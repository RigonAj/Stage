#include "BallTracker.hpp"
#include "Gui.h"
#include "TraceAnalysis.hpp"
#include "util.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <ur3e_catch_msgs/msg/ball_state.hpp>

using namespace std::chrono_literals;

class Pub : public rclcpp::Node {
public:
    Pub()
        : Node("ball_tracking_cpp"),
          resolution(640, 480),
          box(0, 0, resolution.width, resolution.height),
          camera(),
          default_camera_calibration_(camera.calibration),
          gui(camera.Filtered, ui),
	          tracker() {
        gui.SetDisplayView(&camera.Samples);
        gui.SetTraceRawSource(
            &camera.RawFilteredPoints(),
            &camera.RawFilteredTimestamps(),
            &camera.RawFilteredPolarities());
        gui.SetTraceFloatSource(
            &camera.UndistortedFilteredPoints(),
            &camera.UndistortedFilteredTimestamps(),
            &camera.UndistortedFilteredPolarities());

        camera_frame_id_ = this->declare_parameter<std::string>("camera_frame_id", "camera_optical");
        const std::string ball_state_topic =
            this->declare_parameter<std::string>("ball_state_topic", "ball_state");
        const std::string legacy_pose_topic =
            this->declare_parameter<std::string>("legacy_pose_topic", "ball_position_3d_mm");
        publish_legacy_pose_ = this->declare_parameter<bool>("publish_legacy_pose", true);
        const std::string camera_calibration_file =
            this->declare_parameter<std::string>(
                "camera_calibration_file",
                "calibration_camera_DVXplorer_DXA00265-2026_04_23_13_33_50.xml");
        if (!camera.LoadOpenCvCalibrationFile(camera_calibration_file)) {
            throw std::runtime_error("failed to load camera_calibration_file: " + camera_calibration_file);
        }
        default_camera_calibration_ = camera.calibration;
        ui.SetBallRadiusMm(
            static_cast<float>(this->declare_parameter<double>("ball_radius_mm", 20.0)));
        // Lead/coast prediction, initialized from ROS parameters (defaults 0 =
        // publish measurements only). The live-catch launch pins both to 0
        // when the regression node consumes the output; the Option-panel
        // sliders still allow live tuning in standalone runs.
        ui.SetTraceLeadMs(
            static_cast<float>(this->declare_parameter<double>("trace_lead_ms", 0.0)));
        ui.SetTraceHoldMs(
            static_cast<float>(this->declare_parameter<double>("trace_hold_ms", 0.0)));
        // Minimum interval between two trace-analysis runs. The analysis is
        // refreshed from the timer tick as soon as new events arrive (it no
        // longer waits for the 60 FPS render frame); this bounds its CPU cost
        // when live batches arrive at kHz rates. 0 = analyze every new batch.
        gui.SetTraceAnalysisMinPeriodMs(
            static_cast<float>(this->declare_parameter<double>("trace_analysis_period_ms", 4.0)));
        // Input source at startup: false (default) = live DVXplorer camera,
        // true = File/reader playback. Before this parameter the node always
        // started in File mode and silently processed no camera events until
        // the operator clicked "Reader: Camera" (2026-07-16 real-ball session:
        // zero valid Trace samples). The GUI button still toggles it live, and
        // loading a recording re-enables reader mode.
        ui.SetReaderMode(this->declare_parameter<bool>("use_reader", false));
        // Scripted replay of a recordings/ H5 file: forces reader mode and
        // autoplays from the start. Empty (default) = no replay.
        const std::string reader_file =
            this->declare_parameter<std::string>("reader_file", "");
        if (!reader_file.empty()) {
            ui.SetReaderFile(reader_file);
            RCLCPP_INFO(
                this->get_logger(),
                "Replaying recorded events from '%s' (reader_file parameter, autoplay)",
                reader_file.c_str());
        }
        // Preselected file in the reader UI (no mode change): opening the GUI
        // and clicking File/Play replays this session buffer directly.
        const std::string default_reader_file =
            this->declare_parameter<std::string>("default_reader_file", "realtest.h5");
        if (reader_file.empty() && !default_reader_file.empty()) {
            ui.SetDefaultReadFile(default_reader_file);
        }
        // Trace polarity filter: "all" | "positive" | "negative". The former
        // hardcoded GUI default was "negative", which starved the trace of
        // half the ball's events depending on contrast direction.
        const std::string trace_polarity =
            this->declare_parameter<std::string>("trace_polarity_mode", "all");
        if (trace_polarity == "all") {
            ui.SetTracePolarityMode(0);
        }
        else if (trace_polarity == "positive") {
            ui.SetTracePolarityMode(1);
        }
        else if (trace_polarity == "negative") {
            ui.SetTracePolarityMode(2);
        }
        else {
            throw std::runtime_error(
                "trace_polarity_mode must be 'all', 'positive' or 'negative', got '"
                + trace_polarity + "'");
        }
        // Event recording is MANUAL by default (GUI REC toggle, or
        // record:=true to arm it from launch); record_file is the target the
        // writer opens when recording starts. When a writer opens, an
        // existing non-empty target is archived with a timestamp suffix
        // instead of being truncated (2026-07-16 data-loss incident).
        ui.SetSaveFile(this->declare_parameter<std::string>("record_file", "realtest.h5"));
        ui.SetRecord(this->declare_parameter<bool>("record", false));
        gui.SetTracePoseCalibration(camera.calibration, ui.BallRadiusMm());
        // Which 3D estimate feeds BallState: "circle" = per-detection circle-fit
        // pose (legacy algorithm, historical default); "trace" = outlier-filtered
        // mid-window pose from the Trace ribbon pipeline (the primary algorithm:
        // depth from trail width), stamped at the sample's own event time.
        pose_source_ = this->declare_parameter<std::string>("pose_source", "circle");

        if (camera_frame_id_.empty()) {
            throw std::runtime_error("camera_frame_id must not be empty");
        }
        if (pose_source_ != "circle" && pose_source_ != "trace") {
            throw std::runtime_error(
                "pose_source must be 'circle' or 'trace', got '" + pose_source_ + "'");
        }

        ball_state_publisher_ =
            this->create_publisher<ur3e_catch_msgs::msg::BallState>(ball_state_topic, 10);
        if (publish_legacy_pose_) {
            legacy_pose_publisher_ =
                this->create_publisher<std_msgs::msg::Float32MultiArray>(legacy_pose_topic, 10);
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Publishing BallState on '%s' in frame '%s' (pose_source=%s)%s",
            ball_state_topic.c_str(),
            camera_frame_id_.c_str(),
            pose_source_.c_str(),
            publish_legacy_pose_ ? " and legacy ball_position_3d_mm" : "");
        RCLCPP_INFO(
            this->get_logger(),
            "Ball radius set to %.1f mm (ROS parameter ball_radius_mm, adjustable in Option panel)",
            ui.BallRadiusMm());
        RCLCPP_INFO(
            this->get_logger(),
            "Input source: %s (use_reader), trace polarity filter: %s (trace_polarity_mode)",
            ui.UseReader() ? "File/reader" : "live camera",
            trace_polarity.c_str());
        if (ui.Record()) {
            RCLCPP_INFO(
                this->get_logger(),
                "Recording filtered live events to '%s' (record/record_file params; plain names go under recordings/)",
                this->get_parameter("record_file").as_string().c_str());
        }

        timer_ = this->create_wall_timer(1ms, std::bind(&Pub::timer_callback, this));
    }

    ~Pub() override = default;

    void timer_callback();

private:
    // Per-axis quadratic fit of the trace trajectory in the ToMeters world
    // frame: world(t) = c0 + c1*t + c2*t^2, t in seconds relative to originUs.
    struct TraceTrajectoryFit {
        std::array<double, 3> x{{0.0, 0.0, 0.0}};
        std::array<double, 3> y{{0.0, 0.0, 0.0}};
        std::array<double, 3> z{{0.0, 0.0, 0.0}};
        int degree = 1;
        int64_t originUs = 0;
        double tMaxRelSeconds = 0.0;
        rclcpp::Time wallTime;
    };

    void resetTracks();
    void applyInputCalibration();
    void publishBallPose(const BallPose3D &pose);
    void publishBallSample(const cv::Point3f &positionMm, int64_t timestampUs, float confidence);
    void publishTracePose();
    static bool fitTraceTrajectory(const Gui::TraceTrajectory &trajectory, TraceTrajectoryFit &fit);
    static Vector3 evalTraceTrajectoryFit(const TraceTrajectoryFit &fit, double timeSeconds);
    static cv::Point3f traceWorldToCameraMm(const Vector3 &worldMeters);
    builtin_interfaces::msg::Time eventStampToRosTime(int64_t eventTimestampUs);
    void draw2DOverlay(const BallPose3D &pose);
    void drawTrackerResult(const BallTrackerResult &result);
    void drawDbscanClusters(const std::vector<BallTrackerClusterInput> &clusters);

    std::vector<BallTrackerClusterInput> buildTrackerClusters() const;
    BallTrackerSettings trackerSettings() const;
    std::string readerTrackingCacheSignature(
        const BallTrackerSettings &settings,
        int maxEvent,
        int bandwidth,
        uint32_t minNb) const;

private:
    Ui ui;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr legacy_pose_publisher_;
    rclcpp::Publisher<ur3e_catch_msgs::msg::BallState>::SharedPtr ball_state_publisher_;
    std::string camera_frame_id_;
    bool publish_legacy_pose_ = true;
    std::string pose_source_ = "circle";
    // Dedup: latest-sample stamp of the last trace window already consumed.
    int64_t last_trace_source_tmax_us_ = std::numeric_limits<int64_t>::min();
    // Trace-status heartbeat state (peaks between prints, publish counter).
    std::optional<rclcpp::Time> last_trace_status_log_;
    std::size_t trace_status_peak_events_ = 0;
    float trace_status_peak_length_px_ = 0.0f;
    bool trace_status_ribbon_ok_seen_ = false;
    bool trace_status_3d_ok_seen_ = false;
    std::size_t trace_published_count_ = 0;
    // Last live trajectory fit, kept for lead prediction and coasting.
    std::optional<TraceTrajectoryFit> trace_traj_fit_;
    std::optional<int64_t> timestamp_anchor_event_us_;
    std::optional<rclcpp::Time> timestamp_anchor_ros_time_;
    std::optional<rclcpp::Time> last_stamp_conversion_ros_time_;
    cv::Size resolution;
    Box box;

    DvCamera camera;
    CalibrationData default_camera_calibration_;
    Gui gui;
    BallTracker tracker;

    // Reader fast path: last (file, time, window) already read + undistorted.
    // While these are unchanged (paused reader), the tick reuses the cached
    // undistorted window instead of re-reading the H5 and re-undistorting.
    std::string last_reader_processed_path_;
    double last_reader_processed_time_s_ = -1.0;
    double last_reader_processed_window_s_ = -1.0;
    bool reader_processed_valid_ = false;

    std::optional<BallTrackerResult> paused_reader_tracking_cache_;
    double paused_reader_tracking_time_seconds_ = -1.0;
    double paused_reader_tracking_window_seconds_ = -1.0;
    std::string paused_reader_tracking_signature_;
    std::string active_calibration_source_;
    std::string active_reader_path_;
    bool active_reader_calibration_override_ = false;
    bool active_input_state_initialized_ = false;
    bool active_circle_fitting_enabled_ = false;
};
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}

void Pub::applyInputCalibration() {
    const CalibrationData *readerCalibration =
        ui.UseReader() ? gui.ReaderCalibrationOverride() : nullptr;
    const bool useReaderCalibration = readerCalibration != nullptr;
    const CalibrationData &nextCalibration =
        useReaderCalibration ? *readerCalibration : default_camera_calibration_;

    const std::string nextSource = nextCalibration.sourcePath;
    const std::string nextReaderPath = ui.UseReader() ? gui.ReaderEventPath() : "";
    const bool changed =
        !active_input_state_initialized_
        || active_calibration_source_ != nextSource
        || active_reader_path_ != nextReaderPath
        || active_reader_calibration_override_ != useReaderCalibration;

    camera.calibration = nextCalibration;
    gui.SetTracePoseCalibration(camera.calibration, ui.BallRadiusMm());

    if (!changed) {
        return;
    }

    if (active_input_state_initialized_) {
        resetTracks();
    }

    active_calibration_source_ = nextSource;
    active_reader_path_ = nextReaderPath;
    active_reader_calibration_override_ = useReaderCalibration;
    active_input_state_initialized_ = true;

    if (useReaderCalibration) {
        RCLCPP_INFO(
            this->get_logger(),
            "Using sequence calibration for reader: %s",
            nextSource.c_str());
    }
    else {
        RCLCPP_INFO(
            this->get_logger(),
            "Using camera calibration: %s",
            nextSource.c_str());
    }
}

void Pub::timer_callback() {
    using clock = std::chrono::steady_clock;
    const auto t_loop_start = clock::now();

    camera.Samples = dv::EventStore();

    if (gui.ClearPoses) {
        resetTracks();
        gui.ClearPoses = false;
    }

    bool readerNeedsReprocess = false;

    if (!ui.UseReader()) {
        reader_processed_valid_ = false;
        camera.NextBatch();

        if (!camera.isCameraRunning()) {
            camera.Filtered = dv::EventStore();
            gui.ClearCurrentBall3D();
            gui.AddHudText(8.0f, 16.0f,"No DVXplorer camera connected - switch to reader mode or load a .bin file",RED, 22);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "No DVXplorer camera connected: no events processed, no BallState will be published");
            gui.Update(); return;}

        // Live getNextEventBatch() can be empty between real event batches. Do
        // not redraw an empty texture in that case; keep the last camera view
        // on screen until a new batch arrives.
        if (!camera.EventsAvailable()) {
            if (pose_source_ == "trace") {
                publishTracePose();  // keep coasting during event gaps
            }
            return;
        }

        camera.Filter();

        if (ui.Record() && camera.FilteredAvailable()) {
            gui.WriteStore(camera.Filtered);
        }
    }
    else {
        // Paused reader on an unchanged window: the cached undistorted window
        // from the previous tick is still valid, skip the H5 re-read and the
        // full-window re-undistortion (the sliders keep working: clustering
        // reruns below and the trace analysis reruns on any setting change).
        readerNeedsReprocess =
            !reader_processed_valid_
            || gui.ReaderEventPath() != last_reader_processed_path_
            || std::fabs(ui.PlaybackTimeSeconds() - last_reader_processed_time_s_) > 1.0e-9
            || std::fabs(ui.PlaybackWindowSeconds() - last_reader_processed_window_s_) > 1.0e-9;

        if (readerNeedsReprocess) {
            reader_processed_valid_ = false;
            dv::EventStore readerEvents;
            gui.ReadStore(readerEvents);

            if (readerEvents.isEmpty()) {
                camera.Events.reset();
                camera.Filtered = dv::EventStore();
                gui.ClearCurrentBall3D();
                gui.Update();
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "File/reader mode with no events: the live camera is NOT processed "
                    "(set use_reader:=false or click 'Reader' -> Camera in the GUI)");
                if (pose_source_ == "trace") {
                    publishTracePose();  // keep coasting at end of playback
                }
                return;
            }
            camera.Events = std::move(readerEvents);
            camera.Filtered = *camera.Events;
        }
    }

    if (!camera.FilteredAvailable()) {
        if (!ui.UseReader()) {
            if (pose_source_ == "trace") {
                publishTracePose();  // keep coasting while filtered window is empty
            }
            return;
        }
        gui.nb_event = 0;
        gui.ClearCurrentBall3D();
        gui.Update();
        if (pose_source_ == "trace") {
            publishTracePose();
        }
        return;
    }

    gui.ClearCurrentBall3D();
    applyInputCalibration();

    const int maxEvent = static_cast<int>(ui.Maxevent());
    const int bandwidth = ui.Bandwidth();
    const uint32_t minNb = static_cast<uint32_t>(ui.MinNb());

    // Undistortion strategy: the live path undistorts only the fresh batch
    // and maintains a rolling undistorted window (the per-tick full-window
    // recompute was the main CPU cost of the loop). The reader path still
    // recomputes the loaded window, but only when the playback position,
    // window or file actually changed.
    if (!ui.UseReader()) {
        camera.UndistortLiveIncremental(ui.PlaybackWindowSeconds());
    }
    else if (readerNeedsReprocess) {
        camera.Undistort();
        last_reader_processed_path_ = gui.ReaderEventPath();
        last_reader_processed_time_s_ = ui.PlaybackTimeSeconds();
        last_reader_processed_window_s_ = ui.PlaybackWindowSeconds();
        reader_processed_valid_ = true;
    }
    camera.Echantillon(maxEvent);
    gui.nb_event = camera.SampleCount();

    const auto t_pre_end = clock::now();

    const auto t_cluster_start = clock::now();

    // DBSCAN feeds the legacy circle tracker and the cluster overlay of the
    // non-trace views; skip it entirely when neither consumer is active
    // (Trace view with circle fitting off = the live-catch configuration).
    std::vector<BallTrackerClusterInput> trackerClusters;
    if (ui.CircleFittingEnabled() || !ui.ShowTraceView()) {
        camera.Cluster(box, ui.Alpha(), bandwidth, minNb);
        trackerClusters = buildTrackerClusters();
        drawDbscanClusters(trackerClusters);
    }

    const auto t_cluster_end = clock::now();
    const auto t_post_start = clock::now();

    if (active_circle_fitting_enabled_ != ui.CircleFittingEnabled()) {
        active_circle_fitting_enabled_ = ui.CircleFittingEnabled();
        resetTracks();
    }

    const double readerTimeSeconds = ui.PlaybackTimeSeconds();
    const double readerWindowSeconds = ui.PlaybackWindowSeconds();
    const BallTrackerSettings currentTrackerSettings = trackerSettings();
    const std::string readerTrackingSignature =
        readerTrackingCacheSignature(currentTrackerSettings, maxEvent, bandwidth, minNb);
    const bool canReusePausedReaderTracking =
        ui.CircleFittingEnabled()
        &&
        ui.UseReader()
        && !ui.PlaybackPlaying()
        && paused_reader_tracking_cache_.has_value()
        && std::fabs(readerTimeSeconds - paused_reader_tracking_time_seconds_) < 1.0e-9
        && std::fabs(readerWindowSeconds - paused_reader_tracking_window_seconds_) < 1.0e-9
        && readerTrackingSignature == paused_reader_tracking_signature_;

    BallTrackerResult tracking;

    if (canReusePausedReaderTracking) {
        tracking = *paused_reader_tracking_cache_;
    }
    else if (ui.CircleFittingEnabled()) {
        tracking = tracker.Update(
            trackerClusters,
            camera.calibration,
            currentTrackerSettings);

        if (ui.UseReader()) {
            paused_reader_tracking_cache_ = tracking;
            paused_reader_tracking_time_seconds_ = readerTimeSeconds;
            paused_reader_tracking_window_seconds_ = readerWindowSeconds;
            paused_reader_tracking_signature_ = readerTrackingSignature;
        }
        else {
            paused_reader_tracking_cache_.reset();
            paused_reader_tracking_time_seconds_ = -1.0;
            paused_reader_tracking_window_seconds_ = -1.0;
            paused_reader_tracking_signature_.clear();
        }
    }
    else {
        paused_reader_tracking_cache_.reset();
        paused_reader_tracking_time_seconds_ = -1.0;
        paused_reader_tracking_window_seconds_ = -1.0;
        paused_reader_tracking_signature_.clear();
    }

    if (tracking.hasCircle) {
        gui.SetTraceMotionWindow(
            tracking.circle,
            {tracking.imageYFromXFit.a, tracking.imageYFromXFit.b, tracking.imageYFromXFit.c},
            tracking.imageSpaceTrajectory2DValid,
            tracking.imageXMin,
            tracking.imageXMax,
            tracking.circleTimestampUs);
    }
    else {
        gui.ClearTraceMotionWindow();
    }

    // Live: feed the trace with just the fresh batch (its timestamp dedup
    // would skip the rest of the window anyway). Reader: keep the full-window
    // vectors, the loaded window is arbitrary.
    if (ui.TraceUseRawInput()) {
        if (!ui.UseReader()) {
            gui.AppendTraceEvents(
                camera.LiveBatchRawPoints(),
                camera.LiveBatchRawTimestamps(),
                &camera.LiveBatchRawPolarities());
        }
        else {
            gui.AppendTraceEvents(
                camera.RawFilteredPoints(),
                camera.RawFilteredTimestamps(),
                &camera.RawFilteredPolarities());
        }
    }
    else {
        if (!ui.UseReader()) {
            gui.AppendTraceEvents(
                camera.LiveBatchUndistortedPoints(),
                camera.LiveBatchUndistortedTimestamps(),
                &camera.LiveBatchUndistortedPolarities());
        }
        else {
            gui.AppendTraceEvents(
                camera.UndistortedFilteredPoints(),
                camera.UndistortedFilteredTimestamps(),
                &camera.UndistortedFilteredPolarities());
        }
    }

    drawTrackerResult(tracking);

    if (tracking.pose.has_value()) {
        if (pose_source_ == "circle") {
            publishBallPose(*tracking.pose);
        }
        draw2DOverlay(*tracking.pose);
    }
    else if (!ui.CircleFittingEnabled()) {
        gui.AddHudText(8.0f, 42.0f, "Circle fitting: OFF", DARKGRAY, 22);
    }
    else {
        gui.AddHudText(8.0f, 42.0f, "Ball pose: no valid 3D estimate", MAROON, 22);
    }

    // Refresh the trace analysis from this tick's events and publish BEFORE
    // the render gate: a fresh pose no longer waits up to 16.7 ms for the
    // next 60 FPS frame (RefreshTraceAnalysis dedups unchanged runs).
    gui.RefreshTraceAnalysis();
    if (pose_source_ == "trace") {
        publishTracePose();
    }

    const auto t_end = clock::now();
    gui.ms_pre = std::chrono::duration<double, std::milli>(t_pre_end - t_loop_start).count();
    gui.ms_cluster = std::chrono::duration<double, std::milli>(t_cluster_end - t_cluster_start).count();
    gui.ms_post = std::chrono::duration<double, std::milli>(t_end - t_post_start).count();
    gui.ms_loop = std::chrono::duration<double, std::milli>(t_end - t_loop_start).count();

    gui.Update();
}

std::vector<BallTrackerClusterInput> Pub::buildTrackerClusters() const {
    std::vector<BallTrackerClusterInput> output;
    const auto &cameraClusters = camera.Clusters();
    output.reserve(cameraClusters.size());

    for (const auto &cluster : cameraClusters) {
        BallTrackerClusterInput input;
        input.maxTimestamp = cluster.maxTimestamp;
        input.minTimestamp = cluster.minTimestamp;

        const std::vector<cv::Point2f> &points = cluster.points;

        input.points.reserve(points.size());
        input.polarities.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            const cv::Point2f &point = points[i];

            if (camera.calibration.ready
                && (point.x < 0.0f
                    || point.x >= static_cast<float>(camera.calibration.imageSize.width)
                    || point.y < 0.0f
                    || point.y >= static_cast<float>(camera.calibration.imageSize.height))) {
                continue;
            }

            input.points.emplace_back(point);
            input.polarities.emplace_back(polar{cluster.polarities[i], cluster.timestamps[i]});
        }

        if (!input.points.empty()) {
            output.emplace_back(std::move(input));
        }
    }

    return output;
}

BallTrackerSettings Pub::trackerSettings() const {
    BallTrackerSettings settings;
    settings.ballRadiusMm = ui.BallRadiusMm();
    settings.positiveOnly = ui.positive_only;
    settings.coef = ui.Coef();
    settings.filterSize = ui.FilterSize();
    settings.maxResidual = ui.MaxResidual();
    settings.rayonCote = ui.rayon_cote;
    settings.symCoef = ui.Sym_coef();
    settings.symCoef2 = ui.Sym_coef2();
    settings.alpha = ui.Alpha();
    settings.radiusGateEnabled = ui.TraceUseRadiusGate();
    settings.weightedRegressionEnabled = ui.WeightedRegressionEnabled();
    settings.sliceMode = static_cast<BallSliceMode>(ui.SliceMode());
    settings.temporalSliceCount = ui.TemporalSlices();
    settings.eventsPerSlice = ui.EventsPerSlice();
    return settings;
}

std::string Pub::readerTrackingCacheSignature(
    const BallTrackerSettings &settings,
    int maxEvent,
    int bandwidth,
    uint32_t minNb) const {
    std::ostringstream signature;
    signature.setf(std::ios::fixed);
    signature << std::setprecision(6)
              << "maxEvent=" << maxEvent
              << ";bandwidth=" << bandwidth
              << ";minNb=" << minNb
              << ";ballRadiusMm=" << settings.ballRadiusMm
              << ";positiveOnly=" << settings.positiveOnly
              << ";coef=" << settings.coef
              << ";filterSize=" << settings.filterSize
              << ";maxResidual=" << settings.maxResidual
              << ";rayonCote=" << settings.rayonCote
              << ";symCoef=" << settings.symCoef
              << ";symCoef2=" << settings.symCoef2
              << ";alpha=" << settings.alpha
              << ";radiusGate=" << settings.radiusGateEnabled
              << ";weightedRegression=" << settings.weightedRegressionEnabled
              << ";sliceMode=" << static_cast<int>(settings.sliceMode)
              << ";temporalSliceCount=" << settings.temporalSliceCount
              << ";eventsPerSlice=" << settings.eventsPerSlice;
    return signature.str();
}

void Pub::drawTrackerResult(const BallTrackerResult &result) {
    if (result.hasCircle) {
        const Circle &c = result.circle;
        gui.AddCircle(c.x, c.y, c.r, GREEN);
        gui.AddMarker(c.x, c.y, BLUE);
        gui.AddArrow(c.x, c.y, result.arrowEnd.x, result.arrowEnd.y, BLUE);
        gui.AddRect(box.x, box.y, box.w, box.h, RED);
    }

    if (result.imageTrajectory2DValid) {
        gui.SetImageTrajectory2D(
            {result.imageXFit.a, result.imageXFit.b},
            {result.imageYFit.a, result.imageYFit.b, result.imageYFit.c},
            result.imageTMin,
            result.imageTMax,
            true
        );
    }
    if (result.pose.has_value()) {
        std::ostringstream label;
        label.setf(std::ios::fixed);
        label << std::setprecision(3)
              << "Ball 3D position (m): X=" << result.worldPosition.x
              << "  Y=" << result.worldPosition.y
              << "  Z=" << result.worldPosition.z;

        gui.SetBall3D(
            result.worldPosition,
            ui.BallRadiusMm() * 1.0e-3f,
            label.str());

        gui.SetTrajectory3D(
            result.worldTrack,
            result.worldTrackTimes,
            {result.xFit.a, result.xFit.b},
            {result.yFit.a, result.yFit.b},
            {result.zFit.a, result.zFit.b, result.zFit.c},
            result.tMin,
            result.tMax,
            result.trajectoryValid);
    }
}

void Pub::drawDbscanClusters(const std::vector<BallTrackerClusterInput> &clusters) {
    for (const auto &cluster : clusters) {
        if (!cluster.points.empty()) {
            gui.AddClusterView(cluster.points);
        }
    }
}

void Pub::resetTracks() {
    tracker.Reset();
    camera.ResetLiveWindow();
    reader_processed_valid_ = false;
    paused_reader_tracking_cache_.reset();
    paused_reader_tracking_time_seconds_ = -1.0;
    paused_reader_tracking_window_seconds_ = -1.0;
    paused_reader_tracking_signature_.clear();
    gui.ClearImageTrajectory2D();
    gui.ClearTrajectory3D();
    gui.ClearTrace3D();
    gui.ClearTraceMotionWindow();
    gui.ResetTraceAccumulation();
    gui.ClearCurrentBall3D();
    timestamp_anchor_event_us_.reset();
    timestamp_anchor_ros_time_.reset();
    last_stamp_conversion_ros_time_.reset();
    last_trace_source_tmax_us_ = std::numeric_limits<int64_t>::min();
    trace_traj_fit_.reset();
}

builtin_interfaces::msg::Time Pub::eventStampToRosTime(const int64_t eventTimestampUs) {
    const rclcpp::Time now = this->get_clock()->now();
    if (eventTimestampUs <= 0) {
        return now;
    }

    // Re-anchor the event->ROS mapping after every publish gap: the DVS clock
    // drifts against ROS time, and a single session-long anchor would slowly
    // push stamps outside downstream freshness guards (silent perception
    // death). Between-throw silence gives a drift-free re-anchor point while
    // intra-flight stamps stay monotonic (publishes are milliseconds apart).
    constexpr double REANCHOR_AFTER_GAP_S = 0.5;
    const bool gap_expired =
        last_stamp_conversion_ros_time_.has_value()
        && (now - *last_stamp_conversion_ros_time_).seconds() > REANCHOR_AFTER_GAP_S;

    if (!timestamp_anchor_event_us_.has_value()
        || !timestamp_anchor_ros_time_.has_value()
        || eventTimestampUs < *timestamp_anchor_event_us_
        || gap_expired) {
        timestamp_anchor_event_us_ = eventTimestampUs;
        timestamp_anchor_ros_time_ = now;
    }
    last_stamp_conversion_ros_time_ = now;

    const int64_t deltaNs = (eventTimestampUs - *timestamp_anchor_event_us_) * 1000;
    return (*timestamp_anchor_ros_time_ + rclcpp::Duration::from_nanoseconds(deltaNs));
}

void Pub::publishBallPose(const BallPose3D &pose) {
    if (!pose.valid) {
        return;
    }
    publishBallSample(pose.positionMm, pose.timestampUs, 1.0f);
}

cv::Point3f Pub::traceWorldToCameraMm(const Vector3 &worldMeters) {
    // Inverse of the util.hpp ToMeters remap (camera mm {x,y,z} -> world m
    // {x, z, -y}): back to the camera_optical pinhole convention (x right,
    // y down, z forward) that this node declares in frame_id.
    return {
        worldMeters.x * 1.0e3f,
        -worldMeters.z * 1.0e3f,
        worldMeters.y * 1.0e3f,
    };
}

bool Pub::fitTraceTrajectory(const Gui::TraceTrajectory &trajectory, TraceTrajectoryFit &fit) {
    const std::size_t n = trajectory.worldPoints.size();
    if (n < 3 || trajectory.times.size() != n) {
        return false;
    }

    std::vector<double> t(n), xs(n), ys(n), zs(n), weights(n, 1.0);
    double tMin = trajectory.times.front();
    double tMax = trajectory.times.front();
    for (std::size_t i = 0; i < n; ++i) {
        t[i] = static_cast<double>(trajectory.times[i]);
        xs[i] = static_cast<double>(trajectory.worldPoints[i].x);
        ys[i] = static_cast<double>(trajectory.worldPoints[i].y);
        zs[i] = static_cast<double>(trajectory.worldPoints[i].z);
        tMin = std::min(tMin, t[i]);
        tMax = std::max(tMax, t[i]);
    }

    // Need a real time baseline to fit and extrapolate a trajectory.
    if (!(tMax - tMin > 1.0e-4)) {
        return false;
    }

    // Quadratic (ballistic) once there is enough support; linear otherwise.
    const int degree = n >= 5 ? 2 : 1;
    if (!SolveWeightedPolynomialFit(t, xs, weights, degree, fit.x)
        || !SolveWeightedPolynomialFit(t, ys, weights, degree, fit.y)
        || !SolveWeightedPolynomialFit(t, zs, weights, degree, fit.z)) {
        return false;
    }

    fit.degree = degree;
    fit.tMaxRelSeconds = tMax;
    return true;
}

Vector3 Pub::evalTraceTrajectoryFit(const TraceTrajectoryFit &fit, const double timeSeconds) {
    const auto axis = [&](const std::array<double, 3> &c) {
        return c[0] + c[1] * timeSeconds + c[2] * timeSeconds * timeSeconds;
    };
    return Vector3{
        static_cast<float>(axis(fit.x)),
        static_cast<float>(axis(fit.y)),
        static_cast<float>(axis(fit.z)),
    };
}

void Pub::publishTracePose() {
    const double leadSeconds = ui.TraceLeadSeconds();
    const double holdSeconds = ui.TraceHoldSeconds();
    const rclcpp::Time now = this->get_clock()->now();

    // Terminal heartbeat of the trace pipeline stages: without it a live
    // session where the ribbon never validates logs nothing at all
    // (2026-07-16 real-ball session), leaving the failure boundary invisible.
    // Peaks are tracked between prints because ball flights last ~200 ms and
    // a 2 s sample would miss the burst.
    {
        const Gui::TraceDebugStatus st = gui.CurrentTraceStatus();
        trace_status_peak_events_ = std::max(trace_status_peak_events_, st.sourceEvents);
        trace_status_peak_length_px_ = std::max(trace_status_peak_length_px_, st.ribbonLengthPx);
        trace_status_ribbon_ok_seen_ = trace_status_ribbon_ok_seen_ || st.ribbonValid;
        trace_status_3d_ok_seen_ = trace_status_3d_ok_seen_ || st.valid3d;
        if (!last_trace_status_log_.has_value()
            || (now - *last_trace_status_log_).seconds() >= 2.0) {
            RCLCPP_INFO(
                this->get_logger(),
                "trace status: events=%zu (peak %zu) ribbon=%s (peak len %.0fpx, seen ok=%d) "
                "world_pts=%zu 3d=%s (seen ok=%d) published=%zu",
                st.sourceEvents,
                trace_status_peak_events_,
                st.ribbonValid ? "ok" : "invalid",
                trace_status_peak_length_px_,
                trace_status_ribbon_ok_seen_ ? 1 : 0,
                st.worldPoints,
                st.valid3d ? "ok" : "invalid",
                trace_status_3d_ok_seen_ ? 1 : 0,
                trace_published_count_);
            last_trace_status_log_ = now;
            trace_status_peak_events_ = 0;
            trace_status_peak_length_px_ = 0.0f;
            trace_status_ribbon_ok_seen_ = false;
            trace_status_3d_ok_seen_ = false;
        }
    }

    const Gui::TraceTrajectory trajectory = gui.CurrentTraceTrajectory();

    // A fresh window is one that is valid, has enough support and whose latest
    // sample is strictly newer than the last one we already fitted (dedup of
    // repeated render frames on the same accumulated events).
    bool freshWindow = false;
    int64_t tMaxUs = std::numeric_limits<int64_t>::min();
    if (trajectory.valid
        && trajectory.worldPoints.size() >= 3
        && trajectory.worldPoints.size() == trajectory.times.size()) {
        const double tMaxRel = static_cast<double>(
            *std::max_element(trajectory.times.begin(), trajectory.times.end()));
        tMaxUs = trajectory.originUs + static_cast<int64_t>(std::llround(tMaxRel * 1.0e6));
        freshWindow = tMaxUs > last_trace_source_tmax_us_;
    }

    if (freshWindow) {
        TraceTrajectoryFit fit;
        if (fitTraceTrajectory(trajectory, fit)) {
            fit.originUs = trajectory.originUs;
            fit.wallTime = now;
            trace_traj_fit_ = fit;
            last_trace_source_tmax_us_ = tMaxUs;

            // Publish the position predicted at (latest sample + lead).
            const double tEval = fit.tMaxRelSeconds + leadSeconds;
            const Vector3 world = evalTraceTrajectoryFit(fit, tEval);
            const int64_t stampUs =
                fit.originUs + static_cast<int64_t>(std::llround(tEval * 1.0e6));
            publishBallSample(traceWorldToCameraMm(world), stampUs, 1.0f);
        }
        return;
    }

    // No fresh window this frame: coast by extrapolating the last fit forward
    // for up to holdSeconds after the ball left the ROI / the trace stopped.
    if (holdSeconds <= 0.0 || !trace_traj_fit_.has_value()) {
        return;
    }
    const double elapsed = (now - trace_traj_fit_->wallTime).seconds();
    if (elapsed < 0.0) {
        return;
    }
    if (elapsed > holdSeconds) {
        trace_traj_fit_.reset();
        return;
    }

    const double tEval = trace_traj_fit_->tMaxRelSeconds + leadSeconds + elapsed;
    const Vector3 world = evalTraceTrajectoryFit(*trace_traj_fit_, tEval);
    const int64_t stampUs =
        trace_traj_fit_->originUs + static_cast<int64_t>(std::llround(tEval * 1.0e6));
    const float confidence =
        static_cast<float>(std::clamp(1.0 - elapsed / holdSeconds, 0.0, 1.0));
    publishBallSample(traceWorldToCameraMm(world), stampUs, confidence);
}

void Pub::publishBallSample(const cv::Point3f &position, const int64_t timestampUs, const float confidence) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
        return;
    }

    if (ball_state_publisher_) {
        ++trace_published_count_;
        ur3e_catch_msgs::msg::BallState msg;
        msg.header.stamp = eventStampToRosTime(timestampUs);
        msg.header.frame_id = camera_frame_id_;
        msg.position.x = static_cast<double>(position.x) * 1.0e-3;
        msg.position.y = static_cast<double>(position.y) * 1.0e-3;
        msg.position.z = static_cast<double>(position.z) * 1.0e-3;
        // Velocity left at (0,0,0) = "not provided": the downstream consumer
        // recomputes it (EMA / regression). See BallState.msg contract.
        msg.velocity.x = 0.0;
        msg.velocity.y = 0.0;
        msg.velocity.z = 0.0;
        msg.valid = true;
        msg.confidence = confidence;
        ball_state_publisher_->publish(msg);
    }

    if (legacy_pose_publisher_) {
        std_msgs::msg::Float32MultiArray msg;
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label = "xyz_mm";
        msg.layout.dim[0].size = 3;
        msg.layout.dim[0].stride = 3;
        msg.layout.data_offset = 0;
        msg.data = {position.x, position.y, position.z};

        legacy_pose_publisher_->publish(msg);
    }
}

void Pub::draw2DOverlay(const BallPose3D &pose) {
    gui.AddMarker(pose.circle.x, pose.circle.y, YELLOW);

    gui.AddLabel(
        pose.circle.x + 14.0f,
        pose.circle.y - 28.0f,
        ballPoseToString(pose),
        BLACK,
        18);

    std::string s = fmt::format("d={:.1f} mm  r={:.1f} px", pose.depthMm, pose.RadiusPx);

    gui.AddLabel(
        pose.circle.x + 14.0f,
        pose.circle.y - 8.0f,
        s,
        DARKGREEN,
        18);
}
