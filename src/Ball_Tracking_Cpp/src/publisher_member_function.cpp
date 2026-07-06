#include "BallTracker.hpp"
#include "Gui.h"
#include "util.hpp"

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
        gui.SetTracePoseCalibration(camera.calibration, BALL_RADIUS_MM);
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

        timer_ = this->create_wall_timer(1ms, std::bind(&Pub::timer_callback, this));
    }

    ~Pub() override = default;

    void timer_callback();

private:
    static constexpr float BALL_RADIUS_MM = 20.0f;

    void resetTracks();
    void applyInputCalibration();
    void publishBallPose(const BallPose3D &pose);
    void publishBallSample(const cv::Point3f &positionMm, int64_t timestampUs);
    void publishTracePose();
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
    int64_t last_published_trace_stamp_us_ = std::numeric_limits<int64_t>::min();
    std::optional<int64_t> timestamp_anchor_event_us_;
    std::optional<rclcpp::Time> timestamp_anchor_ros_time_;
    std::optional<rclcpp::Time> last_stamp_conversion_ros_time_;
    cv::Size resolution;
    Box box;

    DvCamera camera;
    CalibrationData default_camera_calibration_;
    Gui gui;
    BallTracker tracker;

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
    gui.SetTracePoseCalibration(camera.calibration, BALL_RADIUS_MM);

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

    if (!ui.UseReader()) {
        camera.NextBatch();

        if (!camera.isCameraRunning()) {
            camera.Filtered = dv::EventStore();
            gui.ClearCurrentBall3D();
            gui.AddHudText(8.0f, 16.0f,"No DVXplorer camera connected - switch to reader mode or load a .bin file",RED, 22);
            gui.Update(); return;}

        // Live getNextEventBatch() can be empty between real event batches. Do
        // not redraw an empty texture in that case; keep the last camera view
        // on screen until a new batch arrives.
        if (!camera.EventsAvailable()) {return; }

        camera.Filter();

        if (ui.Record() && camera.FilteredAvailable()) {
            gui.WriteStore(camera.Filtered);
        }

        camera.KeepRecentFiltered(ui.PlaybackWindowSeconds());
    }
    else {
        dv::EventStore readerEvents;
        gui.ReadStore(readerEvents);

        if (readerEvents.isEmpty()) {
            camera.Events.reset();
            camera.Filtered = dv::EventStore();
            gui.ClearCurrentBall3D();
            gui.Update();
            return;
        }
        camera.Events = std::move(readerEvents);
        camera.Filtered = *camera.Events;
    }

    if (!camera.FilteredAvailable()) {
        if (!ui.UseReader()) {
            return;
        }
        gui.nb_event = 0;
        gui.ClearCurrentBall3D();
        gui.Update();
        return;
    }

    gui.ClearCurrentBall3D();
    applyInputCalibration();
    gui.nb_event = camera.FilteredCount();

    // Undistort must run on the full filtered window: the trace accumulation
    // feeds on the undistorted points, and running it after Echantillon would
    // cap the trail density to Maxevent subsampled events per window.
    const int maxEvent = static_cast<int>(ui.Maxevent());
    const int bandwidth = ui.Bandwidth();
    const uint32_t minNb = static_cast<uint32_t>(ui.MinNb());

    camera.Undistort();
    camera.Echantillon(maxEvent);

    const auto t_pre_end = clock::now();

    const auto t_cluster_start = clock::now();

    camera.Cluster(box, ui.Alpha(), bandwidth, minNb);

    const auto trackerClusters = buildTrackerClusters();
    drawDbscanClusters(trackerClusters);

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

    if (ui.TraceUseRawInput()) {
        gui.AppendTraceEvents(
            camera.RawFilteredPoints(),
            camera.RawFilteredTimestamps(),
            &camera.RawFilteredPolarities());
    }
    else {
        gui.AppendTraceEvents(
            camera.UndistortedFilteredPoints(),
            camera.UndistortedFilteredTimestamps(),
            &camera.UndistortedFilteredPolarities());
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

    const auto t_end = clock::now();
    gui.ms_pre = std::chrono::duration<double, std::milli>(t_pre_end - t_loop_start).count();
    gui.ms_cluster = std::chrono::duration<double, std::milli>(t_cluster_end - t_cluster_start).count();
    gui.ms_post = std::chrono::duration<double, std::milli>(t_end - t_post_start).count();
    gui.ms_loop = std::chrono::duration<double, std::milli>(t_end - t_loop_start).count();

    gui.Update();

    // The trace analysis is refreshed inside gui.Update(), so the trace pose
    // is published after it, from this frame's ribbon fit.
    if (pose_source_ == "trace") {
        publishTracePose();
    }
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
    settings.ballRadiusMm = BALL_RADIUS_MM;
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
            BALL_RADIUS_MM * 1.0e-3f,
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
    last_published_trace_stamp_us_ = std::numeric_limits<int64_t>::min();
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
    publishBallSample(pose.positionMm, pose.timestampUs);
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

void Pub::publishTracePose() {
    const Gui::TracePoseSample sample = gui.CurrentTracePoseSample();
    // The mid-window sample only advances when new trace events arrive;
    // requiring a strictly newer stamp deduplicates repeated render frames.
    if (!sample.valid || sample.timestampUs <= last_published_trace_stamp_us_) {
        return;
    }
    last_published_trace_stamp_us_ = sample.timestampUs;
    publishBallSample(traceWorldToCameraMm(sample.worldMeters), sample.timestampUs);
}

void Pub::publishBallSample(const cv::Point3f &position, const int64_t timestampUs) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
        return;
    }

    if (ball_state_publisher_) {
        ur3e_catch_msgs::msg::BallState msg;
        msg.header.stamp = eventStampToRosTime(timestampUs);
        msg.header.frame_id = camera_frame_id_;
        msg.position.x = static_cast<double>(position.x) * 1.0e-3;
        msg.position.y = static_cast<double>(position.y) * 1.0e-3;
        msg.position.z = static_cast<double>(position.z) * 1.0e-3;
        msg.velocity.x = 0.0;
        msg.velocity.y = 0.0;
        msg.velocity.z = 0.0;
        msg.valid = true;
        msg.confidence = 1.0f;
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
