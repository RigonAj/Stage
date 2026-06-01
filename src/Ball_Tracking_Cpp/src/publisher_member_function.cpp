#include "BallTracker.hpp"
#include "Gui.h"
#include "util.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

class Pub : public rclcpp::Node {
public:
    Pub()
        : Node("minimal_publisher"),
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

        timer_ = this->create_wall_timer(1ms, std::bind(&Pub::timer_callback, this));
        pose_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ball_position_3d_mm", 10);
    }

    ~Pub() override = default;

    void timer_callback();

private:
    static constexpr float BALL_RADIUS_MM = 20.0f;

    void resetTracks();
    void applyInputCalibration();
    void publishBallPose(const BallPose3D &pose) const;
    void draw2DOverlay(const BallPose3D &pose);
    void drawTrackerResult(const BallTrackerResult &result);

    std::vector<BallTrackerClusterInput> buildTrackerClusters() const;
    BallTrackerSettings trackerSettings() const;

private:
    Ui ui;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_publisher_;
    cv::Size resolution;
    Box box;

    DvCamera camera;
    CalibrationData default_camera_calibration_;
    Gui gui;
    BallTracker tracker;

    std::optional<BallTrackerResult> paused_reader_tracking_cache_;
    double paused_reader_tracking_time_seconds_ = -1.0;
    double paused_reader_tracking_window_seconds_ = -1.0;
    std::string active_calibration_source_;
    std::string active_reader_path_;
    bool active_reader_calibration_override_ = false;
    bool active_input_state_initialized_ = false;
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
    camera.Filtered = dv::EventStore();

    gui.ClearCurrentBall3D();

    if (gui.ClearPoses) {
        resetTracks();
        gui.ClearPoses = false;
    }

    if (!ui.UseReader()) {
        camera.NextBatch();

        if (!camera.isCameraRunning()) {
            gui.AddHudText(8.0f, 16.0f,"No DVXplorer camera connected - switch to reader mode or load a .bin file",RED, 22);
            gui.Update(); return;}

        if (!camera.EventsAvailable()) {gui.Update();return; }

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
            gui.Update();
            return;
        }
        camera.Events = std::move(readerEvents);
        camera.Filtered = *camera.Events;
    }

    if (!camera.FilteredAvailable()) {
        gui.nb_event = 0;
        gui.Update();
        return;
    }

    applyInputCalibration();
    camera.Undistort();
    gui.nb_event = camera.FilteredCount();

    camera.Echantillon(ui.Maxevent());

    const auto t_pre_end = clock::now();

    const auto t_cluster_start = clock::now();

    camera.Cluster(box,ui.Alpha(), ui.Bandwidth(), static_cast<uint32_t>(ui.MinNb()));

    const auto trackerClusters = buildTrackerClusters();

    const auto t_cluster_end = clock::now();
    const auto t_post_start = clock::now();

    const double readerTimeSeconds = ui.PlaybackTimeSeconds();
    const double readerWindowSeconds = ui.PlaybackWindowSeconds();
    const bool canReusePausedReaderTracking =
        ui.UseReader()
        && !ui.PlaybackPlaying()
        && paused_reader_tracking_cache_.has_value()
        && std::fabs(readerTimeSeconds - paused_reader_tracking_time_seconds_) < 1.0e-9
        && std::fabs(readerWindowSeconds - paused_reader_tracking_window_seconds_) < 1.0e-9;

    BallTrackerResult tracking;

    if (canReusePausedReaderTracking) {
        tracking = *paused_reader_tracking_cache_;
    }
    else {
        tracking = tracker.Update(
            trackerClusters,
            camera.calibration,
            trackerSettings());

        if (ui.UseReader()) {
            paused_reader_tracking_cache_ = tracking;
            paused_reader_tracking_time_seconds_ = readerTimeSeconds;
            paused_reader_tracking_window_seconds_ = readerWindowSeconds;
        }
        else {
            paused_reader_tracking_cache_.reset();
            paused_reader_tracking_time_seconds_ = -1.0;
            paused_reader_tracking_window_seconds_ = -1.0;
        }
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
        publishBallPose(*tracking.pose);
        draw2DOverlay(*tracking.pose);
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
    settings.sliceMode = static_cast<BallSliceMode>(ui.SliceMode());
    settings.temporalSliceCount = ui.TemporalSlices();
    settings.eventsPerSlice = ui.EventsPerSlice();
    return settings;
}

void Pub::drawTrackerResult(const BallTrackerResult &result) {
    for (const auto &clusterView : result.clusterViews) {
        gui.AddClusterView(clusterView);
    }

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

void Pub::resetTracks() {
    tracker.Reset();
    paused_reader_tracking_cache_.reset();
    paused_reader_tracking_time_seconds_ = -1.0;
    paused_reader_tracking_window_seconds_ = -1.0;
    gui.ClearImageTrajectory2D();
    gui.ClearTrajectory3D();
    gui.ClearTrace3D();
    gui.ClearTraceMotionWindow();
    gui.ResetTraceAccumulation();
    gui.ClearCurrentBall3D();
}

void Pub::publishBallPose(const BallPose3D &pose) const {
    if (!pose_publisher_) {
        return;
    }

    std_msgs::msg::Float32MultiArray msg;

    pose_publisher_->publish(msg);
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
