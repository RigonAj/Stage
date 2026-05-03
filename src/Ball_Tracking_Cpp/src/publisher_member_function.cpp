#include "BallTracker.hpp"
#include "Gui.h"
#include "util.hpp"

#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

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
          gui(camera.Filtered, ui),
          tracker() {

        timer_ = this->create_wall_timer(4ms, std::bind(&Pub::timer_callback, this));
        pose_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ball_position_3d_mm", 10);
    }

    ~Pub() override = default;

    void timer_callback();

private:
    static constexpr float BALL_RADIUS_MM = 20.0f;

    void resetTracks();
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
    Gui gui;
    BallTracker tracker;
};
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}

void Pub::timer_callback() {
    using clock = std::chrono::steady_clock;
    const auto t_loop_start = clock::now();

    camera.Samples = dv::EventStore();
    camera.Filtered = dv::EventStore();

    gui.ClearCurrentBall3D();

    if (ui.Lector() < 0.01f) {
        camera.NextBatch();

        if (!camera.isCameraRunning()) {
            gui.AddHudText(8.0f, 16.0f,"No DVXplorer camera connected - switch to reader mode or load a .bin file",RED, 22);
            gui.Update(); return;}

        if (!camera.EventsAvailable()) {gui.Update();return; }

        if (ui.Record() && camera.Events.has_value()) gui.WriteStore(*camera.Events);

        camera.Filter();
        camera.Undistort();
    }
    else {
        dv::EventStore readerEvents;
        gui.ReadStore(readerEvents, ui.Lector(), ui.Time_Slice());

        if (readerEvents.isEmpty()) {
            camera.Events.reset();
            gui.Update();
            return;
        }
        camera.Events = std::move(readerEvents);
        camera.Filtered = *camera.Events;
        camera.Undistort();
    }

    if (!camera.FilteredAvailable()) {
        gui.nb_event = 0;
        gui.Update();
        return;
    }

    gui.nb_event = camera.EventCount();

    camera.Echantillon(ui.Maxevent());

    const auto t_pre_end = clock::now();

    if (gui.ClearPoses) {resetTracks();gui.ClearPoses = false;}

    const auto t_cluster_start = clock::now();

    camera.Cluster(box,ui.Alpha(), ui.Bandwidth(), static_cast<uint32_t>(ui.MinNb()));

    const auto trackerClusters = buildTrackerClusters();

    const auto t_cluster_end = clock::now();
    const auto t_post_start = clock::now();

    const BallTrackerResult tracking = tracker.Update(
        trackerClusters,
        camera.calibration,
        trackerSettings());

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
        input.points = cluster.points;
        input.maxTimestamp = cluster.maxTimestamp;
        input.polarities.reserve(cluster.size());

        for (size_t i = 0; i < cluster.size(); ++i) {
            input.polarities.emplace_back(polar{cluster.polarities[i], cluster.timestamps[i]});
        }

        output.emplace_back(std::move(input));
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

    if (result.parabola2DValid) {
        gui.Parabole(result.parabola2D.a, result.parabola2D.b, result.parabola2D.c);
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
    gui.Parabole(0.0f, 0.0f, 0.0f);
    gui.ClearTrajectory3D();
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

    std::string s = fmt::format("d={:.1f} mm  r_u={:.1f} px", pose.depthMm, pose.RadiusPx);

    gui.AddLabel(
        pose.circle.x + 14.0f,
        pose.circle.y - 8.0f,
        s,
        DARKGREEN,
        18);
}