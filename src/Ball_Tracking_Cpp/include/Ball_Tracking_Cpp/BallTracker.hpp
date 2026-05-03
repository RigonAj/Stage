#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "RegressionAccumulator.hpp"
#include "util.hpp"

struct BallTrackerSettings {
    float ballRadiusMm = 20.0f;
    bool positiveOnly = false;
    float coef = 0.45f;
    float filterSize = 115.0f;
    float maxResidual = 19.0f;
    float rayonCote = 0.0f;
    float symCoef = 29.0f;
    float symCoef2 = 157.0f;
};

struct BallTrackerClusterInput {
    std::vector<cv::Point2f> points;
    std::vector<polar> polarities;
    int64_t maxTimestamp = 0;

    std::size_t size() const { return points.size(); }
};

struct BallTrackerResult {
    std::vector<std::vector<cv::Point2f>> clusterViews;

    bool hasCircle = false;
    Circle circle{0.0f, 0.0f, 0.0f};
    cv::Point2f arrowEnd{0.0f, 0.0f};

    bool parabola2DValid = false;
    coef2 parabola2D{};

    std::optional<BallPose3D> pose;
    int64_t poseTimestampUs = 0;

    bool updatedTrajectory = false;
    Vector3 worldPosition{0.0f, 0.0f, 0.0f};
    std::vector<Vector3> worldTrack;
    coef xFit{};
    coef yFit{};
    coef2 zFit{};
    float tMin = 0.0f;
    float tMax = 0.0f;
    bool trajectoryValid = false;
};

class BallTracker {
public:
    BallTracker();

    void Reset();

    BallTrackerResult Update(
        const std::vector<BallTrackerClusterInput> &clusters,
        const CalibrationData &calibration,
        const BallTrackerSettings &settings);

    const std::optional<BallPose3D>& LastPose() const { return last_ball_pose_; }

private:
    Circle Normalize(const std::vector<cv::Point2f> &points);
    Circle Regression(const std::vector<cv::Point2f> &points, const Circle &norm) const;
    float Rms(const std::vector<cv::Point2f> &points, const Circle &c, float drLocal) const;

    float PolarFilter(
        const std::vector<cv::Point2f> &points,
        const std::vector<polar> &polarityValues,
        const Circle &c,
        const BallTrackerSettings &settings) const;

    std::vector<cv::Point2f> OutlierFilter(
        const std::vector<cv::Point2f> &points,
        float coefValue,
        Circle c) const;

    void Update3DTrack(
        int64_t poseTimestampUs,
        const BallPose3D &pose,
        BallTrackerResult &result);

private:
    cv::Point2f last_pt_{0.0f, 0.0f};
    std::vector<cv::Point2f> circle_centers_;
    float dr_ = 0.0f;

    coef2 parabola_{};
    QuadraticRegression image_parabola_reg_;

    std::vector<Vector3> world_track_;
    std::vector<float> track_time_seconds_;

    LinearRegression x_reg_;
    LinearRegression y_reg_;
    QuadraticRegression z_reg_;

    coef x_fit_{};
    coef y_fit_{};
    coef2 z_fit_{};

    int64_t track_start_timestamp_us_ = -1;
    int64_t last_max_time_ = 0;

    std::optional<BallPose3D> last_ball_pose_;
};
