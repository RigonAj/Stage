#include "BallTracker.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Dense>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

BallTracker::BallTracker() {
    circle_centers_.reserve(50000);
    world_track_.reserve(2000);
    track_time_seconds_.reserve(2000);
}

void BallTracker::Reset() {
    last_pt_ = {0.0f, 0.0f};

    circle_centers_.clear();
    image_parabola_reg_.reset();

    dr_ = 0.0f;
    parabola_ = {};

    world_track_.clear();
    track_time_seconds_.clear();

    x_reg_.reset();
    y_reg_.reset();
    z_reg_.reset();

    x_fit_ = {};
    y_fit_ = {};
    z_fit_ = {};

    track_start_timestamp_us_ = -1;
    last_max_time_ = 0;
    last_ball_pose_.reset();
}

BallTrackerResult BallTracker::Update(
    const std::vector<BallTrackerClusterInput> &clusters,
    const CalibrationData &calibration,
    const BallTrackerSettings &settings) {

    BallTrackerResult result;
    last_ball_pose_.reset();

    for (const auto &cluster : clusters) {
        std::vector<cv::Point2f> points = cluster.points;
        std::vector<polar> polarities = cluster.polarities;

        if (points.size() < 4 || polarities.empty() || polarities.size() != points.size()) {
            if (!points.empty()) result.clusterViews.push_back(points);
            continue;
        }

        Circle norm = Normalize(points);
        Circle c = Regression(points, norm);
        if (c.r <= 0.0f) {
            result.clusterViews.push_back(points);
            continue;
        }

        if (settings.positiveOnly) {
            points = Positif(points, polarities);
            if (points.size() < 4) {
                result.clusterViews.push_back(points);
                continue;
            }

            norm = Normalize(points);
            c = Regression(points, norm);
            if (c.r <= 0.0f) {
                result.clusterViews.push_back(points);
                continue;
            }
        }
        else {
            const float angle = PolarFilter(points, polarities, c, settings);
            if (angle > 99.0f) {
                result.clusterViews.push_back(points);
                continue;
            }
        }

        float rms = Rms(points, c, dr_);
        c.r = c.r + rms * settings.rayonCote;
        points = OutlierFilter(points, settings.coef, c);

        if (points.size() < 4) {
            result.clusterViews.push_back(points);
            continue;
        }

        norm = Normalize(points);
        c = Regression(points, norm);
        if (c.r <= 0.0f) {
            result.clusterViews.push_back(points);
            continue;
        }

        rms = Rms(points, c, dr_);
        c.r = c.r + rms * settings.rayonCote;

        if (!(dr_ * settings.filterSize > c.r && rms < settings.maxResidual)) {
            result.clusterViews.push_back(points);
            continue;
        }

        const cv::Point2f center(c.x, c.y);
        const cv::Point2f direction = Dir(center, last_pt_);
        last_pt_ = center;

        circle_centers_.push_back(center);
        image_parabola_reg_.add(center.x, center.y);

        if (image_parabola_reg_.size() >= 3) {
            parabola_ = image_parabola_reg_.fit();
            result.parabola2DValid = true;
            result.parabola2D = parabola_;
        }

        result.clusterViews.push_back(points);
        result.hasCircle = true;
        result.circle = c;
        result.arrowEnd = {
            c.x + 2.0f * c.r * direction.x,
            c.y + 2.0f * c.r * direction.y
        };

        if (calibration.ready) {
            const auto pose = estimateBallPoseFromCircle(c, calibration, settings.ballRadiusMm);
            if (pose.has_value()) {
                last_ball_pose_ = pose;
                result.pose = pose;
                result.poseTimestampUs = cluster.maxTimestamp;
                Update3DTrack(cluster.maxTimestamp, *pose, result);
            }
        }

        break;
    }

    if (!result.pose.has_value()) {
        result.worldTrack = world_track_;
        result.xFit = x_fit_;
        result.yFit = y_fit_;
        result.zFit = z_fit_;
        result.tMin = track_time_seconds_.empty() ? 0.0f : track_time_seconds_.front();
        result.tMax = track_time_seconds_.empty() ? 0.0f : track_time_seconds_.back();
        result.trajectoryValid = world_track_.size() >= 3;
    }

    return result;
}

Circle BallTracker::Normalize(const std::vector<cv::Point2f> &points) {
    dr_ = 0.0f;

    if (points.size() < 4) return {0.0f, 0.0f, 0.0f};
    
    float cx = 0.0f;
    float cy = 0.0f;

    for (const auto &p : points) {
        cx += p.x;
        cy += p.y;
    }

    cx /= static_cast<float>(points.size());
    cy /= static_cast<float>(points.size());

    for (const auto &p : points) {
        dr_ += std::sqrt(sqr(p.x - cx) + sqr(p.y - cy));
    }
    dr_ /= static_cast<float>(points.size());

    if (dr_ < 0.5f) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {cx, cy, dr_};
}

Circle BallTracker::Regression(const std::vector<cv::Point2f> &points, const Circle &norm) const {
    if (points.size() < 3 || norm.r <= 1e-6f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float cx = norm.x;
    const float cy = norm.y;
    const float drLocal = norm.r;

    Eigen::MatrixXf A(points.size(), 3);
    Eigen::VectorXf B(points.size(), 1);

    int i = 0;
    for (const auto &p : points) {
        const float dx = (p.x - cx) / drLocal;
        const float dy = (p.y - cy) / drLocal;
        A(i, 0) = 2.0f * dx;
        A(i, 1) = 2.0f * dy;
        A(i, 2) = 1.0f;
        B(i, 0) = sqr(dx) + sqr(dy);
        ++i;
    }

    const Eigen::VectorXf s = A.colPivHouseholderQr().solve(B);
    const float radicand = s[2] + sqr(s[1]) + sqr(s[0]);
    if (!std::isfinite(radicand) || radicand <= 0.0f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float x = s[0] * drLocal + cx;
    const float y = s[1] * drLocal + cy;
    const float r = std::sqrt(radicand) * drLocal;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(r)) 
return {0.0f, 0.0f, 0.0f};
    

    return {x, y, r};
}

float BallTracker::Rms(const std::vector<cv::Point2f> &points, const Circle &c, float drLocal) const {
    if (points.empty() || drLocal <= 1e-6f) {
        return std::numeric_limits<float>::infinity();
    }

    float residual = 0.0f;
    for (const auto &p : points) {
        const float dx = (p.x - c.x) / drLocal;
        const float dy = (p.y - c.y) / drLocal;
        const float dist = std::sqrt(sqr(dx) + sqr(dy)) * drLocal;
        residual += sqr(dist - c.r);
    }

    residual /= static_cast<float>(points.size());
    return residual;
}

float BallTracker::PolarFilter(
    const std::vector<cv::Point2f> &points,
    const std::vector<polar> &polarityValues,
    const Circle &c,
    const BallTrackerSettings &settings) const {

    const size_t n = points.size();
    if (n == 0 || polarityValues.size() != n) {
        return 100.0f;
    }

    float sinP = 0.0f;
    float cosP = 0.0f;
    float sinN = 0.0f;
    float cosN = 0.0f;
    size_t countP = 0;
    size_t countN = 0;

    for (size_t i = 0; i < n; ++i) {
        const float dx = points[i].x - c.x;
        const float dy = points[i].y - c.y;
        const float localDr = std::sqrt(dx * dx + dy * dy);
        if (localDr < 1e-6f) {
            continue;
        }

        if (polarityValues[i].polar) {
            cosP += dx / localDr;
            sinP += dy / localDr;
            ++countP;
        }
        else {
            cosN += dx / localDr;
            sinN += dy / localDr;
            ++countN;
        }
    }

    if (countP == 0 || countN == 0) {
        return 100.0f;
    }

    const float symmetry = std::fabs(static_cast<float>(countN) - static_cast<float>(countP))
                         / static_cast<float>(countN + countP);
    if (symmetry > settings.symCoef) {
        return 100.0f;
    }

    const float avgP = std::atan2(sinP, cosP);
    const float avgN = std::atan2(sinN, cosN);

    float separation = avgP - avgN;
    while (separation > kPi) separation -= 2.0f * kPi;
    while (separation < -kPi) separation += 2.0f * kPi;

    if (std::fabs(std::fabs(separation) - kPi) > settings.symCoef2) {
        return 100.0f;
    }

    return avgP;
}

std::vector<cv::Point2f> BallTracker::OutlierFilter(
    const std::vector<cv::Point2f> &points,
    float coefValue,
    Circle c) const {

    std::vector<cv::Point2f> output;
    output.reserve(points.size());

    for (const auto &p : points) {
        const cv::Point2f ref{p.x - c.x, p.y - c.y};
        const float squaredRadius = ref.dot(ref);
        const float lowerLimit = sqr(coefValue * c.r);
        const float upperLimit = sqr(c.r * (2.0f - coefValue));

        if (squaredRadius > lowerLimit && squaredRadius < upperLimit) {
            output.emplace_back(p);
        }
    }

    return output;
}

void BallTracker::Update3DTrack(
    int64_t poseTimestampUs,
    const BallPose3D &pose,
    BallTrackerResult &result) {

    const Vector3 worldPosition = ToMeters(pose.positionMm);
    result.worldPosition = worldPosition;

    if (poseTimestampUs == last_max_time_) {
        result.worldTrack = world_track_;
        result.xFit = x_fit_;
        result.yFit = y_fit_;
        result.zFit = z_fit_;
        result.tMin = track_time_seconds_.empty() ? 0.0f : track_time_seconds_.front();
        result.tMax = track_time_seconds_.empty() ? 0.0f : track_time_seconds_.back();
        result.trajectoryValid = world_track_.size() >= 3;
        return;
    }

    last_max_time_ = poseTimestampUs;

    if (track_start_timestamp_us_ < 0) {
        track_start_timestamp_us_ = poseTimestampUs;
    }

    float tSeconds = static_cast<float>(poseTimestampUs - track_start_timestamp_us_) * 1.0e-6f;

    if (!track_time_seconds_.empty() && tSeconds <= track_time_seconds_.back()) {
        tSeconds = track_time_seconds_.back() + 1.0e-3f;
    }

    track_time_seconds_.push_back(tSeconds);
    world_track_.push_back(worldPosition);

    x_reg_.add(tSeconds, worldPosition.x);
    y_reg_.add(tSeconds, worldPosition.y);
    z_reg_.add(tSeconds, worldPosition.z);

    if (x_reg_.size() >= 2) {
        x_fit_ = x_reg_.fit();
        y_fit_ = y_reg_.fit();
    }

    if (z_reg_.size() >= 3) {
        z_fit_ = z_reg_.fit();
    }

    result.updatedTrajectory = true;
    result.worldTrack = world_track_;
    result.xFit = x_fit_;
    result.yFit = y_fit_;
    result.zFit = z_fit_;
    result.tMin = track_time_seconds_.empty() ? 0.0f : track_time_seconds_.front();
    result.tMax = track_time_seconds_.empty() ? 0.0f : track_time_seconds_.back();
    result.trajectoryValid = world_track_.size() >= 3;
}