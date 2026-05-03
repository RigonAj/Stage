#pragma once

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <opencv2/core.hpp>
#include <raylib.h>

#include "Camera.hpp"

inline bool isZero(const float valeur) {return valeur < 1e-6f && valeur > -1e-6f;}
inline bool isZero(const double valeur) {return valeur < 1e-6 && valeur > -1e-6;}
inline float sqr(const float value) {return value * value;}
struct polar {bool polar = false;int64_t timestamp = 0;};

inline std::optional<BallPose3D> estimateBallPoseFromCircle(
    const Circle &circle,
    const CalibrationData &calibration,
    const float ballRadiusMm) {

    if (!calibration.ready || circle.r <= 0.5f || ballRadiusMm <= 0.0f) return std::nullopt; 

    const float fx = static_cast<float>(calibration.fx());
    const float fy = static_cast<float>(calibration.fy());
    const float cx = static_cast<float>(calibration.cx());
    const float cy = static_cast<float>(calibration.cy());

    if (isZero(fx) || isZero(fy)) return std::nullopt;
    
    const float depthMm = fx * ballRadiusMm / circle.r;

    if (!std::isfinite(depthMm)) return std::nullopt;
    cv::Vec2f centerPx((circle.x - cx) / fx, (circle.y - cy) / fy);
    cv::Vec3f positionMm(centerPx[0] * depthMm , centerPx[1] * depthMm , depthMm) ;


    BallPose3D pose;
    pose.valid = true;
    pose.circle = circle;
    pose.RadiusPx = circle.r;
    pose.positionMm = positionMm;
    pose.depthMm = pose.positionMm.z;
    
    return pose;
}

inline std::string ballPoseToString(const BallPose3D &pose) {

    std::string s = fmt::format(
        "X={:.1f} mm  Y={:.1f} mm  Z={:.1f} mm"
        , pose.positionMm.x, pose.positionMm.y, pose.positionMm.z);

    return s;
}

inline Vector3 ToMeters(const cv::Point3f &cameraPositionMm) {
    return {
        cameraPositionMm.x * 1.0e-3f,
        cameraPositionMm.z * 1.0e-3f,
        -cameraPositionMm.y * 1.0e-3f
    };
}



inline cv::Point2f Dir(cv::Point2f pt, cv::Point2f lastPt) {
    cv::Point2f dir{pt.x - lastPt.x, pt.y - lastPt.y};
    const float norm = std::sqrt(dir.dot(dir));
    if (isZero(norm)) {
        return {1.0f, 0.0f};
    }
    return {dir.x / norm, dir.y / norm};
}

inline std::vector<cv::Point2f> Positif(
    const std::vector<cv::Point2f> &points,
    const std::vector<polar> &polarityValues) {

    std::vector<cv::Point2f> output;
    output.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!polarityValues[i].polar) {
            output.emplace_back(points[i]);
        }
    }

    return output;
}



