#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <dv-processing/io/camera/dvxplorer.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include <opencv2/core.hpp>
class Box;
struct Circle {
    float x = 0.0f;
    float y = 0.0f;
    float r = 0.0f;
};

struct CalibrationData {
    bool ready = false;
    bool useFisheyeModel = false;
    double reprojectionError = -1.0;

    std::string cameraName;
    std::string sourcePath;

    cv::Size imageSize{};
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;

    double fx() const { return cameraMatrix.empty() ? 0.0 : cameraMatrix.at<double>(0, 0); }
    double fy() const { return cameraMatrix.empty() ? 0.0 : cameraMatrix.at<double>(1, 1); }
    double cx() const { return cameraMatrix.empty() ? 0.0 : cameraMatrix.at<double>(0, 2); }
    double cy() const { return cameraMatrix.empty() ? 0.0 : cameraMatrix.at<double>(1, 2); }
};

struct DvCluster {
    int8_t label = -1;
    std::vector<cv::Point2f> points;
    std::vector<bool> polarities;
    std::vector<int64_t> timestamps;
    int64_t maxTimestamp = 0;
    int64_t minTimestamp = 100;

    std::size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
};

struct BallPose3D {
    bool valid = false;
    Circle circle{};
    float RadiusPx = 0.0f;
    float depthMm = 0.0f;
    cv::Point3f positionMm{};
    int64_t timestampUs = 0;
};

class DvCamera {
public:
    DvCamera();

    void NextBatch();
    void Filter();
    void Undistort();
    void KeepRecentFiltered(double windowSeconds);
    void Echantillon(int maxevent);
    void Cluster(Box box,float alpha, int bandwidth, uint32_t minNb);

    const std::vector<DvCluster>& Clusters() const { return clusters_; }
    const std::vector<cv::Point2f>& RawFilteredPoints() const { return rawFilteredPoints_; }
    const std::vector<int64_t>& RawFilteredTimestamps() const { return rawFilteredTimestamps_; }
    const std::vector<bool>& RawFilteredPolarities() const { return rawFilteredPolarities_; }
    const std::vector<cv::Point2f>& UndistortedFilteredPoints() const { return undistortedFilteredPoints_; }
    const std::vector<int64_t>& UndistortedFilteredTimestamps() const { return undistortedFilteredTimestamps_; }
    const std::vector<bool>& UndistortedFilteredPolarities() const { return undistortedFilteredPolarities_; }

    bool ClustersAvailable() const { return !clusters_.empty(); }
    bool isCameraRunning() const { return capture_ && capture_->isRunning(); }
    bool EventsAvailable() const { return Events.has_value() && !Events->isEmpty(); }
    bool FilteredAvailable() const { return !Filtered.isEmpty(); }

    int64_t EventCount() const { return Events ? static_cast<int64_t>(Events->size()) : 0; }
    int64_t FilteredCount() const { return static_cast<int64_t>(Filtered.size()); }

    CalibrationData calibration;

    std::optional<dv::EventStore> Events;
    dv::EventStore Filtered;
    dv::EventStore Samples;

private:
    static constexpr int WIDTH = 640;
    static constexpr int HEIGHT = 480;

    const cv::Size resolution_{WIDTH, HEIGHT};
    const std::string kCalibrationFile = "calibration_camera_DVXplorer_DXA00265-2026_04_23_13_33_50.xml";

    void loadOpenCvCalibrationFile();

    std::unique_ptr<dv::io::camera::DVXplorer> capture_;
    dv::noise::BackgroundActivityNoiseFilter<dv::EventStore> filter_;

    dv::EventStore boxed_;
    dv::EventStore recentFiltered_;
    std::vector<DvCluster> clusters_;
    std::vector<cv::Point2f> rawFilteredPoints_;
    std::vector<int64_t> rawFilteredTimestamps_;
    std::vector<bool> rawFilteredPolarities_;
    std::vector<cv::Point2f> undistortedFilteredPoints_;
    std::vector<int64_t> undistortedFilteredTimestamps_;
    std::vector<bool> undistortedFilteredPolarities_;
};


class Box {
public:
    Box(float xi, float yi, float wi, float hi) : x(xi), y(yi), w(wi), h(hi) {}
    Box() : Box(0.0f, 0.0f, 0.0f, 0.0f) {}

    bool InBox(const dv::Event &event) const {
        return event.x() > x && event.x() < x + w
            && event.y() > y && event.y() < y + h;
    }

    cv::Rect rec() const {
        return cv::Rect(
            static_cast<int>(x),
            static_cast<int>(y),
            static_cast<int>(w),
            static_cast<int>(h)
        );
    }

    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
};
