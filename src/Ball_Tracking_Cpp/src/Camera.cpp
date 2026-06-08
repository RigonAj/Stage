#include "Camera.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include <fmt/core.h>
#include <opencv2/calib3d.hpp>

#include "DBSCAN/dbscan_clustering.hpp"
#include "DBSCAN/dbscan_point_cloud.hpp"

namespace {
constexpr std::size_t DBSCAN_DIMS = 2;
using CoordinateType = float;
using DBPoint = clustering::DBSCANPoint<CoordinateType, DBSCAN_DIMS>;
using DBCloud = clustering::DBSCANPointCloud<CoordinateType, DBSCAN_DIMS>;
using DBSCANType = clustering::DBSCANClustering<CoordinateType, DBSCAN_DIMS>;
}

DvCamera::DvCamera(): filter_(resolution_, std::chrono::milliseconds(1)) {

    loadOpenCvCalibrationFile();

    try {
        capture_ = std::make_unique<dv::io::camera::DVXplorer>();

        if (capture_ && capture_->isRunning()) {
            fmt::print("DVXplorer camera opened successfully.\n");
        }
        else {
            fmt::print("DVXplorer created but camera is not running.\n");
            capture_.reset();
        }
    }
    catch (const std::exception& e) {
        fmt::print(stderr, "No DVXplorer camera available: {}\n", e.what());
        capture_.reset();
    }
}

void DvCamera::loadOpenCvCalibrationFile() {
    const std::string path = kCalibrationFile;

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        fmt::print(stderr, "Failed to open calibration file: {}\n", path);
        return;
    }

    calibration.sourcePath = path;

    cv::FileNode cameraNode;

    for (auto it = fs.root().begin(); it != fs.root().end(); ++it) {
        const cv::FileNode node = *it;

        if (!node["camera_matrix"].empty()) {
            cameraNode = node;
            calibration.cameraName = node.name();
            break;
        }
    }

    if (cameraNode.empty()) {
        fmt::print(stderr, "No valid camera node found in calibration file: {}\n", path);
        return;
    }

    cameraNode["camera_matrix"] >> calibration.cameraMatrix;
    cameraNode["distortion_coefficients"] >> calibration.distortionCoefficients;
    cameraNode["image_width"] >> calibration.imageSize.width;
    cameraNode["image_height"] >> calibration.imageSize.height;

    if (!cameraNode["use_fisheye_model"].empty()) {
        cameraNode["use_fisheye_model"] >> calibration.useFisheyeModel;
    }
    else if (!fs["use_fisheye_model"].empty()) {
        fs["use_fisheye_model"] >> calibration.useFisheyeModel;
    }

    if (!cameraNode["calibration_error"].empty()) {
        cameraNode["calibration_error"] >> calibration.reprojectionError;
    }
    else if (!fs["calibration_error"].empty()) {
        fs["calibration_error"] >> calibration.reprojectionError;
    }

    if (calibration.cameraMatrix.empty()
        || calibration.cameraMatrix.rows != 3
        || calibration.cameraMatrix.cols != 3) {
        fmt::print(stderr, "Invalid camera matrix in calibration file: {}\n", path);
        return;
    }

    if (calibration.distortionCoefficients.empty()) {
        fmt::print(stderr, "Invalid distortion coefficients in calibration file: {}\n", path);
        return;
    }

    if (calibration.imageSize.width <= 0 || calibration.imageSize.height <= 0) {
        calibration.imageSize = resolution_;
    }

    calibration.cameraMatrix.convertTo(calibration.cameraMatrix, CV_64F);

    calibration.distortionCoefficients =
        calibration.distortionCoefficients.reshape(1, 1);

    calibration.distortionCoefficients.convertTo(
        calibration.distortionCoefficients,
        CV_64F
    );

    calibration.ready = true;

    fmt::print(
        "Calibration loaded from {} | camera={} | fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} | RMS={:.3f} px\n",
        calibration.sourcePath,
        calibration.cameraName,
        calibration.fx(),
        calibration.fy(),
        calibration.cx(),
        calibration.cy(),
        calibration.reprojectionError
    );
}

void DvCamera::NextBatch() {
    Events.reset();

    if (!isCameraRunning()) {
        return;
    }

    try {
        Events = capture_->getNextEventBatch();
    }
    catch (const std::exception& e) {
        fmt::print(stderr, "Camera getNextEventBatch failed: {}\n", e.what());
        Events.reset();
        capture_.reset();
    }
}

void DvCamera::Filter() {
    Filtered = dv::EventStore();

    if (!EventsAvailable()) {
        return;
    }

    filter_.accept(*Events);
    Filtered = filter_.generateEvents();
}

void DvCamera::Undistort() {
    rawFilteredPoints_.clear();
    rawFilteredTimestamps_.clear();
    rawFilteredPolarities_.clear();
    undistortedFilteredPoints_.clear();
    undistortedFilteredTimestamps_.clear();
    undistortedFilteredPolarities_.clear();

    const bool useSamples = !Samples.isEmpty();
    dv::EventStore &target = useSamples ? Samples : Filtered;

    if (target.isEmpty()) {
        return;
    }

    std::vector<cv::Point2f> distortedPoints;
    std::vector<int64_t> timestamps;
    std::vector<bool> polarities;

    distortedPoints.reserve(target.size());
    timestamps.reserve(target.size());
    polarities.reserve(target.size());
    rawFilteredPoints_.reserve(target.size());
    rawFilteredTimestamps_.reserve(target.size());
    rawFilteredPolarities_.reserve(target.size());

    for (const auto& e : target) {
        distortedPoints.emplace_back(
            static_cast<float>(e.x()),
            static_cast<float>(e.y())
        );

        timestamps.emplace_back(e.timestamp());
        polarities.emplace_back(e.polarity());
        rawFilteredPoints_.emplace_back(
            static_cast<float>(e.x()),
            static_cast<float>(e.y())
        );
        rawFilteredTimestamps_.emplace_back(e.timestamp());
        rawFilteredPolarities_.emplace_back(e.polarity());
    }

    if (distortedPoints.empty()) {
        return;
    }

    if (!calibration.ready
        || calibration.cameraMatrix.empty()
        || calibration.distortionCoefficients.empty()) {
        return;
    }

    std::vector<cv::Point2f> undistortedPoints;

    if (calibration.useFisheyeModel) {
        cv::fisheye::undistortPoints(
            distortedPoints,
            undistortedPoints,
            calibration.cameraMatrix,
            calibration.distortionCoefficients,
            cv::noArray(),
            calibration.cameraMatrix
        );
    }
    else {
        cv::undistortPoints(
            distortedPoints,
            undistortedPoints,
            calibration.cameraMatrix,
            calibration.distortionCoefficients,
            cv::noArray(),
            calibration.cameraMatrix
        );
    }

    dv::EventStore output;
    undistortedFilteredPoints_.reserve(undistortedPoints.size());
    undistortedFilteredTimestamps_.reserve(undistortedPoints.size());
    undistortedFilteredPolarities_.reserve(undistortedPoints.size());

    for (size_t i = 0; i < undistortedPoints.size(); ++i) {
        const float xf = undistortedPoints[i].x;
        const float yf = undistortedPoints[i].y;
        const int x = static_cast<int>(std::lround(xf));
        const int y = static_cast<int>(std::lround(yf));

        if (x >= 0
            && x < calibration.imageSize.width
            && y >= 0
            && y < calibration.imageSize.height) {

            undistortedFilteredPoints_.emplace_back(xf, yf);
            undistortedFilteredTimestamps_.emplace_back(timestamps[i]);
            undistortedFilteredPolarities_.emplace_back(polarities[i]);

            output.emplace_back(
                timestamps[i],
                static_cast<int16_t>(x),
                static_cast<int16_t>(y),
                polarities[i]
            );
        }
    }

    target = std::move(output);
    if (useSamples) {
        Filtered = target;
    }
}

void DvCamera::KeepRecentFiltered(double windowSeconds) {
    if (Filtered.isEmpty()) {
        return;
    }

    if (windowSeconds <= 0.0) {
        windowSeconds = 0.001;
    }

    for (const auto& e : Filtered) {
        recentFiltered_.emplace_back(e);
    }

    if (recentFiltered_.isEmpty()) {
        return;
    }

    const int64_t newestTimestamp = recentFiltered_.getHighestTime();
    const int64_t windowUs = static_cast<int64_t>(windowSeconds * 1.0e6);
    const int64_t cutoffTimestamp = newestTimestamp - std::max<int64_t>(windowUs, 1);

    dv::EventStore output;
    for (const auto& e : recentFiltered_) {
        if (e.timestamp() >= cutoffTimestamp) {
            output.emplace_back(e);
        }
    }

    recentFiltered_ = std::move(output);
    Filtered = recentFiltered_;
}

void DvCamera::Echantillon(int maxevent) {
    Samples = dv::EventStore();

    if (Filtered.isEmpty()) {
        return;
    }

    if (maxevent <= 0) {
        maxevent = 1;
    }

    const int count = static_cast<int>(Filtered.size());

    int step = count / maxevent;
    if (step <= 0) {
        step = 1;
    }

    for (int i = 0; i < count; i += step) {
        Samples.emplace_back(Filtered[i]);
    }
}

void DvCamera::Cluster(Box box,float alpha, int bandwidth, uint32_t minNb) {
    boxed_ = dv::EventStore();
    clusters_.clear();

    if (Samples.isEmpty()) {
        return;
    }

    if (bandwidth <= 0) {
        bandwidth = 1;
    }
    if (minNb == 0) {
        minNb = 1;
    }

    int64_t avg_time = 0;
    int64_t max_time = std::numeric_limits<int64_t>::min();
    int n = 0;

    for (const auto& e : Samples) {

        if (!box.InBox(e)) continue;

        boxed_.emplace_back(e);
        avg_time += e.timestamp();
        ++n;
        if (e.timestamp() > max_time) max_time = e.timestamp();
    }

    if (n == 0) return;


    avg_time /= n;

    int64_t time_limit = static_cast<int64_t>(static_cast<float>(max_time - avg_time) * alpha) + avg_time;
    if(alpha <= 0.01) time_limit = -1;
    DBCloud cloud;
    std::vector<bool> cloudPolarities;
    std::vector<int64_t> cloudTimestamps;

    cloud.reserve(boxed_.size());
    cloudPolarities.reserve(boxed_.size());
    cloudTimestamps.reserve(boxed_.size());

    for (const auto& e : boxed_) {
        if (e.timestamp() <= time_limit) {
            continue;
        }

        DBPoint p;
        p[0] = static_cast<float>(e.x());
        p[1] = static_cast<float>(e.y());

        cloud.push_back(p);
        cloudPolarities.emplace_back(e.polarity());
        cloudTimestamps.emplace_back(e.timestamp());
    }

    if (cloud.empty()) return;


    DBSCANType dbscan(cloud, bandwidth, minNb, 10000);
    dbscan.formClusters();

    const auto& rawClusters = dbscan.getClusterIndices();

    std::vector<std::pair<int8_t, const std::vector<uint32_t>*>> sortedClusters;
    sortedClusters.reserve(rawClusters.size());

    for (const auto& [label, indices] : rawClusters) {sortedClusters.emplace_back(label, &indices);}

    std::sort(sortedClusters.begin(), sortedClusters.end(), [](const auto& a, const auto& b) {
        return a.second->size() > b.second->size();
    });

    clusters_.reserve(sortedClusters.size());

    for (const auto& [label, indicesPtr] : sortedClusters) {
        const auto& indices = *indicesPtr;
        if (indices.size() < minNb) {
            continue;
        }

        DvCluster cluster;
        cluster.label = label;
        cluster.points.reserve(indices.size());
        cluster.polarities.reserve(indices.size());
        cluster.timestamps.reserve(indices.size());
        cluster.maxTimestamp = std::numeric_limits<int64_t>::min();
        cluster.minTimestamp = std::numeric_limits<int64_t>::max();

        for (const uint32_t idx : indices) {
            if (idx >= cloud.size()) {
                continue;
            }

            cluster.points.emplace_back(cloud[idx][0], cloud[idx][1]);
            cluster.polarities.emplace_back(cloudPolarities[idx]);
            cluster.timestamps.emplace_back(cloudTimestamps[idx]);
            cluster.minTimestamp = std::min(cloudTimestamps[idx],cluster.minTimestamp);
            cluster.maxTimestamp = std::max(cloudTimestamps[idx],cluster.maxTimestamp);
        }

        if (!cluster.points.empty()) {
            clusters_.emplace_back(std::move(cluster));
        }
    }
}
