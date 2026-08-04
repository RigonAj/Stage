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

bool DvCamera::LoadOpenCvCalibrationFile(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        fmt::print(stderr, "Failed to open calibration file: {}\n", path);
        return false;
    }

    CalibrationData nextCalibration;
    nextCalibration.sourcePath = path;

    cv::FileNode cameraNode;

    for (auto it = fs.root().begin(); it != fs.root().end(); ++it) {
        const cv::FileNode node = *it;

        if (!node["camera_matrix"].empty()) {
            cameraNode = node;
            break;
        }
    }

    if (cameraNode.empty()) {
        fmt::print(stderr, "No valid camera node found in calibration file: {}\n", path);
        return false;
    }

    nextCalibration.cameraName = cameraNode.name();
    cameraNode["camera_matrix"] >> nextCalibration.cameraMatrix;
    cameraNode["distortion_coefficients"] >> nextCalibration.distortionCoefficients;
    cameraNode["image_width"] >> nextCalibration.imageSize.width;
    cameraNode["image_height"] >> nextCalibration.imageSize.height;

    if (!cameraNode["use_fisheye_model"].empty()) {
        cameraNode["use_fisheye_model"] >> nextCalibration.useFisheyeModel;
    }
    else if (!fs["use_fisheye_model"].empty()) {
        fs["use_fisheye_model"] >> nextCalibration.useFisheyeModel;
    }

    if (!cameraNode["calibration_error"].empty()) {
        cameraNode["calibration_error"] >> nextCalibration.reprojectionError;
    }
    else if (!fs["calibration_error"].empty()) {
        fs["calibration_error"] >> nextCalibration.reprojectionError;
    }

    if (nextCalibration.cameraMatrix.empty()
        || nextCalibration.cameraMatrix.rows != 3
        || nextCalibration.cameraMatrix.cols != 3) {
        fmt::print(stderr, "Invalid camera matrix in calibration file: {}\n", path);
        return false;
    }

    if (nextCalibration.distortionCoefficients.empty()) {
        fmt::print(stderr, "Invalid distortion coefficients in calibration file: {}\n", path);
        return false;
    }

    if (nextCalibration.imageSize.width <= 0 || nextCalibration.imageSize.height <= 0) {
        nextCalibration.imageSize = resolution_;
    }

    nextCalibration.cameraMatrix.convertTo(nextCalibration.cameraMatrix, CV_64F);

    nextCalibration.distortionCoefficients =
        nextCalibration.distortionCoefficients.reshape(1, 1);

    nextCalibration.distortionCoefficients.convertTo(
        nextCalibration.distortionCoefficients,
        CV_64F
    );

    nextCalibration.ready = true;
    calibration = std::move(nextCalibration);

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
    return true;
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

    if (Filtered.isEmpty()) {
        return;
    }

    std::vector<cv::Point2f> distortedPoints;
    std::vector<int64_t> timestamps;
    std::vector<bool> polarities;

    distortedPoints.reserve(Filtered.size());
    timestamps.reserve(Filtered.size());
    polarities.reserve(Filtered.size());
    rawFilteredPoints_.reserve(Filtered.size());
    rawFilteredTimestamps_.reserve(Filtered.size());
    rawFilteredPolarities_.reserve(Filtered.size());

    for (const auto& e : Filtered) {
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

    Filtered = std::move(output);
}

namespace {
// Drops window-buffer entries older than cutoff, but only once the stale
// prefix dominates: the erase is O(remaining), so amortizing it keeps the
// per-tick cost proportional to the batch instead of the whole window. The
// buffers may briefly hold up to ~2x the window; their only consumer (the
// trace-view fallback display) tolerates that.
void TrimStalePrefix(
    std::vector<cv::Point2f> &points,
    std::vector<int64_t> &timestamps,
    std::vector<bool> &polarities,
    int64_t cutoffTimestamp) {
    if (timestamps.empty()) {
        return;
    }

    const auto firstFresh =
        std::lower_bound(timestamps.begin(), timestamps.end(), cutoffTimestamp);
    const std::size_t stale =
        static_cast<std::size_t>(firstFresh - timestamps.begin());

    if (stale >= 4096 && stale * 2 >= timestamps.size()) {
        points.erase(points.begin(), points.begin() + static_cast<std::ptrdiff_t>(stale));
        timestamps.erase(timestamps.begin(), timestamps.begin() + static_cast<std::ptrdiff_t>(stale));
        polarities.erase(polarities.begin(), polarities.begin() + static_cast<std::ptrdiff_t>(stale));
    }
}
}

void DvCamera::ResetLiveWindow() {
    liveWindow_ = dv::EventStore();
    liveBatchRawPoints_.clear();
    liveBatchRawTimestamps_.clear();
    liveBatchRawPolarities_.clear();
    liveBatchUndistortedPoints_.clear();
    liveBatchUndistortedTimestamps_.clear();
    liveBatchUndistortedPolarities_.clear();
    rawFilteredPoints_.clear();
    rawFilteredTimestamps_.clear();
    rawFilteredPolarities_.clear();
    undistortedFilteredPoints_.clear();
    undistortedFilteredTimestamps_.clear();
    undistortedFilteredPolarities_.clear();
}

void DvCamera::UndistortLiveIncremental(double windowSeconds) {
    liveBatchRawPoints_.clear();
    liveBatchRawTimestamps_.clear();
    liveBatchRawPolarities_.clear();
    liveBatchUndistortedPoints_.clear();
    liveBatchUndistortedTimestamps_.clear();
    liveBatchUndistortedPolarities_.clear();

    if (windowSeconds <= 0.0) {
        windowSeconds = 0.001;
    }

    if (Filtered.isEmpty()) {
        Filtered = liveWindow_;
        return;
    }

    // Backward time jump (camera restart / clock reset): the rolling window
    // is stale, and EventStore::add throws on out-of-order data.
    if (!liveWindow_.isEmpty()
        && Filtered.getLowestTime() < liveWindow_.getHighestTime()) {
        ResetLiveWindow();
    }

    const dv::EventStore batch = Filtered;

    liveBatchRawPoints_.reserve(batch.size());
    liveBatchRawTimestamps_.reserve(batch.size());
    liveBatchRawPolarities_.reserve(batch.size());

    for (const auto& e : batch) {
        liveBatchRawPoints_.emplace_back(
            static_cast<float>(e.x()),
            static_cast<float>(e.y())
        );
        liveBatchRawTimestamps_.emplace_back(e.timestamp());
        liveBatchRawPolarities_.emplace_back(e.polarity());
    }

    dv::EventStore undistortedBatch;

    if (calibration.ready
        && !calibration.cameraMatrix.empty()
        && !calibration.distortionCoefficients.empty()) {
        std::vector<cv::Point2f> undistortedPoints;

        if (calibration.useFisheyeModel) {
            cv::fisheye::undistortPoints(
                liveBatchRawPoints_,
                undistortedPoints,
                calibration.cameraMatrix,
                calibration.distortionCoefficients,
                cv::noArray(),
                calibration.cameraMatrix
            );
        }
        else {
            cv::undistortPoints(
                liveBatchRawPoints_,
                undistortedPoints,
                calibration.cameraMatrix,
                calibration.distortionCoefficients,
                cv::noArray(),
                calibration.cameraMatrix
            );
        }

        liveBatchUndistortedPoints_.reserve(undistortedPoints.size());
        liveBatchUndistortedTimestamps_.reserve(undistortedPoints.size());
        liveBatchUndistortedPolarities_.reserve(undistortedPoints.size());

        for (size_t i = 0; i < undistortedPoints.size(); ++i) {
            const float xf = undistortedPoints[i].x;
            const float yf = undistortedPoints[i].y;
            const int x = static_cast<int>(std::lround(xf));
            const int y = static_cast<int>(std::lround(yf));

            if (x >= 0
                && x < calibration.imageSize.width
                && y >= 0
                && y < calibration.imageSize.height) {

                liveBatchUndistortedPoints_.emplace_back(xf, yf);
                liveBatchUndistortedTimestamps_.emplace_back(liveBatchRawTimestamps_[i]);
                liveBatchUndistortedPolarities_.emplace_back(liveBatchRawPolarities_[i]);

                undistortedBatch.emplace_back(
                    liveBatchRawTimestamps_[i],
                    static_cast<int16_t>(x),
                    static_cast<int16_t>(y),
                    static_cast<bool>(liveBatchRawPolarities_[i])
                );
            }
        }
    }
    else {
        // No calibration: pass raw events through so the pipeline keeps working.
        liveBatchUndistortedPoints_ = liveBatchRawPoints_;
        liveBatchUndistortedTimestamps_ = liveBatchRawTimestamps_;
        liveBatchUndistortedPolarities_ = liveBatchRawPolarities_;
        undistortedBatch = batch;
    }

    liveWindow_.add(undistortedBatch);

    if (liveWindow_.isEmpty()) {
        Filtered = liveWindow_;
        return;
    }

    const int64_t windowUs =
        std::max<int64_t>(static_cast<int64_t>(windowSeconds * 1.0e6), 1);
    const int64_t cutoffTimestamp = liveWindow_.getHighestTime() - windowUs;

    if (liveWindow_.getLowestTime() < cutoffTimestamp) {
        liveWindow_ = liveWindow_.sliceTime(cutoffTimestamp);
    }

    Filtered = liveWindow_;

    // Rolling full-window copies kept for the trace-view fallback source.
    rawFilteredPoints_.insert(
        rawFilteredPoints_.end(), liveBatchRawPoints_.begin(), liveBatchRawPoints_.end());
    rawFilteredTimestamps_.insert(
        rawFilteredTimestamps_.end(), liveBatchRawTimestamps_.begin(), liveBatchRawTimestamps_.end());
    rawFilteredPolarities_.insert(
        rawFilteredPolarities_.end(), liveBatchRawPolarities_.begin(), liveBatchRawPolarities_.end());
    undistortedFilteredPoints_.insert(
        undistortedFilteredPoints_.end(), liveBatchUndistortedPoints_.begin(), liveBatchUndistortedPoints_.end());
    undistortedFilteredTimestamps_.insert(
        undistortedFilteredTimestamps_.end(), liveBatchUndistortedTimestamps_.begin(), liveBatchUndistortedTimestamps_.end());
    undistortedFilteredPolarities_.insert(
        undistortedFilteredPolarities_.end(), liveBatchUndistortedPolarities_.begin(), liveBatchUndistortedPolarities_.end());

    TrimStalePrefix(rawFilteredPoints_, rawFilteredTimestamps_, rawFilteredPolarities_, cutoffTimestamp);
    TrimStalePrefix(
        undistortedFilteredPoints_,
        undistortedFilteredTimestamps_,
        undistortedFilteredPolarities_,
        cutoffTimestamp);
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
