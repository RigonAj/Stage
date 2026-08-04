#include "TraceRuntime.hpp"

#include <algorithm>
#include <cmath>

bool TraceAccumulator::Append(
    const std::vector<cv::Point2f> &points,
    const std::vector<int64_t> &timestamps,
    const std::vector<bool> *polarities,
    const TraceRoi &roi,
    double memorySeconds) {

    if (points.empty() || points.size() != timestamps.size()) {
        return false;
    }

    const bool hasPolarities = polarities != nullptr && polarities->size() >= points.size();

    int64_t newestInBatch = std::numeric_limits<int64_t>::min();
    for (const int64_t timestamp : timestamps) {
        newestInBatch = std::max(newestInBatch, timestamp);
    }

    if (newestInBatch == std::numeric_limits<int64_t>::min()) {
        return false;
    }

    bool changed = false;

    if (lastTimestampUs_ != std::numeric_limits<int64_t>::min()
        && newestInBatch + 1000 < lastTimestampUs_) {
        Reset();
        changed = true;
    }

    const int64_t previousLastTimestamp = lastTimestampUs_;
    const std::size_t sizeBeforeAppend = points_.size();
    points_.reserve(points_.size() + points.size());
    timestamps_.reserve(timestamps_.size() + timestamps.size());
    polarities_.reserve(polarities_.size() + points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
        const int64_t timestamp = timestamps[i];
        if (previousLastTimestamp != std::numeric_limits<int64_t>::min()
            && timestamp <= previousLastTimestamp) {
            continue;
        }

        const cv::Point2f &point = points[i];
        if (!std::isfinite(point.x)
            || !std::isfinite(point.y)
            || point.x < 0.0f
            || point.x >= 640.0f
            || point.y < 0.0f
            || point.y >= 480.0f) {
            continue;
        }

        // Trace accumulation is gated by the fixed work-ROI only (no circle /
        // motion-window dependency): every event inside the ROI feeds the trace.
        if (!roi.Contains(point.x, point.y)) {
            continue;
        }

        points_.emplace_back(point);
        timestamps_.emplace_back(timestamp);
        polarities_.emplace_back(hasPolarities ? (*polarities)[i] : true);
    }

    lastTimestampUs_ = std::max(lastTimestampUs_, newestInBatch);

    if (points_.size() != sizeBeforeAppend) {
        changed = true;
    }

    if (points_.empty()) {
        return changed;
    }

    const int64_t memoryUs = static_cast<int64_t>(memorySeconds * 1.0e6);
    const int64_t cutoffTimestamp = lastTimestampUs_ - std::max<int64_t>(memoryUs, 1);

    // Lazy compaction: the timestamps are appended in order, so the aged-out
    // events form a sorted prefix. Erasing it is O(remaining), so it only
    // happens once the stale prefix dominates - the per-tick cost stays
    // O(batch) instead of O(window) (the old full rewrite ran at 1 kHz over
    // the whole accumulation and froze the loop during event bursts). The
    // analysis applies the exact cutoff itself when reading the buffer.
    const auto firstFresh = std::lower_bound(
        timestamps_.begin(),
        timestamps_.end(),
        cutoffTimestamp);
    const std::size_t stale =
        static_cast<std::size_t>(firstFresh - timestamps_.begin());
    if (stale >= 4096 && stale * 2 >= timestamps_.size()) {
        points_.erase(
            points_.begin(),
            points_.begin() + static_cast<std::ptrdiff_t>(stale));
        timestamps_.erase(
            timestamps_.begin(),
            timestamps_.begin() + static_cast<std::ptrdiff_t>(stale));
        polarities_.erase(
            polarities_.begin(),
            polarities_.begin() + static_cast<std::ptrdiff_t>(stale));
    }

    if (points_.size() > kMaxAccumulatedTraceEvents) {
        const std::size_t removeCount = points_.size() - kMaxAccumulatedTraceEvents;
        points_.erase(points_.begin(), points_.begin() + static_cast<std::ptrdiff_t>(removeCount));
        timestamps_.erase(timestamps_.begin(), timestamps_.begin() + static_cast<std::ptrdiff_t>(removeCount));
        polarities_.erase(polarities_.begin(), polarities_.begin() + static_cast<std::ptrdiff_t>(removeCount));
        changed = true;
    }

    return changed;
}

void TraceAccumulator::Reset() {
    points_.clear();
    timestamps_.clear();
    polarities_.clear();
    lastTimestampUs_ = std::numeric_limits<int64_t>::min();
}

int64_t TraceAccumulator::CutoffUs(double memorySeconds) const {
    if (lastTimestampUs_ == std::numeric_limits<int64_t>::min()) {
        return std::numeric_limits<int64_t>::min();
    }

    const int64_t memoryUs = static_cast<int64_t>(memorySeconds * 1.0e6);
    return lastTimestampUs_ - std::max<int64_t>(memoryUs, 1);
}

TraceRunResult RunTraceAnalysis(
    const TraceAccumulator &accumulator,
    const TracePointSources &sources,
    const TraceRuntimeSettings &settings,
    const CalibrationData &calibration,
    const TraceGroundTruthLookup &lookupGroundTruth) {

    TraceRunResult result;

    static const dv::EventStore kEmptyEvents;
    const dv::EventStore &events = sources.events != nullptr ? *sources.events : kEmptyEvents;

    result.source = BuildTracePointSource(
        accumulator.Points(),
        accumulator.Timestamps(),
        accumulator.Polarities(),
        sources.rawPoints,
        sources.rawTimestamps,
        sources.rawPolarities,
        sources.undistortedPoints,
        sources.undistortedTimestamps,
        sources.undistortedPolarities,
        events,
        settings.useRawInput,
        settings.useRadiusGate,
        sources.motionWindowValid,
        settings.polarityMode,
        accumulator.CutoffUs(settings.memorySeconds),
        settings.maxAnalysisPoints
    );

    result.fit = FitTraceRibbon(
        result.source.points,
        settings.lineBinWidthPx,
        settings.lineWindowPx,
        settings.lineOrder,
        settings.pcaPeriodMs,
        settings.supportEdge,
        settings.edgeRefine
    );

    if (!result.fit.valid) {
        return result;
    }

    result.timeOriginUs = TraceTimeOriginUs(result.source.points);
    result.analysis = AnalyzeTrace3D(
        result.fit,
        calibration,
        settings.ballRadiusMm,
        settings.widthStepPx,
        settings.widthSmoothing,
        result.timeOriginUs,
        lookupGroundTruth
    );

    return result;
}
