#pragma once

// Trace event accumulation and analysis orchestration, extracted from Gui so
// the pipeline can run without a window.
//
// TraceAnalysis.cpp already holds the pure maths (ribbon fit, widths, 3D). What
// stayed in Gui was the *stateful* half: the rolling event window feeding it and
// the call sequence that turns that window into a 3D trajectory. Both the live
// GUI and the offline benchmark need exactly that, and a benchmark that
// reimplements it stops measuring the real algorithm the moment either drifts
// (which is what happened to the previous TraceBenchmark.cpp).

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <opencv2/core.hpp>

#include "Camera.hpp"
#include "TraceAnalysis.hpp"

// Fixed work-ROI in image pixels: only events inside it feed the trace, which
// lets the operator crop out the region where the robot generates its own
// events. Mirrors the Ui::WorkRoi* getters.
struct TraceRoi {
    float x = 0.0f;
    float y = 0.0f;
    float w = 640.0f;
    float h = 480.0f;

    bool Contains(float px, float py) const {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
};

// Every knob the trace pipeline reads, with the Ui constructor defaults. The
// GUI fills this from its sliders; the benchmark fills it from the command line
// so a run is fully described by its manifest.
struct TraceRuntimeSettings {
    double memorySeconds = 0.150;      // trace_memory_ms = 150
    int polarityMode = 0;              // 0 = all (the ROS default; the Ui slider default is 2)
    float lineBinWidthPx = 4.0f;
    float lineWindowPx = 65.69f;
    int lineOrder = 2;
    float pcaPeriodMs = 36.10f;
    float widthStepPx = 8.0f;
    TraceSupportEdgeSettings supportEdge{};
    bool edgeRefine = false;
    bool widthSmoothing = false;
    bool useRawInput = false;
    bool useRadiusGate = false;
    float ballRadiusMm = 20.0f;
    TraceRoi roi{};
    // Hard cap on the points entering the ribbon fit so an event burst cannot
    // blow up the per-run cost (stride subsampling keeps the trail's shape).
    std::size_t maxAnalysisPoints = 24000;
};

// Rolling time window of ROI-gated events feeding the ribbon fit.
class TraceAccumulator {
public:
    // Returns true when the buffer changed (the caller's "analysis is dirty"
    // signal). Drops events outside the ROI or the sensor bounds, dedups on
    // timestamp, and resets on a backward time jump > 1 ms (new recording or
    // seek).
    bool Append(
        const std::vector<cv::Point2f> &points,
        const std::vector<int64_t> &timestamps,
        const std::vector<bool> *polarities,
        const TraceRoi &roi,
        double memorySeconds);

    void Reset();

    const std::vector<cv::Point2f> &Points() const { return points_; }
    const std::vector<int64_t> &Timestamps() const { return timestamps_; }
    const std::vector<bool> &Polarities() const { return polarities_; }
    std::size_t Size() const { return points_.size(); }
    bool Empty() const { return points_.empty(); }
    int64_t LastTimestampUs() const { return lastTimestampUs_; }

    // Exact trace-memory cutoff. Compaction is lazy (an aged-out prefix may
    // still sit in the buffer), so the analysis applies this cutoff itself.
    int64_t CutoffUs(double memorySeconds) const;

private:
    static constexpr std::size_t kMaxAccumulatedTraceEvents = 120000;

    std::vector<cv::Point2f> points_;
    std::vector<int64_t> timestamps_;
    std::vector<bool> polarities_;
    int64_t lastTimestampUs_ = std::numeric_limits<int64_t>::min();
};

// Alternative point sources the ribbon fit may consume instead of the
// accumulated buffer, selected by TraceRuntimeSettings::useRawInput and the
// radius gate. Null members are simply not offered.
struct TracePointSources {
    const std::vector<cv::Point2f> *rawPoints = nullptr;
    const std::vector<int64_t> *rawTimestamps = nullptr;
    const std::vector<bool> *rawPolarities = nullptr;
    const std::vector<cv::Point2f> *undistortedPoints = nullptr;
    const std::vector<int64_t> *undistortedTimestamps = nullptr;
    const std::vector<bool> *undistortedPolarities = nullptr;
    const dv::EventStore *events = nullptr;
    bool motionWindowValid = false;
};

struct TraceRunResult {
    TracePointSourceResult source;
    TraceRibbonFit fit;
    Trace3DAnalysis analysis;
    // Absolute event time (us) the analysis times are relative to.
    int64_t timeOriginUs = 0;
};

// Point source selection -> ribbon fit -> width/3D analysis. An invalid ribbon
// returns a result whose fit and analysis are both invalid (the caller clears
// its 3D state).
TraceRunResult RunTraceAnalysis(
    const TraceAccumulator &accumulator,
    const TracePointSources &sources,
    const TraceRuntimeSettings &settings,
    const CalibrationData &calibration,
    const TraceGroundTruthLookup &lookupGroundTruth);
