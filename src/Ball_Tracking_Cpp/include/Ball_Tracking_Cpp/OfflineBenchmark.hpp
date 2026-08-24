#pragma once

// Headless replay of a recorded/synthetic H5 event sequence through the two
// production perception pipelines, for offline accuracy comparison against a
// ground-truth CSV.
//
//   Trace  : BuildTracePointsFromFloatSource -> FitTraceRibbon -> AnalyzeTrace3D
//   Circle : DvCamera::Echantillon/Cluster   -> BallTracker::Update
//
// Both branches call the same code the live node runs; nothing is
// reimplemented here. Each branch emits one row per grid timestamp, stamped
// with the estimate's OWN event time (trace mid-ribbon sample / circle slice
// max timestamp), not the window end, so the evaluator can interpolate ground
// truth at the instant each method actually describes.

#include <cstdint>
#include <string>
#include <vector>

#include "BallTracker.hpp"
#include "Camera.hpp"
#include "EventWriter.h"

enum class BenchmarkMethod {
    Trace,
    Circle
};

const char *BenchmarkMethodName(BenchmarkMethod method);

struct TraceMethodSettings {
    double traceMemoryMs = 150.0;
    float lineBinWidthPx = 4.0f;
    float localWindowPx = 65.69f;
    int lineOrder = 2;
    float pcaPeriodMs = 36.10f;
    float widthStepPx = 8.0f;
    float supportDivisor = 28.0f;
    int supportMinCount = 3;
    int supportMaxCount = 9;
    float supportRadiusPx = 1.75f;
    float borderRatio = 0.035f;
    // > 0 replaces the width-proportional edge correction with a constant.
    float borderPixels = 0.0f;
    float borderSpacingFactor = 0.0f;
    // 0 = all, 1 = positive, 2 = negative (Ui::TracePolarityMode convention).
    int polarityMode = 0;
    bool useRawInput = false;
    bool edgeRefineEnabled = false;
    bool widthSmoothingEnabled = false;
};

struct CircleMethodSettings {
    double windowMs = 484.32;
    int bandwidth = 50;
    unsigned int minNb = 40;
    int maxEvents = 1000;
    float alpha = 0.5f;
    float coef = 0.45f;
    float filterSize = 115.0f;
    float maxResidual = 19.0f;
    float rayonCote = 0.0f;
    float symCoef = 29.0f;
    float symCoef2 = 157.0f;
    bool positiveOnly = false;
    bool radiusGateEnabled = false;
    bool weightedRegressionEnabled = false;
    int sliceMode = 0;
    float depthJumpGateMm = 250.0f;
    int temporalSliceCount = 5;
    int eventsPerSlice = 100;
    float roiX = 0.0f;
    float roiY = 0.0f;
    float roiW = 640.0f;
    float roiH = 480.0f;
};

struct BenchmarkSettings {
    double outputPeriodMs = 2.0;
    double ballRadiusMeters = 0.02;
    TraceMethodSettings trace;
    CircleMethodSettings circle;
};

struct BenchmarkDetection {
    int64_t timestampUs = 0;
    double runtimeMs = 0.0;
    bool detected = false;
    // Image plane, pixels.
    double centerUPx = 0.0;
    double centerVPx = 0.0;
    // Trace ribbon width / fitted circle radius, pixels.
    double widthPx = 0.0;
    double circleRadiusPx = 0.0;
    // OpenCV camera frame, metres.
    double xCamMeters = 0.0;
    double yCamMeters = 0.0;
    double zCamMeters = 0.0;
    int numEvents = 0;
    int64_t windowStartUs = 0;
    int64_t windowEndUs = 0;
    std::string failureReason;
};

struct BenchmarkMethodRuntime {
    bool ran = false;
    int outputRows = 0;
    int detectedRows = 0;
    double totalRuntimeMs = 0.0;
};

struct BenchmarkRuntime {
    std::string eventsPath;
    std::string cameraPath;
    std::string metadataPath;
    std::string groundTruthPath;
    std::string sequenceName;
    // Echoed back so a run can be audited for using the sequence's synthetic
    // camera model rather than the real DVXplorer calibration.
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int imageWidth = 0;
    int imageHeight = 0;
    int distortionCoefficientCount = 0;
    int64_t inputEventCount = 0;
    int64_t firstTimestampUs = 0;
    int64_t lastTimestampUs = 0;
    std::size_t groundTruthRows = 0;
    std::size_t groundTruthVisibleRows = 0;
    double groundTruthFirstTimestampS = 0.0;
    double groundTruthLastTimestampS = 0.0;
    BenchmarkSettings settings;
    BenchmarkMethodRuntime trace;
    BenchmarkMethodRuntime circle;
};

// Reads the YAML tracker config (flat scalar parsing, `trace:` and `circle:`
// sections) on top of the given defaults.
BenchmarkSettings LoadBenchmarkSettingsYaml(const std::string &path, BenchmarkSettings defaults = {});

std::vector<BenchmarkDetection> RunTraceMethod(
    EventReader &reader,
    DvCamera &camera,
    const CalibrationData &calibration,
    const BenchmarkSettings &settings,
    BenchmarkMethodRuntime &runtime);

std::vector<BenchmarkDetection> RunCircleMethod(
    EventReader &reader,
    DvCamera &camera,
    const CalibrationData &calibration,
    const BenchmarkSettings &settings,
    BenchmarkMethodRuntime &runtime);

void WriteDetectionsCsv(
    const std::string &path,
    const std::vector<BenchmarkDetection> &detections,
    BenchmarkMethod method);

void WriteRuntimeJson(const std::string &path, const BenchmarkRuntime &runtime);
