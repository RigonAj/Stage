#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "EventWriter.h"

struct TraceBenchmarkIntrinsics {
    int width = 640;
    int height = 480;
    double fx = 520.0;
    double fy = 520.0;
    double cx = 320.0;
    double cy = 240.0;
    std::string sourcePath;
};

struct TraceBenchmarkSettings {
    double traceMemoryMs = 530.51;
    double outputPeriodMs = 2.0;
    int minEvents = 40;
    int maxEvents = 120000;
    std::string edgeMode = "support";
    std::string polarityMode = "negative";
    std::string fitInput = "undist";
    std::string lineOrder = "quad";
    bool radiusGate = false;
};

struct TraceBenchmarkMetadata {
    double ballRadiusM = 0.02;
    std::string sequenceName;
    std::string sourcePath;
};

struct TraceGroundTruthSummary {
    std::size_t rowCount = 0;
    std::size_t visibleCount = 0;
    double firstTimestampS = 0.0;
    double lastTimestampS = 0.0;
    std::string sourcePath;
};

struct TraceDetection {
    int64_t timestampUs = 0;
    double runtimeMs = 0.0;
    bool detected = false;
    double centerUPx = 0.0;
    double centerVPx = 0.0;
    double traceWidthPx = 0.0;
    double traceAngleDeg = 0.0;
    double depthEstM = 0.0;
    double xEstM = 0.0;
    double yEstM = 0.0;
    double zEstM = 0.0;
    int numEvents = 0;
    double lineFitErrorPx = 0.0;
    int traceSupportCount = 0;
    bool outlierRejected = false;
    std::string edgeMode = "support";
    std::string polarityMode = "negative";
    int64_t windowStartUs = 0;
    int64_t windowEndUs = 0;
    std::string failureReason;
};

struct TraceBenchmarkRuntime {
    std::string eventsPath;
    std::string cameraPath;
    std::string metadataPath;
    std::string groundTruthPath;
    int64_t inputEventCount = 0;
    int64_t firstTimestampUs = 0;
    int64_t lastTimestampUs = 0;
    double totalRuntimeMs = 0.0;
    int outputRows = 0;
    int detectedRows = 0;
    TraceBenchmarkSettings settings;
    TraceBenchmarkMetadata metadata;
    TraceGroundTruthSummary groundTruth;
};

TraceBenchmarkIntrinsics LoadTraceIntrinsicsJson(const std::string &path);
TraceBenchmarkMetadata LoadTraceMetadataJson(const std::string &path);
TraceGroundTruthSummary LoadTraceGroundTruthCsv(const std::string &path);
TraceBenchmarkSettings LoadTraceSettingsYaml(const std::string &path, TraceBenchmarkSettings defaults = {});

std::vector<TraceDetection> RunTraceBenchmark(
    EventReader &reader,
    const TraceBenchmarkIntrinsics &intrinsics,
    const TraceBenchmarkMetadata &metadata,
    const TraceBenchmarkSettings &settings,
    TraceBenchmarkRuntime &runtime);

void WriteTraceDetectionsCsv(const std::string &path, const std::vector<TraceDetection> &detections);
void WriteTraceRuntimeJson(const std::string &path, const TraceBenchmarkRuntime &runtime);
