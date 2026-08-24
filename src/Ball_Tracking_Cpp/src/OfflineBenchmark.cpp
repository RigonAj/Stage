#include "OfflineBenchmark.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "TraceAnalysis.hpp"
#include "util.hpp"

namespace {

using clock_type = std::chrono::steady_clock;

std::string Trim(std::string value) {
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string Unquote(std::string value) {
    value = Trim(value);
    if (value.size() >= 2
        && ((value.front() == '"' && value.back() == '"')
            || (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

std::string Lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

double ToDouble(const std::string &value, double fallback) {
    try {
        return std::stod(Unquote(value));
    }
    catch (...) {
        return fallback;
    }
}

int ToInt(const std::string &value, int fallback) {
    return static_cast<int>(std::llround(ToDouble(value, static_cast<double>(fallback))));
}

bool ToBool(const std::string &value, bool fallback) {
    const std::string lower = Lower(Unquote(value));
    if (lower == "true" || lower == "yes" || lower == "on" || lower == "1") {
        return true;
    }
    if (lower == "false" || lower == "no" || lower == "off" || lower == "0") {
        return false;
    }
    return fallback;
}

// Ui::TracePolarityMode convention: 0 = all, 1 = positive, 2 = negative.
int ParsePolarityMode(const std::string &value, int fallback) {
    const std::string lower = Lower(Unquote(value));
    if (lower == "all" || lower == "both") return 0;
    if (lower == "positive" || lower == "pos") return 1;
    if (lower == "negative" || lower == "neg") return 2;
    return fallback;
}

std::string CsvEscape(const std::string &value) {
    if (value.find_first_of(",\"\n") == std::string::npos) {
        return value;
    }
    std::string escaped = "\"";
    for (const char c : value) {
        if (c == '"') {
            escaped += "\"\"";
        }
        else {
            escaped += c;
        }
    }
    escaped += '"';
    return escaped;
}

std::string JsonEscape(const std::string &value) {
    std::string escaped;
    for (const char c : value) {
        if (c == '\\' || c == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(c);
    }
    return escaped;
}

TraceSupportEdgeSettings MakeSupportEdge(const TraceMethodSettings &trace) {
    TraceSupportEdgeSettings supportEdge;
    supportEdge.supportDivisor = trace.supportDivisor;
    supportEdge.minLocalSupport = static_cast<std::size_t>(std::max(1, trace.supportMinCount));
    supportEdge.maxLocalSupport =
        static_cast<std::size_t>(std::max(trace.supportMinCount, trace.supportMaxCount));
    supportEdge.supportRadiusPx = trace.supportRadiusPx;
    supportEdge.borderRatio = trace.borderRatio;
    supportEdge.borderPixels = trace.borderPixels;
    supportEdge.borderSpacingFactor = trace.borderSpacingFactor;
    return supportEdge;
}

// The trace pipeline reports positions in the raylib world convention
// (x_cam, z_cam, -y_cam) in metres; ground truth is OpenCV camera metres.
void WorldToCameraMeters(const Vector3 &world, BenchmarkDetection &detection) {
    detection.xCamMeters = static_cast<double>(world.x);
    detection.yCamMeters = static_cast<double>(-world.z);
    detection.zCamMeters = static_cast<double>(world.y);
}

void ProjectToImage(const CalibrationData &calibration, BenchmarkDetection &detection) {
    if (detection.zCamMeters <= 0.0) {
        return;
    }
    detection.centerUPx = calibration.fx() * detection.xCamMeters / detection.zCamMeters + calibration.cx();
    detection.centerVPx = calibration.fy() * detection.yCamMeters / detection.zCamMeters + calibration.cy();
}

}  // namespace

const char *BenchmarkMethodName(BenchmarkMethod method) {
    return method == BenchmarkMethod::Trace ? "trace" : "circle";
}

BenchmarkSettings LoadBenchmarkSettingsYaml(const std::string &path, BenchmarkSettings defaults) {
    if (path.empty()) {
        return defaults;
    }

    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot open tracker config YAML: " + path);
    }

    BenchmarkSettings settings = defaults;
    std::string section;
    std::string line;

    while (std::getline(file, line)) {
        const std::size_t comment = line.find('#');
        const std::string clean = line.substr(0, comment);
        if (Trim(clean).empty()) {
            continue;
        }

        // A top-level key (no indentation) opens a section.
        const bool topLevel = !clean.empty() && !std::isspace(static_cast<unsigned char>(clean.front()));
        const std::size_t colon = clean.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        const std::string key = Trim(clean.substr(0, colon));
        const std::string value = Trim(clean.substr(colon + 1));

        if (topLevel) {
            section = key;
            continue;
        }
        if (value.empty()) {
            continue;
        }

        if (section == "output") {
            if (key == "output_period_ms") settings.outputPeriodMs = ToDouble(value, settings.outputPeriodMs);
        }
        else if (section == "tracker") {
            if (key == "ball_radius_m") settings.ballRadiusMeters = ToDouble(value, settings.ballRadiusMeters);
        }
        else if (section == "trace") {
            TraceMethodSettings &t = settings.trace;
            if (key == "trace_memory_ms") t.traceMemoryMs = ToDouble(value, t.traceMemoryMs);
            else if (key == "line_bin_width_px") t.lineBinWidthPx = static_cast<float>(ToDouble(value, t.lineBinWidthPx));
            else if (key == "local_window_px") t.localWindowPx = static_cast<float>(ToDouble(value, t.localWindowPx));
            else if (key == "line_order") t.lineOrder = Lower(Unquote(value)) == "linear" ? 1 : 2;
            else if (key == "pca_period_ms") t.pcaPeriodMs = static_cast<float>(ToDouble(value, t.pcaPeriodMs));
            else if (key == "width_step_px") t.widthStepPx = static_cast<float>(ToDouble(value, t.widthStepPx));
            else if (key == "support_divisor") t.supportDivisor = static_cast<float>(ToDouble(value, t.supportDivisor));
            else if (key == "support_min_count") t.supportMinCount = ToInt(value, t.supportMinCount);
            else if (key == "support_max_count") t.supportMaxCount = ToInt(value, t.supportMaxCount);
            else if (key == "support_radius_px") t.supportRadiusPx = static_cast<float>(ToDouble(value, t.supportRadiusPx));
            else if (key == "border_percent") t.borderRatio = static_cast<float>(ToDouble(value, t.borderRatio * 100.0) / 100.0);
            else if (key == "border_pixels") t.borderPixels = static_cast<float>(ToDouble(value, t.borderPixels));
            else if (key == "border_spacing_factor") t.borderSpacingFactor = static_cast<float>(ToDouble(value, t.borderSpacingFactor));
            else if (key == "polarity_mode") t.polarityMode = ParsePolarityMode(value, t.polarityMode);
            else if (key == "fit_input") t.useRawInput = Lower(Unquote(value)) == "raw";
            else if (key == "edge_refine") t.edgeRefineEnabled = ToBool(value, t.edgeRefineEnabled);
            else if (key == "width_smoothing") t.widthSmoothingEnabled = ToBool(value, t.widthSmoothingEnabled);
        }
        else if (section == "circle") {
            CircleMethodSettings &c = settings.circle;
            if (key == "window_ms") c.windowMs = ToDouble(value, c.windowMs);
            else if (key == "bandwidth") c.bandwidth = ToInt(value, c.bandwidth);
            else if (key == "min_nb") c.minNb = static_cast<unsigned int>(std::max(1, ToInt(value, static_cast<int>(c.minNb))));
            else if (key == "max_events") c.maxEvents = ToInt(value, c.maxEvents);
            else if (key == "alpha") c.alpha = static_cast<float>(ToDouble(value, c.alpha));
            else if (key == "coef") c.coef = static_cast<float>(ToDouble(value, c.coef));
            else if (key == "filter_size") c.filterSize = static_cast<float>(ToDouble(value, c.filterSize));
            else if (key == "max_residual") c.maxResidual = static_cast<float>(ToDouble(value, c.maxResidual));
            else if (key == "rayon_cote") c.rayonCote = static_cast<float>(ToDouble(value, c.rayonCote));
            else if (key == "sym_coef") c.symCoef = static_cast<float>(ToDouble(value, c.symCoef));
            else if (key == "sym_coef2") c.symCoef2 = static_cast<float>(ToDouble(value, c.symCoef2));
            else if (key == "positive_only") c.positiveOnly = ToBool(value, c.positiveOnly);
            else if (key == "radius_gate") c.radiusGateEnabled = ToBool(value, c.radiusGateEnabled);
            else if (key == "weighted_regression") c.weightedRegressionEnabled = ToBool(value, c.weightedRegressionEnabled);
            else if (key == "slice_mode") c.sliceMode = ToInt(value, c.sliceMode);
            else if (key == "depth_jump_gate_mm") c.depthJumpGateMm = static_cast<float>(ToDouble(value, c.depthJumpGateMm));
            else if (key == "temporal_slices") c.temporalSliceCount = ToInt(value, c.temporalSliceCount);
            else if (key == "events_per_slice") c.eventsPerSlice = ToInt(value, c.eventsPerSlice);
            else if (key == "roi_x") c.roiX = static_cast<float>(ToDouble(value, c.roiX));
            else if (key == "roi_y") c.roiY = static_cast<float>(ToDouble(value, c.roiY));
            else if (key == "roi_w") c.roiW = static_cast<float>(ToDouble(value, c.roiW));
            else if (key == "roi_h") c.roiH = static_cast<float>(ToDouble(value, c.roiH));
        }
    }

    settings.outputPeriodMs = std::max(0.1, settings.outputPeriodMs);
    settings.trace.traceMemoryMs = std::max(1.0, settings.trace.traceMemoryMs);
    settings.circle.windowMs = std::max(1.0, settings.circle.windowMs);
    settings.circle.maxEvents = std::max(1, settings.circle.maxEvents);
    settings.trace.supportMaxCount = std::max(settings.trace.supportMinCount, settings.trace.supportMaxCount);
    return settings;
}

std::vector<BenchmarkDetection> RunTraceMethod(
    EventReader &reader,
    DvCamera &camera,
    const CalibrationData &calibration,
    const BenchmarkSettings &settings,
    BenchmarkMethodRuntime &runtime) {

    std::vector<BenchmarkDetection> detections;
    runtime.ran = true;

    if (reader.empty() || reader.durationUs() <= 0) {
        return detections;
    }

    const TraceMethodSettings &trace = settings.trace;
    const TraceSupportEdgeSettings supportEdge = MakeSupportEdge(trace);
    const float ballRadiusMm = static_cast<float>(settings.ballRadiusMeters * 1000.0);
    const double memorySeconds = trace.traceMemoryMs * 1.0e-3;
    const int64_t memoryUs = static_cast<int64_t>(std::llround(memorySeconds * 1.0e6));

    const int64_t firstUs = reader.startTimestampUs();
    const int64_t lastUs = reader.endTimestampUs();
    const int64_t periodUs =
        std::max<int64_t>(1, static_cast<int64_t>(std::llround(settings.outputPeriodMs * 1000.0)));

    // The GUI feeds the trace analysis from an accumulator holding the last
    // `trace_memory_ms` of events; a sliding read window is the offline
    // equivalent of that same buffer content.
    detections.reserve(static_cast<std::size_t>((lastUs - firstUs) / periodUs + 1));

    for (int64_t timestampUs = firstUs; timestampUs <= lastUs; timestampUs += periodUs) {
        const auto start = clock_type::now();

        BenchmarkDetection detection;
        detection.timestampUs = timestampUs;
        detection.windowStartUs = std::max(firstUs, timestampUs - memoryUs);
        detection.windowEndUs = timestampUs;

        dv::EventStore window;
        reader.readWindowEndingAt(
            window,
            static_cast<double>(timestampUs - firstUs) * 1.0e-6,
            memorySeconds);

        camera.Filtered = window;
        camera.Undistort();

        const std::vector<cv::Point2f> &points =
            trace.useRawInput ? camera.RawFilteredPoints() : camera.UndistortedFilteredPoints();
        const std::vector<int64_t> &timestamps =
            trace.useRawInput ? camera.RawFilteredTimestamps() : camera.UndistortedFilteredTimestamps();
        const std::vector<bool> &polarities =
            trace.useRawInput ? camera.RawFilteredPolarities() : camera.UndistortedFilteredPolarities();

        const std::vector<TracePoint> tracePoints = BuildTracePointsFromFloatSource(
            points,
            timestamps,
            &polarities,
            trace.polarityMode);

        detection.numEvents = static_cast<int>(tracePoints.size());

        const TraceRibbonFit fit = FitTraceRibbon(
            tracePoints,
            trace.lineBinWidthPx,
            trace.localWindowPx,
            trace.lineOrder,
            trace.pcaPeriodMs,
            supportEdge,
            trace.edgeRefineEnabled);

        if (!fit.valid) {
            detection.failureReason = tracePoints.empty() ? "no_events" : "ribbon_fit_failed";
            detection.runtimeMs = std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
            detections.push_back(std::move(detection));
            continue;
        }

        const int64_t traceTimeOriginUs = TraceTimeOriginUs(tracePoints);
        const Trace3DAnalysis analysis = AnalyzeTrace3D(
            fit,
            calibration,
            ballRadiusMm,
            trace.widthStepPx,
            trace.widthSmoothingEnabled,
            traceTimeOriginUs,
            // No ground-truth injection: the estimate must stay independent of
            // the labels it is later scored against.
            [](float, Vector3 &) { return false; });

        if (!analysis.valid) {
            detection.failureReason = "no_valid_3d_sample";
            detection.runtimeMs = std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
            detections.push_back(std::move(detection));
            continue;
        }

        detection.detected = true;
        WorldToCameraMeters(analysis.currentWorld, detection);
        ProjectToImage(calibration, detection);

        // Report the width of the sample actually published, not the ribbon
        // median: worldPoints and times stay parallel through the outlier
        // filter, so the reported sample's time identifies its width estimate.
        const std::size_t midIndex = analysis.worldPoints.size() / 2;
        const float sampleTime = analysis.times[midIndex];
        float bestDelta = std::numeric_limits<float>::max();
        for (const TraceWidthEstimate &estimate : analysis.widthEstimates) {
            if (!estimate.valid || estimate.widthPx < 1.0f) {
                continue;
            }
            const float delta = std::fabs(static_cast<float>(estimate.timeSeconds) - sampleTime);
            if (delta < bestDelta) {
                bestDelta = delta;
                detection.widthPx = static_cast<double>(estimate.widthPx);
            }
        }

        // The trace stamps its estimate with the sample's own event time.
        detection.timestampUs = analysis.currentWorldTimestampUs;
        detection.runtimeMs = std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
        detections.push_back(std::move(detection));
    }

    for (const BenchmarkDetection &detection : detections) {
        runtime.totalRuntimeMs += detection.runtimeMs;
        runtime.detectedRows += detection.detected ? 1 : 0;
    }
    runtime.outputRows = static_cast<int>(detections.size());

    return detections;
}

std::vector<BenchmarkDetection> RunCircleMethod(
    EventReader &reader,
    DvCamera &camera,
    const CalibrationData &calibration,
    const BenchmarkSettings &settings,
    BenchmarkMethodRuntime &runtime) {

    std::vector<BenchmarkDetection> detections;
    runtime.ran = true;

    if (reader.empty() || reader.durationUs() <= 0) {
        return detections;
    }

    const CircleMethodSettings &circle = settings.circle;

    BallTrackerSettings trackerSettings;
    trackerSettings.ballRadiusMm = static_cast<float>(settings.ballRadiusMeters * 1000.0);
    trackerSettings.positiveOnly = circle.positiveOnly;
    trackerSettings.coef = circle.coef;
    trackerSettings.filterSize = circle.filterSize;
    trackerSettings.maxResidual = circle.maxResidual;
    trackerSettings.rayonCote = circle.rayonCote;
    // The config carries GUI slider values; Ui::Sym_coef()/Sym_coef2() divide
    // by 100 before the tracker sees them (a count-imbalance fraction and an
    // angle in radians). Passing the raw slider values would silently disable
    // both symmetry gates.
    trackerSettings.symCoef = circle.symCoef / 100.0f;
    trackerSettings.symCoef2 = circle.symCoef2 / 100.0f;
    // Same value the live node passes to both Cluster() and the tracker.
    trackerSettings.alpha = circle.alpha;
    trackerSettings.radiusGateEnabled = circle.radiusGateEnabled;
    trackerSettings.weightedRegressionEnabled = circle.weightedRegressionEnabled;
    trackerSettings.sliceMode = static_cast<BallSliceMode>(std::clamp(circle.sliceMode, 0, 2));
    trackerSettings.temporalSliceCount = circle.temporalSliceCount;
    trackerSettings.eventsPerSlice = circle.eventsPerSlice;
    trackerSettings.depthJumpGateMm = circle.depthJumpGateMm;

    const Box roi(circle.roiX, circle.roiY, circle.roiW, circle.roiH);
    const double windowSeconds = circle.windowMs * 1.0e-3;
    const int64_t windowUs = static_cast<int64_t>(std::llround(windowSeconds * 1.0e6));

    const int64_t firstUs = reader.startTimestampUs();
    const int64_t lastUs = reader.endTimestampUs();
    const int64_t periodUs =
        std::max<int64_t>(1, static_cast<int64_t>(std::llround(settings.outputPeriodMs * 1000.0)));

    // BallTracker keeps per-throw state (centre history, regressions, the
    // 250 mm depth-jump gate), so one tracker per sequence, fed strictly in
    // chronological order.
    BallTracker tracker;
    tracker.Reset();

    detections.reserve(static_cast<std::size_t>((lastUs - firstUs) / periodUs + 1));

    for (int64_t timestampUs = firstUs; timestampUs <= lastUs; timestampUs += periodUs) {
        const auto start = clock_type::now();

        BenchmarkDetection detection;
        detection.timestampUs = timestampUs;
        detection.windowStartUs = std::max(firstUs, timestampUs - windowUs);
        detection.windowEndUs = timestampUs;

        dv::EventStore window;
        reader.readWindowEndingAt(
            window,
            static_cast<double>(timestampUs - firstUs) * 1.0e-6,
            windowSeconds);

        camera.Filtered = window;
        // No Undistort() here on purpose: the live circle path clusters the
        // raw event coordinates too (Echantillon/Cluster read Filtered), the
        // calibration only enters when the fitted circle becomes a 3D pose.
        camera.Echantillon(circle.maxEvents);
        camera.Cluster(roi, circle.alpha, circle.bandwidth, circle.minNb);

        const std::vector<BallTrackerClusterInput> clusters =
            BuildTrackerClusterInputs(camera.Clusters(), calibration);

        std::size_t clusterEvents = 0;
        for (const BallTrackerClusterInput &cluster : clusters) {
            clusterEvents += cluster.size();
        }
        detection.numEvents = static_cast<int>(clusterEvents);

        const BallTrackerResult result = tracker.Update(clusters, calibration, trackerSettings);

        if (!result.pose.has_value()) {
            detection.failureReason = clusters.empty() ? "no_cluster" : "no_circle_pose";
            detection.runtimeMs = std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
            detections.push_back(std::move(detection));
            continue;
        }

        const BallPose3D &pose = *result.pose;
        detection.detected = true;
        detection.xCamMeters = static_cast<double>(pose.positionMm.x) * 1.0e-3;
        detection.yCamMeters = static_cast<double>(pose.positionMm.y) * 1.0e-3;
        detection.zCamMeters = static_cast<double>(pose.positionMm.z) * 1.0e-3;
        detection.centerUPx = static_cast<double>(pose.circle.x);
        detection.centerVPx = static_cast<double>(pose.circle.y);
        detection.circleRadiusPx = static_cast<double>(pose.RadiusPx);
        detection.widthPx = 2.0 * detection.circleRadiusPx;

        // The circle pose is stamped with the newest event of its slice.
        detection.timestampUs = result.poseTimestampUs;
        detection.runtimeMs = std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
        detections.push_back(std::move(detection));
    }

    for (const BenchmarkDetection &detection : detections) {
        runtime.totalRuntimeMs += detection.runtimeMs;
        runtime.detectedRows += detection.detected ? 1 : 0;
    }
    runtime.outputRows = static_cast<int>(detections.size());

    return detections;
}

void WriteDetectionsCsv(
    const std::string &path,
    const std::vector<BenchmarkDetection> &detections,
    BenchmarkMethod method) {

    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot write detections CSV: " + path);
    }

    const bool isCircle = method == BenchmarkMethod::Circle;

    file << "timestamp_s,timestamp_us,detected,center_u_px,center_v_px,"
         << (isCircle ? "circle_radius_px," : "")
         << "trace_width_px,depth_est_m,x_est_m,y_est_m,z_est_m,"
            "num_events,runtime_ms,failure_reason,window_start_us,window_end_us\n";

    file << std::fixed;
    for (const BenchmarkDetection &detection : detections) {
        file << std::setprecision(9)
             << static_cast<double>(detection.timestampUs) * 1.0e-6 << ','
             << detection.timestampUs << ','
             << (detection.detected ? 1 : 0) << ',';

        if (detection.detected) {
            file << std::setprecision(6);
            file << detection.centerUPx << ',' << detection.centerVPx << ',';
            if (isCircle) {
                file << detection.circleRadiusPx << ',';
            }
            file << detection.widthPx << ','
                 << detection.zCamMeters << ','
                 << detection.xCamMeters << ','
                 << detection.yCamMeters << ','
                 << detection.zCamMeters << ',';
        }
        else {
            file << ",," << (isCircle ? "," : "") << ",,,,,";
        }

        file << detection.numEvents << ','
             << std::setprecision(6) << detection.runtimeMs << ','
             << CsvEscape(detection.failureReason) << ','
             << detection.windowStartUs << ','
             << detection.windowEndUs << '\n';
    }
}

void WriteRuntimeJson(const std::string &path, const BenchmarkRuntime &runtime) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot write runtime JSON: " + path);
    }

    const TraceMethodSettings &t = runtime.settings.trace;
    const CircleMethodSettings &c = runtime.settings.circle;

    auto methodBlock = [&file](const char *name, const BenchmarkMethodRuntime &m) {
        file << "  \"" << name << "\": {\n"
             << "    \"ran\": " << (m.ran ? "true" : "false") << ",\n"
             << "    \"output_rows\": " << m.outputRows << ",\n"
             << "    \"detected_rows\": " << m.detectedRows << ",\n"
             << "    \"total_runtime_ms\": " << m.totalRuntimeMs << "\n"
             << "  },\n";
    };

    file << std::fixed << std::setprecision(6);
    file << "{\n"
         << "  \"events_h5\": \"" << JsonEscape(runtime.eventsPath) << "\",\n"
         << "  \"camera\": \"" << JsonEscape(runtime.cameraPath) << "\",\n"
         << "  \"ground_truth\": \"" << JsonEscape(runtime.groundTruthPath) << "\",\n"
         << "  \"metadata\": \"" << JsonEscape(runtime.metadataPath) << "\",\n"
         << "  \"sequence_name\": \"" << JsonEscape(runtime.sequenceName) << "\",\n"
         << "  \"input_event_count\": " << runtime.inputEventCount << ",\n"
         << "  \"first_timestamp_us\": " << runtime.firstTimestampUs << ",\n"
         << "  \"last_timestamp_us\": " << runtime.lastTimestampUs << ",\n";

    methodBlock("trace", runtime.trace);
    methodBlock("circle", runtime.circle);

    file << "  \"intrinsics\": {\n"
         << "    \"source\": \"" << JsonEscape(runtime.cameraPath) << "\",\n"
         << "    \"width\": " << runtime.imageWidth << ",\n"
         << "    \"height\": " << runtime.imageHeight << ",\n"
         << "    \"fx\": " << runtime.fx << ",\n"
         << "    \"fy\": " << runtime.fy << ",\n"
         << "    \"cx\": " << runtime.cx << ",\n"
         << "    \"cy\": " << runtime.cy << ",\n"
         << "    \"distortion_coefficient_count\": " << runtime.distortionCoefficientCount << "\n"
         << "  },\n"
         << "  \"ball_radius_m\": " << runtime.settings.ballRadiusMeters << ",\n"
         << "  \"output_period_ms\": " << runtime.settings.outputPeriodMs << ",\n"
         << "  \"trace_settings\": {\n"
         << "    \"trace_memory_ms\": " << t.traceMemoryMs << ",\n"
         << "    \"line_bin_width_px\": " << t.lineBinWidthPx << ",\n"
         << "    \"local_window_px\": " << t.localWindowPx << ",\n"
         << "    \"line_order\": " << t.lineOrder << ",\n"
         << "    \"pca_period_ms\": " << t.pcaPeriodMs << ",\n"
         << "    \"width_step_px\": " << t.widthStepPx << ",\n"
         << "    \"support_divisor\": " << t.supportDivisor << ",\n"
         << "    \"support_min_count\": " << t.supportMinCount << ",\n"
         << "    \"support_max_count\": " << t.supportMaxCount << ",\n"
         << "    \"support_radius_px\": " << t.supportRadiusPx << ",\n"
         << "    \"border_ratio\": " << t.borderRatio << ",\n"
         << "    \"border_pixels\": " << t.borderPixels << ",\n"
         << "    \"border_spacing_factor\": " << t.borderSpacingFactor << ",\n"
         << "    \"polarity_mode\": " << t.polarityMode << ",\n"
         << "    \"fit_input\": \"" << (t.useRawInput ? "raw" : "undist") << "\",\n"
         << "    \"edge_refine\": " << (t.edgeRefineEnabled ? "true" : "false") << ",\n"
         << "    \"width_smoothing\": " << (t.widthSmoothingEnabled ? "true" : "false") << "\n"
         << "  },\n"
         << "  \"circle_settings\": {\n"
         << "    \"window_ms\": " << c.windowMs << ",\n"
         << "    \"bandwidth\": " << c.bandwidth << ",\n"
         << "    \"min_nb\": " << c.minNb << ",\n"
         << "    \"max_events\": " << c.maxEvents << ",\n"
         << "    \"alpha\": " << c.alpha << ",\n"
         << "    \"coef\": " << c.coef << ",\n"
         << "    \"filter_size\": " << c.filterSize << ",\n"
         << "    \"max_residual\": " << c.maxResidual << ",\n"
         << "    \"sym_coef\": " << c.symCoef << ",\n"
         << "    \"sym_coef2\": " << c.symCoef2 << ",\n"
         << "    \"slice_mode\": " << c.sliceMode << ",\n"
         << "    \"depth_jump_gate_mm\": " << c.depthJumpGateMm << ",\n"
         << "    \"radius_gate\": " << (c.radiusGateEnabled ? "true" : "false") << "\n"
         << "  },\n"
         << "  \"ground_truth_summary\": {\n"
         << "    \"rows\": " << runtime.groundTruthRows << ",\n"
         << "    \"visible_rows\": " << runtime.groundTruthVisibleRows << ",\n"
         << "    \"first_timestamp_s\": " << runtime.groundTruthFirstTimestampS << ",\n"
         << "    \"last_timestamp_s\": " << runtime.groundTruthLastTimestampS << "\n"
         << "  }\n"
         << "}\n";
}
