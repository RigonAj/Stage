// Offline Trace vs circle-fitting comparison on simulated sequences.
//
// Replays the event stream of one or more Isaac Sim + v2e sequences through
// BOTH 3D estimators and writes, per estimate, the position and the
// time-interpolated ground truth. scripts/compare_trace_vs_circle.py turns
// those CSVs into RMSE / bias / coverage tables.
//
// Design rule: this binary never reimplements an algorithm. It drives the same
// code the live node drives - TraceAccumulator + RunTraceAnalysis for Trace,
// DvCamera::Cluster + BallTracker::Update for circle fitting. The previous
// benchmark (TraceBenchmark.cpp, removed in June) had its own copy of the trace
// maths and stopped measuring the real algorithm as soon as the two diverged.
//
// Intrinsics and ball radius come from the sequence itself
// (camera/intrinsics.json, metadata.json), never from the physical DVXplorer
// calibration: depth is f * D / size, so using the live defaults here would
// bias every single measurement.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <opencv2/core.hpp>

#include "BallTracker.hpp"
#include "Camera.hpp"
#include "EventWriter.h"
#include "SequenceDataset.hpp"
#include "TraceAnalysis.hpp"
#include "TraceRuntime.hpp"
#include "util.hpp"

namespace fs = std::filesystem;
namespace sd = sequence_dataset;

namespace {

constexpr const char *kMethodTrace = "trace";
constexpr const char *kMethodCircle = "circle";

// --------------------------------------------------------------------------
// Options
// --------------------------------------------------------------------------

struct Options {
    std::vector<std::string> datasetRoots;
    std::vector<std::string> sequences;
    std::string outputDir;
    bool runTrace = true;
    bool runCircle = true;

    // Explicit event-file name inside a sequence; empty = try the known layouts.
    std::string eventsName;
    int maxDiscoveryDepth = 4;

    // Replay cadence. 1 ms is the ROS node's timer period.
    double tickMs = 1.0;
    // Trace analysis rate limit, the trace_analysis_period_ms ROS parameter.
    double traceAnalysisPeriodMs = 4.0;
    // Circle branch cadence; 0 = every tick, like the node.
    double circlePeriodMs = 0.0;

    // Trace settings (Ui defaults, except polarity which follows the ROS
    // default "all" rather than the historical slider value).
    TraceRuntimeSettings trace{};

    // Circle settings, pre-scaled exactly like Pub::trackerSettings() scales
    // the Ui getters (FilterSize = filterSize/100, Alpha = alpha/100, ...).
    double circleWindowMs = 484.32;
    int circleMaxEvents = 1000;
    int circleBandwidth = 50;
    unsigned int circleMinNb = 40;
    BallTrackerSettings circle{};

    // Ball radius override in mm; when unset it is read from metadata.json.
    std::optional<double> ballRadiusMmOverride;

    // "file" keeps raw event timestamps as absolute seconds (what the GUI
    // does); "zero" shifts the stream so its first event lands at t = 0.
    std::string timeBase = "file";
    double timeOffsetSeconds = 0.0;

    // "newest" = one estimate per distinct measurement instant (default),
    // "all" = every sample of every analysis run.
    std::string emitMode = "newest";
};

void PrintUsage(const char *argv0) {
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "\n"
        << "Inputs (at least one required):\n"
        << "  --dataset-root DIR        recursively discover sequences under DIR\n"
        << "  --sequence DIR            a single sequence directory (repeatable)\n"
        << "  --events-name NAME        event file name/relative path inside a sequence\n"
        << "  --discovery-depth N       max recursion depth for --dataset-root (default 4)\n"
        << "\n"
        << "Output:\n"
        << "  --out DIR                 output directory (default evaluation/method_comparison/run)\n"
        << "  --methods trace,circle    which estimators to run (default both)\n"
        << "  --emit newest|all         one estimate per instant, or every sample (default newest)\n"
        << "\n"
        << "Replay:\n"
        << "  --tick-ms F               replay step (default 1.0, the node timer period)\n"
        << "  --trace-analysis-period-ms F   trace rate limit (default 4.0)\n"
        << "  --circle-period-ms F      circle cadence, 0 = every tick (default 0)\n"
        << "  --time-base file|zero     absolute time convention (default file)\n"
        << "  --time-offset-s F         extra offset added to event time (default 0)\n"
        << "\n"
        << "Trace parameters (defaults = live GUI defaults):\n"
        << "  --trace-memory-ms F       accumulation window (default 150)\n"
        << "  --trace-polarity all|positive|negative   (default all)\n"
        << "  --trace-width-step-px F   (default 8)\n"
        << "  --trace-edge-refine       enable end-cap trim (default off)\n"
        << "  --trace-width-smoothing   enable robust width(t) (default off)\n"
        << "  --trace-raw-input         fit on raw instead of undistorted points\n"
        << "  --roi X Y W H             work-ROI in pixels (default full frame)\n"
        << "\n"
        << "Circle parameters (defaults = live GUI defaults):\n"
        << "  --circle-window-ms F      trailing event window (default 484.32)\n"
        << "  --circle-max-events N     subsampling budget before DBSCAN (default 1000)\n"
        << "  --circle-bandwidth N      DBSCAN bandwidth (default 50)\n"
        << "  --circle-min-nb N         DBSCAN min events per cluster (default 40)\n"
        << "  --circle-radius-gate      enable the radius/residual gate (default off)\n"
        << "\n"
        << "  --ball-radius-mm F        override metadata.json ball.radius_m\n"
        << "  -h, --help\n";
}

double RequireDouble(const std::string &text, const std::string &flag) {
    try {
        return std::stod(text);
    }
    catch (const std::exception &) {
        throw std::runtime_error("invalid number for " + flag + ": " + text);
    }
}

int RequireInt(const std::string &text, const std::string &flag) {
    try {
        return std::stoi(text);
    }
    catch (const std::exception &) {
        throw std::runtime_error("invalid integer for " + flag + ": " + text);
    }
}

std::vector<std::string> SplitCommaList(const std::string &text) {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string part;
    while (std::getline(stream, part, ',')) {
        if (!part.empty()) {
            parts.push_back(part);
        }
    }
    return parts;
}

Options ParseArgs(int argc, char **argv) {
    Options options;
    // The ROS node defaults trace_polarity_mode to "all"; the Ui slider default
    // (negative) starved the ribbon depending on contrast direction.
    options.trace.polarityMode = 0;
    options.circle.ballRadiusMm = 20.0f;
    options.circle.positiveOnly = false;
    options.circle.coef = 0.45f;
    options.circle.filterSize = 115.0f / 100.0f;
    options.circle.maxResidual = 19.0f;
    options.circle.rayonCote = 0.0f;
    options.circle.symCoef = 29.0f / 100.0f;
    options.circle.symCoef2 = 157.0f / 100.0f;
    options.circle.alpha = 50.0f / 100.0f;
    options.circle.radiusGateEnabled = false;
    options.circle.weightedRegressionEnabled = false;
    options.circle.sliceMode = BallSliceMode::RecentEvents;
    options.circle.temporalSliceCount = 5;
    options.circle.eventsPerSlice = 100;

    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto value = [&](const std::string &flag) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + flag);
            }
            return argv[++i];
        };

        if (key == "--dataset-root") {
            options.datasetRoots.push_back(value(key));
        }
        else if (key == "--sequence") {
            options.sequences.push_back(value(key));
        }
        else if (key == "--events-name") {
            options.eventsName = value(key);
        }
        else if (key == "--discovery-depth") {
            options.maxDiscoveryDepth = RequireInt(value(key), key);
        }
        else if (key == "--out") {
            options.outputDir = value(key);
        }
        else if (key == "--methods") {
            const std::vector<std::string> methods = SplitCommaList(value(key));
            options.runTrace = false;
            options.runCircle = false;
            for (const std::string &method : methods) {
                if (method == kMethodTrace) {
                    options.runTrace = true;
                }
                else if (method == kMethodCircle) {
                    options.runCircle = true;
                }
                else {
                    throw std::runtime_error("unknown method: " + method);
                }
            }
            if (!options.runTrace && !options.runCircle) {
                throw std::runtime_error("--methods selected no method");
            }
        }
        else if (key == "--emit") {
            options.emitMode = value(key);
            if (options.emitMode != "newest" && options.emitMode != "all") {
                throw std::runtime_error("--emit must be 'newest' or 'all'");
            }
        }
        else if (key == "--tick-ms") {
            options.tickMs = RequireDouble(value(key), key);
        }
        else if (key == "--trace-analysis-period-ms") {
            options.traceAnalysisPeriodMs = RequireDouble(value(key), key);
        }
        else if (key == "--circle-period-ms") {
            options.circlePeriodMs = RequireDouble(value(key), key);
        }
        else if (key == "--time-base") {
            options.timeBase = value(key);
            if (options.timeBase != "file" && options.timeBase != "zero") {
                throw std::runtime_error("--time-base must be 'file' or 'zero'");
            }
        }
        else if (key == "--time-offset-s") {
            options.timeOffsetSeconds = RequireDouble(value(key), key);
        }
        else if (key == "--trace-memory-ms") {
            options.trace.memorySeconds = RequireDouble(value(key), key) * 1.0e-3;
        }
        else if (key == "--trace-polarity") {
            const std::string mode = value(key);
            if (mode == "all") {
                options.trace.polarityMode = 0;
            }
            else if (mode == "positive") {
                options.trace.polarityMode = 1;
            }
            else if (mode == "negative") {
                options.trace.polarityMode = 2;
            }
            else {
                throw std::runtime_error("--trace-polarity must be all|positive|negative");
            }
        }
        else if (key == "--trace-width-step-px") {
            options.trace.widthStepPx = static_cast<float>(RequireDouble(value(key), key));
        }
        else if (key == "--trace-edge-refine") {
            options.trace.edgeRefine = true;
        }
        else if (key == "--trace-width-smoothing") {
            options.trace.widthSmoothing = true;
        }
        else if (key == "--trace-raw-input") {
            options.trace.useRawInput = true;
        }
        else if (key == "--roi") {
            options.trace.roi.x = static_cast<float>(RequireDouble(value(key), key));
            options.trace.roi.y = static_cast<float>(RequireDouble(value(key), key));
            options.trace.roi.w = static_cast<float>(RequireDouble(value(key), key));
            options.trace.roi.h = static_cast<float>(RequireDouble(value(key), key));
        }
        else if (key == "--circle-window-ms") {
            options.circleWindowMs = RequireDouble(value(key), key);
        }
        else if (key == "--circle-max-events") {
            options.circleMaxEvents = RequireInt(value(key), key);
        }
        else if (key == "--circle-bandwidth") {
            options.circleBandwidth = RequireInt(value(key), key);
        }
        else if (key == "--circle-min-nb") {
            options.circleMinNb = static_cast<unsigned int>(RequireInt(value(key), key));
        }
        else if (key == "--circle-radius-gate") {
            options.circle.radiusGateEnabled = true;
        }
        else if (key == "--ball-radius-mm") {
            options.ballRadiusMmOverride = RequireDouble(value(key), key);
        }
        else if (key == "--help" || key == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        else {
            throw std::runtime_error("unknown argument: " + key);
        }
    }

    if (options.datasetRoots.empty() && options.sequences.empty()) {
        throw std::runtime_error("need at least one --dataset-root or --sequence");
    }
    if (options.outputDir.empty()) {
        options.outputDir = "evaluation/method_comparison/run";
    }
    if (options.tickMs <= 0.0) {
        throw std::runtime_error("--tick-ms must be > 0");
    }

    return options;
}

// --------------------------------------------------------------------------
// Sequence discovery
// --------------------------------------------------------------------------

struct SequenceRef {
    fs::path directory;
    fs::path eventsPath;
    fs::path intrinsicsPath;
    fs::path groundTruthPath;
    fs::path metadataPath;
    std::string name;
};

bool HasSuffixCaseInsensitive(const std::string &value, const std::string &suffix) {
    if (value.size() < suffix.size()) {
        return false;
    }
    return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin(), [](char a, char b) {
        return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b));
    });
}

bool IsEventFileName(const std::string &filename) {
    return HasSuffixCaseInsensitive(filename, ".h5")
        || HasSuffixCaseInsensitive(filename, ".hdf5")
        || HasSuffixCaseInsensitive(filename, ".bin");
}

// The generator's layout is events_v2e/events.h5, but runs differ (filtered vs
// raw streams, an "events" directory instead of "events_v2e"). Try the known
// names, then fall back to the first event file under any events* directory,
// then anywhere in the sequence.
fs::path FindEventsFile(const fs::path &sequenceDir, const std::string &explicitName) {
    std::error_code ec;

    if (!explicitName.empty()) {
        const fs::path direct = sequenceDir / explicitName;
        if (fs::is_regular_file(direct, ec)) {
            return direct;
        }
        // Also accept a bare file name found anywhere in the sequence.
        for (const fs::directory_entry &entry : fs::recursive_directory_iterator(sequenceDir, ec)) {
            std::error_code entryEc;
            if (!entry.is_regular_file(entryEc)) {
                continue;
            }
            if (entry.path().filename().string() == explicitName) {
                return entry.path();
            }
        }
        return {};
    }

    static const char *kKnownPaths[] = {
        "events_v2e/events.h5",
        "events_v2e/events_filtered.h5",
        "events_v2e/events_raw.h5",
        "events/events.h5",
        "events.h5",
    };
    for (const char *known : kKnownPaths) {
        const fs::path candidate = sequenceDir / known;
        if (fs::is_regular_file(candidate, ec)) {
            return candidate;
        }
    }

    std::vector<fs::path> fallback;
    for (const fs::directory_entry &entry : fs::recursive_directory_iterator(sequenceDir, ec)) {
        std::error_code entryEc;
        if (!entry.is_regular_file(entryEc)) {
            continue;
        }
        if (IsEventFileName(entry.path().filename().string())) {
            fallback.push_back(entry.path());
        }
    }

    if (fallback.empty()) {
        return {};
    }

    std::sort(fallback.begin(), fallback.end());
    return fallback.front();
}

// A directory is a sequence when it carries both sidecars the comparison needs:
// the ground truth to compare against and the intrinsics to project with.
std::optional<SequenceRef> MakeSequenceRef(
    const fs::path &directory,
    const std::string &eventsName,
    std::string &rejectReason) {

    std::error_code ec;
    const fs::path groundTruth = directory / "labels" / "ground_truth.csv";
    const fs::path intrinsics = directory / "camera" / "intrinsics.json";

    if (!fs::is_regular_file(groundTruth, ec)) {
        rejectReason = "no labels/ground_truth.csv";
        return std::nullopt;
    }
    if (!fs::is_regular_file(intrinsics, ec)) {
        rejectReason = "no camera/intrinsics.json";
        return std::nullopt;
    }

    const fs::path events = FindEventsFile(directory, eventsName);
    if (events.empty()) {
        rejectReason = "no event file (looked for events_v2e/events.h5 and any *.h5/*.bin)";
        return std::nullopt;
    }

    SequenceRef ref;
    ref.directory = directory;
    ref.eventsPath = events;
    ref.intrinsicsPath = intrinsics;
    ref.groundTruthPath = groundTruth;
    ref.name = directory.filename().string();

    const fs::path metadata = directory / "metadata.json";
    if (fs::is_regular_file(metadata, ec)) {
        ref.metadataPath = metadata;
    }

    return ref;
}

void DiscoverSequences(
    const fs::path &root,
    int maxDepth,
    const std::string &eventsName,
    std::vector<SequenceRef> &found) {

    std::error_code ec;
    if (!fs::is_directory(root, ec)) {
        std::cerr << "discovery: not a directory, skipped: " << root.string() << "\n";
        return;
    }

    std::string reason;
    if (auto ref = MakeSequenceRef(root, eventsName, reason)) {
        found.push_back(*ref);
        return;
    }

    if (maxDepth <= 0) {
        return;
    }

    std::vector<fs::path> children;
    for (const fs::directory_entry &entry : fs::directory_iterator(root, ec)) {
        std::error_code entryEc;
        if (entry.is_directory(entryEc)) {
            children.push_back(entry.path());
        }
    }
    std::sort(children.begin(), children.end());

    for (const fs::path &child : children) {
        DiscoverSequences(child, maxDepth - 1, eventsName, found);
    }
}

// --------------------------------------------------------------------------
// Detection rows
// --------------------------------------------------------------------------

struct DetectionRow {
    std::string method;
    std::string sequence;
    int runIndex = 0;
    double tEstSeconds = 0.0;
    // camera_optical metres: x right, y down, z depth.
    Vector3 estimate{0.0f, 0.0f, 0.0f};
    bool gtValid = false;
    bool gtVisible = false;
    Vector3 groundTruth{0.0f, 0.0f, 0.0f};
    // Apparent size the depth came from: ribbon width for trace, circle radius
    // for circle fitting (both in pixels; not the same quantity, hence the
    // separate size_kind column).
    double sizePx = 0.0;
    std::string sizeKind;
    int windowEvents = 0;
    int windowPoints = 0;
    double runtimeMs = 0.0;
};

class DetectionWriter {
public:
    explicit DetectionWriter(const fs::path &path) : file_(path) {
        if (!file_) {
            throw std::runtime_error("cannot write " + path.string());
        }
        file_ << std::fixed;
        file_ << "method,sequence,run_index,t_est_s,"
                 "x_cam_m,y_cam_m,z_cam_m,"
                 "gt_valid,gt_visible,gt_x_cam_m,gt_y_cam_m,gt_z_cam_m,"
                 "err_x_m,err_y_m,err_z_m,err_norm_m,"
                 "size_px,size_kind,window_events,window_points,runtime_ms\n";
    }

    void Write(const DetectionRow &row) {
        const float ex = row.estimate.x - row.groundTruth.x;
        const float ey = row.estimate.y - row.groundTruth.y;
        const float ez = row.estimate.z - row.groundTruth.z;
        const float norm = std::sqrt(ex * ex + ey * ey + ez * ez);

        file_ << row.method << ','
              << row.sequence << ','
              << row.runIndex << ','
              << std::setprecision(9) << row.tEstSeconds << ','
              << std::setprecision(6)
              << row.estimate.x << ',' << row.estimate.y << ',' << row.estimate.z << ','
              << (row.gtValid ? 1 : 0) << ',' << (row.gtVisible ? 1 : 0) << ',';

        if (row.gtValid) {
            file_ << row.groundTruth.x << ',' << row.groundTruth.y << ',' << row.groundTruth.z << ','
                  << ex << ',' << ey << ',' << ez << ',' << norm << ',';
        }
        else {
            file_ << ",,,,,,,";
        }

        file_ << std::setprecision(4) << row.sizePx << ','
              << row.sizeKind << ','
              << row.windowEvents << ','
              << row.windowPoints << ','
              << std::setprecision(4) << row.runtimeMs << '\n';

        ++count_;
    }

    std::size_t count() const { return count_; }

private:
    std::ofstream file_;
    std::size_t count_ = 0;
};

// --------------------------------------------------------------------------
// Per-sequence run
// --------------------------------------------------------------------------

struct MethodStats {
    std::size_t estimates = 0;
    std::size_t gtMatched = 0;
    std::size_t runs = 0;
    std::size_t validRuns = 0;
    double totalRuntimeMs = 0.0;
    double firstEstimateSeconds = std::numeric_limits<double>::quiet_NaN();
};

struct SequenceReport {
    std::string name;
    std::string eventsPath;
    std::string intrinsicsPath;
    std::string groundTruthPath;
    std::string metadataPath;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int width = 0;
    int height = 0;
    std::string distortionSource;
    double ballRadiusMm = 0.0;
    std::string ballRadiusSource;
    int64_t eventCount = 0;
    int64_t tStartUs = 0;
    int64_t tEndUs = 0;
    std::size_t groundTruthRows = 0;
    double groundTruthFirstSeconds = 0.0;
    double groundTruthLastSeconds = 0.0;
    MethodStats trace;
    MethodStats circle;
    std::string error;
};

std::string EscapeJson(const std::string &text) {
    std::string out;
    out.reserve(text.size() + 8);
    for (const char c : text) {
        if (c == '"' || c == '\\') {
            out.push_back('\\');
            out.push_back(c);
        }
        else if (c == '\n') {
            out += "\\n";
        }
        else {
            out.push_back(c);
        }
    }
    return out;
}

SequenceReport RunSequence(
    const SequenceRef &ref,
    const Options &options,
    const fs::path &outputDir) {

    SequenceReport report;
    report.name = ref.name;
    report.eventsPath = ref.eventsPath.string();
    report.intrinsicsPath = ref.intrinsicsPath.string();
    report.groundTruthPath = ref.groundTruthPath.string();
    report.metadataPath = ref.metadataPath.string();

    // --- Camera: the sequence's own intrinsics, never the DVXplorer XML. ---
    const CalibrationData calibration = sd::LoadCalibrationFromIntrinsicsJson(ref.intrinsicsPath);
    if (!calibration.ready) {
        report.error = "invalid intrinsics at " + ref.intrinsicsPath.string();
        return report;
    }
    report.fx = calibration.fx();
    report.fy = calibration.fy();
    report.cx = calibration.cx();
    report.cy = calibration.cy();
    report.width = calibration.imageSize.width;
    report.height = calibration.imageSize.height;
    report.distortionSource = calibration.useFisheyeModel ? "fisheye" : "opencv";

    // --- Ball radius: the sequence's own metadata, not the 60 mm default. ---
    double ballRadiusMm = 0.0;
    if (options.ballRadiusMmOverride.has_value()) {
        ballRadiusMm = *options.ballRadiusMmOverride;
        report.ballRadiusSource = "--ball-radius-mm";
    }
    else if (!ref.metadataPath.empty()) {
        if (const auto radius = sd::LoadBallRadiusMmFromMetadata(ref.metadataPath)) {
            ballRadiusMm = *radius;
            report.ballRadiusSource = ref.metadataPath.string();
        }
    }
    if (ballRadiusMm <= 0.0) {
        report.error =
            "no ball radius: metadata.json missing or without ball.radius_m, "
            "and no --ball-radius-mm given";
        return report;
    }
    report.ballRadiusMm = ballRadiusMm;

    // --- Ground truth ---
    const sd::GroundTruthTable groundTruth = sd::LoadGroundTruthCsv(ref.groundTruthPath);
    if (groundTruth.empty()) {
        report.error = "empty or unreadable ground truth at " + ref.groundTruthPath.string();
        return report;
    }
    report.groundTruthRows = groundTruth.size();
    report.groundTruthFirstSeconds = groundTruth.firstTimeSeconds();
    report.groundTruthLastSeconds = groundTruth.lastTimeSeconds();

    // --- Events ---
    EventReader reader(ref.eventsPath.string());
    if (reader.empty()) {
        report.error = "no events in " + ref.eventsPath.string();
        return report;
    }
    report.eventCount = static_cast<int64_t>(reader.count());
    report.tStartUs = reader.startTimestampUs();
    report.tEndUs = reader.endTimestampUs();

    // Absolute event time -> ground-truth time base.
    const double timeShiftSeconds =
        (options.timeBase == "zero" ? -static_cast<double>(reader.startTimestampUs()) * 1.0e-6 : 0.0)
        + options.timeOffsetSeconds;

    TraceRuntimeSettings traceSettings = options.trace;
    traceSettings.ballRadiusMm = static_cast<float>(ballRadiusMm);

    BallTrackerSettings circleSettings = options.circle;
    circleSettings.ballRadiusMm = static_cast<float>(ballRadiusMm);

    // One DvCamera drives both branches: no DVXplorer is attached (the
    // constructor degrades gracefully), and Filtered is reassigned between the
    // trace slice and the circle window within each tick.
    DvCamera camera;
    camera.calibration = calibration;

    TraceAccumulator accumulator;
    BallTracker tracker;
    const Box box(0.0f, 0.0f,
                  static_cast<float>(calibration.imageSize.width),
                  static_cast<float>(calibration.imageSize.height));

    std::optional<DetectionWriter> traceWriter;
    std::optional<DetectionWriter> circleWriter;
    if (options.runTrace) {
        traceWriter.emplace(outputDir / ("detections_" + ref.name + "_trace.csv"));
    }
    if (options.runCircle) {
        circleWriter.emplace(outputDir / ("detections_" + ref.name + "_circle.csv"));
    }

    const auto lookup = [&groundTruth](float timeSeconds, Vector3 &worldPoint) {
        return sd::LookupGroundTruthWorld(groundTruth, timeSeconds, worldPoint);
    };

    const double tickSeconds = options.tickMs * 1.0e-3;
    const double durationSeconds = reader.durationSeconds();
    const double circlePeriodSeconds = options.circlePeriodMs * 1.0e-3;
    const double traceAnalysisPeriodSeconds = options.traceAnalysisPeriodMs * 1.0e-3;

    double lastTraceAnalysisSeconds = -1.0e9;
    double lastCircleSeconds = -1.0e9;
    int64_t lastEmittedTraceTimeUs = std::numeric_limits<int64_t>::min();
    int64_t lastEmittedCircleTimeUs = std::numeric_limits<int64_t>::min();
    bool traceDirty = false;
    int runIndex = 0;

    dv::EventStore slice;
    dv::EventStore window;

    // Integer tick index: accumulating `t += tickSeconds` over thousands of
    // iterations drifts, and the slice boundaries must tile the stream exactly
    // or events fall between two reads.
    const int64_t tickCount =
        static_cast<int64_t>(std::ceil(durationSeconds / tickSeconds)) + 1;

    for (int64_t tick = 0; tick <= tickCount; ++tick) {
        const double t = static_cast<double>(tick) * tickSeconds;
        // ---------------- Trace branch ----------------
        if (options.runTrace) {
            reader.readWindow(slice, t, tickSeconds);
            if (!slice.isEmpty()) {
                camera.Filtered = slice;
                camera.Undistort();

                // Same choice the node makes before AppendTraceEvents: the
                // accumulator is the ribbon's primary source, so raw-vs-
                // undistorted has to be decided here, not in the fit.
                const bool raw = traceSettings.useRawInput;
                if (accumulator.Append(
                        raw ? camera.RawFilteredPoints() : camera.UndistortedFilteredPoints(),
                        raw ? camera.RawFilteredTimestamps() : camera.UndistortedFilteredTimestamps(),
                        raw ? &camera.RawFilteredPolarities() : &camera.UndistortedFilteredPolarities(),
                        traceSettings.roi,
                        traceSettings.memorySeconds)) {
                    traceDirty = true;
                }
            }

            if (traceDirty && (t - lastTraceAnalysisSeconds) >= traceAnalysisPeriodSeconds) {
                lastTraceAnalysisSeconds = t;
                traceDirty = false;
                ++report.trace.runs;

                TracePointSources sources;
                sources.events = nullptr;
                sources.motionWindowValid = false;

                const auto started = std::chrono::steady_clock::now();
                const TraceRunResult run = RunTraceAnalysis(
                    accumulator, sources, traceSettings, calibration, lookup);
                const double runtimeMs = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started).count();
                report.trace.totalRuntimeMs += runtimeMs;

                if (run.analysis.valid
                    && run.analysis.worldPoints.size() == run.analysis.times.size()) {
                    ++report.trace.validRuns;
                    ++runIndex;

                    const std::size_t pointCount = run.analysis.worldPoints.size();
                    for (std::size_t i = 0; i < pointCount; ++i) {
                        const int64_t sampleTimeUs =
                            run.timeOriginUs
                            + static_cast<int64_t>(std::llround(
                                  static_cast<double>(run.analysis.times[i]) * 1.0e6));

                        // "newest": one estimate per distinct measurement
                        // instant, taken from the first window that covered it.
                        // Without this, overlapping windows re-estimate the same
                        // instants and inflate every statistic.
                        if (options.emitMode == "newest"
                            && lastEmittedTraceTimeUs != std::numeric_limits<int64_t>::min()
                            && sampleTimeUs <= lastEmittedTraceTimeUs) {
                            continue;
                        }

                        DetectionRow row;
                        row.method = kMethodTrace;
                        row.sequence = ref.name;
                        row.runIndex = runIndex;
                        row.tEstSeconds =
                            static_cast<double>(sampleTimeUs) * 1.0e-6 + timeShiftSeconds;
                        row.estimate = sd::WorldToCameraOptical(run.analysis.worldPoints[i]);

                        Vector3 gtWorld{};
                        row.gtValid = sd::LookupGroundTruthWorld(
                            groundTruth, static_cast<float>(row.tEstSeconds), gtWorld);
                        if (row.gtValid) {
                            row.groundTruth = sd::WorldToCameraOptical(gtWorld);
                            row.gtVisible = sd::LookupGroundTruthVisible(
                                groundTruth, static_cast<float>(row.tEstSeconds));
                            ++report.trace.gtMatched;
                        }

                        row.sizePx = run.fit.widthPx;
                        row.sizeKind = "ribbon_width";
                        row.windowEvents = static_cast<int>(run.source.points.size());
                        row.windowPoints = static_cast<int>(pointCount);
                        row.runtimeMs = runtimeMs;

                        traceWriter->Write(row);
                        ++report.trace.estimates;
                        if (std::isnan(report.trace.firstEstimateSeconds)) {
                            report.trace.firstEstimateSeconds = row.tEstSeconds;
                        }

                        lastEmittedTraceTimeUs = std::max(lastEmittedTraceTimeUs, sampleTimeUs);
                    }
                }
            }
        }

        // ---------------- Circle branch ----------------
        if (options.runCircle && (t - lastCircleSeconds) >= circlePeriodSeconds) {
            lastCircleSeconds = t;
            reader.readWindowEndingAt(window, t, options.circleWindowMs * 1.0e-3);

            if (!window.isEmpty()) {
                ++report.circle.runs;
                camera.Filtered = window;

                const auto started = std::chrono::steady_clock::now();
                camera.Echantillon(options.circleMaxEvents);
                camera.Cluster(box, circleSettings.alpha, options.circleBandwidth, options.circleMinNb);
                const std::vector<BallTrackerClusterInput> clusters =
                    BuildTrackerClusters(camera.Clusters(), calibration);
                const BallTrackerResult tracking =
                    tracker.Update(clusters, calibration, circleSettings);
                const double runtimeMs = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started).count();
                report.circle.totalRuntimeMs += runtimeMs;

                if (tracking.pose.has_value()) {
                    ++report.circle.validRuns;
                    const int64_t poseTimeUs = tracking.poseTimestampUs;

                    // Same dedup rule as the trace branch: the circle window
                    // slides by one tick but spans ~484 ms, so the identical
                    // detection reappears on many consecutive ticks.
                    const bool fresh =
                        options.emitMode == "all"
                        || lastEmittedCircleTimeUs == std::numeric_limits<int64_t>::min()
                        || poseTimeUs > lastEmittedCircleTimeUs;

                    if (fresh) {
                        const BallPose3D &pose = *tracking.pose;

                        DetectionRow row;
                        row.method = kMethodCircle;
                        row.sequence = ref.name;
                        row.runIndex = static_cast<int>(report.circle.runs);
                        row.tEstSeconds =
                            static_cast<double>(poseTimeUs) * 1.0e-6 + timeShiftSeconds;
                        // estimateBallPoseFromCircle works in camera_optical
                        // millimetres already; only the scale changes here.
                        row.estimate = {
                            pose.positionMm.x * 1.0e-3f,
                            pose.positionMm.y * 1.0e-3f,
                            pose.positionMm.z * 1.0e-3f
                        };

                        Vector3 gtWorld{};
                        row.gtValid = sd::LookupGroundTruthWorld(
                            groundTruth, static_cast<float>(row.tEstSeconds), gtWorld);
                        if (row.gtValid) {
                            row.groundTruth = sd::WorldToCameraOptical(gtWorld);
                            row.gtVisible = sd::LookupGroundTruthVisible(
                                groundTruth, static_cast<float>(row.tEstSeconds));
                            ++report.circle.gtMatched;
                        }

                        row.sizePx = pose.RadiusPx;
                        row.sizeKind = "circle_radius";
                        row.windowEvents = static_cast<int>(camera.SampleCount());
                        row.windowPoints = 1;
                        row.runtimeMs = runtimeMs;

                        circleWriter->Write(row);
                        ++report.circle.estimates;
                        if (std::isnan(report.circle.firstEstimateSeconds)) {
                            report.circle.firstEstimateSeconds = row.tEstSeconds;
                        }

                        lastEmittedCircleTimeUs = std::max(lastEmittedCircleTimeUs, poseTimeUs);
                    }
                }
            }
        }
    }

    return report;
}

void WriteMethodStatsJson(std::ostream &out, const char *name, const MethodStats &stats) {
    out << "      \"" << name << "\": {\n"
        << "        \"estimates\": " << stats.estimates << ",\n"
        << "        \"ground_truth_matched\": " << stats.gtMatched << ",\n"
        << "        \"runs\": " << stats.runs << ",\n"
        << "        \"valid_runs\": " << stats.validRuns << ",\n"
        << "        \"total_runtime_ms\": " << stats.totalRuntimeMs << ",\n"
        << "        \"first_estimate_s\": ";
    if (std::isnan(stats.firstEstimateSeconds)) {
        out << "null";
    }
    else {
        out << stats.firstEstimateSeconds;
    }
    out << "\n      }";
}

void WriteManifest(
    const fs::path &path,
    const Options &options,
    const std::vector<SequenceReport> &reports) {

    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path.string());
    }

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"tool\": \"method_comparison_benchmark\",\n";
    out << "  \"emit_mode\": \"" << options.emitMode << "\",\n";
    out << "  \"time_base\": \"" << options.timeBase << "\",\n";
    out << "  \"time_offset_s\": " << options.timeOffsetSeconds << ",\n";
    out << "  \"tick_ms\": " << options.tickMs << ",\n";
    out << "  \"methods\": [";
    if (options.runTrace) {
        out << "\"trace\"";
    }
    if (options.runTrace && options.runCircle) {
        out << ", ";
    }
    if (options.runCircle) {
        out << "\"circle\"";
    }
    out << "],\n";

    out << "  \"trace_settings\": {\n"
        << "    \"memory_ms\": " << options.trace.memorySeconds * 1.0e3 << ",\n"
        << "    \"analysis_period_ms\": " << options.traceAnalysisPeriodMs << ",\n"
        << "    \"polarity_mode\": " << options.trace.polarityMode << ",\n"
        << "    \"line_bin_width_px\": " << options.trace.lineBinWidthPx << ",\n"
        << "    \"line_window_px\": " << options.trace.lineWindowPx << ",\n"
        << "    \"line_order\": " << options.trace.lineOrder << ",\n"
        << "    \"pca_period_ms\": " << options.trace.pcaPeriodMs << ",\n"
        << "    \"width_step_px\": " << options.trace.widthStepPx << ",\n"
        << "    \"support_divisor\": " << options.trace.supportEdge.supportDivisor << ",\n"
        << "    \"support_min\": " << options.trace.supportEdge.minLocalSupport << ",\n"
        << "    \"support_max\": " << options.trace.supportEdge.maxLocalSupport << ",\n"
        << "    \"support_radius_px\": " << options.trace.supportEdge.supportRadiusPx << ",\n"
        << "    \"border_ratio\": " << options.trace.supportEdge.borderRatio << ",\n"
        << "    \"edge_refine\": " << (options.trace.edgeRefine ? "true" : "false") << ",\n"
        << "    \"width_smoothing\": " << (options.trace.widthSmoothing ? "true" : "false") << ",\n"
        << "    \"use_raw_input\": " << (options.trace.useRawInput ? "true" : "false") << ",\n"
        << "    \"roi\": [" << options.trace.roi.x << ", " << options.trace.roi.y << ", "
        << options.trace.roi.w << ", " << options.trace.roi.h << "],\n"
        << "    \"max_analysis_points\": " << options.trace.maxAnalysisPoints << "\n"
        << "  },\n";

    out << "  \"circle_settings\": {\n"
        << "    \"window_ms\": " << options.circleWindowMs << ",\n"
        << "    \"period_ms\": " << options.circlePeriodMs << ",\n"
        << "    \"max_events\": " << options.circleMaxEvents << ",\n"
        << "    \"bandwidth\": " << options.circleBandwidth << ",\n"
        << "    \"min_nb\": " << options.circleMinNb << ",\n"
        << "    \"alpha\": " << options.circle.alpha << ",\n"
        << "    \"coef\": " << options.circle.coef << ",\n"
        << "    \"filter_size\": " << options.circle.filterSize << ",\n"
        << "    \"max_residual\": " << options.circle.maxResidual << ",\n"
        << "    \"rayon_cote\": " << options.circle.rayonCote << ",\n"
        << "    \"sym_coef\": " << options.circle.symCoef << ",\n"
        << "    \"sym_coef2\": " << options.circle.symCoef2 << ",\n"
        << "    \"positive_only\": " << (options.circle.positiveOnly ? "true" : "false") << ",\n"
        << "    \"radius_gate\": " << (options.circle.radiusGateEnabled ? "true" : "false") << ",\n"
        << "    \"weighted_regression\": "
        << (options.circle.weightedRegressionEnabled ? "true" : "false") << ",\n"
        << "    \"slice_mode\": " << static_cast<int>(options.circle.sliceMode) << ",\n"
        << "    \"temporal_slice_count\": " << options.circle.temporalSliceCount << ",\n"
        << "    \"events_per_slice\": " << options.circle.eventsPerSlice << "\n"
        << "  },\n";

    out << "  \"sequences\": [\n";
    for (std::size_t i = 0; i < reports.size(); ++i) {
        const SequenceReport &report = reports[i];
        out << "    {\n"
            << "      \"name\": \"" << EscapeJson(report.name) << "\",\n"
            << "      \"events_path\": \"" << EscapeJson(report.eventsPath) << "\",\n"
            << "      \"intrinsics_path\": \"" << EscapeJson(report.intrinsicsPath) << "\",\n"
            << "      \"ground_truth_path\": \"" << EscapeJson(report.groundTruthPath) << "\",\n"
            << "      \"metadata_path\": \"" << EscapeJson(report.metadataPath) << "\",\n"
            << "      \"fx\": " << report.fx << ",\n"
            << "      \"fy\": " << report.fy << ",\n"
            << "      \"cx\": " << report.cx << ",\n"
            << "      \"cy\": " << report.cy << ",\n"
            << "      \"width\": " << report.width << ",\n"
            << "      \"height\": " << report.height << ",\n"
            << "      \"distortion_model\": \"" << report.distortionSource << "\",\n"
            << "      \"ball_radius_mm\": " << report.ballRadiusMm << ",\n"
            << "      \"ball_radius_source\": \"" << EscapeJson(report.ballRadiusSource) << "\",\n"
            << "      \"event_count\": " << report.eventCount << ",\n"
            << "      \"t_start_us\": " << report.tStartUs << ",\n"
            << "      \"t_end_us\": " << report.tEndUs << ",\n"
            << "      \"ground_truth_rows\": " << report.groundTruthRows << ",\n"
            << "      \"ground_truth_first_s\": " << report.groundTruthFirstSeconds << ",\n"
            << "      \"ground_truth_last_s\": " << report.groundTruthLastSeconds << ",\n"
            << "      \"error\": \"" << EscapeJson(report.error) << "\",\n";
        WriteMethodStatsJson(out, "trace", report.trace);
        out << ",\n";
        WriteMethodStatsJson(out, "circle", report.circle);
        out << "\n    }";
        if (i + 1 < reports.size()) {
            out << ',';
        }
        out << '\n';
    }
    out << "  ]\n";
    out << "}\n";
}

}  // namespace

int main(int argc, char **argv) {
    try {
        const Options options = ParseArgs(argc, argv);

        std::vector<SequenceRef> sequences;
        for (const std::string &explicitSequence : options.sequences) {
            std::string reason;
            if (auto ref = MakeSequenceRef(fs::path(explicitSequence), options.eventsName, reason)) {
                sequences.push_back(*ref);
            }
            else {
                std::cerr << "sequence rejected: " << explicitSequence << " (" << reason << ")\n";
            }
        }
        for (const std::string &root : options.datasetRoots) {
            DiscoverSequences(fs::path(root), options.maxDiscoveryDepth, options.eventsName, sequences);
        }

        if (sequences.empty()) {
            std::cerr << "No usable sequence found. A sequence directory must contain "
                         "labels/ground_truth.csv, camera/intrinsics.json and an event file.\n";
            return 1;
        }

        const fs::path outputDir(options.outputDir);
        std::error_code ec;
        fs::create_directories(outputDir, ec);
        if (ec) {
            throw std::runtime_error("cannot create output directory " + outputDir.string());
        }

        std::cout << "Output directory: " << outputDir.string() << "\n";
        std::cout << "Sequences: " << sequences.size() << "\n\n";

        std::vector<SequenceReport> reports;
        reports.reserve(sequences.size());

        for (const SequenceRef &ref : sequences) {
            std::cout << "== " << ref.name << " ==\n"
                      << "   events:     " << ref.eventsPath.string() << "\n"
                      << "   intrinsics: " << ref.intrinsicsPath.string() << "\n";

            // One unreadable sequence must not abort a whole dataset run.
            SequenceReport report;
            try {
                report = RunSequence(ref, options, outputDir);
            }
            catch (const std::exception &e) {
                report.name = ref.name;
                report.eventsPath = ref.eventsPath.string();
                report.intrinsicsPath = ref.intrinsicsPath.string();
                report.groundTruthPath = ref.groundTruthPath.string();
                report.metadataPath = ref.metadataPath.string();
                report.error = e.what();
            }

            if (!report.error.empty()) {
                std::cerr << "   SKIPPED: " << report.error << "\n\n";
                reports.push_back(std::move(report));
                continue;
            }

            std::cout << std::fixed << std::setprecision(2)
                      << "   fx=" << report.fx << " fy=" << report.fy
                      << " cx=" << report.cx << " cy=" << report.cy
                      << " (" << report.width << "x" << report.height << ")\n"
                      << "   ball radius: " << report.ballRadiusMm << " mm from "
                      << (report.ballRadiusSource.empty() ? "n/a" : report.ballRadiusSource) << "\n"
                      << "   events: " << report.eventCount
                      << "  t_start=" << static_cast<double>(report.tStartUs) * 1.0e-6 << " s"
                      << "  t_end=" << static_cast<double>(report.tEndUs) * 1.0e-6 << " s\n"
                      << "   ground truth: " << report.groundTruthRows << " rows, "
                      << report.groundTruthFirstSeconds << " s .. "
                      << report.groundTruthLastSeconds << " s\n";

            if (options.runTrace) {
                std::cout << "   trace:  " << report.trace.estimates << " estimates, "
                          << report.trace.gtMatched << " matched to ground truth ("
                          << report.trace.validRuns << "/" << report.trace.runs << " valid runs)\n";
            }
            if (options.runCircle) {
                std::cout << "   circle: " << report.circle.estimates << " estimates, "
                          << report.circle.gtMatched << " matched to ground truth ("
                          << report.circle.validRuns << "/" << report.circle.runs << " valid runs)\n";
            }

            // A near-zero match rate almost always means the event stream and
            // the ground truth do not share a time origin.
            const auto warnTimeBase = [&](const char *name, const MethodStats &stats) {
                if (stats.estimates > 0 && stats.gtMatched * 4 < stats.estimates) {
                    std::cerr << "   WARNING (" << name << "): only " << stats.gtMatched << "/"
                              << stats.estimates << " estimates matched a ground-truth instant. "
                              << "Event stream spans "
                              << static_cast<double>(report.tStartUs) * 1.0e-6 << ".."
                              << static_cast<double>(report.tEndUs) * 1.0e-6
                              << " s while ground truth spans " << report.groundTruthFirstSeconds
                              << ".." << report.groundTruthLastSeconds
                              << " s; try --time-base zero.\n";
                }
            };
            warnTimeBase("trace", report.trace);
            warnTimeBase("circle", report.circle);

            std::cout << "\n";
            reports.push_back(std::move(report));
        }

        const fs::path manifestPath = outputDir / "run_manifest.json";
        WriteManifest(manifestPath, options, reports);
        std::cout << "Manifest: " << manifestPath.string() << "\n";
        std::cout << "Next: python3 scripts/compare_trace_vs_circle.py " << outputDir.string() << "\n";

        return 0;
    }
    catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
