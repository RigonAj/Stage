#include "OfflineBenchmark.hpp"
#include "SequenceData.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace {

struct Args {
    std::string eventsH5;
    std::string groundTruth;
    std::string camera;
    std::string metadata;
    std::string outputTrace;
    std::string outputCircle;
    std::string runtimeOutput;
    std::string config;
    std::string mode = "both";
};

void PrintUsage(const char *argv0) {
    std::cerr
        << "Usage: " << argv0 << " --events-h5 FILE --ground-truth FILE --camera FILE\n"
        << "       --metadata FILE --runtime-output FILE [--config FILE]\n"
        << "       [--mode trace|circle|both] [--output-trace FILE] [--output-circle FILE]\n"
        << "\n"
        << "  --output FILE   alias for --output-trace (backward compatibility)\n"
        << "  --headless      accepted and ignored, the benchmark never opens a window\n";
}

Args ParseArgs(int argc, char **argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto requireValue = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + name);
            }
            return argv[++i];
        };

        if (key == "--events-h5") args.eventsH5 = requireValue(key);
        else if (key == "--ground-truth") args.groundTruth = requireValue(key);
        else if (key == "--camera") args.camera = requireValue(key);
        else if (key == "--metadata") args.metadata = requireValue(key);
        else if (key == "--output-trace" || key == "--output") args.outputTrace = requireValue(key);
        else if (key == "--output-circle") args.outputCircle = requireValue(key);
        else if (key == "--runtime-output") args.runtimeOutput = requireValue(key);
        else if (key == "--config") args.config = requireValue(key);
        else if (key == "--mode") args.mode = requireValue(key);
        else if (key == "--headless") continue;
        else if (key == "--help" || key == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        else {
            throw std::runtime_error("Unknown argument: " + key);
        }
    }
    return args;
}

void RequireReadable(const std::string &path, const std::string &label) {
    if (path.empty()) {
        throw std::runtime_error("Missing required argument: " + label);
    }
    if (!fs::exists(path)) {
        throw std::runtime_error(label + " does not exist: " + path);
    }
}

void EnsureParentDirectory(const std::string &path) {
    const fs::path parent = fs::path(path).parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent);
    }
}

}  // namespace

int main(int argc, char **argv) {
    try {
        const Args args = ParseArgs(argc, argv);

        const bool runTrace = args.mode == "trace" || args.mode == "both";
        const bool runCircle = args.mode == "circle" || args.mode == "both";
        if (!runTrace && !runCircle) {
            throw std::runtime_error("--mode must be one of: trace, circle, both");
        }

        RequireReadable(args.eventsH5, "--events-h5");
        RequireReadable(args.groundTruth, "--ground-truth");
        RequireReadable(args.camera, "--camera");
        RequireReadable(args.metadata, "--metadata");
        if (args.runtimeOutput.empty()) {
            throw std::runtime_error("Missing required argument: --runtime-output");
        }
        if (runTrace && args.outputTrace.empty()) {
            throw std::runtime_error("Missing required argument: --output-trace");
        }
        if (runCircle && args.outputCircle.empty()) {
            throw std::runtime_error("Missing required argument: --output-circle");
        }

        // The synthetic sequences carry their own camera model. Refusing to run
        // without it is deliberate: silently falling back to the real DVXplorer
        // calibration would corrupt every depth estimate.
        const CalibrationData calibration = LoadCalibrationFromIntrinsicsJson(args.camera);
        if (!calibration.ready) {
            throw std::runtime_error("Invalid or unreadable intrinsics JSON: " + args.camera);
        }

        BenchmarkSettings settings;
        if (!args.config.empty()) {
            RequireReadable(args.config, "--config");
            settings = LoadBenchmarkSettingsYaml(args.config, settings);
        }

        double ballRadiusMeters = settings.ballRadiusMeters;
        if (LoadBallRadiusFromMetadataJson(args.metadata, ballRadiusMeters)) {
            settings.ballRadiusMeters = ballRadiusMeters;
        }

        BenchmarkRuntime runtime;
        runtime.eventsPath = args.eventsH5;
        runtime.cameraPath = args.camera;
        runtime.groundTruthPath = args.groundTruth;
        runtime.metadataPath = args.metadata;
        runtime.sequenceName = JsonString(ReadTextFile(args.metadata), "sequence_name");
        runtime.fx = calibration.fx();
        runtime.fy = calibration.fy();
        runtime.cx = calibration.cx();
        runtime.cy = calibration.cy();
        runtime.imageWidth = calibration.imageSize.width;
        runtime.imageHeight = calibration.imageSize.height;
        runtime.distortionCoefficientCount = calibration.distortionCoefficients.cols;
        runtime.settings = settings;

        const std::vector<SequenceGroundTruthSample> groundTruth =
            LoadSequenceGroundTruth(args.groundTruth);
        runtime.groundTruthRows = groundTruth.size();
        for (const SequenceGroundTruthSample &sample : groundTruth) {
            runtime.groundTruthVisibleRows += sample.visible ? 1 : 0;
        }
        if (!groundTruth.empty()) {
            runtime.groundTruthFirstTimestampS = groundTruth.front().timeSeconds;
            runtime.groundTruthLastTimestampS = groundTruth.back().timeSeconds;
        }

        EventReader reader(args.eventsH5);
        runtime.inputEventCount = static_cast<int64_t>(reader.count());
        runtime.firstTimestampUs = reader.startTimestampUs();
        runtime.lastTimestampUs = reader.endTimestampUs();

        // Constructed once: without a DVXplorer attached the constructor just
        // reports "no camera available" and leaves the capture handle null,
        // which is all the offline path needs.
        DvCamera camera;
        camera.calibration = calibration;

        if (runTrace) {
            EnsureParentDirectory(args.outputTrace);
            const std::vector<BenchmarkDetection> detections =
                RunTraceMethod(reader, camera, calibration, settings, runtime.trace);
            WriteDetectionsCsv(args.outputTrace, detections, BenchmarkMethod::Trace);
        }

        if (runCircle) {
            EnsureParentDirectory(args.outputCircle);
            const std::vector<BenchmarkDetection> detections =
                RunCircleMethod(reader, camera, calibration, settings, runtime.circle);
            WriteDetectionsCsv(args.outputCircle, detections, BenchmarkMethod::Circle);
        }

        EnsureParentDirectory(args.runtimeOutput);
        WriteRuntimeJson(args.runtimeOutput, runtime);

        std::cerr << "[ball_tracker_h5_benchmark] " << runtime.sequenceName
                  << " fx=" << runtime.fx << " cx=" << runtime.cx
                  << " radius=" << settings.ballRadiusMeters << "m";
        if (runTrace) {
            std::cerr << " | trace rows=" << runtime.trace.outputRows
                      << " detected=" << runtime.trace.detectedRows;
        }
        if (runCircle) {
            std::cerr << " | circle rows=" << runtime.circle.outputRows
                      << " detected=" << runtime.circle.detectedRows;
        }
        std::cerr << "\n";
        return 0;
    }
    catch (const std::exception &e) {
        std::cerr << "[ball_tracker_h5_benchmark] ERROR: " << e.what() << "\n";
        PrintUsage(argv[0]);
        return 1;
    }
}
