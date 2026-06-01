#include "TraceBenchmark.hpp"

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
    std::string output;
    std::string runtimeOutput;
    std::string config;
    std::string mode = "trace";
    bool headless = false;
};

void PrintUsage(const char *argv0) {
    std::cerr
        << "Usage: " << argv0 << " --events-h5 FILE --ground-truth FILE --camera FILE --metadata FILE "
        << "--output FILE --runtime-output FILE [--config FILE] [--mode trace] [--headless]\n";
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
        if (key == "--events-h5") {
            args.eventsH5 = requireValue(key);
        }
        else if (key == "--ground-truth") {
            args.groundTruth = requireValue(key);
        }
        else if (key == "--camera") {
            args.camera = requireValue(key);
        }
        else if (key == "--metadata") {
            args.metadata = requireValue(key);
        }
        else if (key == "--output") {
            args.output = requireValue(key);
        }
        else if (key == "--runtime-output") {
            args.runtimeOutput = requireValue(key);
        }
        else if (key == "--config") {
            args.config = requireValue(key);
        }
        else if (key == "--mode") {
            args.mode = requireValue(key);
        }
        else if (key == "--headless") {
            args.headless = true;
        }
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

}  // namespace

int main(int argc, char **argv) {
    try {
        const Args args = ParseArgs(argc, argv);
        if (args.mode != "trace") {
            throw std::runtime_error("Only --mode trace is supported by this benchmark prototype.");
        }
        RequireReadable(args.eventsH5, "--events-h5");
        RequireReadable(args.groundTruth, "--ground-truth");
        RequireReadable(args.camera, "--camera");
        RequireReadable(args.metadata, "--metadata");
        if (args.output.empty()) {
            throw std::runtime_error("Missing required argument: --output");
        }
        if (args.runtimeOutput.empty()) {
            throw std::runtime_error("Missing required argument: --runtime-output");
        }

        TraceBenchmarkSettings settings;
        if (!args.config.empty()) {
            RequireReadable(args.config, "--config");
            settings = LoadTraceSettingsYaml(args.config, settings);
        }

        TraceBenchmarkRuntime runtime;
        runtime.eventsPath = args.eventsH5;
        runtime.cameraPath = args.camera;
        runtime.groundTruthPath = args.groundTruth;
        runtime.metadataPath = args.metadata;
        runtime.settings = settings;

        TraceBenchmarkIntrinsics intrinsics = LoadTraceIntrinsicsJson(args.camera);
        TraceBenchmarkMetadata metadata = LoadTraceMetadataJson(args.metadata);
        TraceGroundTruthSummary groundTruth = LoadTraceGroundTruthCsv(args.groundTruth);
        runtime.metadata = metadata;
        runtime.groundTruth = groundTruth;

        EventReader reader(args.eventsH5);
        runtime.inputEventCount = static_cast<int64_t>(reader.count());
        runtime.firstTimestampUs = reader.startTimestampUs();
        runtime.lastTimestampUs = reader.endTimestampUs();

        fs::create_directories(fs::path(args.output).parent_path());
        fs::create_directories(fs::path(args.runtimeOutput).parent_path());

        std::vector<TraceDetection> detections = RunTraceBenchmark(
            reader,
            intrinsics,
            metadata,
            settings,
            runtime);
        WriteTraceDetectionsCsv(args.output, detections);
        WriteTraceRuntimeJson(args.runtimeOutput, runtime);

        std::cerr << "[ball_tracker_h5_benchmark] rows=" << runtime.outputRows
                  << " detected=" << runtime.detectedRows
                  << " output=" << args.output << "\n";
        return 0;
    }
    catch (const std::exception &e) {
        std::cerr << "[ball_tracker_h5_benchmark] ERROR: " << e.what() << "\n";
        PrintUsage(argv[0]);
        return 1;
    }
}
