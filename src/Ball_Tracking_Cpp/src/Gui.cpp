#define RAYGUI_IMPLEMENTATION
#include "Gui.h"
#undef RAYGUI_IMPLEMENTATION

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <execution>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <sstream>
#include <string_view>
#include <utility>
#include "RegressionAccumulator.hpp"
#include "TraceAnalysis.hpp"
#include <raymath.h>

namespace {
namespace fs = std::filesystem;

constexpr std::string_view kRecordingDirectory = "recordings";
constexpr std::string_view kSequenceDirectory = "sequences";
constexpr std::string_view kSequenceEventsPath = "events_v2e/events.h5";

bool EndsWithCaseInsensitive(std::string_view value, std::string_view suffix) {
    return value.size() >= suffix.size()
        && std::equal(suffix.rbegin(), suffix.rend(), value.rbegin(), [](char a, char b) {
               return std::tolower(static_cast<unsigned char>(a))
                   == std::tolower(static_cast<unsigned char>(b));
           });
}

void EnsureRecordingDirectory() {
    std::error_code ec;
    fs::create_directories(kRecordingDirectory, ec);
}

void EnsureSequenceDirectory() {
    std::error_code ec;
    fs::create_directories(kSequenceDirectory, ec);
}

void AddFilesFromDirectory(const fs::path &directory, std::vector<std::string> &files) {
    std::error_code ec;
    for (const fs::directory_entry &entry : fs::directory_iterator(directory, ec)) {
        std::error_code entryEc;
        if (ec || !entry.is_regular_file(entryEc)) continue;

        const std::string filename = entry.path().filename().string();
        if (EndsWithCaseInsensitive(filename, ".h5")
            || EndsWithCaseInsensitive(filename, ".hdf5")
            || EndsWithCaseInsensitive(filename, ".bin")) {
            files.push_back(filename);
        }
    }
}

bool IsEventFileName(std::string_view filename) {
    return EndsWithCaseInsensitive(filename, ".h5")
        || EndsWithCaseInsensitive(filename, ".hdf5")
        || EndsWithCaseInsensitive(filename, ".bin");
}

std::vector<std::string> BuildSequenceEventList() {
    EnsureSequenceDirectory();
    std::vector<fs::path> candidates;
    std::error_code ec;

    for (const fs::directory_entry &sequenceEntry : fs::directory_iterator(kSequenceDirectory, ec)) {
        std::error_code sequenceEc;
        if (ec || !sequenceEntry.is_directory(sequenceEc)) continue;

        std::error_code walkEc;
        for (const fs::directory_entry &entry : fs::recursive_directory_iterator(sequenceEntry.path(), walkEc)) {
            std::error_code entryEc;
            if (walkEc || !entry.is_regular_file(entryEc)) continue;

            const std::string filename = entry.path().filename().string();
            if (IsEventFileName(filename)) {
                candidates.push_back(entry.path());
            }
        }
    }

    std::sort(candidates.begin(), candidates.end());
    std::vector<std::string> files;
    files.reserve(candidates.size());

    for (const fs::path &eventsPath : candidates) {
        std::error_code relativeEc;
        fs::path relativePath = fs::relative(eventsPath, kSequenceDirectory, relativeEc);
        files.push_back(relativeEc ? eventsPath.string() : relativePath.string());
    }

    return files;
}

std::vector<std::string> BuildReaderFileList(bool useSequenceDirectory) {
    if (useSequenceDirectory) {
        return BuildSequenceEventList();
    }

    EnsureRecordingDirectory();

    std::vector<std::string> files;
    AddFilesFromDirectory(fs::path(kRecordingDirectory), files);
    std::sort(files.begin(), files.end());
    return files;
}

std::string MakeEventPath(std::string name, std::string_view fallbackBase) {
    if (name.empty()) {
        name = std::string(fallbackBase);
    }

    fs::path path(name);
    if (!EndsWithCaseInsensitive(name, ".h5")
        && !EndsWithCaseInsensitive(name, ".hdf5")
        && !EndsWithCaseInsensitive(name, ".bin")) {
        path += ".h5";
    }

    if (path.has_parent_path()) {
        return path.string();
    }

    EnsureRecordingDirectory();
    return (fs::path(kRecordingDirectory) / path).string();
}

std::string MakeReaderEventPath(std::string name, bool useSequenceDirectory) {
    if (!useSequenceDirectory) {
        return MakeEventPath(std::move(name), "events3");
    }

    EnsureSequenceDirectory();

    if (name.empty()) {
        const std::vector<std::string> files = BuildSequenceEventList();
        if (!files.empty()) {
            name = files.front();
        }
    }

    if (name.empty()) {
        return MakeEventPath("events3.h5", "events3");
    }

    fs::path path(name);
    if (path.has_parent_path()) {
        std::error_code ec;
        if (fs::is_directory(path, ec)) {
            return (path / std::string(kSequenceEventsPath)).string();
        }
        if (path.is_absolute()) {
            return path.string();
        }
        return (fs::path(kSequenceDirectory) / path).string();
    }

    if (EndsWithCaseInsensitive(name, ".h5")
        || EndsWithCaseInsensitive(name, ".hdf5")
        || EndsWithCaseInsensitive(name, ".bin")) {
        return (fs::path(kSequenceDirectory) / path).string();
    }

    return (fs::path(kSequenceDirectory) / path / std::string(kSequenceEventsPath)).string();
}

std::string DefaultReaderPath() {
    const std::string sequencePath = MakeReaderEventPath({}, true);
    if (sequencePath != MakeEventPath("events3.h5", "events3")) {
        return sequencePath;
    }

    return MakeEventPath("events3.h5", "events3");
}

std::vector<std::string> SplitCsvLine(const std::string &line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

int CsvColumnIndex(const std::vector<std::string> &header, std::string_view name) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool ParseDouble(std::string_view text, double &value) {
    std::string owned(text);
    char *end = nullptr;
    value = std::strtod(owned.c_str(), &end);
    return end != owned.c_str() && std::isfinite(value);
}

bool JsonNumber(std::string_view json, std::string_view key, double &value) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const std::size_t keyPos = json.find(needle);
    if (keyPos == std::string_view::npos) {
        return false;
    }

    const std::size_t colonPos = json.find(':', keyPos + needle.size());
    if (colonPos == std::string_view::npos) {
        return false;
    }

    std::size_t valuePos = colonPos + 1;
    while (valuePos < json.size() && std::isspace(static_cast<unsigned char>(json[valuePos]))) {
        ++valuePos;
    }

    return ParseDouble(json.substr(valuePos), value);
}

std::string JsonString(std::string_view json, std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const std::size_t keyPos = json.find(needle);
    if (keyPos == std::string_view::npos) {
        return {};
    }

    const std::size_t colonPos = json.find(':', keyPos + needle.size());
    if (colonPos == std::string_view::npos) {
        return {};
    }

    std::size_t quoteStart = json.find('"', colonPos + 1);
    if (quoteStart == std::string_view::npos) {
        return {};
    }
    ++quoteStart;

    const std::size_t quoteEnd = json.find('"', quoteStart);
    if (quoteEnd == std::string_view::npos || quoteEnd <= quoteStart) {
        return {};
    }

    return std::string(json.substr(quoteStart, quoteEnd - quoteStart));
}

std::vector<double> JsonNumberArray(std::string_view json, std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const std::size_t keyPos = json.find(needle);
    if (keyPos == std::string_view::npos) {
        return {};
    }

    const std::size_t bracketStart = json.find('[', keyPos + needle.size());
    const std::size_t bracketEnd = json.find(']', bracketStart);
    if (bracketStart == std::string_view::npos
        || bracketEnd == std::string_view::npos
        || bracketEnd <= bracketStart) {
        return {};
    }

    std::vector<double> values;
    std::stringstream stream(std::string(json.substr(bracketStart + 1, bracketEnd - bracketStart - 1)));
    std::string field;
    while (std::getline(stream, field, ',')) {
        double value = 0.0;
        if (ParseDouble(field, value)) {
            values.push_back(value);
        }
    }
    return values;
}

std::string ReadTextFile(const fs::path &path) {
    std::ifstream file(path);
    if (!file) {
        return {};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

fs::path FindSidecarPathForEventPath(const std::string &eventPath, const fs::path &relativeSidecar) {
    if (eventPath.empty()) {
        return {};
    }

    std::error_code ec;
    fs::path directory = fs::absolute(fs::path(eventPath), ec).parent_path();
    if (ec) {
        directory = fs::path(eventPath).parent_path();
    }

    while (!directory.empty()) {
        const fs::path candidate = directory / relativeSidecar;
        std::error_code fileEc;
        if (fs::is_regular_file(candidate, fileEc)) {
            return candidate;
        }

        const fs::path parent = directory.parent_path();
        if (parent == directory) {
            break;
        }
        directory = parent;
    }

    return {};
}

fs::path IntrinsicsPathForEventPath(const std::string &eventPath) {
    return FindSidecarPathForEventPath(eventPath, fs::path("camera") / "intrinsics.json");
}

fs::path GroundTruthPathForEventPath(const std::string &eventPath) {
    return FindSidecarPathForEventPath(eventPath, fs::path("labels") / "ground_truth.csv");
}

CalibrationData LoadCalibrationFromIntrinsicsJson(const fs::path &intrinsicsPath) {
    CalibrationData calibration;

    const std::string json = ReadTextFile(intrinsicsPath);
    if (json.empty()) {
        return calibration;
    }

    double width = 0.0;
    double height = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    if (!JsonNumber(json, "width", width)
        || !JsonNumber(json, "height", height)
        || !JsonNumber(json, "fx", fx)
        || !JsonNumber(json, "fy", fy)
        || !JsonNumber(json, "cx", cx)
        || !JsonNumber(json, "cy", cy)
        || width <= 0.0
        || height <= 0.0
        || fx <= 0.0
        || fy <= 0.0) {
        return calibration;
    }

    std::vector<double> distortion = JsonNumberArray(json, "distortion_coefficients");
    if (distortion.empty()) {
        distortion.assign(5, 0.0);
    }

    calibration.ready = true;
    calibration.reprojectionError = 0.0;
    calibration.sourcePath = intrinsicsPath.string();
    calibration.cameraName = intrinsicsPath.parent_path().parent_path().filename().string();
    calibration.imageSize = cv::Size(
        static_cast<int>(std::lround(width)),
        static_cast<int>(std::lround(height)));
    calibration.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    calibration.distortionCoefficients = cv::Mat(1, static_cast<int>(distortion.size()), CV_64F);
    for (std::size_t i = 0; i < distortion.size(); ++i) {
        calibration.distortionCoefficients.at<double>(0, static_cast<int>(i)) = distortion[i];
    }

    std::string distortionModel = JsonString(json, "distortion_model");
    std::transform(distortionModel.begin(), distortionModel.end(), distortionModel.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    calibration.useFisheyeModel =
        distortionModel == "fisheye"
        || distortionModel == "equidistant";

    return calibration;
}

float FittedCameraPlaneSpeedXZ(const LineFit3D &fitX, const QuadFit3D &fitZ, float t) {
    const float vx = fitX.a;
    const float vz = 2.0f * fitZ.a * t + fitZ.b;
    return std::sqrt(vx * vx + vz * vz);
}

Color SpeedToHeatColor(float speed, float maxSpeed) {
    if (!std::isfinite(speed) || maxSpeed <= 1.0e-6f) {
        return ORANGE;
    }

    const float normalized = std::clamp(speed / maxSpeed, 0.0f, 1.0f);
    const float hue = (1.0f - normalized) * 240.0f;
    return ColorFromHSV(hue, 0.85f, 0.95f);
}

float EstimateUpperParallelOffset(const std::vector<Vector3> &points, const LineFit3D &fit) {
    std::vector<float> positiveResiduals;
    positiveResiduals.reserve(points.size());

    for (const Vector3 &p : points) {
        const float residual = p.y - (fit.a * p.x + fit.b);
        if (std::isfinite(residual) && residual > 0.0f) {
            positiveResiduals.emplace_back(residual);
        }
    }

    if (positiveResiduals.empty()) {
        return 0.0f;
    }

    constexpr float kUpperResidualQuantile = 0.90f;
    const std::size_t index = std::min(
        positiveResiduals.size() - 1,
        static_cast<std::size_t>(std::floor(kUpperResidualQuantile * static_cast<float>(positiveResiduals.size() - 1)))
    );

    std::nth_element(
        positiveResiduals.begin(),
        positiveResiduals.begin() + static_cast<std::ptrdiff_t>(index),
        positiveResiduals.end()
    );

    return positiveResiduals[index];
}

TraceSupportEdgeSettings MakeTraceSupportEdgeSettings(const Ui &ui) {
    TraceSupportEdgeSettings settings;
    settings.supportDivisor = ui.TraceSupportDivisor();
    settings.minLocalSupport = static_cast<std::size_t>(ui.TraceSupportMinCount());
    settings.maxLocalSupport = static_cast<std::size_t>(ui.TraceSupportMaxCount());
    settings.supportRadiusPx = ui.TraceSupportRadiusPx();
    settings.borderRatio = ui.TraceBorderRatio();
    return settings;
}

}

Gui::Gui(const dv::EventStore &view, Ui &uii, int screenWidth, int screenHeight)
    : screenWidth(screenWidth), screenHeight(screenHeight), View(view), ui(uii)
     {
    EnsureRecordingDirectory();
    path_reader_ = DefaultReaderPath();
    std::snprintf(ui.read_file, sizeof(ui.read_file), "%s", path_reader_.c_str());
    LoadReaderCalibrationForReader(path_reader_);
    LoadGroundTruthForReader(path_reader_);
    ui.SetRecordingFiles(BuildReaderFileList(ui.UseSequenceDirectory()));

    InitWindow(screenWidth, screenHeight, "Event Viewer");
    SetTargetFPS((int)RENDER_FPS);

    GuiSetStyle(SLIDER, BASE_COLOR_NORMAL, ColorToInt(DARKGRAY));
    GuiSetStyle(SLIDER, BASE_COLOR_FOCUSED, ColorToInt(GREEN));
    GuiSetStyle(SLIDER, BASE_COLOR_PRESSED, ColorToInt(RED));
    GuiSetStyle(SLIDER, BORDER_COLOR_NORMAL, ColorToInt(BLACK));
    GuiSetStyle(SLIDER, TEXT_COLOR_NORMAL, ColorToInt(YELLOW));

    clusters.reserve(10);
    pixelBuffer.resize(1280 * 480, BLANK);
    Image img = GenImageColor(1280, 480, BLANK);
    cpuTexture = LoadTextureFromImage(img);
    UnloadImage(img);

    sceneCamera.position = {2.2f, 1.6f, 2.8f};
    sceneCamera.target = {0.0f, 0.6f, 1.2f};
    sceneCamera.up = {0.0f, 1.0f, 0.0f};
    sceneCamera.fovy = 45.0f;
    sceneCamera.projection = CAMERA_PERSPECTIVE;


}

Gui::~Gui() {
    if (cpuTexture.id > 0) {
        UnloadTexture(cpuTexture);
    }
    if (IsWindowReady()) {
        CloseWindow();
    }
}

Vector3 Gui::WorldToScene(const Vector3 &world) const {
    return {world.x, world.z, world.y};
}

void Gui::LoadReaderCalibrationForReader(const std::string &eventPath) {
    readerCalibrationOverride = {};
    readerCalibrationOverrideReady = false;

    const fs::path intrinsicsPath = IntrinsicsPathForEventPath(eventPath);
    if (intrinsicsPath.empty()) {
        return;
    }

    CalibrationData calibration = LoadCalibrationFromIntrinsicsJson(intrinsicsPath);
    if (!calibration.ready) {
        std::cerr << "Sequence calibration ignored: invalid intrinsics at "
                  << intrinsicsPath.string() << "\n";
        return;
    }

    readerCalibrationOverride = std::move(calibration);
    readerCalibrationOverrideReady = true;
    std::cerr << "Sequence calibration loaded: fx=" << readerCalibrationOverride.fx()
              << " fy=" << readerCalibrationOverride.fy()
              << " cx=" << readerCalibrationOverride.cx()
              << " cy=" << readerCalibrationOverride.cy()
              << " distortion=" << readerCalibrationOverride.distortionCoefficients.cols
              << " coeffs from " << readerCalibrationOverride.sourcePath << "\n";
}

void Gui::LoadGroundTruthForReader(const std::string &eventPath) {
    groundTruthTimesSeconds.clear();
    groundTruthWorld3D.clear();
    groundTruthSourcePath.clear();
    traceGroundTruthWorld3D.clear();
    traceGroundTruthEstimateWorld3D.clear();

    const fs::path groundTruthPath = GroundTruthPathForEventPath(eventPath);
    if (groundTruthPath.empty()) {
        return;
    }

    std::ifstream file(groundTruthPath);
    if (!file) {
        return;
    }

    std::string line;
    if (!std::getline(file, line)) {
        return;
    }

    const std::vector<std::string> header = SplitCsvLine(line);
    const int timeColumn = CsvColumnIndex(header, "timestamp_s");
    const int xColumn = CsvColumnIndex(header, "ball_x_cam_m");
    const int yColumn = CsvColumnIndex(header, "ball_y_cam_m");
    const int zColumn = CsvColumnIndex(header, "ball_z_cam_m");
    if (timeColumn < 0 || xColumn < 0 || yColumn < 0 || zColumn < 0) {
        return;
    }

    const int requiredColumn = std::max({timeColumn, xColumn, yColumn, zColumn});
    std::vector<std::pair<float, Vector3>> samples;

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = SplitCsvLine(line);
        if (static_cast<int>(fields.size()) <= requiredColumn) {
            continue;
        }

        double timeSeconds = 0.0;
        double xCam = 0.0;
        double yCam = 0.0;
        double zCam = 0.0;
        if (!ParseDouble(fields[static_cast<std::size_t>(timeColumn)], timeSeconds)
            || !ParseDouble(fields[static_cast<std::size_t>(xColumn)], xCam)
            || !ParseDouble(fields[static_cast<std::size_t>(yColumn)], yCam)
            || !ParseDouble(fields[static_cast<std::size_t>(zColumn)], zCam)) {
            continue;
        }

        samples.push_back({
            static_cast<float>(timeSeconds),
            {
                static_cast<float>(xCam),
                static_cast<float>(zCam),
                static_cast<float>(-yCam)
            }
        });
    }

    std::sort(samples.begin(), samples.end(), [](const auto &a, const auto &b) {
        return a.first < b.first;
    });

    groundTruthTimesSeconds.reserve(samples.size());
    groundTruthWorld3D.reserve(samples.size());
    for (const auto &sample : samples) {
        groundTruthTimesSeconds.push_back(sample.first);
        groundTruthWorld3D.push_back(sample.second);
    }

    if (!groundTruthTimesSeconds.empty()) {
        groundTruthSourcePath = groundTruthPath.string();
        std::cerr << "Ground truth loaded: " << groundTruthTimesSeconds.size()
                  << " poses from " << groundTruthSourcePath << "\n";
    }
}

bool Gui::LookupGroundTruthWorld(float timeSeconds, Vector3 &worldPoint) const {
    if (!std::isfinite(timeSeconds)
        || groundTruthTimesSeconds.empty()
        || groundTruthTimesSeconds.size() != groundTruthWorld3D.size()) {
        return false;
    }

    constexpr float kEndpointToleranceSeconds = 0.01f;
    const auto upper = std::lower_bound(
        groundTruthTimesSeconds.begin(),
        groundTruthTimesSeconds.end(),
        timeSeconds);

    if (upper == groundTruthTimesSeconds.begin()) {
        if (std::fabs(timeSeconds - groundTruthTimesSeconds.front()) <= kEndpointToleranceSeconds) {
            worldPoint = groundTruthWorld3D.front();
            return true;
        }
        return false;
    }

    if (upper == groundTruthTimesSeconds.end()) {
        if (std::fabs(timeSeconds - groundTruthTimesSeconds.back()) <= kEndpointToleranceSeconds) {
            worldPoint = groundTruthWorld3D.back();
            return true;
        }
        return false;
    }

    const std::size_t hi = static_cast<std::size_t>(
        std::distance(groundTruthTimesSeconds.begin(), upper));
    const std::size_t lo = hi - 1;
    const float t0 = groundTruthTimesSeconds[lo];
    const float t1 = groundTruthTimesSeconds[hi];
    const float dt = t1 - t0;
    if (dt <= 1.0e-9f) {
        worldPoint = groundTruthWorld3D[lo];
        return true;
    }

    const float alpha = std::clamp((timeSeconds - t0) / dt, 0.0f, 1.0f);
    const Vector3 &p0 = groundTruthWorld3D[lo];
    const Vector3 &p1 = groundTruthWorld3D[hi];
    worldPoint = {
        p0.x + (p1.x - p0.x) * alpha,
        p0.y + (p1.y - p0.y) * alpha,
        p0.z + (p1.z - p0.z) * alpha
    };
    return true;
}

void Gui::Draw() {
    std::fill(std::execution::par_unseq, pixelBuffer.begin(), pixelBuffer.end(), BLANK);
    if (!ui.ShowTraceView()) {
        for (int y = 0; y < 480; ++y) {
            for (int x = 640; x < 1280; ++x) {
                pixelBuffer[y * 1280 + x] = BLACK;
            }
        }
    }

    DrawView();
    if (!ui.ShowTraceView()) {
        DrawClusters();
    }
    UpdateTexture(cpuTexture, pixelBuffer.data());

    UpdateTraceAnalysis();

    BeginDrawing();
    ClearBackground(RAYWHITE);

    if (ui.Show3D()) {
        Draw3DScene();
    } else if (ui.ShowTopView()) {
        DrawTopView();
    } else if (ui.ShowQualityView()) {
        DrawTrajectoryQualityPanel();
    } else if (ui.ShowTraceView()) {
        DrawTraceScene();
    } else {
        Draw2DScene();
    }

    DrawHudTexts();
    DrawPerformance();
    const auto scanNow = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(scanNow - last_reader_file_scan_).count() >= 1.0) {
        ui.SetRecordingFiles(BuildReaderFileList(ui.UseSequenceDirectory()));
        last_reader_file_scan_ = scanNow;
    }
    ui.DrawPanel();
    DrawText(TextFormat("Nombre evenment    : %ld", nb_event), 8, 84, 20, ORANGE);
    EndDrawing();
}

void Gui::Draw2DScene() {
    DrawText(TextFormat("Mouse Position: (%.0f, %.0f)", mousePosition.x, mousePosition.y), 190, 200, 20, LIGHTGRAY);
    DrawTextureEx(cpuTexture, {offset.x, offset.y}, 0.0f, scale, WHITE);
    DrawRectangleLinesEx({offset.x, offset.y, 1280 * scale, 480 * scale}, 2, RED);
    DrawOverlays();
    DrawImageTrajectory2D();
}

// Runs the full trace pipeline once per rendered frame and caches the
// results in members; every view (Trace, 3D, TOP, RMSE) reads from this
// cache so the computation never runs twice in a frame.
void Gui::UpdateTraceAnalysis() {
    const int polarityMode = ui.TracePolarityMode();
    const TraceSupportEdgeSettings supportEdge = MakeTraceSupportEdgeSettings(ui);

    TracePointSourceResult source = BuildTracePointSource(
        traceAccumulatedPoints,
        traceAccumulatedTimestamps,
        traceAccumulatedPolarities,
        traceRawPoints,
        traceRawTimestamps,
        traceRawPolarities,
        traceFloatPoints,
        traceFloatTimestamps,
        traceFloatPolarities,
        View,
        ui.TraceUseRawInput(),
        ui.TraceUseRadiusGate(),
        traceMotionWindowValid,
        polarityMode
    );
    traceSourceLabel_ = std::move(source.label);
    traceSourceColor_ = source.color;
    traceSourcePoints_ = std::move(source.points);

    traceFit_ = FitTraceRibbon(
        traceSourcePoints_,
        ui.TraceLineBinWidthPx(),
        ui.TraceLineWindowPx(),
        ui.TraceLineOrder(),
        ui.TracePcaPeriodMs(),
        supportEdge,
        ui.TraceEdgeRefineEnabled()
    );

    if (!traceFit_.valid) {
        ClearTrace3D();
        return;
    }

    traceAnalysis_ = AnalyzeTrace3D(
        traceFit_,
        traceCalibration,
        traceBallRadiusMm,
        ui.TraceWidthStepPx(),
        ui.TraceWidthSmoothingEnabled(),
        TraceTimeOriginUs(traceSourcePoints_),
        [this](float timeSeconds, Vector3 &worldPoint) {
            return this->LookupGroundTruthWorld(timeSeconds, worldPoint);
        }
    );

    traceWorld3D = std::move(traceAnalysis_.worldPoints);
    traceTimes3D = std::move(traceAnalysis_.times);
    traceGroundTruthWorld3D = std::move(traceAnalysis_.groundTruthPoints);
    traceGroundTruthEstimateWorld3D = std::move(traceAnalysis_.groundTruthEstimatePoints);
    trace3DValid = traceAnalysis_.valid;
    traceCurrentWorld3D = traceAnalysis_.currentWorld;
    tracePoseText3D = traceAnalysis_.poseText;
}

void Gui::DrawTraceScene() {
    const float top = 385.0f;
    const float margin = 80.0f;
    const float availableWidth = static_cast<float>(screenWidth) - 2.0f * margin;
    const float availableHeight = static_cast<float>(screenHeight) - top - 40.0f;
    const float traceScale = std::max(
        0.1f,
        std::min(availableWidth / 640.0f, availableHeight / 480.0f)
    );

    const Rectangle source = {0.0f, 0.0f, 640.0f, 480.0f};
    const Rectangle dest = {
        margin,
        top,
        640.0f * traceScale,
        480.0f * traceScale
    };

    DrawRectangleRec(dest, RAYWHITE);
    DrawTexturePro(cpuTexture, source, dest, {0.0f, 0.0f}, 0.0f, WHITE);
    DrawRectangleLinesEx(dest, 2.0f, DARKGRAY);
    DrawText("Trace events - real-time trail", static_cast<int>(dest.x), static_cast<int>(dest.y - 30.0f), 22, BLACK);

    const float lineWindowPx = ui.TraceLineWindowPx();
    const int lineOrder = ui.TraceLineOrder();
    const float followWindowPx = ui.TraceFollowWindowPx();
    const std::vector<TracePoint> &tracePoints = traceSourcePoints_;
    DrawText(
        TextFormat("events: %zu accumulated | %ld current", tracePoints.size(), nb_event),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y),
        20,
        DARKGRAY
    );

    auto project = [&](const Vector2 &p) -> Vector2 {
        return {
            dest.x + p.x * traceScale,
            dest.y + p.y * traceScale
        };
    };

    auto drawMotionWindow = [&]() {
        if (!traceMotionWindowValid) {
            return;
        }

        const Vector2 center{traceMotionCircle.x, traceMotionCircle.y};
        const Color guideColor = traceMotionParabolaValid ? DARKBLUE : MAROON;
        DrawCircleV(project(center), 4.0f, guideColor);

        if (!traceMotionParabolaValid) {
            DrawCircleLinesV(project(center), followWindowPx * traceScale, Fade(guideColor, 0.70f));
            return;
        }

        const auto curveY = [&](float x) {
            return traceMotionYFromXFit.a * x * x
                 + traceMotionYFromXFit.b * x
                 + traceMotionYFromXFit.c;
        };
        const auto curveSlope = [&](float x) {
            return 2.0f * traceMotionYFromXFit.a * x + traceMotionYFromXFit.b;
        };

        const float lateralWindowPx = TraceFollowLateralWindowPx(followWindowPx, traceMotionCircle.r);
        constexpr int kWindowSegments = 42;
        std::vector<Vector2> centerCurve;
        std::vector<Vector2> upperGuide;
        std::vector<Vector2> lowerGuide;
        centerCurve.reserve(kWindowSegments + 1);
        upperGuide.reserve(kWindowSegments + 1);
        lowerGuide.reserve(kWindowSegments + 1);

        const float xStart = std::clamp(center.x - followWindowPx, 0.0f, 639.0f);
        const float xEnd = std::clamp(center.x + followWindowPx, 0.0f, 639.0f);

        for (int i = 0; i <= kWindowSegments; ++i) {
            const float ratio = static_cast<float>(i) / static_cast<float>(kWindowSegments);
            const float x = xStart + (xEnd - xStart) * ratio;
            const float y = curveY(x);
            const float slope = curveSlope(x);

            if (!std::isfinite(y) || !std::isfinite(slope)) {
                continue;
            }

            const Vector2 tangent = NormalizeOr({1.0f, slope}, {1.0f, 0.0f});
            const Vector2 normal{-tangent.y, tangent.x};
            const Vector2 point{x, y};
            centerCurve.emplace_back(point);
            upperGuide.emplace_back(Vector2{
                point.x + normal.x * lateralWindowPx,
                point.y + normal.y * lateralWindowPx
            });
            lowerGuide.emplace_back(Vector2{
                point.x - normal.x * lateralWindowPx,
                point.y - normal.y * lateralWindowPx
            });
        }

        auto drawGuideCurve = [&](const std::vector<Vector2> &curve, Color color, float thickness) {
            if (curve.size() < 2) {
                return;
            }

            Vector2 previous = project(curve.front());
            for (std::size_t i = 1; i < curve.size(); ++i) {
                const Vector2 current = project(curve[i]);
                DrawLineEx(previous, current, thickness, color);
                previous = current;
            }
        };

        drawGuideCurve(upperGuide, Fade(SKYBLUE, 0.65f), 1.5f);
        drawGuideCurve(lowerGuide, Fade(SKYBLUE, 0.65f), 1.5f);
        drawGuideCurve(centerCurve, Fade(guideColor, 0.85f), 2.0f);
    };

    auto drawTraceEvents = [&]() {
        if (tracePoints.empty()) {
            return;
        }

        int64_t tMin = tracePoints.front().timestampUs;
        int64_t tMax = tracePoints.front().timestampUs;
        for (const TracePoint &tracePoint : tracePoints) {
            tMin = std::min(tMin, tracePoint.timestampUs);
            tMax = std::max(tMax, tracePoint.timestampUs);
        }

        const int64_t tRange = std::max<int64_t>(1, tMax - tMin);
        const std::size_t step = std::max<std::size_t>(1, tracePoints.size() / 45000);
        const float pointSize = std::clamp(traceScale * 1.15f, 1.0f, 2.2f);

        for (std::size_t i = 0; i < tracePoints.size(); i += step) {
            const TracePoint &tracePoint = tracePoints[i];
            const Vector2 screenPoint = project(tracePoint.point);
            if (screenPoint.x < dest.x
                || screenPoint.x >= dest.x + dest.width
                || screenPoint.y < dest.y
                || screenPoint.y >= dest.y + dest.height) {
                continue;
            }

            const float ratio = static_cast<float>(tracePoint.timestampUs - tMin)
                / static_cast<float>(tRange);
            const Color eventColor = {
                static_cast<unsigned char>(30.0f + 225.0f * ratio),
                static_cast<unsigned char>(125.0f + 90.0f * ratio),
                static_cast<unsigned char>(255.0f * (1.0f - ratio)),
                210
            };
            DrawRectangle(
                static_cast<int>(screenPoint.x),
                static_cast<int>(screenPoint.y),
                static_cast<int>(std::ceil(pointSize)),
                static_cast<int>(std::ceil(pointSize)),
                eventColor
            );
        }
    };

    drawTraceEvents();
    drawMotionWindow();

    const TraceRibbonFit &fit = traceFit_;
    if (!fit.valid) {
        DrawText(
            "trace ribbon: not enough coherent events",
            static_cast<int>(dest.x + dest.width + 24.0f),
            static_cast<int>(dest.y + 34.0f),
            18,
            MAROON
        );
        if (tracePoints.empty() && ui.TraceUseRadiusGate()) {
            DrawText(
                "no followed ball window accumulated",
                static_cast<int>(dest.x + dest.width + 24.0f),
                static_cast<int>(dest.y + 58.0f),
                18,
                MAROON
            );
        }
        return;
    }

    auto drawCurve = [&](const std::vector<Vector2> &curve, Color color, float thickness) {
        if (curve.size() < 2) {
            return;
        }

        Vector2 previous = project(curve.front());
        for (std::size_t i = 1; i < curve.size(); ++i) {
            const Vector2 current = project(curve[i]);
            DrawLineEx(previous, current, thickness, color);
            previous = current;
        }
    };

    const float edgeLineThicknessPx = std::clamp(traceScale, 1.0f, 1.6f);
    const float middleLineThicknessPx = std::clamp(traceScale * 1.15f, 1.2f, 1.9f);
    drawCurve(fit.upperCurve, GREEN, edgeLineThicknessPx);
    drawCurve(fit.lowerCurve, MAGENTA, edgeLineThicknessPx);
    drawCurve(fit.middleCurve, YELLOW, middleLineThicknessPx);

    if (fit.temporalPcaSlices.size() > 1) {
        const float pcaDirectionLength = std::max(22.0f, fit.widthPx * 2.0f);
        for (const TracePcaSlice &slice : fit.temporalPcaSlices) {
            if (!slice.valid) {
                continue;
            }

            const Vector2 start{
                slice.center.x - slice.tangent.x * pcaDirectionLength * 0.5f,
                slice.center.y - slice.tangent.y * pcaDirectionLength * 0.5f
            };
            const Vector2 end{
                slice.center.x + slice.tangent.x * pcaDirectionLength * 0.5f,
                slice.center.y + slice.tangent.y * pcaDirectionLength * 0.5f
            };
            DrawLineEx(project(start), project(end), 2.0f, Fade(BLUE, 0.75f));
            DrawCircleV(project(slice.center), 2.5f, Fade(BLUE, 0.85f));
        }
    }

    auto drawSparseFitPoints = [&](const std::vector<Vector2> &points) {
        for (const Vector2 &sparsePoint : points) {
            const Vector2 screenPoint = project(sparsePoint);
            DrawCircleV(screenPoint, 4.0f, MAROON);
            DrawCircleLinesV(screenPoint, 5.0f, RED);
        }
    };

    drawSparseFitPoints(fit.sparseUpperFitPoints);
    drawSparseFitPoints(fit.sparseLowerFitPoints);
    drawSparseFitPoints(fit.sparseMiddleFitPoints);

    const int sparseFitCount =
        static_cast<int>(fit.sparseUpperFitPoints.size()
            + fit.sparseMiddleFitPoints.size()
            + fit.sparseLowerFitPoints.size());

    const Vector2 middleStart = project(fit.middleCurve.front());
    const Vector2 middleEnd = project(fit.middleCurve.back());
    DrawCircleV(middleStart, 4.0f, YELLOW);
    DrawCircleV(middleEnd, 4.0f, YELLOW);

    const Trace3DAnalysis &analysis = traceAnalysis_;
    const std::vector<TraceWidthEstimate> &widthEstimates = analysis.widthEstimates;
    const std::vector<float> &widthSamples = analysis.widthSamples;
    const int widthSampleCount = analysis.widthSampleCount;
    const int traceRejectedWorldOutliers = analysis.rejectedWorldOutliers;

    for (const TraceWidthEstimate &estimate : widthEstimates) {
        if (!estimate.valid || estimate.widthPx < 1.0f) {
            continue;
        }

        const Color widthColor = Fade(ORANGE, 0.85f);
        DrawLineEx(project(estimate.upper), project(estimate.lower), 2.0f, widthColor);
        DrawCircleV(project(estimate.middle), 3.0f, widthColor);
        const float directionLength = std::max(16.0f, estimate.widthPx * 1.35f);
        const Vector2 directionStart{
            estimate.middle.x - estimate.tangent.x * directionLength * 0.5f,
            estimate.middle.y - estimate.tangent.y * directionLength * 0.5f
        };
        const Vector2 directionEnd{
            estimate.middle.x + estimate.tangent.x * directionLength * 0.5f,
            estimate.middle.y + estimate.tangent.y * directionLength * 0.5f
        };
        DrawLineEx(project(directionStart), project(directionEnd), 1.5f, Fade(SKYBLUE, 0.85f));
    }

    DrawText(
        TextFormat("ribbon inliers: %d", static_cast<int>(fit.inlierCount)),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 34.0f),
        18,
        DARKGRAY
    );
    DrawText(
        TextFormat("length: %.1f px", fit.lengthPx),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 58.0f),
        18,
        DARKGRAY
    );
    DrawText(
        TextFormat(
            "width normal: %.1f px (%s)",
            widthSamples.empty() ? fit.widthPx : Quantile(widthSamples, 0.50f),
            ui.TraceWidthSmoothingEnabled() ? "smooth" : "raw"
        ),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 82.0f),
        18,
        DARKGRAY
    );
    DrawText(
        TextFormat(
            "width samples: %d/%d | 3D outliers: %d",
            static_cast<int>(widthEstimates.size()),
            widthSampleCount,
            traceRejectedWorldOutliers
        ),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 106.0f),
        18,
        traceRejectedWorldOutliers == 0 ? DARKGRAY : MAROON
    );
    DrawText(
        TextFormat(
            "line: binw=%.1fpx bins=%d win=%.0f support div=%.0f [%d-%d]",
            fit.lineBinWidthPx,
            fit.lineBinCount,
            lineWindowPx,
            fit.supportEdge.supportDivisor,
            static_cast<int>(fit.supportEdge.minLocalSupport),
            static_cast<int>(fit.supportEdge.maxLocalSupport)
        ),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 130.0f),
        18,
        DARKGRAY
    );
    DrawText(
        TextFormat(
            "order=%d pca=%d @ %.0fms follow=%.0fpx rad=%.2f border=%.1f%% drop=%d sparse=%d",
            lineOrder,
            static_cast<int>(fit.temporalPcaSlices.size()),
            fit.pcaPeriodMs,
            followWindowPx,
            fit.supportEdge.supportRadiusPx,
            fit.supportEdge.borderRatio * 100.0f,
            fit.ransacRejectedSamples,
            sparseFitCount
        ),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 154.0f),
        18,
        sparseFitCount == 0 ? DARKGREEN : MAROON
    );
    DrawText(
        TextFormat("fit input: %s", traceSourceLabel_.c_str()),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 178.0f),
        18,
        traceSourceColor_
    );
    if (trace3DValid) {
        DrawText(
            tracePoseText3D.c_str(),
            static_cast<int>(dest.x + dest.width + 24.0f),
            static_cast<int>(dest.y + 202.0f),
            18,
            DARKBLUE
        );
    }
    else if (!traceCalibration.ready) {
        DrawText(
            "trace 3D: missing calibration",
            static_cast<int>(dest.x + dest.width + 24.0f),
            static_cast<int>(dest.y + 202.0f),
            18,
            MAROON
        );
    }

    if (traceCalibration.ready) {
        DrawText(
            TextFormat(
                "calib: RMS=%.2f px model=%s",
                traceCalibration.reprojectionError,
                traceCalibration.useFisheyeModel ? "fisheye" : "opencv"
            ),
            static_cast<int>(dest.x + dest.width + 24.0f),
            static_cast<int>(dest.y + 226.0f),
            18,
            traceCalibration.reprojectionError >= 0.0 && traceCalibration.reprojectionError <= 1.0 ? DARKGRAY : MAROON
        );
    }

    const Vector2 topLabel = project(fit.upperCurve.back());
    const Vector2 middleLabel = project(fit.middleCurve.back());
    const Vector2 bottomLabel = project(fit.lowerCurve.back());
    DrawText("top edge", static_cast<int>(topLabel.x + 8.0f), static_cast<int>(topLabel.y - 18.0f), 16, GREEN);
    DrawText("middle", static_cast<int>(middleLabel.x + 8.0f), static_cast<int>(middleLabel.y - 8.0f), 16, GOLD);
    DrawText("bottom edge", static_cast<int>(bottomLabel.x + 8.0f), static_cast<int>(bottomLabel.y + 4.0f), 16, MAGENTA);
}

// Temporal stabilization of the blue trace trajectory: averages the curve
// coefficients over the recently computed fits, weighting recent fits more
// (exponential recency) and fits far from the consensus curve less (robust
// 1/(1+(d/scale)^2) on the curve-space distance). O(history) per frame with
// a small bounded history.
bool Gui::StabilizeTraceCurve(LineFit3D &fitX, LineFit3D &fitY, QuadFit3D &fitZ, float tMin, float tMax) {
    constexpr double kMemorySeconds = 0.8;
    constexpr std::size_t kMaxSamples = 48;
    constexpr int kEvalPoints = 5;

    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // A jump of the trace time range means the reader was scrubbed or the
    // sequence restarted: the history belongs to another playback position.
    if (!traceCurveHistory_.empty()) {
        const TraceCurveSample &last = traceCurveHistory_.back();
        if (tMax < last.tMax - 0.05f || tMax > last.tMax + 0.35f) {
            traceCurveHistory_.clear();
        }
    }

    traceCurveHistory_.push_back({fitX, fitY, fitZ, tMin, tMax, now});

    std::size_t firstFresh = 0;
    while (firstFresh < traceCurveHistory_.size()
        && (traceCurveHistory_.size() - firstFresh > kMaxSamples
            || now - traceCurveHistory_[firstFresh].wallSeconds > kMemorySeconds)) {
        ++firstFresh;
    }
    if (firstFresh > 0) {
        traceCurveHistory_.erase(
            traceCurveHistory_.begin(),
            traceCurveHistory_.begin() + static_cast<std::ptrdiff_t>(firstFresh));
    }

    const std::size_t count = traceCurveHistory_.size();
    if (count < 3) {
        return false;
    }

    const auto evalSample = [](const TraceCurveSample &sample, float t, float out[3]) {
        out[0] = sample.x.a * t + sample.x.b;
        out[1] = sample.y.a * t + sample.y.b;
        out[2] = sample.z.a * t * t + sample.z.b * t + sample.z.c;
    };

    std::vector<double> recencyWeights(count);
    double consensusX[2] = {0.0, 0.0};
    double consensusY[2] = {0.0, 0.0};
    double consensusZ[3] = {0.0, 0.0, 0.0};
    double recencySum = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const TraceCurveSample &sample = traceCurveHistory_[i];
        const double age = std::clamp((now - sample.wallSeconds) / kMemorySeconds, 0.0, 1.0);
        const double weight = std::exp(-3.0 * age);
        recencyWeights[i] = weight;
        recencySum += weight;
        consensusX[0] += weight * sample.x.a;
        consensusX[1] += weight * sample.x.b;
        consensusY[0] += weight * sample.y.a;
        consensusY[1] += weight * sample.y.b;
        consensusZ[0] += weight * sample.z.a;
        consensusZ[1] += weight * sample.z.b;
        consensusZ[2] += weight * sample.z.c;
    }
    if (recencySum <= 1.0e-9) {
        return false;
    }
    for (double &value : consensusX) value /= recencySum;
    for (double &value : consensusY) value /= recencySum;
    for (double &value : consensusZ) value /= recencySum;

    TraceCurveSample consensus{};
    consensus.x = {static_cast<float>(consensusX[0]), static_cast<float>(consensusX[1])};
    consensus.y = {static_cast<float>(consensusY[0]), static_cast<float>(consensusY[1])};
    consensus.z = {
        static_cast<float>(consensusZ[0]),
        static_cast<float>(consensusZ[1]),
        static_cast<float>(consensusZ[2])
    };

    std::vector<float> distances(count, 0.0f);
    for (std::size_t i = 0; i < count; ++i) {
        float total = 0.0f;
        for (int k = 0; k < kEvalPoints; ++k) {
            const float t = tMin + (tMax - tMin) * static_cast<float>(k)
                / static_cast<float>(kEvalPoints - 1);
            float a[3];
            float b[3];
            evalSample(traceCurveHistory_[i], t, a);
            evalSample(consensus, t, b);
            const float dx = a[0] - b[0];
            const float dy = a[1] - b[1];
            const float dz = a[2] - b[2];
            total += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        distances[i] = total / static_cast<float>(kEvalPoints);
    }

    std::vector<float> sortedDistances = distances;
    std::nth_element(
        sortedDistances.begin(),
        sortedDistances.begin() + static_cast<std::ptrdiff_t>(count / 2),
        sortedDistances.end());
    const float scale = std::max(1.0e-4f, sortedDistances[count / 2] * 1.4826f);

    double finalX[2] = {0.0, 0.0};
    double finalY[2] = {0.0, 0.0};
    double finalZ[3] = {0.0, 0.0, 0.0};
    double weightSum = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const double ratio = static_cast<double>(distances[i]) / static_cast<double>(scale);
        const double weight = recencyWeights[i] / (1.0 + ratio * ratio);
        const TraceCurveSample &sample = traceCurveHistory_[i];
        weightSum += weight;
        finalX[0] += weight * sample.x.a;
        finalX[1] += weight * sample.x.b;
        finalY[0] += weight * sample.y.a;
        finalY[1] += weight * sample.y.b;
        finalZ[0] += weight * sample.z.a;
        finalZ[1] += weight * sample.z.b;
        finalZ[2] += weight * sample.z.c;
    }
    if (weightSum <= 1.0e-9) {
        return false;
    }

    fitX = {static_cast<float>(finalX[0] / weightSum), static_cast<float>(finalX[1] / weightSum)};
    fitY = {static_cast<float>(finalY[0] / weightSum), static_cast<float>(finalY[1] / weightSum)};
    fitZ = {
        static_cast<float>(finalZ[0] / weightSum),
        static_cast<float>(finalZ[1] / weightSum),
        static_cast<float>(finalZ[2] / weightSum)
    };
    return true;
}

void Gui::Draw3DScene() {
    static bool wasControlling = false;
    const bool controlCamera = IsMouseButtonDown(MOUSE_RIGHT_BUTTON);
    bool traceFitValid = false;
    coef traceXFit{};
    coef traceYFit{};
    coef2 traceZFit{};
    float traceFitTMin = 0.0f;
    float traceFitTMax = 0.0f;

    if (trace3DValid && traceTimes3D.size() == traceWorld3D.size() && traceWorld3D.size() >= 3) {
        LinearRegression traceXReg;
        LinearRegression traceYReg;
        QuadraticRegression traceZReg;

        traceFitTMin = traceTimes3D.front();
        traceFitTMax = traceTimes3D.front();
        for (std::size_t i = 0; i < traceWorld3D.size(); ++i) {
            const float t = traceTimes3D[i];
            traceFitTMin = std::min(traceFitTMin, t);
            traceFitTMax = std::max(traceFitTMax, t);
            traceXReg.add(t, traceWorld3D[i].x);
            traceYReg.add(t, traceWorld3D[i].y);
            traceZReg.add(t, traceWorld3D[i].z);
        }

        traceXFit = traceXReg.fit();
        traceYFit = traceYReg.fit();
        traceZFit = traceZReg.fit();

        if (ui.WeightedRegressionEnabled()) {
            std::vector<float> xs;
            std::vector<float> ys;
            std::vector<float> zs;
            xs.reserve(traceWorld3D.size());
            ys.reserve(traceWorld3D.size());
            zs.reserve(traceWorld3D.size());
            for (const Vector3 &point : traceWorld3D) {
                xs.push_back(point.x);
                ys.push_back(point.y);
                zs.push_back(point.z);
            }

            traceXFit = WeightedLinearFit(traceTimes3D, xs, traceXFit);
            traceYFit = WeightedLinearFit(traceTimes3D, ys, traceYFit);
            traceZFit = WeightedQuadraticFit(traceTimes3D, zs, traceZFit);
        }

        traceFitValid = traceFitTMax > traceFitTMin;

        if (traceFitValid && ui.TraceCurveAverageEnabled()) {
            LineFit3D stabilizedX{traceXFit.a, traceXFit.b};
            LineFit3D stabilizedY{traceYFit.a, traceYFit.b};
            QuadFit3D stabilizedZ{traceZFit.a, traceZFit.b, traceZFit.c};
            if (StabilizeTraceCurve(stabilizedX, stabilizedY, stabilizedZ, traceFitTMin, traceFitTMax)) {
                traceXFit = {stabilizedX.a, stabilizedX.b};
                traceYFit = {stabilizedY.a, stabilizedY.b};
                traceZFit = {stabilizedZ.a, stabilizedZ.b, stabilizedZ.c};
            }
        }
        else if (!ui.TraceCurveAverageEnabled()) {
            traceCurveHistory_.clear();
        }
    }

    if (controlCamera) {
        if (!wasControlling) {
            DisableCursor();
        }
        UpdateCamera(&sceneCamera, CAMERA_FREE);
    }
    else {
        if (wasControlling) {
            EnableCursor();
        }
    }

    wasControlling = controlCamera;

    BeginMode3D(sceneCamera);
    DrawGrid(40, 0.25f);

    DrawLine3D({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, RED);
    DrawLine3D({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, GREEN);
    DrawLine3D({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 2.0f}, BLUE);

    for (const auto &worldPoint : trajectoryWorld3D) {
        DrawSphere(WorldToScene(worldPoint), 0.012f, Fade(GRAY, 0.85f));
    }

    if (trajectory3DValid && fitTMax3D > fitTMin3D) {
        constexpr int kSegments = 240;
        for (int i = 0; i < kSegments; ++i) {
            const float alpha0 = (float)i / (float)kSegments;
            const float alpha1 = (float)(i + 1) / (float)kSegments;
            const float t0 = fitTMin3D - 1 + alpha0 * (fitTMax3D + 3 - fitTMin3D);
            const float t1 = fitTMin3D - 1 + alpha1 * (fitTMax3D + 3 - fitTMin3D);

            const Vector3 world0 = {
                fitX3D.a * t0 + fitX3D.b,
                fitY3D.a * t0 + fitY3D.b,
                fitZ3D.a * t0 * t0 + fitZ3D.b * t0 + fitZ3D.c
            };
            const Vector3 world1 = {
                fitX3D.a * t1 + fitX3D.b,
                fitY3D.a * t1 + fitY3D.b,
                fitZ3D.a * t1 * t1 + fitZ3D.b * t1 + fitZ3D.c
            };

            DrawCylinderEx(WorldToScene(world0), WorldToScene(world1), 0.005f, 0.005f, 12, RED);

        }
    }

    if (hasCurrentBall3D) {
        const Vector3 ballScene = WorldToScene(currentBallWorld3D);
        DrawSphere(ballScene, currentBallRadius3D, ORANGE);
        DrawSphereWires(ballScene, currentBallRadius3D, 16, 16, MAROON);
        DrawLine3D(ballScene, {ballScene.x, 0.0f, ballScene.z}, Fade(RED, 0.6f));
    }

    if (trace3DValid && traceWorld3D.size() >= 3) {
        for (const Vector3 &worldPoint : traceWorld3D) {
            DrawSphere(WorldToScene(worldPoint), 0.010f, Fade(SKYBLUE, 0.9f));
        }

        if (!traceGroundTruthWorld3D.empty()) {
            for (std::size_t i = 0; i < traceGroundTruthWorld3D.size(); ++i) {
                const Vector3 truthScene = WorldToScene(traceGroundTruthWorld3D[i]);
                DrawSphere(truthScene, 0.013f, RED);
                DrawSphereWires(truthScene, 0.013f, 10, 10, MAROON);

                if (i < traceGroundTruthEstimateWorld3D.size()) {
                    DrawCylinderEx(
                        WorldToScene(traceGroundTruthEstimateWorld3D[i]),
                        truthScene,
                        0.0025f,
                        0.0025f,
                        8,
                        Fade(RED, 0.45f)
                    );
                }
            }

            for (std::size_t i = 1; i < traceGroundTruthWorld3D.size(); ++i) {
                DrawCylinderEx(
                    WorldToScene(traceGroundTruthWorld3D[i - 1]),
                    WorldToScene(traceGroundTruthWorld3D[i]),
                    0.004f,
                    0.004f,
                    10,
                    RED
                );
            }
        }

        if (traceFitValid) {
            constexpr int kTraceSegments = 80;
            for (int i = 0; i < kTraceSegments; ++i) {
                const float alpha0 = static_cast<float>(i) / static_cast<float>(kTraceSegments);
                const float alpha1 = static_cast<float>(i + 1) / static_cast<float>(kTraceSegments);
                const float t0 = traceFitTMin + alpha0 * (traceFitTMax - traceFitTMin);
                const float t1 = traceFitTMin + alpha1 * (traceFitTMax - traceFitTMin);

                const Vector3 world0{
                    traceXFit.a * t0 + traceXFit.b,
                    traceYFit.a * t0 + traceYFit.b,
                    traceZFit.a * t0 * t0 + traceZFit.b * t0 + traceZFit.c
                };
                const Vector3 world1{
                    traceXFit.a * t1 + traceXFit.b,
                    traceYFit.a * t1 + traceYFit.b,
                    traceZFit.a * t1 * t1 + traceZFit.b * t1 + traceZFit.c
                };

                DrawCylinderEx(WorldToScene(world0), WorldToScene(world1), 0.006f, 0.006f, 12, BLUE);
            }
        }
        else {
            for (std::size_t i = 1; i < traceWorld3D.size(); ++i) {
                DrawCylinderEx(
                    WorldToScene(traceWorld3D[i - 1]),
                    WorldToScene(traceWorld3D[i]),
                    0.006f,
                    0.006f,
                    12,
                    BLUE
                );
            }
        }

        const Vector3 traceScene = WorldToScene(traceCurrentWorld3D);
        DrawSphere(traceScene, traceBallRadiusMm * 1.0e-3f, BLUE);
        DrawSphereWires(traceScene, traceBallRadiusMm * 1.0e-3f, 16, 16, DARKBLUE);
    }

    EndMode3D();

    if (hasCurrentBall3D) {
        DrawText(currentBallText3D.c_str(), 8, 258, 20, BLACK);
    }
    else {
        DrawText("Ball pose: unavailable", 8, 258, 20, MAROON);
    }

    if (trajectory3DValid) {
        DrawText(TextFormat("x(t) = %.3f t + %.3f", fitX3D.a, fitX3D.b), 8, 286, 18, DARKBLUE);
        DrawText(TextFormat("y(t) = %.3f t + %.3f", fitY3D.a, fitY3D.b), 8, 308, 18, DARKBLUE);
        DrawText(TextFormat("z(t) = %.3f t^2 + %.3f t + %.3f", fitZ3D.a, fitZ3D.b, fitZ3D.c), 8, 330, 18, DARKBLUE);
    }

    if (trace3DValid) {
        DrawText(tracePoseText3D.c_str(), 8, 356, 18, BLUE);
        if (traceFitValid) {
            DrawText(
                ui.WeightedRegressionEnabled()
                    ? "Trace fit: weighted recent + robust"
                    : "Trace fit: unweighted",
                8,
                378,
                18,
                BLUE);
            DrawText(TextFormat("Trace x(t) = %.3f t + %.3f", traceXFit.a, traceXFit.b), 8, 400, 18, BLUE);
            DrawText(TextFormat("Trace y(t) = %.3f t + %.3f", traceYFit.a, traceYFit.b), 8, 422, 18, BLUE);
            DrawText(TextFormat("Trace z(t) = %.3f t^2 + %.3f t + %.3f", traceZFit.a, traceZFit.b, traceZFit.c), 8, 444, 18, BLUE);
        }
        else {
            DrawText("Trace 3D fit: blue x/y linear, z quadratic", 8, 378, 18, BLUE);
        }
        if (!traceGroundTruthWorld3D.empty()) {
            DrawText(
                TextFormat("Trace GT red: %zu / %zu matched positions",
                    traceGroundTruthWorld3D.size(),
                    traceWorld3D.size()),
                8,
                traceFitValid ? 466 : 400,
                18,
                RED
            );
        }
    }

}

void Gui::DrawHudTexts() {
    for (const auto &t : overlay_hud_texts) {
        DrawText(t.text.c_str(), (int)t.pos.x, (int)t.pos.y, t.size, t.color);
    }
    overlay_hud_texts.clear();
}

void Gui::DrawView() {
    int64_t t_min = View.getLowestTime();
    int64_t t_max = View.getHighestTime();
    int64_t t_range = t_max - t_min;
    Color min_color = {0, 0, 255, 255};
    Color max_color = {255, 128, 0, 255};
    std::for_each(std::execution::par_unseq, View.begin(), View.end(), [&](const auto &e) {
        int x = static_cast<int>(e.x());
        int y = static_cast<int>(e.y());

        if (x < 0 || x >= 1280 || y < 0 || y >= 480) return;

        int idx = y * 1280 + x;

        if (ui.Color_switch()) {
            float ratio = t_range > 0 ? (float)(e.timestamp() - t_min) / t_range : 0.5f;
            Color col = {
                (unsigned char)((1.0f - ratio) * min_color.r + ratio * max_color.r),
                (unsigned char)((1.0f - ratio) * min_color.g + ratio * max_color.g),
                (unsigned char)((1.0f - ratio) * min_color.b + ratio * max_color.b),
                255
            };
            pixelBuffer[idx] = col;
        }
        else {
            pixelBuffer[idx] = e.polarity() ? RED : BLUE;
        }
    });
}

void Gui::Update() {
    auto now = std::chrono::steady_clock::now();

    if (last_playback_time_.time_since_epoch().count() == 0) {
        last_playback_time_ = now;
    }

    const double playbackDeltaSeconds =
        std::chrono::duration<double>(now - last_playback_time_).count();
    last_playback_time_ = now;
    ui.AdvancePlayback(playbackDeltaSeconds);

    const double dure = std::chrono::duration<double, std::milli>(now - last_render_time).count();
    if (dure >= RENDER_PERIOD_MS) {

        if (IsKeyPressed(KEY_C) || ui.ConsumeResetRequested()) {
            ClearPoses = true;
            ClearTrajectory3D();
            ClearCurrentBall3D();
            ClearImageTrajectory2D();
        }
        if (IsKeyPressed(KEY_B)) ui.NextView();
        if (IsKeyPressed(KEY_V)) ui.PreviousView();
        if (IsKeyPressed(KEY_SPACE)) ui.TogglePlayback();
        if(IsKeyPressed(KEY_Z) && !IsKeyDown(KEY_LEFT_SHIFT)) ui.lector += 0.001;
        if(IsKeyPressed(KEY_X) && !IsKeyDown(KEY_LEFT_SHIFT)) ui.lector += 0.0001;
        if(IsKeyPressed(KEY_Z) && IsKeyDown(KEY_LEFT_SHIFT)) ui.lector -= 0.001;
        if(IsKeyPressed(KEY_X) && IsKeyDown(KEY_LEFT_SHIFT)) ui.lector -= 0.0001;
        ui.lector = std::clamp(ui.lector, 0.0f, 1.0f);

        mousePosition = GetMousePosition();
        Draw();
        clusters.clear();
        last_render_time = now;
        if(ui.read == 1){
            ui.read = 0;
            path_reader_ = MakeReaderEventPath(std::string(ui.read_file), ui.UseSequenceDirectory());
            if (event_reader_ != nullptr) event_reader_->close();
            if(event_writer_ != nullptr){
                event_writer_->close();
                std::cerr << "Writer closed, " << event_writer_->count() << " events saved\n";
            }
            event_reader_ = std::make_unique<EventReader>(path_reader_);
            LoadReaderCalibrationForReader(path_reader_);
            LoadGroundTruthForReader(path_reader_);
            ui.SetReaderDuration(event_reader_->durationSeconds());
            ui.EnableReaderMode();
            ui.ClearFileTextFocus();
        }
        if(ui.save == 1){
            ui.save = 0;
            path_writer_ = MakeEventPath(std::string(ui.save_file), "events3");
            if (event_writer_ != nullptr) event_writer_->close();
            if (event_reader_ != nullptr) event_reader_->close();
            ui.PausePlayback();
            event_writer_ = std::make_unique<EventWriter>(path_writer_);
            ui.ClearFileTextFocus();
        }
	}
}

void Gui::ReadStore(dv::EventStore &event) {
    if (event_writer_ != nullptr) {
        event_writer_->close();
        std::cerr << "Writer closed, " << event_writer_->count() << " events saved\n";
        event_writer_ = nullptr;
    }

    if (event_reader_ == nullptr) {
        try {
            path_reader_ = MakeReaderEventPath(std::string(ui.read_file), ui.UseSequenceDirectory());
            event_reader_ = std::make_unique<EventReader>(path_reader_);
            LoadReaderCalibrationForReader(path_reader_);
            LoadGroundTruthForReader(path_reader_);
            ui.SetReaderDuration(event_reader_->durationSeconds());
            std::cerr << "Reader ready with " << event_reader_->count() << " events\n";
        }
        catch (const std::exception &e) {
            std::cerr << "Reader disabled: " << e.what() << "\n";
            event_reader_ = nullptr;
            event = dv::EventStore();
            return;
        }
    }

    if (event_reader_) {
        ui.SetReaderDuration(event_reader_->durationSeconds());
        event_reader_->readWindowEndingAt(event, ui.PlaybackTimeSeconds(), ui.PlaybackWindowSeconds());
    }
}

void Gui::AddClusterView(std::vector<cv::Point2f> cluster) {
    clusters.emplace_back(std::move(cluster));
}

void Gui::DrawPerformance() {
    DrawText(TextFormat("Pre-processing : %.2f ms", ms_pre), 8, 110, 20, DARKGRAY);
    DrawText(TextFormat("Clustering     : %.2f ms", ms_cluster), 8, 134, 20, DARKGRAY);
    DrawText(TextFormat("Post-processing: %.2f ms", ms_post), 8, 158, 20, DARKGRAY);
    DrawText(TextFormat("Loop total     : %.2f ms", ms_loop), 8, 182, 20, DARKGRAY);
}

void Gui::DrawClusters() {
    const int colorCount = static_cast<int>(Colors.size());

    int id = 0;
    for (const auto &cluster : clusters) {
        const Color col = Colors[id % colorCount];

        std::for_each(std::execution::par_unseq, cluster.begin(), cluster.end(), [&](const auto &p) {
            int x = 640 + static_cast<int>(p.x);
            int y = static_cast<int>(p.y);

            if (x < 0 || x >= 1280 || y < 0 || y >= 480) return;

            pixelBuffer[y * 1280 + x] = col;
        });
        ++id;
    }
}

void Gui::AddCircle(float cx, float cy, float r, Color col, int thickness) { overlay_circles.push_back({{cx, cy}, r, col, thickness}); }
void Gui::AddArrow(float x1, float y1, float x2, float y2, Color col) { overlay_arrows.push_back({{x1, y1}, {x2, y2}, col}); }
void Gui::AddMarker(float x, float y, Color col) { overlay_markers.push_back({{x, y}, col}); }
void Gui::AddLine(float x1, float y1, float x2, float y2, Color col, int t) { overlay_lines.push_back({{x1, y1}, {x2, y2}, col, t}); }
void Gui::AddRect(float x, float y, float w, float h, Color col) { overlay_rects.push_back({{x, y, w, h}, col}); }
void Gui::AddLabel(float x, float y, const std::string &txt, Color col, int size) { overlay_texts.push_back({{x, y}, txt, col, size}); }
void Gui::AddHudText(float x, float y, const std::string &txt, Color col, int size) { overlay_hud_texts.push_back({{x, y}, txt, col, size}); }

Vector2 Gui::CamToScreen(float x, float y) {
    return {offset.x + (x + 640.0f) * scale, offset.y + y * scale};
}

void Gui::DrawOverlays() {
    for (auto &c : overlay_circles) {
        Vector2 sc = CamToScreen(c.center.x, c.center.y);
        DrawCircleLinesV(sc, c.radius * scale, c.color);
    }
    for (auto &a : overlay_arrows) {
        Vector2 s = CamToScreen(a.from.x, a.from.y);
        Vector2 e = CamToScreen(a.to.x, a.to.y);
        DrawLineV(s, e, a.color);

        Vector2 dir = Vector2Normalize(Vector2Subtract(e, s));
        Vector2 left = {dir.x * 8 - dir.y * 4, dir.y * 8 + dir.x * 4};
        Vector2 right = {dir.x * 8 + dir.y * 4, dir.y * 8 - dir.x * 4};
        DrawLineV(e, Vector2Subtract(e, left), a.color);
        DrawLineV(e, Vector2Subtract(e, right), a.color);
    }
    for (auto &m : overlay_markers) {
        Vector2 sc = CamToScreen(m.pos.x, m.pos.y);
        DrawLine(sc.x - 6, sc.y, sc.x + 6, sc.y, m.color);
        DrawLine(sc.x, sc.y - 6, sc.x, sc.y + 6, m.color);
    }
    for (auto &lline : overlay_lines) {
        Vector2 s = CamToScreen(lline.p1.x, lline.p1.y);
        Vector2 e = CamToScreen(lline.p2.x, lline.p2.y);
        DrawLineEx(s, e, (float)lline.thickness, lline.color);
    }
    for (auto &r : overlay_rects) {
        Rectangle sr = {offset.x + r.rect.x * scale,
                        offset.y + (480.f - r.rect.y - r.rect.height) * scale,
                        r.rect.width * scale, r.rect.height * scale};
        DrawRectangleLinesEx(sr, 1.0f, r.color);
    }
    for (auto &t : overlay_texts) {
        Vector2 sc = CamToScreen(t.pos.x, t.pos.y);
        DrawText(t.text.c_str(), (int)sc.x, (int)sc.y, t.size, t.color);
    }
    overlay_circles.clear();
    overlay_arrows.clear();
    overlay_markers.clear();
    overlay_lines.clear();
    overlay_rects.clear();
    overlay_texts.clear();
}

void Gui::DrawImageTrajectory2D() {
    if (!imageTrajectory2DValid || imageTMax2D <= imageTMin2D) {
        return;
    }

    constexpr int kSegments = 120;

    const float tStart = imageTMin2D;
    const float tEnd = imageTMax2D + 0.15f;

    bool hasPrevious = false;
    Vector2 previous {-1, -1};

    for (int i = 0; i <= kSegments; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(kSegments);
        const float t = tStart + alpha * (tEnd - tStart);
        const float x = imageXFit2D.a * t + imageXFit2D.b;
        const float y = imageYFit2D.a * t * t + imageYFit2D.b * t + imageYFit2D.c;

        const Vector2 current = CamToScreen(x, y);
        if( previous.x != -1 && previous.y != -1 )DrawLineEx(previous, current, 2.0f, BLUE);

        previous = current;

    }
}

void Gui::SetBall3D(const Vector3 &worldPosition, float radiusMeters, const std::string &label) {
    currentBallWorld3D = worldPosition;
    currentBallRadius3D = radiusMeters;
    currentBallText3D = label;
    hasCurrentBall3D = true;
}

void Gui::ClearCurrentBall3D() {
    hasCurrentBall3D = false;
    currentBallText3D = "Ball pose: unavailable";
}

void Gui::SetTrajectory3D(const std::vector<Vector3> &worldTrack, const std::vector<float> &worldTrackTimes,
                          const LineFit3D &fitX, const LineFit3D &fitY,
                          const QuadFit3D &fitZ, float tMin, float tMax, bool valid) {
    trajectoryWorld3D = worldTrack;
    trajectoryTimes3D = worldTrackTimes;
    fitX3D = fitX;
    fitY3D = fitY;
    fitZ3D = fitZ;
    fitTMin3D = tMin;
    fitTMax3D = tMax;
    trajectory3DValid = valid;
    UpdateTrajectoryQuality();
}

void Gui::ClearTrajectory3D() {
    trajectoryWorld3D.clear();
    trajectoryTimes3D.clear();
    trajectoryQuality.clear();
    fitX3D = {};
    fitY3D = {};
    fitZ3D = {};
    fitTMin3D = 0.0f;
    fitTMax3D = 0.0f;
    trajectory3DValid = false;
}

void Gui::SetTracePoseCalibration(const CalibrationData &calibration, float ballRadiusMm) {
    traceCalibration = calibration;
    traceBallRadiusMm = ballRadiusMm;
}

const CalibrationData *Gui::ReaderCalibrationOverride() const {
    return readerCalibrationOverrideReady ? &readerCalibrationOverride : nullptr;
}

void Gui::SetTraceMotionWindow(
    const Circle &circle,
    const QuadFit3D &yFromXFit,
    bool parabolaValid,
    float xMin,
    float xMax,
    int64_t timestampUs) {

    if (!std::isfinite(circle.x)
        || !std::isfinite(circle.y)
        || !std::isfinite(circle.r)
        || circle.r <= 0.0f) {
        ClearTraceMotionWindow();
        return;
    }

    traceMotionWindowValid = true;
    traceMotionCircle = circle;
    traceMotionYFromXFit = yFromXFit;
    traceMotionParabolaValid =
        parabolaValid
        && std::isfinite(yFromXFit.a)
        && std::isfinite(yFromXFit.b)
        && std::isfinite(yFromXFit.c)
        && std::isfinite(xMin)
        && std::isfinite(xMax)
        && std::fabs(xMax - xMin) >= 6.0f;
    traceMotionXMin = std::min(xMin, xMax);
    traceMotionXMax = std::max(xMin, xMax);
    traceMotionTimestampUs = timestampUs;
}

void Gui::ClearTraceMotionWindow() {
    traceMotionWindowValid = false;
    traceMotionParabolaValid = false;
    traceMotionCircle = {};
    traceMotionYFromXFit = {};
    traceMotionXMin = 0.0f;
    traceMotionXMax = 0.0f;
    traceMotionTimestampUs = std::numeric_limits<int64_t>::min();
}

bool Gui::TraceMotionWindowContains(const cv::Point2f &point) const {
    if (!traceMotionWindowValid
        || !std::isfinite(point.x)
        || !std::isfinite(point.y)
        || point.x < 0.0f
        || point.x >= 640.0f
        || point.y < 0.0f
        || point.y >= 480.0f) {
        return false;
    }

    const float followWindowPx = ui.TraceFollowWindowPx();
    const Vector2 center{traceMotionCircle.x, traceMotionCircle.y};
    const Vector2 sample{point.x, point.y};

    if (!traceMotionParabolaValid) {
        const Vector2 delta{sample.x - center.x, sample.y - center.y};
        return Dot2(delta, delta) <= followWindowPx * followWindowPx;
    }

    const auto curveY = [&](float x) {
        return traceMotionYFromXFit.a * x * x
             + traceMotionYFromXFit.b * x
             + traceMotionYFromXFit.c;
    };
    const auto curveSlope = [&](float x) {
        return 2.0f * traceMotionYFromXFit.a * x + traceMotionYFromXFit.b;
    };

    const float anchorY = curveY(center.x);
    const Vector2 anchor = std::isfinite(anchorY) ? Vector2{center.x, anchorY} : center;
    const float anchorSlope = curveSlope(center.x);
    const Vector2 tangent = NormalizeOr({1.0f, anchorSlope}, {1.0f, 0.0f});
    const Vector2 alongDelta{sample.x - anchor.x, sample.y - anchor.y};
    const float alongDistance = Dot2(alongDelta, tangent);

    if (std::fabs(alongDistance) > followWindowPx) {
        return false;
    }

    const float yOnCurve = curveY(point.x);
    const float slopeAtPoint = curveSlope(point.x);
    const float normalScale = std::sqrt(1.0f + slopeAtPoint * slopeAtPoint);
    if (!std::isfinite(yOnCurve) || !std::isfinite(normalScale) || normalScale <= 1.0e-6f) {
        const Vector2 delta{sample.x - center.x, sample.y - center.y};
        return Dot2(delta, delta) <= followWindowPx * followWindowPx;
    }

    const float normalDistance = std::fabs(point.y - yOnCurve) / normalScale;
    const float lateralWindowPx = TraceFollowLateralWindowPx(followWindowPx, traceMotionCircle.r);
    return normalDistance <= lateralWindowPx;
}

void Gui::AppendTraceEvents(
    const std::vector<cv::Point2f> &points,
    const std::vector<int64_t> &timestamps,
    const std::vector<bool> *polarities) {

    if (points.empty() || points.size() != timestamps.size()) {
        return;
    }

    const bool hasPolarities = polarities != nullptr && polarities->size() >= points.size();

    int64_t newestInBatch = std::numeric_limits<int64_t>::min();
    for (const int64_t timestamp : timestamps) {
        newestInBatch = std::max(newestInBatch, timestamp);
    }

    if (newestInBatch == std::numeric_limits<int64_t>::min()) {
        return;
    }

    if (traceLastAccumulatedTimestampUs != std::numeric_limits<int64_t>::min()
        && newestInBatch + 1000 < traceLastAccumulatedTimestampUs) {
        ResetTraceAccumulation();
    }

    const int64_t previousLastTimestamp = traceLastAccumulatedTimestampUs;
    traceAccumulatedPoints.reserve(traceAccumulatedPoints.size() + points.size());
    traceAccumulatedTimestamps.reserve(traceAccumulatedTimestamps.size() + timestamps.size());
    traceAccumulatedPolarities.reserve(traceAccumulatedPolarities.size() + points.size());

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

        if (!TraceMotionWindowContains(point)) {
            continue;
        }

        traceAccumulatedPoints.emplace_back(point);
        traceAccumulatedTimestamps.emplace_back(timestamp);
        traceAccumulatedPolarities.emplace_back(hasPolarities ? (*polarities)[i] : true);
    }

    traceLastAccumulatedTimestampUs = std::max(traceLastAccumulatedTimestampUs, newestInBatch);

    if (traceAccumulatedPoints.empty()) {
        return;
    }

    constexpr std::size_t kMaxAccumulatedTraceEvents = 120000;
    const int64_t memoryUs = static_cast<int64_t>(ui.TraceMemorySeconds() * 1.0e6);
    const int64_t cutoffTimestamp = traceLastAccumulatedTimestampUs - std::max<int64_t>(memoryUs, 1);

    std::size_t write = 0;
    for (std::size_t read = 0; read < traceAccumulatedPoints.size(); ++read) {
        if (traceAccumulatedTimestamps[read] < cutoffTimestamp) {
            continue;
        }

        if (write != read) {
            traceAccumulatedPoints[write] = traceAccumulatedPoints[read];
            traceAccumulatedTimestamps[write] = traceAccumulatedTimestamps[read];
            traceAccumulatedPolarities[write] = traceAccumulatedPolarities[read];
        }
        ++write;
    }

    traceAccumulatedPoints.resize(write);
    traceAccumulatedTimestamps.resize(write);
    traceAccumulatedPolarities.resize(write);

    if (traceAccumulatedPoints.size() > kMaxAccumulatedTraceEvents) {
        const std::size_t removeCount = traceAccumulatedPoints.size() - kMaxAccumulatedTraceEvents;
        traceAccumulatedPoints.erase(traceAccumulatedPoints.begin(), traceAccumulatedPoints.begin() + static_cast<std::ptrdiff_t>(removeCount));
        traceAccumulatedTimestamps.erase(traceAccumulatedTimestamps.begin(), traceAccumulatedTimestamps.begin() + static_cast<std::ptrdiff_t>(removeCount));
        traceAccumulatedPolarities.erase(traceAccumulatedPolarities.begin(), traceAccumulatedPolarities.begin() + static_cast<std::ptrdiff_t>(removeCount));
    }
}

void Gui::ResetTraceAccumulation() {
    traceAccumulatedPoints.clear();
    traceAccumulatedTimestamps.clear();
    traceAccumulatedPolarities.clear();
    traceLastAccumulatedTimestampUs = std::numeric_limits<int64_t>::min();
}

void Gui::ClearTrace3D() {
    trace3DValid = false;
    traceCurrentWorld3D = {0.0f, 0.0f, 0.0f};
    traceWorld3D.clear();
    traceTimes3D.clear();
    traceGroundTruthWorld3D.clear();
    traceGroundTruthEstimateWorld3D.clear();
    tracePoseText3D = "Trace pose: unavailable";
    traceAnalysis_ = Trace3DAnalysis{};
    traceCurveHistory_.clear();
}


void Gui::UpdateTrajectoryQuality() {
    trajectoryQuality.clear();

    const std::size_t n = trajectoryWorld3D.size();
    if (n < 4 || trajectoryTimes3D.size() != n) {
        return;
    }

    for (int percent = 10; percent <= 90; percent += 10) {
        std::size_t trainCount = (n * static_cast<std::size_t>(percent) + 99) / 100;
        trainCount = std::clamp<std::size_t>(trainCount, 3, n - 1);

        const std::size_t testCount = n - trainCount;
        if (testCount == 0) {
            continue;
        }

        LinearRegression xReg;
        LinearRegression yReg;
        QuadraticRegression zReg;

        for (std::size_t i = 0; i < trainCount; ++i) {
            const float t = trajectoryTimes3D[i];
            const Vector3 &p = trajectoryWorld3D[i];
            xReg.add(t, p.x);
            yReg.add(t, p.y);
            zReg.add(t, p.z);
        }

        const coef xFit = xReg.fit();
        const coef yFit = yReg.fit();
        const coef2 zFit = zReg.fit();

        double squaredError = 0.0;
        for (std::size_t i = trainCount; i < n; ++i) {
            const float t = trajectoryTimes3D[i];
            const Vector3 prediction{
                xFit.a * t + xFit.b,
                yFit.a * t + yFit.b,
                zFit.a * t * t + zFit.b * t + zFit.c
            };

            const Vector3 &actual = trajectoryWorld3D[i];
            const double dx = static_cast<double>(prediction.x - actual.x);
            const double dy = static_cast<double>(prediction.y - actual.y);
            const double dz = static_cast<double>(prediction.z - actual.z);
            squaredError += dx * dx + dy * dy + dz * dz;
        }

        const float rmse = static_cast<float>(std::sqrt(squaredError / static_cast<double>(testCount)));
        trajectoryQuality.push_back({
            static_cast<float>(percent) / 100.0f,
            rmse,
            static_cast<int>(trainCount),
            static_cast<int>(testCount)
        });
    }
}

void Gui::DrawTrajectoryQualityPanel() {
    const Rectangle panel = {80.0f, 360.0f, 1740.0f, 600.0f};
    DrawRectangleLinesEx(panel, 2.0f, DARKGRAY);

    DrawText("Trajectory quality", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 18.0f), 22, BLACK);
    DrawText("Train first 10..90%, test on remaining points", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 48.0f), 18, DARKGRAY);

    if (trajectoryWorld3D.size() < 4 || trajectoryTimes3D.size() != trajectoryWorld3D.size()) {
        DrawText("Not enough 3D points for validation", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 92.0f), 20, MAROON);
        DrawText(TextFormat("3D points: %d", static_cast<int>(trajectoryWorld3D.size())), static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 122.0f), 18, DARKGRAY);
        return;
    }

    if (trajectoryQuality.empty()) {
        UpdateTrajectoryQuality();
    }

    float maxRmse = 0.0f;
    for (const auto &q : trajectoryQuality) {
        maxRmse = std::max(maxRmse, q.rmseMeters);
    }

    if (maxRmse <= 1.0e-6f) {
        maxRmse = 1.0e-6f;
    }

    const Rectangle chart = {panel.x + 18.0f, panel.y + 115.0f, 980.0f, 330.0f};
    DrawRectangleLinesEx(chart, 1.0f, GRAY);

    DrawText(TextFormat("max RMSE = %.4f m", maxRmse), static_cast<int>(chart.x), static_cast<int>(chart.y - 26.0f), 18, DARKBLUE);

    const int count = static_cast<int>(trajectoryQuality.size());
    const float gap = 8.0f;
    const float barWidth = count > 0 ? (chart.width - gap * static_cast<float>(count + 1)) / static_cast<float>(count) : 0.0f;

    for (int i = 0; i < count; ++i) {
        const auto &q = trajectoryQuality[i];
        const float normalized = std::clamp(q.rmseMeters / maxRmse, 0.0f, 1.0f);
        const float barHeight = normalized * (chart.height - 30.0f);
        const float x = chart.x + gap + static_cast<float>(i) * (barWidth + gap);
        const float y = chart.y + chart.height - 24.0f - barHeight;

        DrawRectangle(static_cast<int>(x), static_cast<int>(y), static_cast<int>(barWidth), static_cast<int>(barHeight), ORANGE);
        DrawRectangleLines(static_cast<int>(x), static_cast<int>(y), static_cast<int>(barWidth), static_cast<int>(barHeight), BROWN);

        DrawText(TextFormat("%d", static_cast<int>(q.trainRatio * 100.0f + 0.5f)), static_cast<int>(x + 3.0f), static_cast<int>(chart.y + chart.height - 20.0f), 14, BLACK);
        DrawText(TextFormat("%.3f", q.rmseMeters), static_cast<int>(x), static_cast<int>(y - 18.0f), 12, DARKGRAY);
    }

    DrawText("train %", static_cast<int>(chart.x + chart.width - 62.0f), static_cast<int>(chart.y + chart.height + 8.0f), 16, BLACK);
    DrawText("RMSE test 3D (m)", static_cast<int>(chart.x + 420.0f), static_cast<int>(chart.y + chart.height + 30.0f), 16, BLACK);

    const float tableX = panel.x + 1040.0f;
    float y = panel.y + 112.0f;
    DrawText("Results", static_cast<int>(tableX), static_cast<int>(y), 20, BLACK);
    y += 32.0f;
    DrawText("%   train  rest  RMSE (m)", static_cast<int>(tableX), static_cast<int>(y), 15, DARKGRAY);
    y += 24.0f;

    for (const auto &q : trajectoryQuality) {
        DrawText(
            TextFormat("%2d  %5d  %4d  %.5f",
                static_cast<int>(q.trainRatio * 100.0f + 0.5f),
                q.trainCount,
                q.testCount,
                q.rmseMeters),
            static_cast<int>(tableX),
            static_cast<int>(y),
            14,
            BLACK
        );
        y += 23.0f;
    }

    const auto &last = trajectoryQuality.back();
    DrawText(TextFormat("Current points: %d", static_cast<int>(trajectoryWorld3D.size())), static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + panel.height - 90.0f), 18, DARKGRAY);
    DrawText(TextFormat("Last test RMSE: %.4f m", last.rmseMeters), static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + panel.height - 62.0f), 18, DARKBLUE);
}



void Gui::SetImageTrajectory2D(
        const LineFit3D &fitX,
        const QuadFit3D &fitY,
        float tMin,
        float tMax,
        bool valid) {

    imageXFit2D = fitX;
    imageYFit2D = fitY;
    imageTMin2D = tMin;
    imageTMax2D = tMax;
    imageTrajectory2DValid = valid;
}

void Gui::ClearImageTrajectory2D() {
    imageXFit2D = {};
    imageYFit2D = {};
    imageTMin2D = 0.0f;
    imageTMax2D = 0.0f;
    imageTrajectory2DValid = false;
}


void Gui::DrawTopResidualSpeedPanel(const LineFit3D &topFit) {
    const Rectangle panel = {1040.0f, 360.0f, 780.0f, 600.0f};
    DrawRectangleLinesEx(panel, 2.0f, DARKGRAY);

    DrawText("Top line error vs fitted camera-plane speed", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 18.0f), 22, BLACK);
    DrawText("perpendicular distance to TOP line / camera XZ speed from 3D regression", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 48.0f), 17, DARKGRAY);

    if (trajectoryWorld3D.size() < 3 || trajectoryTimes3D.size() != trajectoryWorld3D.size() || !trajectory3DValid) {
        DrawText("Not enough valid 3D regression data", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 92.0f), 20, MAROON);
        return;
    }

    struct ResidualSpeedSample {
        float speedXZ = 0.0f;
        float distance = 0.0f;
    };

    std::vector<ResidualSpeedSample> samples;
    samples.reserve(trajectoryWorld3D.size());
    LinearRegression SpeedXZReg;
    const float denominator = std::sqrt(topFit.a * topFit.a + 1.0f);
    if (denominator <= 1.0e-6f) {
        DrawText("Invalid TOP regression", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 92.0f), 20, MAROON);
        return;
    }

    for (std::size_t i = 0; i < trajectoryWorld3D.size(); ++i) {
        const float t = trajectoryTimes3D[i];
        const Vector3 &p = trajectoryWorld3D[i];


        const float speedXZ = FittedCameraPlaneSpeedXZ(fitX3D, fitZ3D, t);

        const float distance = std::fabs(topFit.a * p.x - p.y + topFit.b) / denominator;

        if (std::isfinite(speedXZ) && std::isfinite(distance)) {
            samples.push_back({speedXZ, distance});
            SpeedXZReg.add(speedXZ, distance);
        }
    }

    if (samples.empty()) {
        DrawText("No valid speed sample", static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 92.0f), 20, MAROON);
        return;
    }


    float maxSpeed = 0.0f;
    float minSpeed = 400.0f;
    float minDistance = 400.0f;
    float maxDistance = 0.0f;
    float sumDistance = 0.0f;

    for (const auto &sample : samples) {
        maxSpeed = std::max(maxSpeed, sample.speedXZ);
        maxDistance = std::max(maxDistance, sample.distance);
        minSpeed = std::min(minSpeed,sample.speedXZ);
        minDistance = std::min(minDistance,sample.distance);
        sumDistance += sample.distance;
    }

    float deltaSpeed = maxSpeed - minSpeed;

    float deltaDistance = maxDistance - minDistance;

    coef CoefXZReg = SpeedXZReg.fit();

    const float meanDistance = sumDistance / static_cast<float>(samples.size());

    const Rectangle chart = {panel.x + 60.0f, panel.y + 155.0f, 640.0f, 330.0f};
    DrawRectangleLinesEx(chart, 1.0f, GRAY);

    auto project = [&](float speed, float distance) -> Vector2 {
        const float sx = chart.x + ((speed - minSpeed) / deltaSpeed) * chart.width;
        const float sy = chart.y + chart.height - ((distance - minDistance) / deltaDistance) * chart.height;
        return {sx, sy};
    };
    auto ClipLineToRect =  [&] (float x0,float y0) -> Vector2{
        if (y0 > maxDistance) {
            x0 = (maxDistance - CoefXZReg.b) / CoefXZReg.a;
            y0 = maxDistance;
            return {x0, y0};
        }
        else if (y0 < 0.0f) {
            x0 = (0.0f - CoefXZReg.b) / CoefXZReg.a;
            y0 = 0.0f;
            return {x0, y0};
        }
        return{x0,y0};
    };

    for (const auto &sample : samples) {
        const Vector2 screen = project(sample.speedXZ, sample.distance);
        DrawCircleV(screen, 4.0f, BLUE);
    }

    DrawLineEx({chart.x, chart.y + chart.height}, {chart.x + chart.width, chart.y + chart.height}, 2.0f, BLACK);
    DrawLineEx({chart.x, chart.y}, {chart.x, chart.y + chart.height}, 2.0f, BLACK);

    Vector2 lineStart = ClipLineToRect(minSpeed, CoefXZReg.a * minSpeed + CoefXZReg.b);
    lineStart = project(lineStart.x, lineStart.y);
    Vector2 lineEnd = ClipLineToRect(maxSpeed, CoefXZReg.a * maxSpeed + CoefXZReg.b);
    lineEnd = project(lineEnd.x, lineEnd.y);
    DrawLineEx(lineStart, lineEnd, 2.0f, RED);

    DrawText(TextFormat("max Speed = %.3f m/s", maxSpeed), static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 86.0f), 17, DARKBLUE);
    DrawText(TextFormat("max dist = %.4f m | mean dist = %.4f m", maxDistance, meanDistance), static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 108.0f), 17, DARKBLUE);
    DrawText(TextFormat("vx=%.3f | vz(t)=%.3ft + %.3f | Slope = %.2f", fitX3D.a, 2.0f * fitZ3D.a, fitZ3D.b,CoefXZReg.a), static_cast<int>(panel.x + 18.0f), static_cast<int>(panel.y + 130.0f), 15, BLACK);


    DrawText("Fitted camera-plane XZ speed (m/s)", static_cast<int>(chart.x + 165.0f), static_cast<int>(chart.y + chart.height + 30.0f), 17, BLACK);
    DrawText("perp. distance", static_cast<int>(chart.x + chart.width + 16.0f), static_cast<int>(chart.y + 145.0f), 17, BLACK);
    DrawText("to TOP line (m)", static_cast<int>(chart.x + chart.width + 16.0f), static_cast<int>(chart.y + 166.0f), 17, BLACK);

    DrawText(TextFormat("%.3f", maxDistance), static_cast<int>(chart.x - 50.0f), static_cast<int>(chart.y - 6.0f), 14, BLACK);
    DrawText(TextFormat("%.3f", minDistance), static_cast<int>(chart.x - 50.0f), static_cast<int>(chart.y + chart.height - 6.0f), 14, BLACK);
    DrawText(TextFormat("%.2f", minSpeed), static_cast<int>(chart.x - 18.0f), static_cast<int>(chart.y + chart.height - 15.0f), 14, BLACK);
    DrawText(TextFormat("%.2f", maxSpeed), static_cast<int>(chart.x + chart.width - 35.0f), static_cast<int>(chart.y + chart.height + 8.0f), 14, BLACK);
}

void Gui::DrawTopView() {
    DrawText("Top view - X/Y plane", 8, 258, 22, BLACK);
    DrawText("Regression: Y = aX + b", 8, 284, 18, DARKGRAY);

    Rectangle area = {80.0f, 360.0f, 900.0f, 600.0f};
    DrawRectangleLinesEx(area, 2.0f, DARKGRAY);

    if (trajectoryWorld3D.empty()) {
        DrawText("No 3D points available", 100, 390, 20, MAROON);
        return;
    }

    float minX = trajectoryWorld3D.front().x;
    float maxX = trajectoryWorld3D.front().x;
    float minY = trajectoryWorld3D.front().y;
    float maxY = trajectoryWorld3D.front().y;

    for (const auto &p : trajectoryWorld3D) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }

    const float margin = 0.15f;
    minX -= margin;
    maxX += margin;
    minY -= margin;
    maxY += margin;

    const float rangeX = std::max(maxX - minX, 0.001f);
    const float rangeY = std::max(maxY - minY, 0.001f);

    auto project = [&](float x, float y) -> Vector2 {
        const float sx = area.x + ((x - minX) / rangeX) * area.width;
        const float sy = area.y + area.height - ((y - minY) / rangeY) * area.height;
        return {sx, sy};
    };

    LinearRegression topRegression;

    float maxCameraPlaneSpeed = 0.0f;
    const bool canColorBySpeed = trajectory3DValid && trajectoryTimes3D.size() == trajectoryWorld3D.size();
    if (canColorBySpeed) {
        for (const float t : trajectoryTimes3D) {
            const float speed = FittedCameraPlaneSpeedXZ(fitX3D, fitZ3D, t);
            if (std::isfinite(speed)) {
                maxCameraPlaneSpeed = std::max(maxCameraPlaneSpeed, speed);
            }
        }
    }

    for (std::size_t i = 0; i < trajectoryWorld3D.size(); ++i) {
        const auto &p = trajectoryWorld3D[i];
        topRegression.add(p.x, p.y);

        Color pointColor = ORANGE;
        if (canColorBySpeed && i < trajectoryTimes3D.size()) {
            const float speed = FittedCameraPlaneSpeedXZ(fitX3D, fitZ3D, trajectoryTimes3D[i]);
            pointColor = SpeedToHeatColor(speed, maxCameraPlaneSpeed);
        }

        const Vector2 screen = project(p.x, p.y);
        DrawCircleV(screen, 4.5f, pointColor);
        DrawCircleLinesV(screen, 5.0f, Fade(BLACK, 0.45f));
    }

    if (canColorBySpeed && maxCameraPlaneSpeed > 1.0e-6f) {
        const Rectangle legend = {area.x + 18.0f, area.y + 18.0f, 230.0f, 18.0f};
        constexpr int kLegendSteps = 80;
        for (int i = 0; i < kLegendSteps; ++i) {
            const float ratio0 = static_cast<float>(i) / static_cast<float>(kLegendSteps);
            const float speed = ratio0 * maxCameraPlaneSpeed;
            const Color col = SpeedToHeatColor(speed, maxCameraPlaneSpeed);
            const float x = legend.x + ratio0 * legend.width;
            DrawRectangle(static_cast<int>(x), static_cast<int>(legend.y),
                          static_cast<int>(legend.width / static_cast<float>(kLegendSteps) + 1.0f),
                          static_cast<int>(legend.height), col);
        }
        DrawRectangleLinesEx(legend, 1.0f, BLACK);
        DrawText("camera XZ speed", static_cast<int>(legend.x), static_cast<int>(legend.y - 20.0f), 14, BLACK);
        DrawText("0", static_cast<int>(legend.x), static_cast<int>(legend.y + 24.0f), 13, DARKGRAY);
        DrawText(TextFormat("%.2f m/s", maxCameraPlaneSpeed), static_cast<int>(legend.x + legend.width - 65.0f), static_cast<int>(legend.y + 24.0f), 13, DARKGRAY);
    }

    if (topRegression.size() >= 2) {
        const coef rawTopFit = topRegression.fit();
        const float upperOffset = EstimateUpperParallelOffset(trajectoryWorld3D, {rawTopFit.a, rawTopFit.b});
        const LineFit3D upperTopFit{rawTopFit.a, rawTopFit.b + upperOffset};
        auto ClipLineToRect =  [&] (float& x0,float& y0){
        if (y0 > maxY) {
            x0 = (maxY - rawTopFit.b) / rawTopFit.a;
            y0 = maxY;
        }
        else if (y0 < minY) {
            x0 = (minY - rawTopFit.b) / rawTopFit.a;
            y0 = minY;
        }};
        float x0 = minX;
        float x1 = maxX;

        float rawY0 = rawTopFit.a * x0 + rawTopFit.b;
        float rawY1 = rawTopFit.a * x1 + rawTopFit.b;

        ClipLineToRect(x0, rawY0);
        ClipLineToRect(x1, rawY1);
        DrawLineEx(project(x0, rawY0), project(x1, rawY1), 1.5f, Fade(DARKGRAY, 0.45f));

        float y0 = upperTopFit.a * x0 + upperTopFit.b;
        float y1 = upperTopFit.a * x1 + upperTopFit.b;
        ClipLineToRect(x0, y0);
        ClipLineToRect(x1, y1);
        DrawLineEx(project(x0, y0), project(x1, y1), 3.0f, RED);

        DrawText(
            TextFormat("Y = %.3f X + %.3f | upper offset = %.3f m", upperTopFit.a, upperTopFit.b, upperOffset),
            100,
            320,
            20,
            DARKBLUE
        );

        DrawTopResidualSpeedPanel(upperTopFit);
    }
    else {
        DrawText("Need at least 2 points for TOP regression", 1040, 390, 20, MAROON);
    }

    DrawText("X lateral position (m)", static_cast<int>(area.x + 300), static_cast<int>(area.y + area.height + 18), 18, BLACK);
    DrawText("Y depth (m)", static_cast<int>(area.x + area.width + 20), static_cast<int>(area.y + 260), 18, BLACK);
}
