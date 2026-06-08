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

struct TracePoint {
    Vector2 point{0.0f, 0.0f};
    int64_t timestampUs = 0;
    bool polarity = true;
};

struct TraceProjection {
    float s = 0.0f;
    float h = 0.0f;
    Vector2 point{0.0f, 0.0f};
    double timeSeconds = 0.0;
};

struct TraceWidthEstimate {
    bool valid = false;
    Vector2 middle{0.0f, 0.0f};
    Vector2 upper{0.0f, 0.0f};
    Vector2 lower{0.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    Vector2 tangent{1.0f, 0.0f};
    float widthPx = 0.0f;
    double timeSeconds = 0.0;
    int localEventCount = 0;
};

struct LocalTraceSection {
    bool valid = false;
    Vector2 middle{0.0f, 0.0f};
    Vector2 upper{0.0f, 0.0f};
    Vector2 lower{0.0f, 0.0f};
    Vector2 tangent{1.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    float widthPx = 0.0f;
    float s = 0.0f;
    double timeSeconds = 0.0;
    int localEventCount = 0;
};

struct PolynomialSample {
    double x = 0.0;
    double y = 0.0;
};

struct LocalQuadraticFit {
    bool valid = false;
    bool fallback = false;
    bool sparse = false;
    int degree = 2;
    int supportCount = 0;
    float supportSpanPx = 0.0f;
    double scale = 1.0;
    double c = 0.0;
    double b = 0.0;
    double a = 0.0;
};

struct LocalQuadraticModel {
    bool valid = false;
    int order = 2;
    double bandwidth = 24.0;
    std::vector<PolynomialSample> samples;
};

struct TracePcaResult {
    bool valid = false;
    Vector2 center{0.0f, 0.0f};
    Vector2 direction{1.0f, 0.0f};
    double lambda1 = 0.0;
    double lambda2 = 0.0;
    int eventCount = 0;
};

struct TracePcaSlice {
    bool valid = false;
    Vector2 center{0.0f, 0.0f};
    Vector2 tangent{1.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    double timeStartSeconds = 0.0;
    double timeEndSeconds = 0.0;
    int eventCount = 0;
};

struct TraceSupportEdgeSettings {
    float supportDivisor = 28.0f;
    std::size_t minLocalSupport = 3;
    std::size_t maxLocalSupport = 9;
    float supportRadiusPx = 1.75f;
    float borderRatio = 0.035f;
};

struct TraceRibbonFit {
    bool valid = false;
    Vector2 center{0.0f, 0.0f};
    Vector2 direction{1.0f, 0.0f};
    Vector2 normal{0.0f, 1.0f};
    LocalQuadraticModel middleModel{};
    LocalQuadraticModel upperModel{};
    LocalQuadraticModel lowerModel{};
    float sMin = 0.0f;
    float sMax = 0.0f;
    float lengthPx = 0.0f;
    float widthPx = 0.0f;
    std::size_t inlierCount = 0;
    int lineBinCount = 0;
    float lineBinWidthPx = 0.0f;
    TraceSupportEdgeSettings supportEdge{};
    float pcaPeriodMs = 8.0f;
    int ransacRejectedSamples = 0;
    std::vector<TraceProjection> projections;
    std::vector<Vector2> middleCurve;
    std::vector<Vector2> upperCurve;
    std::vector<Vector2> lowerCurve;
    std::vector<Vector2> sparseMiddleFitPoints;
    std::vector<Vector2> sparseUpperFitPoints;
    std::vector<Vector2> sparseLowerFitPoints;
    std::vector<TracePcaSlice> temporalPcaSlices;
};

bool AcceptTracePolarity(bool polarity, int polarityMode) {
    if (polarityMode == 1) {
        return polarity;
    }
    if (polarityMode == 2) {
        return !polarity;
    }
    return true;
}

const char *TracePolarityModeName(int polarityMode) {
    if (polarityMode == 1) {
        return "positive";
    }
    if (polarityMode == 2) {
        return "negative";
    }
    return "all";
}

std::vector<TracePoint> BuildTracePointsFromEvents(const dv::EventStore &events, int polarityMode) {
    std::vector<TracePoint> points;
    points.reserve(events.size());

    for (const auto &e : events) {
        const bool polarity = e.polarity();
        if (!AcceptTracePolarity(polarity, polarityMode)) {
            continue;
        }

        const float x = static_cast<float>(e.x());
        const float y = static_cast<float>(e.y());
        if (x >= 0.0f && x < 640.0f && y >= 0.0f && y < 480.0f) {
            points.push_back({{x, y}, e.timestamp(), polarity});
        }
    }

    return points;
}

std::vector<TracePoint> BuildTracePointsFromFloatSource(
    const std::vector<cv::Point2f> &pointsSource,
    const std::vector<int64_t> &timestamps,
    const std::vector<bool> *polarities,
    int polarityMode) {

    std::vector<TracePoint> points;
    const std::size_t count = std::min(pointsSource.size(), timestamps.size());
    const bool hasPolarities = polarities != nullptr && polarities->size() >= count;
    points.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        const bool polarity = hasPolarities ? (*polarities)[i] : true;
        if (hasPolarities && !AcceptTracePolarity(polarity, polarityMode)) {
            continue;
        }

        const float x = pointsSource[i].x;
        const float y = pointsSource[i].y;
        if (std::isfinite(x)
            && std::isfinite(y)
            && x >= 0.0f
            && x < 640.0f
            && y >= 0.0f
            && y < 480.0f) {
            points.push_back({{x, y}, timestamps[i], polarity});
        }
    }

    return points;
}

struct TracePointSourceResult {
    std::vector<TracePoint> points;
    std::string label;
    Color color = MAROON;
};

TracePointSourceResult BuildTracePointSource(
    const std::vector<cv::Point2f> &accumulatedPoints,
    const std::vector<int64_t> &accumulatedTimestamps,
    const std::vector<bool> &accumulatedPolarities,
    const std::vector<cv::Point2f> *rawPoints,
    const std::vector<int64_t> *rawTimestamps,
    const std::vector<bool> *rawPolarities,
    const std::vector<cv::Point2f> *floatPoints,
    const std::vector<int64_t> *floatTimestamps,
    const std::vector<bool> *floatPolarities,
    const dv::EventStore &events,
    bool useRawInput,
    bool useRadiusGate,
    bool motionWindowValid,
    int polarityMode) {

    TracePointSourceResult result;
    result.label = useRawInput
        ? "raw events in moving ball window"
        : "undistorted events in moving ball window";
    result.label += useRadiusGate ? " + radius gate" : " without radius gate";
    result.label += std::string(" | pol=") + TracePolarityModeName(polarityMode);
    result.color = motionWindowValid ? DARKGREEN : MAROON;

    if (!accumulatedPoints.empty()
        && accumulatedPoints.size() == accumulatedTimestamps.size()
        && accumulatedPoints.size() == accumulatedPolarities.size()) {
        result.points = BuildTracePointsFromFloatSource(
            accumulatedPoints,
            accumulatedTimestamps,
            &accumulatedPolarities,
            polarityMode);
    }

    if (result.points.empty()
        && !useRadiusGate
        && useRawInput
        && rawPoints != nullptr
        && rawTimestamps != nullptr
        && !rawPoints->empty()
        && rawPoints->size() == rawTimestamps->size()) {
        result.points = BuildTracePointsFromFloatSource(
            *rawPoints,
            *rawTimestamps,
            rawPolarities,
            polarityMode);
        result.label = std::string("raw current window fallback | pol=")
            + TracePolarityModeName(polarityMode);
        result.color = MAROON;
    }

    if (result.points.empty()
        && !useRadiusGate
        && floatPoints != nullptr
        && floatTimestamps != nullptr
        && !floatPoints->empty()
        && floatPoints->size() == floatTimestamps->size()) {
        result.points = BuildTracePointsFromFloatSource(
            *floatPoints,
            *floatTimestamps,
            floatPolarities,
            polarityMode);
        result.label = std::string("undistorted current window fallback | pol=")
            + TracePolarityModeName(polarityMode);
        result.color = MAROON;
    }

    if (result.points.empty() && !useRadiusGate) {
        result.points = BuildTracePointsFromEvents(events, polarityMode);
        result.label = std::string("rounded current window fallback | pol=")
            + TracePolarityModeName(polarityMode);
        result.color = MAROON;
    }

    return result;
}

float Quantile(std::vector<float> values, float q) {
    if (values.empty()) {
        return 0.0f;
    }

    q = std::clamp(q, 0.0f, 1.0f);
    const std::size_t index = std::min(
        values.size() - 1,
        static_cast<std::size_t>(std::floor(q * static_cast<float>(values.size() - 1)))
    );

    std::nth_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(index),
        values.end()
    );

    return values[index];
}

struct TraceEdgeEstimate {
    bool valid = false;
    float low = 0.0f;
    float middle = 0.0f;
    float high = 0.0f;
    float width = 0.0f;
    int supportCount = 0;
};

TraceEdgeEstimate EstimateSupportedEdges(
    const std::vector<float> &values,
    std::size_t minSupportCount,
    const TraceSupportEdgeSettings &settings) {

    TraceEdgeEstimate estimate;
    std::vector<float> finiteValues;
    finiteValues.reserve(values.size());
    for (float value : values) {
        if (std::isfinite(value)) {
            finiteValues.push_back(value);
        }
    }

    if (finiteValues.size() < std::max<std::size_t>(minSupportCount, 8)) {
        return estimate;
    }

    std::sort(finiteValues.begin(), finiteValues.end());

    const std::size_t feasibleSupport = std::max<std::size_t>(1, finiteValues.size());
    const std::size_t minLocalSupport = std::min(
        std::max<std::size_t>(settings.minLocalSupport, 1),
        feasibleSupport
    );
    const std::size_t maxLocalSupport = std::min(
        std::max(settings.maxLocalSupport, minLocalSupport),
        feasibleSupport
    );
    const float supportDivisor = std::max(settings.supportDivisor, 1.0f);
    const std::size_t localSupport = std::clamp<std::size_t>(
        static_cast<std::size_t>(std::floor(static_cast<float>(finiteValues.size()) / supportDivisor)),
        minLocalSupport,
        maxLocalSupport
    );
    const float supportRadius = std::max(settings.supportRadiusPx, 0.25f);

    auto supportedLow = [&]() -> float {
        std::size_t right = 0;
        for (std::size_t left = 0; left < finiteValues.size(); ++left) {
            while (right < finiteValues.size()
                && finiteValues[right] - finiteValues[left] <= supportRadius) {
                ++right;
            }
            if (right - left >= localSupport) {
                return finiteValues[left];
            }
        }
        return finiteValues.front();
    };

    auto supportedHigh = [&]() -> float {
        for (std::size_t reverseIndex = finiteValues.size(); reverseIndex > 0; --reverseIndex) {
            const std::size_t right = reverseIndex - 1;
            const auto begin = finiteValues.begin();
            const auto end = begin + static_cast<std::ptrdiff_t>(right + 1);
            const auto left = std::lower_bound(begin, end, finiteValues[right] - supportRadius);
            if (static_cast<std::size_t>(std::distance(left, end)) >= localSupport) {
                return finiteValues[right];
            }
        }
        return finiteValues.back();
    };

    float low = supportedLow();
    float high = supportedHigh();
    if (!std::isfinite(low) || !std::isfinite(high) || high <= low) {
        return estimate;
    }

    const float rawWidth = high - low;
    if (rawWidth < 2.0f) {
        return estimate;
    }

    const float pixelCenterToBorderPx = std::clamp(rawWidth * settings.borderRatio, 0.0f, 1.5f);
    low -= pixelCenterToBorderPx;
    high += pixelCenterToBorderPx;

    int supportCount = 0;
    for (float value : finiteValues) {
        if (value >= low && value <= high) {
            ++supportCount;
        }
    }

    if (supportCount < static_cast<int>(minSupportCount)) {
        return estimate;
    }

    estimate.valid = true;
    estimate.low = low;
    estimate.high = high;
    estimate.middle = 0.5f * (low + high);
    estimate.width = high - low;
    estimate.supportCount = supportCount;
    return estimate;
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

float MedianValue(std::vector<float> values) {
    return values.empty() ? 0.0f : Quantile(std::move(values), 0.50f);
}

float MedianAbsoluteDeviation(std::vector<float> values, float median) {
    if (values.empty()) {
        return 0.0f;
    }

    for (float &value : values) {
        value = std::fabs(value - median);
    }

    return MedianValue(std::move(values));
}

bool LocalLinearPrediction(
    const std::vector<float> &xValues,
    const std::vector<float> &yValues,
    std::size_t index,
    int radius,
    float &prediction,
    float &mad) {

    if (xValues.size() != yValues.size() || index >= xValues.size()) {
        return false;
    }

    const int count = static_cast<int>(xValues.size());
    const int begin = std::max(0, static_cast<int>(index) - radius);
    const int end = std::min(count - 1, static_cast<int>(index) + radius);

    int used = 0;
    double meanX = 0.0;
    double meanY = 0.0;
    for (int i = begin; i <= end; ++i) {
        if (i == static_cast<int>(index)) {
            continue;
        }

        meanX += xValues[static_cast<std::size_t>(i)];
        meanY += yValues[static_cast<std::size_t>(i)];
        ++used;
    }

    if (used < 4) {
        return false;
    }

    meanX /= static_cast<double>(used);
    meanY /= static_cast<double>(used);

    double numerator = 0.0;
    double denominator = 0.0;
    for (int i = begin; i <= end; ++i) {
        if (i == static_cast<int>(index)) {
            continue;
        }

        const double dx = static_cast<double>(xValues[static_cast<std::size_t>(i)]) - meanX;
        const double dy = static_cast<double>(yValues[static_cast<std::size_t>(i)]) - meanY;
        numerator += dx * dy;
        denominator += dx * dx;
    }

    const double slope = denominator > 1.0e-9 ? numerator / denominator : 0.0;
    const double intercept = meanY - slope * meanX;
    prediction = static_cast<float>(slope * static_cast<double>(xValues[index]) + intercept);

    std::vector<float> residuals;
    residuals.reserve(static_cast<std::size_t>(used));
    for (int i = begin; i <= end; ++i) {
        if (i == static_cast<int>(index)) {
            continue;
        }

        const double expected = slope * static_cast<double>(xValues[static_cast<std::size_t>(i)]) + intercept;
        residuals.push_back(static_cast<float>(std::fabs(static_cast<double>(yValues[static_cast<std::size_t>(i)]) - expected)));
    }

    mad = MedianValue(std::move(residuals));
    return std::isfinite(prediction);
}

void FilterCoherentRibbonSamples(
    std::vector<PolynomialSample> &middleSamples,
    std::vector<PolynomialSample> &lowSamples,
    std::vector<PolynomialSample> &highSamples,
    std::vector<float> &localWidths) {

    const std::size_t count = middleSamples.size();
    if (count < 7
        || lowSamples.size() != count
        || highSamples.size() != count
        || localWidths.size() != count) {
        return;
    }

    std::vector<float> sValues;
    std::vector<float> middleValues;
    std::vector<float> lowValues;
    std::vector<float> highValues;
    sValues.reserve(count);
    middleValues.reserve(count);
    lowValues.reserve(count);
    highValues.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        sValues.push_back(static_cast<float>(middleSamples[i].x));
        middleValues.push_back(static_cast<float>(middleSamples[i].y));
        lowValues.push_back(static_cast<float>(lowSamples[i].y));
        highValues.push_back(static_cast<float>(highSamples[i].y));
    }

    const float medianWidth = MedianValue(localWidths);
    const float widthMad = MedianAbsoluteDeviation(localWidths, medianWidth);
    const float minEdgeLimit = std::max(2.8f, medianWidth * 0.18f);
    const float minWidthLimit = std::max(3.2f, medianWidth * 0.22f);

    std::vector<char> keep(count, 1);
    for (std::size_t i = 0; i < count; ++i) {
        float predictedHigh = 0.0f;
        float predictedLow = 0.0f;
        float predictedMiddle = 0.0f;
        float predictedWidth = 0.0f;
        float highMad = 0.0f;
        float lowMad = 0.0f;
        float middleMad = 0.0f;
        float localWidthMad = 0.0f;

        const bool hasHighPrediction = LocalLinearPrediction(sValues, highValues, i, 4, predictedHigh, highMad);
        const bool hasLowPrediction = LocalLinearPrediction(sValues, lowValues, i, 4, predictedLow, lowMad);
        const bool hasMiddlePrediction = LocalLinearPrediction(sValues, middleValues, i, 4, predictedMiddle, middleMad);
        const bool hasWidthPrediction = LocalLinearPrediction(sValues, localWidths, i, 4, predictedWidth, localWidthMad);
        if (!hasHighPrediction || !hasLowPrediction || !hasMiddlePrediction || !hasWidthPrediction) {
            continue;
        }

        const float highLimit = std::max(minEdgeLimit, highMad * 5.0f + 1.0f);
        const float lowLimit = std::max(minEdgeLimit, lowMad * 5.0f + 1.0f);
        const float middleLimit = std::max(minEdgeLimit, middleMad * 5.0f + 1.0f);
        const float widthLimit = std::max(minWidthLimit, std::max(widthMad, localWidthMad) * 5.0f + 1.0f);

        const bool highSpike = std::fabs(highValues[i] - predictedHigh) > highLimit;
        const bool lowSpike = std::fabs(lowValues[i] - predictedLow) > lowLimit;
        const bool middleSpike = std::fabs(middleValues[i] - predictedMiddle) > middleLimit;
        const bool widthSpike = std::fabs(localWidths[i] - predictedWidth) > widthLimit;

        if (widthSpike || highSpike || lowSpike || middleSpike) {
            keep[i] = 0;
        }
    }

    std::vector<PolynomialSample> filteredMiddle;
    std::vector<PolynomialSample> filteredLow;
    std::vector<PolynomialSample> filteredHigh;
    std::vector<float> filteredWidths;
    filteredMiddle.reserve(count);
    filteredLow.reserve(count);
    filteredHigh.reserve(count);
    filteredWidths.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        if (!keep[i]) {
            continue;
        }

        filteredMiddle.push_back(middleSamples[i]);
        filteredLow.push_back(lowSamples[i]);
        filteredHigh.push_back(highSamples[i]);
        filteredWidths.push_back(localWidths[i]);
    }

    if (filteredMiddle.size() >= std::max<std::size_t>(7, count / 2)) {
        middleSamples = std::move(filteredMiddle);
        lowSamples = std::move(filteredLow);
        highSamples = std::move(filteredHigh);
        localWidths = std::move(filteredWidths);
    }
}

void FilterTraceWidthSpikes(std::vector<TraceWidthEstimate> &estimates) {
    const std::size_t count = estimates.size();
    if (count < 7) {
        return;
    }

    std::vector<float> sValues;
    std::vector<float> widths;
    sValues.reserve(count);
    widths.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        sValues.push_back(static_cast<float>(i));
        widths.push_back(estimates[i].widthPx);
    }

    const float medianWidth = MedianValue(widths);
    const float widthMad = MedianAbsoluteDeviation(widths, medianWidth);
    const float minWidthLimit = std::max(3.0f, medianWidth * 0.22f);

    std::vector<char> keep(count, 1);
    for (std::size_t i = 0; i < count; ++i) {
        float predictedWidth = 0.0f;
        float localWidthMad = 0.0f;
        if (!LocalLinearPrediction(sValues, widths, i, 3, predictedWidth, localWidthMad)) {
            continue;
        }

        const float widthLimit = std::max(minWidthLimit, std::max(widthMad, localWidthMad) * 5.0f + 1.0f);
        if (std::fabs(widths[i] - predictedWidth) > widthLimit) {
            keep[i] = 0;
        }
    }

    std::vector<TraceWidthEstimate> filtered;
    filtered.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (keep[i]) {
            filtered.push_back(estimates[i]);
        }
    }

    if (filtered.size() >= std::max<std::size_t>(5, count / 2)) {
        estimates = std::move(filtered);
    }
}

float Distance3D(const Vector3 &a, const Vector3 &b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool IsFinite3D(const Vector3 &point) {
    return std::isfinite(point.x)
        && std::isfinite(point.y)
        && std::isfinite(point.z);
}

int FilterTraceWorldOutliers(std::vector<Vector3> &points, std::vector<float> &times) {
    const std::size_t count = points.size();
    if (count < 5 || times.size() != count) {
        return 0;
    }

    std::vector<char> keep(count, 1);
    auto keptCount = [&]() {
        return static_cast<std::size_t>(std::count(keep.begin(), keep.end(), 1));
    };

    auto dropTemporalJumps = [&]() {
        if (count < 3) {
            return 0;
        }

        std::vector<float> stepDistances;
        stepDistances.reserve(count - 1);
        for (std::size_t i = 1; i < count; ++i) {
            if (IsFinite3D(points[i - 1]) && IsFinite3D(points[i])) {
                stepDistances.push_back(Distance3D(points[i - 1], points[i]));
            }
        }

        if (stepDistances.empty()) {
            return 0;
        }

        const float medianStep = std::max(1.0e-4f, MedianValue(stepDistances));
        const float jumpLimit = std::max(0.16f, medianStep * 7.0f);
        const float bridgeLimit = std::max(0.12f, medianStep * 3.5f);
        int removed = 0;

        auto markDrop = [&](std::size_t index) {
            if (keep[index] && keptCount() > std::max<std::size_t>(4, count / 2)) {
                keep[index] = 0;
                ++removed;
            }
        };

        if (Distance3D(points[0], points[1]) > jumpLimit
            && Distance3D(points[1], points[2]) <= bridgeLimit) {
            markDrop(0);
        }

        if (Distance3D(points[count - 1], points[count - 2]) > jumpLimit
            && Distance3D(points[count - 2], points[count - 3]) <= bridgeLimit) {
            markDrop(count - 1);
        }

        for (std::size_t i = 1; i + 1 < count; ++i) {
            if (!keep[i]) {
                continue;
            }

            const float prevDistance = Distance3D(points[i], points[i - 1]);
            const float nextDistance = Distance3D(points[i], points[i + 1]);
            const float bridgeDistance = Distance3D(points[i - 1], points[i + 1]);
            if (prevDistance > jumpLimit
                && nextDistance > jumpLimit
                && bridgeDistance <= bridgeLimit) {
                markDrop(i);
            }
        }

        return removed;
    };

    auto dropFitResiduals = [&]() {
        if (keptCount() < 5) {
            return 0;
        }

        LinearRegression xReg;
        LinearRegression yReg;
        QuadraticRegression zReg;
        for (std::size_t i = 0; i < count; ++i) {
            if (!keep[i] || !IsFinite3D(points[i]) || !std::isfinite(times[i])) {
                continue;
            }

            xReg.add(times[i], points[i].x);
            yReg.add(times[i], points[i].y);
            zReg.add(times[i], points[i].z);
        }

        if (xReg.size() < 5 || yReg.size() < 5 || zReg.size() < 5) {
            return 0;
        }

        const coef xFit = xReg.fit();
        const coef yFit = yReg.fit();
        const coef2 zFit = zReg.fit();

        std::vector<float> residuals;
        residuals.reserve(keptCount());
        for (std::size_t i = 0; i < count; ++i) {
            if (!keep[i]) {
                continue;
            }

            const float t = times[i];
            const Vector3 predicted{
                xFit.a * t + xFit.b,
                yFit.a * t + yFit.b,
                zFit.a * t * t + zFit.b * t + zFit.c
            };
            residuals.push_back(Distance3D(points[i], predicted));
        }

        if (residuals.size() < 5) {
            return 0;
        }

        const float medianResidual = MedianValue(residuals);
        const float residualMad = MedianAbsoluteDeviation(residuals, medianResidual);
        const float threshold = std::max(
            0.10f,
            medianResidual + std::max(0.015f, residualMad) * 6.0f
        );

        std::vector<std::size_t> rejected;
        rejected.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            if (!keep[i]) {
                continue;
            }

            const float t = times[i];
            const Vector3 predicted{
                xFit.a * t + xFit.b,
                yFit.a * t + yFit.b,
                zFit.a * t * t + zFit.b * t + zFit.c
            };
            if (Distance3D(points[i], predicted) > threshold) {
                rejected.push_back(i);
            }
        }

        if (rejected.empty()
            || keptCount() - rejected.size() < std::max<std::size_t>(4, count / 2)) {
            return 0;
        }

        for (std::size_t index : rejected) {
            keep[index] = 0;
        }
        return static_cast<int>(rejected.size());
    };

    int removed = 0;
    for (int iteration = 0; iteration < 3; ++iteration) {
        const int before = removed;
        removed += dropTemporalJumps();
        removed += dropFitResiduals();
        if (removed == before) {
            break;
        }
    }

    if (removed == 0) {
        return 0;
    }

    std::vector<Vector3> filteredPoints;
    std::vector<float> filteredTimes;
    filteredPoints.reserve(keptCount());
    filteredTimes.reserve(keptCount());
    for (std::size_t i = 0; i < count; ++i) {
        if (!keep[i]) {
            continue;
        }
        filteredPoints.push_back(points[i]);
        filteredTimes.push_back(times[i]);
    }

    points = std::move(filteredPoints);
    times = std::move(filteredTimes);
    return removed;
}

Vector2 PointOnRibbon(const TraceRibbonFit &fit, float s, float h) {
    return {
        fit.center.x + fit.direction.x * s + fit.normal.x * h,
        fit.center.y + fit.direction.y * s + fit.normal.y * h
    };
}

float Dot2(const Vector2 &a, const Vector2 &b) {
    return a.x * b.x + a.y * b.y;
}

float Norm2(const Vector2 &v) {
    return std::sqrt(Dot2(v, v));
}

Vector2 NormalizeOr(const Vector2 &v, const Vector2 &fallback) {
    const float norm = Norm2(v);
    if (norm <= 1.0e-6f) {
        return fallback;
    }

    return {v.x / norm, v.y / norm};
}

float TraceFollowLateralWindowPx(float followWindowPx, float circleRadiusPx) {
    return std::clamp(
        std::max({18.0f, circleRadiusPx * 2.6f, followWindowPx * 0.35f}),
        18.0f,
        110.0f
    );
}

TracePcaResult FitTracePca(
    const std::vector<const TracePoint *> &points,
    const Vector2 &fallbackDirection) {

    TracePcaResult result;
    if (points.size() < 2) {
        return result;
    }

    double meanX = 0.0;
    double meanY = 0.0;
    int used = 0;
    for (const TracePoint *point : points) {
        if (point == nullptr
            || !std::isfinite(point->point.x)
            || !std::isfinite(point->point.y)) {
            continue;
        }

        meanX += static_cast<double>(point->point.x);
        meanY += static_cast<double>(point->point.y);
        ++used;
    }

    if (used < 2) {
        return result;
    }

    meanX /= static_cast<double>(used);
    meanY /= static_cast<double>(used);

    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;
    for (const TracePoint *point : points) {
        if (point == nullptr
            || !std::isfinite(point->point.x)
            || !std::isfinite(point->point.y)) {
            continue;
        }

        const double dx = static_cast<double>(point->point.x) - meanX;
        const double dy = static_cast<double>(point->point.y) - meanY;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }

    cxx /= static_cast<double>(used);
    cxy /= static_cast<double>(used);
    cyy /= static_cast<double>(used);

    const double trace = cxx + cyy;
    if (!std::isfinite(trace) || trace <= 1.0e-9) {
        return result;
    }

    const double delta = std::sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
    const double lambda1 = 0.5 * (trace + delta);
    const double lambda2 = 0.5 * (trace - delta);

    Vector2 direction{
        static_cast<float>(cxy),
        static_cast<float>(lambda1 - cxx)
    };

    if (std::fabs(direction.x) + std::fabs(direction.y) < 1.0e-6f) {
        direction = fallbackDirection;
    }

    direction = NormalizeOr(direction, NormalizeOr(fallbackDirection, {1.0f, 0.0f}));
    if (Dot2(direction, fallbackDirection) < 0.0f) {
        direction.x *= -1.0f;
        direction.y *= -1.0f;
    }

    result.valid = true;
    result.center = {static_cast<float>(meanX), static_cast<float>(meanY)};
    result.direction = direction;
    result.lambda1 = lambda1;
    result.lambda2 = lambda2;
    result.eventCount = used;
    return result;
}

std::vector<TracePcaSlice> BuildTemporalPcaSlices(
    const std::vector<TracePoint> &points,
    float requestedSlicePeriodMs,
    int64_t timeOriginUs,
    const Vector2 &fallbackDirection) {

    std::vector<TracePcaSlice> slices;
    if (points.size() < 2) {
        return slices;
    }

    int64_t tMin = points.front().timestampUs;
    int64_t tMax = points.front().timestampUs;
    for (const TracePoint &point : points) {
        tMin = std::min(tMin, point.timestampUs);
        tMax = std::max(tMax, point.timestampUs);
    }

    if (tMax <= tMin) {
        return slices;
    }

    constexpr int kMaxTimedPcaSlices = 160;
    const double requestedPeriodUs =
        static_cast<double>(std::clamp(requestedSlicePeriodMs, 2.0f, 80.0f)) * 1000.0;
    const double durationUs = static_cast<double>(tMax - tMin);
    const int sliceCount = std::clamp(
        static_cast<int>(std::ceil(durationUs / std::max(1.0, requestedPeriodUs))),
        1,
        kMaxTimedPcaSlices
    );
    slices.reserve(static_cast<std::size_t>(sliceCount));

    std::vector<std::vector<const TracePoint *>> slicePoints(static_cast<std::size_t>(sliceCount));
    for (const TracePoint &point : points) {
        double position = static_cast<double>(point.timestampUs - tMin) / durationUs;
        int slice = static_cast<int>(std::floor(position * static_cast<double>(sliceCount)));
        slice = std::clamp(slice, 0, sliceCount - 1);
        slicePoints[static_cast<std::size_t>(slice)].push_back(&point);
    }

    Vector2 previousDirection = NormalizeOr(fallbackDirection, {1.0f, 0.0f});
    for (int slice = 0; slice < sliceCount; ++slice) {
        const auto &currentPoints = slicePoints[static_cast<std::size_t>(slice)];
        const std::size_t minSliceSupport = std::max<std::size_t>(
            12,
            points.size() / static_cast<std::size_t>(sliceCount * 12)
        );
        if (currentPoints.size() < minSliceSupport) {
            continue;
        }

        TracePcaResult pca = FitTracePca(currentPoints, previousDirection);
        if (!pca.valid) {
            continue;
        }

        if (Dot2(pca.direction, previousDirection) < 0.0f) {
            pca.direction.x *= -1.0f;
            pca.direction.y *= -1.0f;
        }

        const int64_t sliceStartUs = tMin + static_cast<int64_t>(
            std::floor(static_cast<double>(slice) * durationUs / static_cast<double>(sliceCount))
        );
        const int64_t sliceEndUs = tMin + static_cast<int64_t>(
            std::ceil(static_cast<double>(slice + 1) * durationUs / static_cast<double>(sliceCount))
        );
        TracePcaSlice pcaSlice;
        pcaSlice.valid = true;
        pcaSlice.center = pca.center;
        pcaSlice.tangent = pca.direction;
        pcaSlice.normal = {-pca.direction.y, pca.direction.x};
        pcaSlice.timeStartSeconds = static_cast<double>(sliceStartUs - timeOriginUs) * 1.0e-6;
        pcaSlice.timeEndSeconds = static_cast<double>(sliceEndUs - timeOriginUs) * 1.0e-6;
        pcaSlice.eventCount = pca.eventCount;
        slices.push_back(pcaSlice);
        previousDirection = pca.direction;
    }

    return slices;
}

Vector2 CombineTemporalPcaDirection(
    const std::vector<TracePcaSlice> &slices,
    const Vector2 &globalDirection) {

    if (slices.size() < 2) {
        return globalDirection;
    }

    Vector2 weightedDirection{0.0f, 0.0f};
    const TracePcaSlice *first = nullptr;
    const TracePcaSlice *last = nullptr;
    for (const TracePcaSlice &slice : slices) {
        if (!slice.valid) {
            continue;
        }

        Vector2 tangent = slice.tangent;
        if (Dot2(tangent, globalDirection) < 0.0f) {
            tangent.x *= -1.0f;
            tangent.y *= -1.0f;
        }

        const float weight = static_cast<float>(std::max(1, slice.eventCount));
        weightedDirection.x += tangent.x * weight;
        weightedDirection.y += tangent.y * weight;

        if (first == nullptr) {
            first = &slice;
        }
        last = &slice;
    }

    if (first == nullptr || last == nullptr || first == last) {
        return globalDirection;
    }

    weightedDirection = NormalizeOr(weightedDirection, globalDirection);
    if (Dot2(weightedDirection, globalDirection) < 0.0f) {
        weightedDirection.x *= -1.0f;
        weightedDirection.y *= -1.0f;
    }

    Vector2 temporalDirection{
        last->center.x - first->center.x,
        last->center.y - first->center.y
    };
    temporalDirection = NormalizeOr(temporalDirection, weightedDirection);
    if (Dot2(temporalDirection, globalDirection) < 0.0f) {
        temporalDirection.x *= -1.0f;
        temporalDirection.y *= -1.0f;
    }

    return NormalizeOr(
        {
            globalDirection.x * 0.45f + weightedDirection.x * 0.35f + temporalDirection.x * 0.20f,
            globalDirection.y * 0.45f + weightedDirection.y * 0.35f + temporalDirection.y * 0.20f
        },
        globalDirection
    );
}

bool SolveSmallLinearSystem(
    std::array<std::array<double, 5>, 5> matrix,
    std::array<double, 5> rhs,
    int size,
    std::array<double, 5> &solution) {

    for (int pivot = 0; pivot < size; ++pivot) {
        int bestRow = pivot;
        double bestValue = std::fabs(matrix[pivot][pivot]);

        for (int row = pivot + 1; row < size; ++row) {
            const double value = std::fabs(matrix[row][pivot]);
            if (value > bestValue) {
                bestValue = value;
                bestRow = row;
            }
        }

        if (bestValue < 1.0e-12) {
            return false;
        }

        if (bestRow != pivot) {
            std::swap(matrix[pivot], matrix[bestRow]);
            std::swap(rhs[pivot], rhs[bestRow]);
        }

        const double divisor = matrix[pivot][pivot];
        for (int col = pivot; col < size; ++col) {
            matrix[pivot][col] /= divisor;
        }
        rhs[pivot] /= divisor;

        for (int row = 0; row < size; ++row) {
            if (row == pivot) {
                continue;
            }

            const double factor = matrix[row][pivot];
            if (std::fabs(factor) < 1.0e-18) {
                continue;
            }

            for (int col = pivot; col < size; ++col) {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    solution = {};
    for (int i = 0; i < size; ++i) {
        solution[i] = rhs[i];
    }

    return true;
}

LocalQuadraticModel MakeLocalQuadraticModel(
    std::vector<PolynomialSample> samples,
    double bandwidth,
    int order) {

    LocalQuadraticModel model;
    model.order = std::clamp(order, 1, 2);
    model.valid = samples.size() >= static_cast<std::size_t>(model.order + 1);
    model.bandwidth = std::max(1.0, bandwidth);
    model.samples = std::move(samples);
    return model;
}

LocalQuadraticFit FitLocalQuadraticAt(const LocalQuadraticModel &model, float s) {
    LocalQuadraticFit fit;
    const int degree = std::clamp(model.order, 1, 2);
    const int fitSize = degree + 1;
    if (!model.valid || model.samples.size() < static_cast<std::size_t>(fitSize)) {
        return fit;
    }

    fit.degree = degree;
    fit.scale = std::max(1.0, model.bandwidth);

    auto solveWeighted = [&](bool localOnly, LocalQuadraticFit &output) {
        std::array<std::array<double, 5>, 5> normal{};
        std::array<double, 5> rhs{};
        int used = 0;
        double supportMin = std::numeric_limits<double>::max();
        double supportMax = std::numeric_limits<double>::lowest();

        for (const PolynomialSample &sample : model.samples) {
            const double x = (sample.x - static_cast<double>(s)) / fit.scale;
            const double distance = std::fabs(x);
            double weight = 0.0;

            if (distance <= 1.0) {
                const double oneMinusD3 = 1.0 - distance * distance * distance;
                weight = oneMinusD3 * oneMinusD3 * oneMinusD3;
            }
            else if (!localOnly) {
                weight = std::exp(-0.5 * distance * distance);
            }

            if (weight <= 1.0e-9) {
                continue;
            }

            const double x2 = x * x;
            const std::array<double, 5> powers{1.0, x, x2, x2 * x, x2 * x2};
            for (int row = 0; row < fitSize; ++row) {
                rhs[row] += weight * sample.y * powers[row];
                for (int col = 0; col < fitSize; ++col) {
                    normal[row][col] += weight * powers[row + col];
                }
            }
            supportMin = std::min(supportMin, sample.x);
            supportMax = std::max(supportMax, sample.x);
            ++used;
        }

        if (used < fitSize) {
            return false;
        }

        for (int i = 0; i < fitSize; ++i) {
            normal[i][i] += 1.0e-8;
        }

        std::array<double, 5> solution{};
        if (!SolveSmallLinearSystem(normal, rhs, fitSize, solution)) {
            return false;
        }

        output.valid = true;
        output.fallback = !localOnly;
        output.degree = degree;
        output.supportCount = used;
        output.supportSpanPx = static_cast<float>(std::max(0.0, supportMax - supportMin));
        output.sparse = output.fallback
            || output.supportCount <= degree + 1
            || output.supportSpanPx < static_cast<float>(fit.scale * 0.35);
        output.scale = fit.scale;
        output.c = solution[0];
        output.b = solution[1];
        output.a = degree >= 2 ? solution[2] : 0.0;
        return true;
    };

    if (solveWeighted(true, fit)) {
        return fit;
    }

    solveWeighted(false, fit);
    return fit;
}

float RibbonCurveH(const LocalQuadraticModel &curve, float s) {
    const LocalQuadraticFit fit = FitLocalQuadraticAt(curve, s);
    return fit.valid ? static_cast<float>(fit.c) : 0.0f;
}

float RibbonMiddleH(const TraceRibbonFit &fit, float s) {
    return RibbonCurveH(fit.middleModel, s);
}

struct GlobalPolynomialModel {
    bool valid = false;
    int degree = 2;
    double c = 0.0;
    double b = 0.0;
    double a = 0.0;
};

double EvaluateGlobalPolynomial(const GlobalPolynomialModel &model, double x) {
    return model.c + model.b * x + model.a * x * x;
}

bool FitGlobalPolynomial(
    const std::vector<PolynomialSample> &samples,
    const std::vector<char> &mask,
    int degree,
    GlobalPolynomialModel &model) {

    degree = std::clamp(degree, 1, 2);
    const int fitSize = degree + 1;

    std::array<std::array<double, 5>, 5> normal{};
    std::array<double, 5> rhs{};
    int used = 0;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (!mask.empty() && !mask[i]) {
            continue;
        }

        const double x = samples[i].x;
        const double y = samples[i].y;
        const double x2 = x * x;
        const std::array<double, 5> powers{1.0, x, x2, x2 * x, x2 * x2};

        for (int row = 0; row < fitSize; ++row) {
            rhs[row] += y * powers[row];
            for (int col = 0; col < fitSize; ++col) {
                normal[row][col] += powers[row + col];
            }
        }
        ++used;
    }

    if (used < fitSize) {
        return false;
    }

    for (int i = 0; i < fitSize; ++i) {
        normal[i][i] += 1.0e-8;
    }

    std::array<double, 5> solution{};
    if (!SolveSmallLinearSystem(normal, rhs, fitSize, solution)) {
        return false;
    }

    model.valid = true;
    model.degree = degree;
    model.c = solution[0];
    model.b = solution[1];
    model.a = degree >= 2 ? solution[2] : 0.0;
    return true;
}

int FilterRibbonSamplesRansac(
    std::vector<PolynomialSample> &middleSamples,
    std::vector<PolynomialSample> &lowSamples,
    std::vector<PolynomialSample> &highSamples,
    std::vector<float> &localWidths) {

    const std::size_t count = middleSamples.size();
    if (count < 10
        || lowSamples.size() != count
        || highSamples.size() != count
        || localWidths.size() != count) {
        return 0;
    }

    const int degree = count >= 12 ? 2 : 1;
    const int fitSize = degree + 1;
    const float medianWidth = MedianValue(localWidths);
    const float widthMad = MedianAbsoluteDeviation(localWidths, medianWidth);
    const float residualThreshold = std::max(4.0f, medianWidth * 0.34f);

    GlobalPolynomialModel bestModel;
    int bestInlierCount = 0;
    double bestError = std::numeric_limits<double>::max();

    auto evaluateCandidate = [&](const std::vector<char> &candidateMask) {
        GlobalPolynomialModel candidate;
        if (!FitGlobalPolynomial(middleSamples, candidateMask, degree, candidate)) {
            return;
        }

        int inlierCount = 0;
        double error = 0.0;
        for (const PolynomialSample &sample : middleSamples) {
            const double residual = std::fabs(sample.y - EvaluateGlobalPolynomial(candidate, sample.x));
            if (residual <= residualThreshold) {
                ++inlierCount;
                error += residual;
            }
            else {
                error += residualThreshold * 2.0;
            }
        }

        const bool better =
            inlierCount > bestInlierCount
            || (inlierCount == bestInlierCount && error < bestError);
        if (better) {
            bestModel = candidate;
            bestInlierCount = inlierCount;
            bestError = error;
        }
    };

    std::vector<char> allMask(count, 1);
    evaluateCandidate(allMask);

    std::uint32_t state = 2166136261u ^ static_cast<std::uint32_t>(count * 16777619u);
    auto nextIndex = [&]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<std::size_t>(state % static_cast<std::uint32_t>(count));
    };

    constexpr int kIterations = 96;
    for (int iteration = 0; iteration < kIterations; ++iteration) {
        std::vector<char> candidateMask(count, 0);
        int selected = 0;
        int guard = 0;
        while (selected < fitSize && guard < 32) {
            const std::size_t index = nextIndex();
            if (!candidateMask[index]) {
                candidateMask[index] = 1;
                ++selected;
            }
            ++guard;
        }

        if (selected == fitSize) {
            evaluateCandidate(candidateMask);
        }
    }

    if (!bestModel.valid || bestInlierCount < std::max<int>(7, static_cast<int>(count * 0.55f))) {
        return 0;
    }

    std::vector<float> inlierResiduals;
    inlierResiduals.reserve(count);
    for (const PolynomialSample &sample : middleSamples) {
        const float residual = static_cast<float>(
            std::fabs(sample.y - EvaluateGlobalPolynomial(bestModel, sample.x))
        );
        if (residual <= residualThreshold) {
            inlierResiduals.push_back(residual);
        }
    }

    const float residualMedian = MedianValue(inlierResiduals);
    const float residualMad = MedianAbsoluteDeviation(inlierResiduals, residualMedian);
    const float finalResidualThreshold = std::max(
        residualThreshold,
        residualMedian + residualMad * 5.0f + 1.0f
    );
    const float widthThreshold = std::max(
        medianWidth * 0.65f,
        widthMad * 6.0f + 2.0f
    );

    std::vector<char> finalMask(count, 0);
    for (std::size_t i = 0; i < count; ++i) {
        const float residual = static_cast<float>(
            std::fabs(middleSamples[i].y - EvaluateGlobalPolynomial(bestModel, middleSamples[i].x))
        );
        const float widthResidual = std::fabs(localWidths[i] - medianWidth);
        if (residual <= finalResidualThreshold && widthResidual <= widthThreshold) {
            finalMask[i] = 1;
        }
    }

    GlobalPolynomialModel refinedModel;
    if (FitGlobalPolynomial(middleSamples, finalMask, degree, refinedModel)) {
        bestModel = refinedModel;
        for (std::size_t i = 0; i < count; ++i) {
            const float residual = static_cast<float>(
                std::fabs(middleSamples[i].y - EvaluateGlobalPolynomial(bestModel, middleSamples[i].x))
            );
            const float widthResidual = std::fabs(localWidths[i] - medianWidth);
            finalMask[i] = (residual <= finalResidualThreshold && widthResidual <= widthThreshold) ? 1 : 0;
        }
    }

    std::size_t keptCount = 0;
    for (char keep : finalMask) {
        if (keep) {
            ++keptCount;
        }
    }

    if (keptCount < std::max<std::size_t>(7, count / 2)) {
        return 0;
    }

    std::vector<PolynomialSample> filteredMiddle;
    std::vector<PolynomialSample> filteredLow;
    std::vector<PolynomialSample> filteredHigh;
    std::vector<float> filteredWidths;
    filteredMiddle.reserve(keptCount);
    filteredLow.reserve(keptCount);
    filteredHigh.reserve(keptCount);
    filteredWidths.reserve(keptCount);

    for (std::size_t i = 0; i < count; ++i) {
        if (!finalMask[i]) {
            continue;
        }
        filteredMiddle.push_back(middleSamples[i]);
        filteredLow.push_back(lowSamples[i]);
        filteredHigh.push_back(highSamples[i]);
        filteredWidths.push_back(localWidths[i]);
    }

    const int removed = static_cast<int>(count - keptCount);
    if (removed > 0) {
        middleSamples = std::move(filteredMiddle);
        lowSamples = std::move(filteredLow);
        highSamples = std::move(filteredHigh);
        localWidths = std::move(filteredWidths);
    }

    return removed;
}

Vector2 RibbonCoordinates(const TraceRibbonFit &fit, const Vector2 &point) {
    const Vector2 delta{
        point.x - fit.center.x,
        point.y - fit.center.y
    };

    return {
        Dot2(delta, fit.direction),
        Dot2(delta, fit.normal)
    };
}

Vector2 RibbonCurvePoint(const TraceRibbonFit &fit, const LocalQuadraticModel &curve, float s) {
    return PointOnRibbon(fit, s, RibbonCurveH(curve, s));
}

Vector2 RibbonTangent(const TraceRibbonFit &fit, float s) {
    const LocalQuadraticFit localFit = FitLocalQuadraticAt(fit.middleModel, s);
    const float dhds = localFit.valid ? static_cast<float>(localFit.b / localFit.scale) : 0.0f;
    return NormalizeOr(
        {
            fit.direction.x + fit.normal.x * dhds,
            fit.direction.y + fit.normal.y * dhds
        },
        fit.direction
    );
}

bool CurveIntersectionOnNormal(
    const TraceRibbonFit &fit,
    const LocalQuadraticModel &curve,
    float referenceS,
    const Vector2 &middle,
    const Vector2 &lineTangent,
    Vector2 &intersection) {

    if (!curve.valid) {
        return false;
    }

    auto lineValue = [&](float s) {
        const Vector2 point = RibbonCurvePoint(fit, curve, s);
        return Dot2({point.x - middle.x, point.y - middle.y}, lineTangent);
    };

    constexpr int kSearchSteps = 64;
    const float searchRadius = std::max(12.0f, fit.widthPx * 4.0f);
    const float s0 = std::max(fit.sMin, referenceS - searchRadius);
    const float s1 = std::min(fit.sMax, referenceS + searchRadius);

    if (s1 <= s0) {
        return false;
    }

    float bestS = referenceS;
    float bestLineError = std::numeric_limits<float>::max();
    bool found = false;

    auto rememberCandidate = [&](float s, float value) {
        const float lineError = std::fabs(value);
        const float referenceBias = 0.05f * std::fabs(s - referenceS);
        const float score = lineError + referenceBias;
        if (score < bestLineError) {
            bestLineError = score;
            bestS = s;
            found = true;
        }
    };

    float previousS = s0;
    float previousValue = lineValue(previousS);
    rememberCandidate(previousS, previousValue);

    for (int i = 1; i <= kSearchSteps; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(kSearchSteps);
        float currentS = s0 + alpha * (s1 - s0);
        float currentValue = lineValue(currentS);
        rememberCandidate(currentS, currentValue);

        if (std::isfinite(previousValue)
            && std::isfinite(currentValue)
            && previousValue * currentValue <= 0.0f) {

            float lo = previousS;
            float hi = currentS;
            float loValue = previousValue;

            for (int iter = 0; iter < 18; ++iter) {
                const float mid = 0.5f * (lo + hi);
                const float midValue = lineValue(mid);

                if (loValue * midValue <= 0.0f) {
                    hi = mid;
                }
                else {
                    lo = mid;
                    loValue = midValue;
                }
            }

            const float root = 0.5f * (lo + hi);
            rememberCandidate(root, lineValue(root));
        }

        previousS = currentS;
        previousValue = currentValue;
    }

    if (!found) {
        return false;
    }

    intersection = RibbonCurvePoint(fit, curve, bestS);
    return true;
}

Vector3 TraceImagePointToWorldMeters(
    const Vector2 &imagePoint,
    float widthPx,
    const Vector2 &widthDirection,
    const CalibrationData &calibration,
    float ballRadiusMm) {

    if (!calibration.ready || widthPx <= 1.0e-3f || ballRadiusMm <= 0.0f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float fx = static_cast<float>(calibration.fx());
    const float fy = static_cast<float>(calibration.fy());
    const float cx = static_cast<float>(calibration.cx());
    const float cy = static_cast<float>(calibration.cy());

    if (std::fabs(fx) <= 1.0e-6f || std::fabs(fy) <= 1.0e-6f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float widthNorm = std::sqrt(widthDirection.x * widthDirection.x + widthDirection.y * widthDirection.y);
    if (widthNorm <= 1.0e-6f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float nx = widthDirection.x / widthNorm;
    const float ny = widthDirection.y / widthNorm;
    const float fEff = std::sqrt((fx * nx) * (fx * nx) + (fy * ny) * (fy * ny));
    const float diameterMm = 2.0f * ballRadiusMm;
    const float depthMm = fEff * diameterMm / widthPx;

    if (!std::isfinite(depthMm) || depthMm <= 0.0f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float xMm = ((imagePoint.x - cx) / fx) * depthMm;
    const float yMm = ((imagePoint.y - cy) / fy) * depthMm;

    const cv::Point3f cameraPositionMm{xMm, yMm, depthMm};
    return {
        cameraPositionMm.x * 1.0e-3f,
        cameraPositionMm.z * 1.0e-3f,
        -cameraPositionMm.y * 1.0e-3f
    };
}

std::vector<Vector2> BuildRibbonCurve(
    const TraceRibbonFit &fit,
    const LocalQuadraticModel &curve,
    int segments,
    std::vector<Vector2> *sparseFitPoints = nullptr) {

    std::vector<Vector2> output;
    output.reserve(static_cast<std::size_t>(segments + 1));

    for (int i = 0; i <= segments; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(segments);
        const float s = fit.sMin + alpha * (fit.sMax - fit.sMin);
        const LocalQuadraticFit localFit = FitLocalQuadraticAt(curve, s);
        const float h = localFit.valid ? static_cast<float>(localFit.c) : 0.0f;
        const Vector2 point = PointOnRibbon(fit, s, h);
        output.push_back(point);

        if (sparseFitPoints != nullptr && (!localFit.valid || localFit.sparse)) {
            sparseFitPoints->push_back(point);
        }
    }

    return output;
}

LocalTraceSection EstimateLocalFlowSection(
    const TraceRibbonFit &fit,
    const std::vector<const TraceProjection *> &timeLocal,
    float fallbackS,
    double sampleTime,
    double timeStep,
    float halfTangentWindow) {

    LocalTraceSection section;
    if (timeLocal.size() < 20) {
        return section;
    }

    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> ss;
    xs.reserve(timeLocal.size());
    ys.reserve(timeLocal.size());
    ss.reserve(timeLocal.size());

    double meanTime = 0.0;
    double meanX = 0.0;
    double meanY = 0.0;

    for (const TraceProjection *projection : timeLocal) {
        xs.push_back(projection->point.x);
        ys.push_back(projection->point.y);
        ss.push_back(projection->s);
        meanTime += projection->timeSeconds;
        meanX += projection->point.x;
        meanY += projection->point.y;
    }

    const double invCount = 1.0 / static_cast<double>(timeLocal.size());
    meanTime *= invCount;
    meanX *= invCount;
    meanY *= invCount;

    const Vector2 robustCenter{
        Quantile(xs, 0.50f),
        Quantile(ys, 0.50f)
    };
    const float medianS = std::clamp(Quantile(ss, 0.50f), fit.sMin, fit.sMax);
    const float referenceS = std::clamp(
        std::isfinite(medianS) ? medianS : fallbackS,
        fit.sMin,
        fit.sMax
    );

    double timeVariance = 0.0;
    double covXTime = 0.0;
    double covYTime = 0.0;
    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;

    for (const TraceProjection *projection : timeLocal) {
        const double dt = projection->timeSeconds - meanTime;
        const double dx = static_cast<double>(projection->point.x) - meanX;
        const double dy = static_cast<double>(projection->point.y) - meanY;

        timeVariance += dt * dt;
        covXTime += dt * dx;
        covYTime += dt * dy;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }

    Vector2 geometricTangent = fit.direction;
    if (fit.middleModel.valid) {
        geometricTangent = RibbonTangent(fit, referenceS);
    }

    Vector2 tangent = geometricTangent;
    bool hasFlowTangent = false;

    if (timeVariance > 1.0e-10) {
        Vector2 flowDirection{
            static_cast<float>(covXTime / timeVariance),
            static_cast<float>(covYTime / timeVariance)
        };
        const float flowSpeed = Norm2(flowDirection);

        if (flowSpeed > 1.0e-3f) {
            flowDirection = NormalizeOr(flowDirection, geometricTangent);
            if (Dot2(flowDirection, geometricTangent) < 0.0f) {
                flowDirection.x *= -1.0f;
                flowDirection.y *= -1.0f;
            }

            const float flowDisplacement = flowSpeed * static_cast<float>(std::max(timeStep, 1.0e-6));
            const float flowWeight = std::clamp(
                flowDisplacement / std::max(3.0f, fit.widthPx),
                0.0f,
                0.90f
            );

            tangent = NormalizeOr(
                {
                    geometricTangent.x * (1.0f - flowWeight) + flowDirection.x * flowWeight,
                    geometricTangent.y * (1.0f - flowWeight) + flowDirection.y * flowWeight
                },
                flowDirection
            );
            hasFlowTangent = true;
        }
    }

    if (!hasFlowTangent) {
        const double trace = cxx + cyy;
        const double delta = std::sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
        const double lambda1 = 0.5 * (trace + delta);

        Vector2 pcaTangent{
            static_cast<float>(cxy),
            static_cast<float>(lambda1 - cxx)
        };

        pcaTangent = NormalizeOr(pcaTangent, geometricTangent);
        if (Dot2(pcaTangent, geometricTangent) < 0.0f) {
            pcaTangent.x *= -1.0f;
            pcaTangent.y *= -1.0f;
        }

        tangent = NormalizeOr(
            {
                geometricTangent.x * 0.35f + pcaTangent.x * 0.65f,
                geometricTangent.y * 0.35f + pcaTangent.y * 0.65f
            },
            geometricTangent
        );
    }

    Vector2 normal{-tangent.y, tangent.x};
    if (Dot2(normal, fit.normal) < 0.0f) {
        normal.x *= -1.0f;
        normal.y *= -1.0f;
    }

    const Vector2 sectionCenter = fit.middleModel.valid
        ? PointOnRibbon(fit, referenceS, RibbonMiddleH(fit, referenceS))
        : robustCenter;

    std::vector<float> normalOffsets;
    normalOffsets.reserve(timeLocal.size());

    for (const TraceProjection *projection : timeLocal) {
        const Vector2 delta{
            projection->point.x - sectionCenter.x,
            projection->point.y - sectionCenter.y
        };
        const float tangentOffset = Dot2(delta, tangent);
        if (std::fabs(tangentOffset) > halfTangentWindow) {
            continue;
        }

        normalOffsets.push_back(Dot2(delta, normal));
    }

    if (normalOffsets.size() < 12) {
        return section;
    }

    const float medianOffset = Quantile(normalOffsets, 0.50f);
    std::vector<float> absoluteDeviations;
    absoluteDeviations.reserve(normalOffsets.size());
    for (float offset : normalOffsets) {
        absoluteDeviations.push_back(std::fabs(offset - medianOffset));
    }

    const float robustSigma = 1.4826f * Quantile(absoluteDeviations, 0.50f);
    const float trimLimit = std::max(6.0f, std::max(fit.widthPx * 0.95f, robustSigma * 3.0f));

    std::vector<float> trimmedOffsets;
    trimmedOffsets.reserve(normalOffsets.size());
    for (float offset : normalOffsets) {
        if (std::fabs(offset - medianOffset) <= trimLimit) {
            trimmedOffsets.push_back(offset);
        }
    }

    if (trimmedOffsets.size() < 12) {
        trimmedOffsets = std::move(normalOffsets);
    }

    const TraceEdgeEstimate edgeEstimate = EstimateSupportedEdges(
        trimmedOffsets,
        12,
        fit.supportEdge
    );
    if (!edgeEstimate.valid) {
        return section;
    }

    const float eventLow = edgeEstimate.low;
    const float eventHigh = edgeEstimate.high;
    const float widthPx = eventHigh - eventLow;
    if (!std::isfinite(widthPx) || widthPx < 1.0f) {
        return section;
    }

    const float middleOffset = 0.5f * (eventLow + eventHigh);
    const Vector2 middle{
        sectionCenter.x + normal.x * middleOffset,
        sectionCenter.y + normal.y * middleOffset
    };
    Vector2 endA{
        sectionCenter.x + normal.x * eventLow,
        sectionCenter.y + normal.y * eventLow
    };
    Vector2 endB{
        sectionCenter.x + normal.x * eventHigh,
        sectionCenter.y + normal.y * eventHigh
    };

    if (endA.y < endB.y) {
        section.upper = endA;
        section.lower = endB;
    }
    else {
        section.upper = endB;
        section.lower = endA;
    }

    const Vector2 ribbonMiddle = RibbonCoordinates(fit, middle);
    section.valid = true;
    section.middle = middle;
    section.tangent = tangent;
    section.normal = NormalizeOr(
        {
            section.upper.x - section.lower.x,
            section.upper.y - section.lower.y
        },
        normal
    );
    section.widthPx = widthPx;
    section.s = std::clamp(ribbonMiddle.x, fit.sMin, fit.sMax);
    section.timeSeconds = sampleTime;
    section.localEventCount = edgeEstimate.supportCount;
    return section;
}

std::vector<TraceWidthEstimate> EstimateTraceWidths(const TraceRibbonFit &fit, int sampleCount) {
    std::vector<TraceWidthEstimate> estimates;
    if (!fit.valid || fit.projections.empty() || sampleCount < 1) {
        return estimates;
    }

    double tMin = std::numeric_limits<double>::max();
    double tMax = std::numeric_limits<double>::lowest();
    for (const TraceProjection &projection : fit.projections) {
        tMin = std::min(tMin, projection.timeSeconds);
        tMax = std::max(tMax, projection.timeSeconds);
    }

    if (!std::isfinite(tMin) || !std::isfinite(tMax) || tMax <= tMin) {
        return estimates;
    }

    const double timeStep = (tMax - tMin) / static_cast<double>(sampleCount);
    const double halfTimeWindow = std::max(timeStep * 0.75, 1.0e-4);
    const float halfTangentWindow = std::max(
        8.0f,
        std::max(fit.widthPx * 1.35f, fit.lengthPx / static_cast<float>(sampleCount * 2))
    );
    const float roughHalfWidthLimit = std::max(12.0f, fit.widthPx * 1.8f);

    estimates.reserve(static_cast<std::size_t>(sampleCount));

    for (int sample = 0; sample < sampleCount; ++sample) {
        const double sampleTime = tMin + (static_cast<double>(sample) + 0.5) * timeStep;

        std::vector<const TraceProjection *> timeLocal;
        timeLocal.reserve(fit.projections.size() / static_cast<std::size_t>(sampleCount) + 16);

        for (const TraceProjection &projection : fit.projections) {
            if (std::fabs(projection.timeSeconds - sampleTime) > halfTimeWindow) {
                continue;
            }

            const float middleH = RibbonMiddleH(fit, projection.s);
            if (std::fabs(projection.h - middleH) <= roughHalfWidthLimit) {
                timeLocal.push_back(&projection);
            }
        }

        if (timeLocal.size() < 20) {
            continue;
        }

        std::vector<float> localS;
        localS.reserve(timeLocal.size());
        for (const TraceProjection *projection : timeLocal) {
            localS.push_back(projection->s);
        }

        const float sampleS = std::clamp(Quantile(localS, 0.50f), fit.sMin, fit.sMax);

        const LocalTraceSection localSection = EstimateLocalFlowSection(
            fit,
            timeLocal,
            sampleS,
            sampleTime,
            timeStep,
            halfTangentWindow
        );

        const Vector2 sectionMiddle = PointOnRibbon(fit, sampleS, RibbonMiddleH(fit, sampleS));
        const Vector2 geometricTangent = RibbonTangent(fit, sampleS);
        Vector2 geometricNormal{-geometricTangent.y, geometricTangent.x};
        if (Dot2(geometricNormal, fit.normal) < 0.0f) {
            geometricNormal.x *= -1.0f;
            geometricNormal.y *= -1.0f;
        }

        std::vector<float> normalOffsets;
        normalOffsets.reserve(timeLocal.size());

        for (const TraceProjection *projection : timeLocal) {
            const Vector2 delta{
                projection->point.x - sectionMiddle.x,
                projection->point.y - sectionMiddle.y
            };

            const float tangentOffset = Dot2(delta, geometricTangent);
            if (std::fabs(tangentOffset) > halfTangentWindow) {
                continue;
            }

            normalOffsets.push_back(Dot2(delta, geometricNormal));
        }

        if (normalOffsets.size() < 12) {
            continue;
        }

        const TraceEdgeEstimate edgeEstimate = EstimateSupportedEdges(
            normalOffsets,
            12,
            fit.supportEdge
        );
        if (!edgeEstimate.valid) {
            continue;
        }

        const float eventLow = edgeEstimate.low;
        const float eventHigh = edgeEstimate.high;
        const float eventWidth = eventHigh - eventLow;

        if (!std::isfinite(eventWidth) || eventWidth < 2.0f) {
            continue;
        }

        Vector2 endA{
            sectionMiddle.x + geometricNormal.x * eventLow,
            sectionMiddle.y + geometricNormal.y * eventLow
        };
        Vector2 endB{
            sectionMiddle.x + geometricNormal.x * eventHigh,
            sectionMiddle.y + geometricNormal.y * eventHigh
        };
        float endAOffset = eventLow;
        float endBOffset = eventHigh;

        Vector2 upperIntersection{};
        Vector2 lowerIntersection{};
        const bool hasUpperIntersection = CurveIntersectionOnNormal(
            fit,
            fit.upperModel,
            sampleS,
            sectionMiddle,
            geometricTangent,
            upperIntersection
        );
        const bool hasLowerIntersection = CurveIntersectionOnNormal(
            fit,
            fit.lowerModel,
            sampleS,
            sectionMiddle,
            geometricTangent,
            lowerIntersection
        );

        float widthPx = eventWidth;
        if (hasUpperIntersection && hasLowerIntersection) {
            const Vector2 upperDelta{
                upperIntersection.x - sectionMiddle.x,
                upperIntersection.y - sectionMiddle.y
            };
            const Vector2 lowerDelta{
                lowerIntersection.x - sectionMiddle.x,
                lowerIntersection.y - sectionMiddle.y
            };
            const float upperOffset = Dot2(upperDelta, geometricNormal);
            const float lowerOffset = Dot2(lowerDelta, geometricNormal);
            const float envelopeWidth = std::fabs(upperOffset - lowerOffset);
            const float widthRatio = envelopeWidth / eventWidth;

            if (std::isfinite(envelopeWidth)
                && envelopeWidth >= 2.0f
                && widthRatio >= 0.55f
                && widthRatio <= 1.70f) {

                widthPx = envelopeWidth;
                endAOffset = upperOffset;
                endBOffset = lowerOffset;
                endA = {
                    sectionMiddle.x + geometricNormal.x * endAOffset,
                    sectionMiddle.y + geometricNormal.y * endAOffset
                };
                endB = {
                    sectionMiddle.x + geometricNormal.x * endBOffset,
                    sectionMiddle.y + geometricNormal.y * endBOffset
                };
            }
        }

        if (!std::isfinite(widthPx) || widthPx < 1.0f) {
            continue;
        }

        TraceWidthEstimate estimate;
        estimate.valid = true;
        estimate.middle = sectionMiddle;
        estimate.widthPx = widthPx;
        estimate.timeSeconds = sampleTime;
        estimate.localEventCount = localSection.valid ? localSection.localEventCount : edgeEstimate.supportCount;
        estimate.tangent = geometricTangent;

        if (endA.y < endB.y) {
            estimate.upper = endA;
            estimate.lower = endB;
        }
        else {
            estimate.upper = endB;
            estimate.lower = endA;
        }
        estimate.normal = geometricNormal;

        estimates.push_back(estimate);
    }

    FilterTraceWidthSpikes(estimates);
    return estimates;
}

TraceRibbonFit FitTraceRibbon(
    const std::vector<TracePoint> &points,
    float lineBinWidthPx,
    float localWindowPx,
    int localOrder,
    float pcaPeriodMs,
    const TraceSupportEdgeSettings &supportEdge) {

    TraceRibbonFit fit;

    constexpr std::size_t kMinRawEvents = 500;
    constexpr std::size_t kMinInlierEvents = 250;
    constexpr int kMinValidBins = 7;
    constexpr int kSparseDiagnosticSegments = 90;
    const float requestedLineBinWidth = std::clamp(lineBinWidthPx, 4.0f, 48.0f);
    const float localWindow = std::clamp(localWindowPx, 8.0f, 240.0f);
    const int fitOrder = std::clamp(localOrder, 1, 2);
    const float timedPcaPeriodMs = std::clamp(pcaPeriodMs, 2.0f, 80.0f);
    TraceSupportEdgeSettings edgeSettings = supportEdge;
    edgeSettings.supportDivisor = std::clamp(edgeSettings.supportDivisor, 8.0f, 60.0f);
    edgeSettings.minLocalSupport = std::clamp<std::size_t>(edgeSettings.minLocalSupport, 1, 20);
    edgeSettings.maxLocalSupport = std::clamp<std::size_t>(
        std::max(edgeSettings.maxLocalSupport, edgeSettings.minLocalSupport),
        edgeSettings.minLocalSupport,
        30
    );
    edgeSettings.supportRadiusPx = std::clamp(edgeSettings.supportRadiusPx, 0.5f, 4.0f);
    edgeSettings.borderRatio = std::clamp(edgeSettings.borderRatio, 0.0f, 0.10f);

    if (points.size() < kMinRawEvents) {
        return fit;
    }

    int64_t timeOriginUs = points.front().timestampUs;
    for (const TracePoint &p : points) {
        timeOriginUs = std::min(timeOriginUs, p.timestampUs);
    }

    std::vector<float> xs;
    std::vector<float> ys;
    xs.reserve(points.size());
    ys.reserve(points.size());

    for (const TracePoint &p : points) {
        xs.push_back(p.point.x);
        ys.push_back(p.point.y);
    }

    const Vector2 robustCenter{
        Quantile(xs, 0.50f),
        Quantile(ys, 0.50f)
    };

    std::vector<float> squaredDistances;
    squaredDistances.reserve(points.size());
    for (const TracePoint &p : points) {
        const float dx = p.point.x - robustCenter.x;
        const float dy = p.point.y - robustCenter.y;
        squaredDistances.push_back(dx * dx + dy * dy);
    }

    const float radialLimit = Quantile(squaredDistances, 0.97f);

    std::vector<TracePoint> inliers;
    inliers.reserve(points.size());
    for (const TracePoint &p : points) {
        const float dx = p.point.x - robustCenter.x;
        const float dy = p.point.y - robustCenter.y;
        if (dx * dx + dy * dy <= radialLimit) {
            inliers.push_back(p);
        }
    }

    if (inliers.size() < kMinInlierEvents) {
        return fit;
    }

    std::vector<const TracePoint *> inlierRefs;
    inlierRefs.reserve(inliers.size());
    for (const TracePoint &p : inliers) {
        inlierRefs.push_back(&p);
    }

    const TracePcaResult globalPca = FitTracePca(inlierRefs, {1.0f, 0.0f});
    if (!globalPca.valid) {
        return fit;
    }

    std::vector<TracePcaSlice> temporalPcaSlices = BuildTemporalPcaSlices(
        inliers,
        timedPcaPeriodMs,
        timeOriginUs,
        globalPca.direction
    );
    Vector2 direction = temporalPcaSlices.size() > 1
        ? CombineTemporalPcaDirection(temporalPcaSlices, globalPca.direction)
        : globalPca.direction;
    direction = NormalizeOr(direction, globalPca.direction);

    const Vector2 center = globalPca.center;
    const Vector2 normal{-direction.y, direction.x};

    for (TracePcaSlice &slice : temporalPcaSlices) {
        if (Dot2(slice.tangent, direction) < 0.0f) {
            slice.tangent.x *= -1.0f;
            slice.tangent.y *= -1.0f;
        }
        slice.normal = {-slice.tangent.y, slice.tangent.x};
    }

    std::vector<float> sValues;
    std::vector<float> hValues;
    sValues.reserve(inliers.size());
    hValues.reserve(inliers.size());

    for (const TracePoint &p : inliers) {
        const float dx = p.point.x - center.x;
        const float dy = p.point.y - center.y;
        sValues.push_back(direction.x * dx + direction.y * dy);
        hValues.push_back(normal.x * dx + normal.y * dy);
    }

    fit.center = center;
    fit.direction = direction;
    fit.normal = normal;
    fit.supportEdge = edgeSettings;
    fit.pcaPeriodMs = timedPcaPeriodMs;
    fit.temporalPcaSlices = std::move(temporalPcaSlices);
    fit.sMin = Quantile(sValues, 0.03f);
    fit.sMax = Quantile(sValues, 0.97f);
    fit.lengthPx = fit.sMax - fit.sMin;
    fit.inlierCount = inliers.size();

    if (fit.lengthPx < 35.0f) {
        return fit;
    }

    std::vector<TraceProjection> projections;
    projections.reserve(inliers.size());

    for (std::size_t i = 0; i < sValues.size(); ++i) {
        if (sValues[i] >= fit.sMin && sValues[i] <= fit.sMax) {
            projections.push_back({
                sValues[i],
                hValues[i],
                inliers[i].point,
                static_cast<double>(inliers[i].timestampUs - timeOriginUs) * 1.0e-6
            });
        }
    }

    if (projections.size() < kMinInlierEvents) {
        return fit;
    }

    const int binCount = std::clamp(
        static_cast<int>(std::ceil(fit.lengthPx / requestedLineBinWidth)),
        8,
        260
    );
    fit.lineBinCount = binCount;
    fit.lineBinWidthPx = fit.lengthPx / static_cast<float>(binCount);

    std::vector<std::vector<float>> binS(static_cast<std::size_t>(binCount));
    std::vector<std::vector<float>> binH(static_cast<std::size_t>(binCount));

    const float binWidth = fit.lengthPx / static_cast<float>(binCount);
    if (binWidth <= 1.0e-6f) {
        return fit;
    }

    for (const TraceProjection &projection : projections) {
        int bin = static_cast<int>((projection.s - fit.sMin) / binWidth);
        bin = std::clamp(bin, 0, binCount - 1);
        binS[static_cast<std::size_t>(bin)].push_back(projection.s);
        binH[static_cast<std::size_t>(bin)].push_back(projection.h);
    }

    const std::size_t minEventsPerBin = std::max<std::size_t>(
        10,
        projections.size() / static_cast<std::size_t>(binCount * 10)
    );

    std::vector<PolynomialSample> middleSamples;
    std::vector<PolynomialSample> lowSamples;
    std::vector<PolynomialSample> highSamples;
    std::vector<float> localWidths;

    int validBins = 0;
    for (int bin = 0; bin < binCount; ++bin) {
        std::vector<float> &sBin = binS[static_cast<std::size_t>(bin)];
        std::vector<float> &hBin = binH[static_cast<std::size_t>(bin)];

        if (hBin.size() < minEventsPerBin) {
            continue;
        }

        const float s = Quantile(sBin, 0.50f);
        const TraceEdgeEstimate edgeEstimate = EstimateSupportedEdges(
            hBin,
            minEventsPerBin,
            edgeSettings
        );
        if (!edgeEstimate.valid) {
            continue;
        }

        const float hLow = edgeEstimate.low;
        const float hMiddle = edgeEstimate.middle;
        const float hHigh = edgeEstimate.high;
        const float localWidth = hHigh - hLow;

        if (!std::isfinite(localWidth) || localWidth < 2.0f) {
            continue;
        }

        middleSamples.push_back({s, hMiddle});
        lowSamples.push_back({s, hLow});
        highSamples.push_back({s, hHigh});
        localWidths.push_back(localWidth);
        ++validBins;
    }

    fit.ransacRejectedSamples = FilterRibbonSamplesRansac(
        middleSamples,
        lowSamples,
        highSamples,
        localWidths
    );
    FilterCoherentRibbonSamples(middleSamples, lowSamples, highSamples, localWidths);
    validBins = static_cast<int>(middleSamples.size());

    if (validBins < kMinValidBins
        || middleSamples.size() < static_cast<std::size_t>(fitOrder + 1)
        || lowSamples.size() < static_cast<std::size_t>(fitOrder + 1)
        || highSamples.size() < static_cast<std::size_t>(fitOrder + 1)) {
        return fit;
    }

    float sampleSMin = static_cast<float>(middleSamples.front().x);
    float sampleSMax = sampleSMin;
    for (const PolynomialSample &sample : middleSamples) {
        sampleSMin = std::min(sampleSMin, static_cast<float>(sample.x));
        sampleSMax = std::max(sampleSMax, static_cast<float>(sample.x));
    }

    if (sampleSMax - sampleSMin >= 35.0f) {
        fit.sMin = sampleSMin;
        fit.sMax = sampleSMax;
        fit.lengthPx = fit.sMax - fit.sMin;

        std::vector<TraceProjection> clippedProjections;
        clippedProjections.reserve(projections.size());
        for (const TraceProjection &projection : projections) {
            if (projection.s >= fit.sMin && projection.s <= fit.sMax) {
                clippedProjections.push_back(projection);
            }
        }
        projections = std::move(clippedProjections);
    }

    if (projections.size() < kMinInlierEvents) {
        return fit;
    }

    fit.widthPx = Quantile(localWidths, 0.50f);
    if (fit.widthPx < 1.0f) {
        return fit;
    }

    const int visualCurveSegments = std::clamp(
        static_cast<int>(std::ceil(std::max(1.0f, fit.lengthPx))),
        kSparseDiagnosticSegments,
        1400
    );

    const double localBandwidth = static_cast<double>(localWindow);
    const LocalQuadraticModel middleCurve = MakeLocalQuadraticModel(middleSamples, localBandwidth, fitOrder);
    const LocalQuadraticModel lowCurve = MakeLocalQuadraticModel(lowSamples, localBandwidth, fitOrder);
    const LocalQuadraticModel highCurve = MakeLocalQuadraticModel(highSamples, localBandwidth, fitOrder);
    if (!middleCurve.valid || !lowCurve.valid || !highCurve.valid) {
        return fit;
    }

    std::vector<Vector2> sparseLowFitPoints;
    std::vector<Vector2> sparseHighFitPoints;
    (void)BuildRibbonCurve(
        fit,
        lowCurve,
        kSparseDiagnosticSegments,
        &sparseLowFitPoints
    );
    (void)BuildRibbonCurve(
        fit,
        highCurve,
        kSparseDiagnosticSegments,
        &sparseHighFitPoints
    );

    std::vector<Vector2> lowImageCurve = BuildRibbonCurve(
        fit,
        lowCurve,
        visualCurveSegments
    );
    std::vector<Vector2> highImageCurve = BuildRibbonCurve(
        fit,
        highCurve,
        visualCurveSegments
    );

    float lowMeanY = 0.0f;
    float highMeanY = 0.0f;
    for (const Vector2 &point : lowImageCurve) {
        lowMeanY += point.y;
    }
    for (const Vector2 &point : highImageCurve) {
        highMeanY += point.y;
    }
    lowMeanY /= static_cast<float>(lowImageCurve.size());
    highMeanY /= static_cast<float>(highImageCurve.size());

    fit.middleModel = middleCurve;
    fit.projections = std::move(projections);
    fit.middleCurve = BuildRibbonCurve(fit, fit.middleModel, visualCurveSegments);
    (void)BuildRibbonCurve(fit, fit.middleModel, kSparseDiagnosticSegments, &fit.sparseMiddleFitPoints);
    if (lowMeanY < highMeanY) {
        fit.upperModel = lowCurve;
        fit.lowerModel = highCurve;
        fit.upperCurve = std::move(lowImageCurve);
        fit.lowerCurve = std::move(highImageCurve);
        fit.sparseUpperFitPoints = std::move(sparseLowFitPoints);
        fit.sparseLowerFitPoints = std::move(sparseHighFitPoints);
    }
    else {
        fit.upperModel = highCurve;
        fit.lowerModel = lowCurve;
        fit.upperCurve = std::move(highImageCurve);
        fit.lowerCurve = std::move(lowImageCurve);
        fit.sparseUpperFitPoints = std::move(sparseHighFitPoints);
        fit.sparseLowerFitPoints = std::move(sparseLowFitPoints);
    }

    fit.valid = true;
    return fit;
}

int64_t TraceTimeOriginUs(const std::vector<TracePoint> &tracePoints) {
    int64_t originUs = 0;
    if (!tracePoints.empty()) {
        originUs = tracePoints.front().timestampUs;
        for (const TracePoint &tracePoint : tracePoints) {
            originUs = std::min(originUs, tracePoint.timestampUs);
        }
    }
    return originUs;
}

struct Trace3DAnalysis {
    bool valid = false;
    Vector3 currentWorld{0.0f, 0.0f, 0.0f};
    std::vector<Vector3> worldPoints;
    std::vector<float> times;
    std::vector<Vector3> groundTruthPoints;
    std::vector<Vector3> groundTruthEstimatePoints;
    std::vector<TraceWidthEstimate> widthEstimates;
    std::vector<float> widthSamples;
    int widthSampleCount = 0;
    int rejectedWorldOutliers = 0;
    std::string poseText = "Trace pose: unavailable";
};

template <typename GroundTruthLookup>
Trace3DAnalysis AnalyzeTrace3D(
    const TraceRibbonFit &fit,
    const CalibrationData &calibration,
    float ballRadiusMm,
    float widthStepPx,
    int64_t traceTimeOriginUs,
    GroundTruthLookup lookupGroundTruth) {

    Trace3DAnalysis analysis;
    if (!fit.valid) {
        return analysis;
    }

    analysis.widthSampleCount = std::clamp(
        static_cast<int>(std::ceil(fit.lengthPx / std::max(widthStepPx, 1.0f))),
        3,
        80
    );
    analysis.widthEstimates = EstimateTraceWidths(fit, analysis.widthSampleCount);
    analysis.widthSamples.reserve(analysis.widthEstimates.size());

    for (const TraceWidthEstimate &estimate : analysis.widthEstimates) {
        if (!estimate.valid || estimate.widthPx < 1.0f) {
            continue;
        }

        analysis.widthSamples.push_back(estimate.widthPx);

        if (!calibration.ready) {
            continue;
        }

        const Vector2 widthVector{
            estimate.upper.x - estimate.lower.x,
            estimate.upper.y - estimate.lower.y
        };
        const Vector3 worldPoint = TraceImagePointToWorldMeters(
            estimate.middle,
            estimate.widthPx,
            widthVector,
            calibration,
            ballRadiusMm
        );

        if (std::isfinite(worldPoint.x)
            && std::isfinite(worldPoint.y)
            && std::isfinite(worldPoint.z)
            && (std::fabs(worldPoint.x) + std::fabs(worldPoint.y) + std::fabs(worldPoint.z)) > 1.0e-6f) {
            analysis.worldPoints.push_back(worldPoint);
            analysis.times.push_back(static_cast<float>(estimate.timeSeconds));
        }
    }

    analysis.rejectedWorldOutliers = FilterTraceWorldOutliers(analysis.worldPoints, analysis.times);

    if (analysis.worldPoints.size() == analysis.times.size()) {
        for (std::size_t i = 0; i < analysis.worldPoints.size(); ++i) {
            const float absoluteTimeSeconds =
                static_cast<float>(traceTimeOriginUs) * 1.0e-6f + analysis.times[i];
            Vector3 groundTruthPoint{};
            if (lookupGroundTruth(absoluteTimeSeconds, groundTruthPoint)) {
                analysis.groundTruthPoints.push_back(groundTruthPoint);
                analysis.groundTruthEstimatePoints.push_back(analysis.worldPoints[i]);
            }
        }
    }

    if (analysis.worldPoints.size() >= 3 && !analysis.widthSamples.empty()) {
        analysis.valid = true;
        analysis.currentWorld = analysis.worldPoints[analysis.worldPoints.size() / 2];
        const float medianWidth = Quantile(analysis.widthSamples, 0.50f);
        analysis.poseText = std::format(
            "Trace 3D position (m): X={:.3f}  Y={:.3f}  Z={:.3f} | W={:.1f}px",
            analysis.currentWorld.x,
            analysis.currentWorld.y,
            analysis.currentWorld.z,
            medianWidth
        );
    }

    return analysis;
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

    if (!ui.ShowTraceView()) {
        UpdateTraceAnalysis();
    }

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
    ui.SetRecordingFiles(BuildReaderFileList(ui.UseSequenceDirectory()));
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

void Gui::UpdateTraceAnalysis() {
    const int polarityMode = ui.TracePolarityMode();
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

    const TraceRibbonFit fit = FitTraceRibbon(
        source.points,
        ui.TraceLineBinWidthPx(),
        ui.TraceLineWindowPx(),
        ui.TraceLineOrder(),
        ui.TracePcaPeriodMs(),
        MakeTraceSupportEdgeSettings(ui)
    );

    if (!fit.valid) {
        ClearTrace3D();
        return;
    }

    const Trace3DAnalysis analysis = AnalyzeTrace3D(
        fit,
        traceCalibration,
        traceBallRadiusMm,
        ui.TraceWidthStepPx(),
        TraceTimeOriginUs(source.points),
        [this](float timeSeconds, Vector3 &worldPoint) {
            return this->LookupGroundTruthWorld(timeSeconds, worldPoint);
        }
    );

    traceWorld3D = analysis.worldPoints;
    traceTimes3D = analysis.times;
    traceGroundTruthWorld3D = analysis.groundTruthPoints;
    traceGroundTruthEstimateWorld3D = analysis.groundTruthEstimatePoints;
    trace3DValid = analysis.valid;
    traceCurrentWorld3D = analysis.currentWorld;
    tracePoseText3D = analysis.poseText;
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

    const float lineBinWidthPx = ui.TraceLineBinWidthPx();
    const float lineWindowPx = ui.TraceLineWindowPx();
    const int lineOrder = ui.TraceLineOrder();
    const float pcaPeriodMs = ui.TracePcaPeriodMs();
    const float followWindowPx = ui.TraceFollowWindowPx();
    const TraceSupportEdgeSettings supportEdge = MakeTraceSupportEdgeSettings(ui);
    const int polarityMode = ui.TracePolarityMode();
    TracePointSourceResult traceSource = BuildTracePointSource(
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
    std::string traceInputLabel = std::move(traceSource.label);
    Color traceInputColor = traceSource.color;
    std::vector<TracePoint> tracePoints = std::move(traceSource.points);
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

    const TraceRibbonFit fit = FitTraceRibbon(
        tracePoints,
        lineBinWidthPx,
        lineWindowPx,
        lineOrder,
        pcaPeriodMs,
        supportEdge
    );
    if (!fit.valid) {
        ClearTrace3D();
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

    const Trace3DAnalysis analysis = AnalyzeTrace3D(
        fit,
        traceCalibration,
        traceBallRadiusMm,
        ui.TraceWidthStepPx(),
        TraceTimeOriginUs(tracePoints),
        [this](float timeSeconds, Vector3 &worldPoint) {
            return this->LookupGroundTruthWorld(timeSeconds, worldPoint);
        }
    );
    traceWorld3D = analysis.worldPoints;
    traceTimes3D = analysis.times;
    traceGroundTruthWorld3D = analysis.groundTruthPoints;
    traceGroundTruthEstimateWorld3D = analysis.groundTruthEstimatePoints;
    trace3DValid = analysis.valid;
    traceCurrentWorld3D = analysis.currentWorld;
    tracePoseText3D = analysis.poseText;

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
            "width normal: %.1f px",
            widthSamples.empty() ? fit.widthPx : Quantile(widthSamples, 0.50f)
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
        TextFormat("fit input: %s", traceInputLabel.c_str()),
        static_cast<int>(dest.x + dest.width + 24.0f),
        static_cast<int>(dest.y + 178.0f),
        18,
        traceInputColor
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
