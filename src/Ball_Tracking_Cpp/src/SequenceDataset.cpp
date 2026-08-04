#include "SequenceDataset.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <utility>

#include <opencv2/core.hpp>

namespace sequence_dataset {

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

fs::path MetadataPathForEventPath(const std::string &eventPath) {
    return FindSidecarPathForEventPath(eventPath, fs::path("metadata.json"));
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

std::optional<double> LoadBallRadiusMmFromMetadata(const fs::path &metadataPath) {
    const std::string json = ReadTextFile(metadataPath);
    if (json.empty()) {
        return std::nullopt;
    }

    // "ball": { "radius_m": 0.02 } - search from the "ball" object so a
    // "radius_m" appearing elsewhere in the file cannot be picked up first.
    const std::size_t ballPos = json.find("\"ball\"");
    const std::string_view scope =
        ballPos == std::string::npos
            ? std::string_view(json)
            : std::string_view(json).substr(ballPos);

    double radiusM = 0.0;
    if (!JsonNumber(scope, "radius_m", radiusM) || radiusM <= 0.0) {
        return std::nullopt;
    }

    return radiusM * 1.0e3;
}

GroundTruthTable LoadGroundTruthCsv(const fs::path &groundTruthPath) {
    GroundTruthTable table;

    std::ifstream file(groundTruthPath);
    if (!file) {
        return table;
    }

    std::string line;
    if (!std::getline(file, line)) {
        return table;
    }

    const std::vector<std::string> header = SplitCsvLine(line);
    const int timeColumn = CsvColumnIndex(header, "timestamp_s");
    const int xColumn = CsvColumnIndex(header, "ball_x_cam_m");
    const int yColumn = CsvColumnIndex(header, "ball_y_cam_m");
    const int zColumn = CsvColumnIndex(header, "ball_z_cam_m");
    if (timeColumn < 0 || xColumn < 0 || yColumn < 0 || zColumn < 0) {
        return table;
    }

    // "visible" is optional (older generator runs omit it). Invisible rows are
    // kept so the interpolation matches what the GUI overlay does; the flag is
    // carried alongside so a consumer can reject an estimate that falls in a
    // stretch where the ball is out of frame.
    const int visibleColumn = CsvColumnIndex(header, "visible");

    const int requiredColumn = std::max({timeColumn, xColumn, yColumn, zColumn});
    struct Sample {
        float timeSeconds;
        Vector3 world;
        bool visible;
    };
    std::vector<Sample> samples;

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = SplitCsvLine(line);
        if (static_cast<int>(fields.size()) <= requiredColumn) {
            continue;
        }

        bool visible = true;
        if (visibleColumn >= 0 && static_cast<int>(fields.size()) > visibleColumn) {
            double visibleValue = 1.0;
            if (ParseDouble(fields[static_cast<std::size_t>(visibleColumn)], visibleValue)) {
                visible = visibleValue >= 0.5;
            }
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
            },
            visible
        });
    }

    std::sort(samples.begin(), samples.end(), [](const Sample &a, const Sample &b) {
        return a.timeSeconds < b.timeSeconds;
    });

    table.timesSeconds.reserve(samples.size());
    table.world.reserve(samples.size());
    table.visible.reserve(samples.size());
    for (const Sample &sample : samples) {
        table.timesSeconds.push_back(sample.timeSeconds);
        table.world.push_back(sample.world);
        table.visible.push_back(static_cast<char>(sample.visible ? 1 : 0));
    }

    if (!table.timesSeconds.empty()) {
        table.sourcePath = groundTruthPath.string();
    }

    return table;
}

bool LookupGroundTruthWorld(const GroundTruthTable &table, float timeSeconds, Vector3 &worldPoint) {
    if (!std::isfinite(timeSeconds)
        || table.timesSeconds.empty()
        || table.timesSeconds.size() != table.world.size()) {
        return false;
    }

    constexpr float kEndpointToleranceSeconds = 0.01f;
    const auto upper = std::lower_bound(
        table.timesSeconds.begin(),
        table.timesSeconds.end(),
        timeSeconds);

    if (upper == table.timesSeconds.begin()) {
        if (std::fabs(timeSeconds - table.timesSeconds.front()) <= kEndpointToleranceSeconds) {
            worldPoint = table.world.front();
            return true;
        }
        return false;
    }

    if (upper == table.timesSeconds.end()) {
        if (std::fabs(timeSeconds - table.timesSeconds.back()) <= kEndpointToleranceSeconds) {
            worldPoint = table.world.back();
            return true;
        }
        return false;
    }

    const std::size_t hi = static_cast<std::size_t>(
        std::distance(table.timesSeconds.begin(), upper));
    const std::size_t lo = hi - 1;
    const float t0 = table.timesSeconds[lo];
    const float t1 = table.timesSeconds[hi];
    const float dt = t1 - t0;
    if (dt <= 1.0e-9f) {
        worldPoint = table.world[lo];
        return true;
    }

    const float alpha = std::clamp((timeSeconds - t0) / dt, 0.0f, 1.0f);
    const Vector3 &p0 = table.world[lo];
    const Vector3 &p1 = table.world[hi];
    worldPoint = {
        p0.x + (p1.x - p0.x) * alpha,
        p0.y + (p1.y - p0.y) * alpha,
        p0.z + (p1.z - p0.z) * alpha
    };
    return true;
}

bool LookupGroundTruthVisible(const GroundTruthTable &table, float timeSeconds) {
    if (table.visible.size() != table.timesSeconds.size() || table.timesSeconds.empty()) {
        // No visibility column in this CSV: treat every labelled instant as
        // visible, which is what the generator produces for a full flight.
        return true;
    }

    const auto upper = std::lower_bound(
        table.timesSeconds.begin(),
        table.timesSeconds.end(),
        timeSeconds);

    if (upper == table.timesSeconds.begin()) {
        return table.visible.front() != 0;
    }
    if (upper == table.timesSeconds.end()) {
        return table.visible.back() != 0;
    }

    const std::size_t hi = static_cast<std::size_t>(
        std::distance(table.timesSeconds.begin(), upper));
    const std::size_t lo = hi - 1;

    // Both bracketing samples must be visible: an estimate landing on the
    // boundary of an occlusion has no trustworthy reference.
    return table.visible[lo] != 0 && table.visible[hi] != 0;
}

}  // namespace sequence_dataset
