#include "SequenceData.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <system_error>

namespace fs = std::filesystem;

std::string ReadTextFile(const fs::path &path) {
    std::ifstream file(path);
    if (!file) {
        return {};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

bool ParseDouble(std::string_view text, double &value) {
    std::string owned(text);
    char *end = nullptr;
    value = std::strtod(owned.c_str(), &end);
    return end != owned.c_str() && std::isfinite(value);
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

bool LoadBallRadiusFromMetadataJson(const fs::path &metadataPath, double &radiusMeters) {
    const std::string json = ReadTextFile(metadataPath);
    if (json.empty()) {
        return false;
    }

    double value = 0.0;
    if (!JsonNumber(json, "radius_m", value) || !(value > 0.0)) {
        return false;
    }

    radiusMeters = value;
    return true;
}

std::vector<SequenceGroundTruthSample> LoadSequenceGroundTruth(const fs::path &groundTruthPath) {
    std::vector<SequenceGroundTruthSample> samples;

    std::ifstream file(groundTruthPath);
    if (!file) {
        return samples;
    }

    std::string line;
    if (!std::getline(file, line)) {
        return samples;
    }

    const std::vector<std::string> header = SplitCsvLine(line);
    const int timeColumn = CsvColumnIndex(header, "timestamp_s");
    const int xColumn = CsvColumnIndex(header, "ball_x_cam_m");
    const int yColumn = CsvColumnIndex(header, "ball_y_cam_m");
    const int zColumn = CsvColumnIndex(header, "ball_z_cam_m");
    if (timeColumn < 0 || xColumn < 0 || yColumn < 0 || zColumn < 0) {
        return samples;
    }

    // Optional columns: image projection is only used by the benchmark.
    const int uColumn = CsvColumnIndex(header, "u_px");
    const int vColumn = CsvColumnIndex(header, "v_px");
    const int radiusColumn = CsvColumnIndex(header, "radius_px");
    const int visibleColumn = CsvColumnIndex(header, "visible");

    const int requiredColumn = std::max({timeColumn, xColumn, yColumn, zColumn});

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = SplitCsvLine(line);
        if (static_cast<int>(fields.size()) <= requiredColumn) {
            continue;
        }

        SequenceGroundTruthSample sample;
        if (!ParseDouble(fields[static_cast<std::size_t>(timeColumn)], sample.timeSeconds)
            || !ParseDouble(fields[static_cast<std::size_t>(xColumn)], sample.xCamMeters)
            || !ParseDouble(fields[static_cast<std::size_t>(yColumn)], sample.yCamMeters)
            || !ParseDouble(fields[static_cast<std::size_t>(zColumn)], sample.zCamMeters)) {
            continue;
        }

        auto optionalColumn = [&](int column, double &target) {
            if (column >= 0 && static_cast<int>(fields.size()) > column) {
                ParseDouble(fields[static_cast<std::size_t>(column)], target);
            }
        };
        optionalColumn(uColumn, sample.uPx);
        optionalColumn(vColumn, sample.vPx);
        optionalColumn(radiusColumn, sample.radiusPx);

        if (visibleColumn >= 0 && static_cast<int>(fields.size()) > visibleColumn) {
            double visible = 1.0;
            if (ParseDouble(fields[static_cast<std::size_t>(visibleColumn)], visible)) {
                sample.visible = visible >= 0.5;
            }
        }

        samples.push_back(sample);
    }

    std::sort(samples.begin(), samples.end(), [](const auto &a, const auto &b) {
        return a.timeSeconds < b.timeSeconds;
    });

    return samples;
}
