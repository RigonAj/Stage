#pragma once

// Sidecar loading for generated sequence directories (Isaac Sim / v2e datasets).
//
// A sequence directory looks like:
//   <sequence>/camera/intrinsics.json
//   <sequence>/labels/ground_truth.csv
//   <sequence>/metadata.json
//   <sequence>/events_v2e/events.h5
//
// The GUI reader and the offline benchmark both resolve those sidecars from
// the events file path, so the synthetic camera model can never be confused
// with the real DVXplorer calibration.

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "Camera.hpp"

std::string ReadTextFile(const std::filesystem::path &path);

bool ParseDouble(std::string_view text, double &value);

std::vector<std::string> SplitCsvLine(const std::string &line);

int CsvColumnIndex(const std::vector<std::string> &header, std::string_view name);

bool JsonNumber(std::string_view json, std::string_view key, double &value);

std::string JsonString(std::string_view json, std::string_view key);

std::vector<double> JsonNumberArray(std::string_view json, std::string_view key);

// Walks up from the events file until <dir>/relativeSidecar exists. Empty path
// when no ancestor carries the sidecar.
std::filesystem::path FindSidecarPathForEventPath(
    const std::string &eventPath,
    const std::filesystem::path &relativeSidecar);

std::filesystem::path IntrinsicsPathForEventPath(const std::string &eventPath);

std::filesystem::path GroundTruthPathForEventPath(const std::string &eventPath);

std::filesystem::path MetadataPathForEventPath(const std::string &eventPath);

// Returns a non-ready CalibrationData when the JSON is missing or invalid.
// Callers must treat that as an error instead of falling back to another
// calibration source.
CalibrationData LoadCalibrationFromIntrinsicsJson(const std::filesystem::path &intrinsicsPath);

// Ball radius in metres from a sequence metadata.json ("ball": {"radius_m"}).
// Returns false when the key is absent or not positive.
bool LoadBallRadiusFromMetadataJson(const std::filesystem::path &metadataPath, double &radiusMeters);

struct SequenceGroundTruthSample {
    double timeSeconds = 0.0;
    // OpenCV camera frame, metres, as exported by the generator.
    double xCamMeters = 0.0;
    double yCamMeters = 0.0;
    double zCamMeters = 0.0;
    double uPx = 0.0;
    double vPx = 0.0;
    double radiusPx = 0.0;
    bool visible = true;
};

// Sorted by time. Empty when the CSV is missing or lacks the required columns.
std::vector<SequenceGroundTruthSample> LoadSequenceGroundTruth(
    const std::filesystem::path &groundTruthPath);
