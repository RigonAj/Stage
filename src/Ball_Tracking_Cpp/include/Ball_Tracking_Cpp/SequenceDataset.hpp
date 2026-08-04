#pragma once

// Simulated-sequence dataset access, extracted from Gui.cpp so the GUI and the
// headless benchmark read a sequence exactly the same way.
//
// A sequence produced by the Isaac Sim + v2e generator (project
// "ball_event_dataset_v0") looks like:
//
//   <sequence>/camera/intrinsics.json     fx, fy, cx, cy, distortion
//   <sequence>/labels/ground_truth.csv    timestamp_s, ball_{x,y,z}_cam_m, ...
//   <sequence>/metadata.json              ball.radius_m, trajectory, conventions
//   <sequence>/events_v2e/events.h5       event stream
//
// The intrinsics of these sequences have nothing to do with the physical
// DVXplorer calibration (fx = fy = 520, cx = 320, cy = 240, no distortion), and
// the ball radius is 20 mm, not the 60 mm BallTrackerSettings default. Depth is
// f * D / size, so reading both from the sequence instead of the live defaults
// is what makes an offline comparison meaningful.

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <raylib.h>

#include "Camera.hpp"

namespace sequence_dataset {

namespace fs = std::filesystem;

// --- Minimal JSON / CSV helpers -------------------------------------------
// Deliberately small readers for the flat, generator-produced files above; the
// package has no JSON dependency and these files have a fixed shape.

std::vector<std::string> SplitCsvLine(const std::string &line);
int CsvColumnIndex(const std::vector<std::string> &header, std::string_view name);
bool ParseDouble(std::string_view text, double &value);

bool JsonNumber(std::string_view json, std::string_view key, double &value);
std::string JsonString(std::string_view json, std::string_view key);
std::vector<double> JsonNumberArray(std::string_view json, std::string_view key);
std::string ReadTextFile(const fs::path &path);

// --- Sidecar discovery ----------------------------------------------------
// Walks up from the event file until it finds the relative sidecar, so both
// "<seq>/events_v2e/events.h5" and a deeper layout resolve to the same
// "<seq>/camera/intrinsics.json".

fs::path FindSidecarPathForEventPath(const std::string &eventPath, const fs::path &relativeSidecar);
fs::path IntrinsicsPathForEventPath(const std::string &eventPath);
fs::path GroundTruthPathForEventPath(const std::string &eventPath);
fs::path MetadataPathForEventPath(const std::string &eventPath);

// --- Sequence camera and ball --------------------------------------------

CalibrationData LoadCalibrationFromIntrinsicsJson(const fs::path &intrinsicsPath);

// metadata.json -> ball.radius_m, in millimetres (the unit used everywhere in
// the tracker). std::nullopt when the file is missing or malformed.
std::optional<double> LoadBallRadiusMmFromMetadata(const fs::path &metadataPath);

// --- Ground truth ---------------------------------------------------------

// Ground-truth poses in the internal world convention used by the trace and
// circle 3D outputs: camera_optical {x, y, z} is stored as {x, z, -y} (the
// util.hpp ToMeters remap). Convert back with WorldToCameraOptical below
// before reporting anything against the CSV columns.
struct GroundTruthTable {
    std::vector<float> timesSeconds;
    std::vector<Vector3> world;
    // Optional "visible" column, 1 per row when the CSV carries it. Rows are
    // never dropped: dropping them would change what the GUI overlay draws.
    std::vector<char> visible;
    std::string sourcePath;

    bool empty() const { return timesSeconds.empty(); }
    std::size_t size() const { return timesSeconds.size(); }
    float firstTimeSeconds() const { return timesSeconds.empty() ? 0.0f : timesSeconds.front(); }
    float lastTimeSeconds() const { return timesSeconds.empty() ? 0.0f : timesSeconds.back(); }
};

GroundTruthTable LoadGroundTruthCsv(const fs::path &groundTruthPath);

// Linear interpolation between the two bracketing samples, with a 10 ms
// tolerance outside the table (a trace window can end a few ms past the last
// labelled frame).
bool LookupGroundTruthWorld(const GroundTruthTable &table, float timeSeconds, Vector3 &worldPoint);

// True when both bracketing ground-truth rows are flagged visible (always true
// for a CSV without a "visible" column).
bool LookupGroundTruthVisible(const GroundTruthTable &table, float timeSeconds);

// Inverse of the util.hpp ToMeters remap: internal world metres back to the
// camera_optical pinhole convention (x right, y down, z forward/depth), so the
// z column is directly comparable to ball_z_cam_m.
inline Vector3 WorldToCameraOptical(const Vector3 &worldMeters) {
    return {worldMeters.x, -worldMeters.z, worldMeters.y};
}

}  // namespace sequence_dataset
