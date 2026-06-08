#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <opencv2/core.hpp>
#include <raylib.h>

#include "EventWriter.h"
#include "Camera.hpp"
#include "raygui.h"

static constexpr double RENDER_FPS = 60.0;
static constexpr double RENDER_PERIOD_MS = 1000.0 / RENDER_FPS;

struct OverlayCircle  { Vector2 center; float radius; Color color; int thickness; };
struct OverlayArrow   { Vector2 from, to; Color color; };
struct OverlayMarker  { Vector2 pos; Color color; };
struct OverlayLine    { Vector2 p1, p2; Color color; int thickness; };
struct OverlayRect    { Rectangle rect; Color color; };
struct OverlayText    { Vector2 pos; std::string text; Color color; int size; };
struct OverlayHudText { Vector2 pos; std::string text; Color color; int size; };
struct CoefParabolique { float a = 0.0f; float b = 0.0f; float c = 0.0f; };
struct LineFit3D { float a = 0.0f; float b = 0.0f; };
struct QuadFit3D { float a = 0.0f; float b = 0.0f; float c = 0.0f; };

struct TrajectoryQualityPoint {
    float trainRatio = 0.0f;
    float rmseMeters = 0.0f;
    int trainCount = 0;
    int testCount = 0;
};

class Ui;

class Gui {
private:
    int screenWidth;
    int screenHeight;
    const dv::EventStore &View;
    std::vector<std::vector<cv::Point2f>> clusters;
    std::array<Color, 9> Colors = {RED, GREEN, BLUE, YELLOW, MAGENTA, ORANGE, PINK, VIOLET, BROWN};
    Vector2 offset = {0.0f, 360.0f};
    float scale = 1.5f;
    Vector2 mousePosition = {0.0f, 0.0f};
    std::vector<Color> pixelBuffer;
    Texture2D cpuTexture{};
    std::vector<OverlayCircle> overlay_circles;
    std::vector<OverlayArrow> overlay_arrows;
    std::vector<OverlayMarker> overlay_markers;
    std::vector<OverlayLine> overlay_lines;
    std::vector<OverlayRect> overlay_rects;
    std::vector<OverlayText> overlay_texts;
    std::vector<OverlayHudText> overlay_hud_texts;
    LineFit3D imageXFit2D{};
    QuadFit3D imageYFit2D{};
    float imageTMin2D = 0.0f;
    float imageTMax2D = 0.0f;
    bool imageTrajectory2DValid = false;
    Ui &ui;
    std::chrono::steady_clock::time_point last_render_time{};

    Camera3D sceneCamera{};
    bool hasCurrentBall3D = false;
    Vector3 currentBallWorld3D{0.0f, 0.0f, 0.0f};
    float currentBallRadius3D = 0.04f;
    std::string currentBallText3D = "Ball pose: unavailable";
    std::vector<Vector3> trajectoryWorld3D;
    std::vector<float> trajectoryTimes3D;
    std::vector<TrajectoryQualityPoint> trajectoryQuality;
    bool trajectory3DValid = false;
    LineFit3D fitX3D{};
    LineFit3D fitY3D{};
    QuadFit3D fitZ3D{};
    float fitTMin3D = 0.0f;
    float fitTMax3D = 0.0f;
    CalibrationData traceCalibration{};
    float traceBallRadiusMm = 20.0f;
    const std::vector<cv::Point2f> *traceRawPoints = nullptr;
    const std::vector<int64_t> *traceRawTimestamps = nullptr;
    const std::vector<bool> *traceRawPolarities = nullptr;
    const std::vector<cv::Point2f> *traceFloatPoints = nullptr;
    const std::vector<int64_t> *traceFloatTimestamps = nullptr;
    const std::vector<bool> *traceFloatPolarities = nullptr;
    std::vector<cv::Point2f> traceAccumulatedPoints;
    std::vector<int64_t> traceAccumulatedTimestamps;
    std::vector<bool> traceAccumulatedPolarities;
    int64_t traceLastAccumulatedTimestampUs = std::numeric_limits<int64_t>::min();
    bool traceMotionWindowValid = false;
    bool traceMotionParabolaValid = false;
    Circle traceMotionCircle{};
    QuadFit3D traceMotionYFromXFit{};
    float traceMotionXMin = 0.0f;
    float traceMotionXMax = 0.0f;
    int64_t traceMotionTimestampUs = std::numeric_limits<int64_t>::min();
    bool trace3DValid = false;
    Vector3 traceCurrentWorld3D{0.0f, 0.0f, 0.0f};
    std::vector<Vector3> traceWorld3D;
    std::vector<float> traceTimes3D;
    std::vector<Vector3> traceGroundTruthWorld3D;
    std::vector<Vector3> traceGroundTruthEstimateWorld3D;
    std::string tracePoseText3D = "Trace pose: unavailable";
    std::vector<float> groundTruthTimesSeconds;
    std::vector<Vector3> groundTruthWorld3D;
    std::string groundTruthSourcePath;
    CalibrationData readerCalibrationOverride{};
    bool readerCalibrationOverrideReady = false;

    Vector3 WorldToScene(const Vector3 &world) const;
    void LoadReaderCalibrationForReader(const std::string &eventPath);
    void LoadGroundTruthForReader(const std::string &eventPath);
    bool LookupGroundTruthWorld(float timeSeconds, Vector3 &worldPoint) const;
    void Draw2DScene();
    void DrawTraceScene();
    void UpdateTraceAnalysis();
    void Draw3DScene();
    void DrawHudTexts();
    void UpdateTrajectoryQuality();
    void DrawTrajectoryQualityPanel();
    void DrawTopResidualSpeedPanel(const LineFit3D &topFit);
    bool TraceMotionWindowContains(const cv::Point2f &point) const;

    std::unique_ptr<EventWriter> event_writer_;
    std::unique_ptr<EventReader> event_reader_;
    std::string path_writer_ = "recordings/tmp.h5";
    std::string path_reader_ = "recordings/events.h5";
    std::chrono::steady_clock::time_point last_playback_time_{};

public:
    Gui(const dv::EventStore &view, Ui &uii, int screenWidth = 1920, int screenHeight = 1080);
    ~Gui();

    void Draw();
    void Update();
    void DrawTopView();
    void DrawView();
    void DrawClusters();
    void DrawPerformance();
    void AddClusterView(std::vector<cv::Point2f> cluster);
    void AddCircle(float cx, float cy, float r, Color col, int thickness = 2);
    void AddArrow(float x1, float y1, float x2, float y2, Color col);
    void AddMarker(float x, float y, Color col);
    void AddLine(float x1, float y1, float x2, float y2, Color col, int t = 1);
    void AddRect(float x, float y, float w, float h, Color col);
    void AddLabel(float x, float y, const std::string &txt, Color col, int size = 14);
    void AddHudText(float x, float y, const std::string &txt, Color col, int size = 20);
    void SetImageTrajectory2D(
        const LineFit3D &fitX,
        const QuadFit3D &fitY,
        float tMin,
        float tMax,
        bool valid);
    void ClearImageTrajectory2D();
    void DrawImageTrajectory2D();
    Vector2 CamToScreen(float x, float y);
    void DrawOverlays();


    void SetBall3D(const Vector3 &worldPosition, float radiusMeters, const std::string &label);
    void ClearCurrentBall3D();
    void SetTrajectory3D(const std::vector<Vector3> &worldTrack, const std::vector<float> &worldTrackTimes,
                         const LineFit3D &fitX, const LineFit3D &fitY,
                         const QuadFit3D &fitZ, float tMin, float tMax, bool valid);
    void ClearTrajectory3D();
    void SetTracePoseCalibration(const CalibrationData &calibration, float ballRadiusMm);
    void SetTraceFloatSource(
        const std::vector<cv::Point2f> *points,
        const std::vector<int64_t> *timestamps,
        const std::vector<bool> *polarities) {
        traceFloatPoints = points;
        traceFloatTimestamps = timestamps;
        traceFloatPolarities = polarities;
    }
    void SetTraceRawSource(
        const std::vector<cv::Point2f> *points,
        const std::vector<int64_t> *timestamps,
        const std::vector<bool> *polarities) {
        traceRawPoints = points;
        traceRawTimestamps = timestamps;
        traceRawPolarities = polarities;
    }
    void AppendTraceEvents(
        const std::vector<cv::Point2f> &points,
        const std::vector<int64_t> &timestamps,
        const std::vector<bool> *polarities);
    void SetTraceMotionWindow(
        const Circle &circle,
        const QuadFit3D &yFromXFit,
        bool parabolaValid,
        float xMin,
        float xMax,
        int64_t timestampUs);
    void ClearTraceMotionWindow();
    void ResetTraceAccumulation();
    void ClearTrace3D();
    const CalibrationData *ReaderCalibrationOverride() const;
    const std::string &ReaderEventPath() const { return path_reader_; }

    void WriteStore(const dv::EventStore &event) {
        if (event_reader_ != nullptr) {
            event_reader_->close();
            event_reader_ = nullptr;
        }

        if (event_writer_ == nullptr) {
            event_writer_ = std::make_unique<EventWriter>("recordings/events3.h5");
            std::cerr << "Writer ready with " << event_writer_->count() << " events\n";
        }

        if (event_writer_) {
            event_writer_->writeStore(event);
        }
    }

    void ReadStore(dv::EventStore &event);

    double ms_pre = 0.0;
    double ms_cluster = 0.0;
    double ms_post = 0.0;
    double ms_loop = 0.0;

    int64_t nb_event = 0;
    bool ClearPoses = false;
};

class Ui {
public:
    Ui() {
        bandwidth = 50.0f;
        minNb = 40.0f;
        coef = 0.45f;
        filterSize = 115.0f;
        maxResidual = 19.0f;
        alpha = 50.0f;
        sym_coef = 29.0f;
        sym_coef2 = 157.0f;
        color_switch = false;
        record = false;
        lector = 0.0f;
        timeslice = 484.32f;
        trace_width_step_px = 8.0f;
        trace_line_window_px = 65.69f;
        trace_memory_ms = 40.0f;
        trace_line_bin_width_px = 4.0f;
        trace_line_order = 2.0f;
        trace_pca_period_ms = 36.10f;
        trace_follow_window_px = 98.17f;
        trace_support_divisor = 28.0f;
        trace_support_min = 3.0f;
        trace_support_max = 9.0f;
        trace_support_radius_px = 1.75f;
        trace_border_percent = 3.5f;
        trace_use_raw_input = false;
        trace_radius_gate_enabled = false;
        weighted_regression_enabled = false;
        trace_polarity_mode = 2;
        temporal_slices = 5.0f;
        events_per_slice = 100.0f;
        slice_mode = 0;
        circle_fitting_enabled = false;
        reader_source_sequences = true;
        reader_mode = true;
        playback_playing = false;
        reader_duration_seconds = 0.0;
        reset_requested = false;
        option = false;
        save_f = false;
        read_f = false;
        save = 0;
        read = 0;
    }

    void DrawPanel() {
        constexpr int kViewModeCount = 5;
        const bool traceView = ShowTraceView();
        const float panelX = 300.0f;
        const float panelY = 20.0f;
        const float w = 350.0f;
        const float h = 40.0f;
        const float gapX = 400.0f;
        const float gapY = 70.0f;
        const float panelRows = traceView ? 5.0f : 5.0f;
        float px = panelX;
        float py = panelY;

        DrawRectangle(
            static_cast<int>(panelX) - 5,
            static_cast<int>(panelY) - 20,
            1625,
            static_cast<int>(gapY * panelRows) + 10,
            Fade(DARKGRAY, 0.3f)
        );

        auto setCell = [&](int col, int row) {
            px = panelX + static_cast<float>(col) * gapX;
            py = panelY + static_cast<float>(row) * gapY;
        };

        auto sliderAt = [&](int col, int row, const char *label, float &val, float lo, float hi) {
            setCell(col, row);
            DrawText(label, static_cast<int>(px), static_cast<int>(py) - 18, 17, BLACK);
            GuiSliderBar({px, py, w, h}, nullptr, nullptr, &val, lo, hi);
            DrawText(TextFormat("%.2f", val), static_cast<int>(px + w + 6.0f), static_cast<int>(py), 14, BLACK);
        };

        if (traceView) {
            sliderAt(0, 0, "Window ms", timeslice, 1.0f, 500.0f);
            sliderAt(1, 0, "Trace ms", trace_memory_ms, 40.0f, 3000.0f);
            sliderAt(2, 0, "Bin width px", trace_line_bin_width_px, 1.0f, 48.0f);
            sliderAt(3, 0, "Local window", trace_line_window_px, 8.0f, 240.0f);
            sliderAt(0, 1, "Width step px", trace_width_step_px, 8.0f, 90.0f);
            sliderAt(1, 1, "PCA ms", trace_pca_period_ms, 2.0f, 80.0f);
            sliderAt(2, 1, "Follow window px", trace_follow_window_px, 20.0f, 260.0f);
            sliderAt(3, 1, "Support div", trace_support_divisor, 8.0f, 60.0f);
            sliderAt(0, 2, "Support min", trace_support_min, 1.0f, 20.0f);
            sliderAt(1, 2, "Support max", trace_support_max, 2.0f, 30.0f);
            sliderAt(2, 2, "Support radius px", trace_support_radius_px, 0.5f, 4.0f);
            sliderAt(3, 2, "Border %", trace_border_percent, 0.0f, 10.0f);
            trace_support_max = std::max(trace_support_max, trace_support_min);
        }
        else {
            sliderAt(0, 0, "Bandwidth", bandwidth, 1.0f, 300.0f);
            sliderAt(1, 0, "Min events/cluster", minNb, 1.0f, 1000.0f);
            sliderAt(2, 0, "Filtre rayon", coef, 0.0f, 1.0f);
            sliderAt(3, 0, "RMS max", maxResidual, 0.0f, 200.0f);

            sliderAt(0, 1, "Coef time", alpha, 0.0f, 100.0f);
            sliderAt(1, 1, "Symetrie coef", sym_coef, 0.0f, 120.0f);
            sliderAt(2, 1, "Symetrie coef2", sym_coef2, 0.0f, 1000.0f);
            sliderAt(3, 1, "Window ms", timeslice, 1.0f, 500.0f);

            sliderAt(0, 2, "Ball slices", temporal_slices, 1.0f, 10.0f);
            sliderAt(1, 2, "Max Events", maxevent, 1.0f, 10000.0f);
            sliderAt(2, 2, "Rayon cote", rayon_cote, 0.0f, 2.0f);
            sliderAt(3, 2, "Events / slice", events_per_slice, 4.0f, 1000.0f);
        }

        py = panelY + (traceView ? 3.0f : 3.0f) * gapY;
        px = panelX;

        if (traceView) {
            DrawText("Fit input", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
            GuiToggle({px, py, 120.0f, h}, trace_use_raw_input ? "Raw" : "Undist", &trace_use_raw_input);
            px += 140.0f;
            DrawText("Radius gate", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
            GuiToggle({px, py, 120.0f, h}, trace_radius_gate_enabled ? "ON" : "OFF", &trace_radius_gate_enabled);
            px += 140.0f;

            DrawText("Line order", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
            if (GuiButton({px, py, 100.0f, h}, trace_line_order >= 1.5f ? "Quad" : "Linear")) {
                trace_line_order = trace_line_order >= 1.5f ? 1.0f : 2.0f;
            }
            px += 120.0f;
        }

        DrawText("Reader", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        if (GuiButton({px, py, 120.0f, h}, reader_mode ? "File" : "Camera")) {
            reader_mode = !reader_mode;
            if (!reader_mode) {
                playback_playing = false;
            }
        }
        px += 140.0f;

        DrawText("Playback", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        if (GuiButton({px, py, 120.0f, h}, playback_playing ? "Pause" : "Play")) {
            reader_mode = true;
            playback_playing = !playback_playing;
        }
        px += 150.0f;

        DrawText("Lecteur", static_cast<int>(px), static_cast<int>(py) - 18, 17, BLACK);
        GuiSliderBar({px, py, 520.0f, h}, nullptr, nullptr, &lector, 0.0f, 1.0f);
        DrawText(TextFormat("%.3f", lector), static_cast<int>(px + 526.0f), static_cast<int>(py), 14, BLACK);
        px += 600.0f;

        DrawText(
            TextFormat("Time %.3f / %.3f s", PlaybackTimeSeconds(), reader_duration_seconds),
            static_cast<int>(px),
            static_cast<int>(py + 9.0f),
            18,
            playback_playing ? DARKGREEN : DARKGRAY
        );

        py = panelY + (traceView ? 4.0f : 4.0f) * gapY;
        px = panelX;

        DrawText("Color mode", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        GuiToggle({px, py, 90.0f, h}, color_switch ? "ON" : "OFF", &color_switch);
        px += 120.0f;

        DrawText("Circle fit", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        GuiToggle({px, py, 90.0f, h}, circle_fitting_enabled ? "ON" : "OFF", &circle_fitting_enabled);
        px += 120.0f;

        DrawText("Weighted reg", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        GuiToggle({px, py, 110.0f, h}, weighted_regression_enabled ? "ON" : "OFF", &weighted_regression_enabled);
        px += 140.0f;

        DrawText("Record", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        GuiToggle({px, py, 90.0f, h}, record ? "REC" : "---", &record);
        px += 130.0f;

        if (!traceView) {
            DrawText("Slice mode", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
            const char *sliceLabel = "Recent";

            if (slice_mode == 1) {
                sliceLabel = "Time windows";
            } else if (slice_mode == 2) {
                sliceLabel = "Event windows";
            }

            if (GuiButton({px, py, 170.0f, h}, sliceLabel)) {
                slice_mode = (slice_mode + 1) % 3;
            }

            px += 200.0f;
        }

        DrawText("View", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        view_mode = std::clamp(view_mode, 0, kViewModeCount - 1);
        if (GuiDropdownBox({px, py, 170.0f, h}, "2D;3D;TOP;RMSE;Trace", &view_mode, view_dropdown_edit_mode)) {
            view_dropdown_edit_mode = !view_dropdown_edit_mode;
        }


        px += 200.0f;

        if (!traceView) {
            if (GuiButton({px, py, 170.0f, h}, "Reset track")) {
                reset_requested = true;
            }

            px += 200.0f;
        }

        GuiToggle({px, py, 90.0f, h}, "Option", &option);
        px += 130.0f;
        if (traceView) {
            const char *polarityLabel = "All";
            if (trace_polarity_mode == 1) {
                polarityLabel = "Positive";
            } else if (trace_polarity_mode == 2) {
                polarityLabel = "Negative";
            }
            DrawText("Polarity", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
            if (GuiButton({px, py, 120.0f, h}, polarityLabel)) {
                trace_polarity_mode = (trace_polarity_mode + 1) % 3;
            }
            px += 140.0f;
        }
        if (!traceView) {
            GuiToggle({px, py, 90.0f, h}, "Positif only", &positive_only);
        }

        if (option) {
            const float x = 900.0f;
            const float y = 500.0f;
            const float textbox_width = 260.0f;
            const float button_h = 30.0f;
            const float inner_gap_x = 50.0f;
            const float inner_gap_y = 30.0f;
            const float option_width = textbox_width + 2.0f * inner_gap_x;
            const float row_count = 8.0f;
            const float dy = button_h + 20.0f;
            const float option_height = dy * row_count + 2.0f * inner_gap_y;

            DrawRectangle(x - inner_gap_x, y - inner_gap_y, option_width, option_height, RED);

            Rectangle save_rec = {x, y + dy, textbox_width, button_h};
            Rectangle read_rec = {x, y + 7.0f * dy, textbox_width, button_h};
            const Vector2 currentMousePosition = GetMousePosition();

            if (CheckCollisionPointRec(currentMousePosition, save_rec) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                save_f = true;
                read_f = false;
            }

            if (CheckCollisionPointRec(currentMousePosition, read_rec) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                save_f = false;
                read_f = true;
            }

            DrawText("Save file:", static_cast<int>(x), static_cast<int>(y), 14, BLACK);
            save = GuiTextBox(save_rec, save_file, sizeof(save_file), save_f);
            DrawText("Source:", static_cast<int>(x), static_cast<int>(y + 2.0f * dy), 14, BLACK);
            if (GuiButton(
                    {x, y + 3.0f * dy, textbox_width, button_h},
                    reader_source_sequences ? "Sequences" : "Recordings")) {
                reader_source_sequences = !reader_source_sequences;
                recording_files.clear();
                recording_dropdown_text = reader_source_sequences ? "No sequence events" : "No recordings";
                recording_dropdown_index = 0;
                recording_dropdown_edit_mode = false;
                read_file[0] = '\0';
                read_f = false;
            }

            DrawText(
                reader_source_sequences ? "Choose sequence event:" : "Choose recording:",
                static_cast<int>(x),
                static_cast<int>(y + 4.0f * dy),
                14,
                BLACK);
            const bool wasDropdownOpen = recording_dropdown_edit_mode;
            if (GuiDropdownBox(
                    {x, y + 5.0f * dy, textbox_width, button_h},
                    recording_dropdown_text.c_str(),
                    &recording_dropdown_index,
                    recording_dropdown_edit_mode)) {
                recording_dropdown_edit_mode = !recording_dropdown_edit_mode;
                if (wasDropdownOpen
                    && !recording_files.empty()
                    && recording_dropdown_index >= 0
                    && recording_dropdown_index < static_cast<int>(recording_files.size())) {
                    std::snprintf(
                        read_file,
                        sizeof(read_file),
                        "%s",
                        recording_files[static_cast<std::size_t>(recording_dropdown_index)].c_str());
                    read = 1;
                    reader_mode = true;
                    playback_playing = false;
                    save_f = false;
                    read_f = false;
                }
            }

            DrawText("Read file:", static_cast<int>(x), static_cast<int>(y + 6.0f * dy), 14, BLACK);
            read = GuiTextBox(read_rec, read_file, sizeof(read_file), read_f);
        }
    }

    int Bandwidth() const { return static_cast<int>(bandwidth); }
    float Maxevent() const { return maxevent; }
    int MinNb() const { return static_cast<int>(minNb); }
    float Coef() const { return coef; }
    float FilterSize() const { return filterSize / 100.0f; }
    float MaxResidual() const { return maxResidual; }
    float Alpha() const { return alpha / 100.0f; }
    float Sym_coef() const { return sym_coef / 100.0f; }
    float Sym_coef2() const { return sym_coef2 / 100.0f; }
    bool Color_switch() const { return color_switch; }
    bool Record() const { return record; }
    float Lector() const { return lector; }
    float Time_Slice() const { return timeslice * 1.0e-3f; }
    bool UseReader() const { return reader_mode || lector > 0.01f; }
    bool UseSequenceDirectory() const { return reader_source_sequences; }
    bool PlaybackPlaying() const { return playback_playing; }
    void EnableReaderMode() { reader_mode = true; }
    void PausePlayback() { playback_playing = false; }
    void SetRecordingFiles(const std::vector<std::string> &files) {
        std::string currentSelection;
        if (recording_dropdown_index >= 0
            && recording_dropdown_index < static_cast<int>(recording_files.size())) {
            currentSelection = recording_files[static_cast<std::size_t>(recording_dropdown_index)];
        }

        if (files == recording_files) return;

        recording_files = files;
        recording_dropdown_text.clear();
        if (recording_files.empty()) {
            recording_dropdown_text = reader_source_sequences ? "No sequence events" : "No recordings";
            recording_dropdown_index = 0;
            return;
        }

        recording_dropdown_index = 0;
        for (std::size_t i = 0; i < recording_files.size(); ++i) {
            if (i > 0) recording_dropdown_text += ';';
            recording_dropdown_text += recording_files[i];
            if (!currentSelection.empty() && recording_files[i] == currentSelection) {
                recording_dropdown_index = static_cast<int>(i);
            }
        }
    }
    void TogglePlayback() {
        if (save_f || read_f) return;
        reader_mode = true;
        playback_playing = !playback_playing;
    }
    void ClearFileTextFocus() {
        save_f = false;
        read_f = false;
    }
    double PlaybackTimeSeconds() const { return static_cast<double>(lector) * reader_duration_seconds; }
    double PlaybackWindowSeconds() const { return std::max(0.001, static_cast<double>(Time_Slice())); }
    void SetReaderDuration(double durationSeconds) {
        reader_duration_seconds = std::max(0.0, durationSeconds);
        lector = std::clamp(lector, 0.0f, 1.0f);
    }
    void AdvancePlayback(double deltaSeconds) {
        if (!playback_playing || reader_duration_seconds <= 0.0 || deltaSeconds <= 0.0) {
            return;
        }

        const double nextTime = PlaybackTimeSeconds() + deltaSeconds;
        if (nextTime >= reader_duration_seconds) {
            lector = 1.0f;
            playback_playing = false;
            return;
        }

        lector = static_cast<float>(nextTime / reader_duration_seconds);
    }
    int TemporalSlices() const {
        return std::clamp(static_cast<int>(std::round(temporal_slices)), 1, 10);
    }
    int EventsPerSlice() const {
        return std::clamp(static_cast<int>(std::round(events_per_slice)), 1, 10000);
    }
    float TraceWidthStepPx() const {
        return std::clamp(trace_width_step_px, 8.0f, 90.0f);
    }
    float TraceLineWindowPx() const {
        return std::clamp(trace_line_window_px, 8.0f, 240.0f);
    }
    float TraceLineBinWidthPx() const {
        return std::clamp(trace_line_bin_width_px, 1.0f, 48.0f);
    }
    int TraceLineOrder() const {
        return std::clamp(static_cast<int>(std::round(trace_line_order)), 1, 2);
    }
    float TracePcaPeriodMs() const {
        return std::clamp(trace_pca_period_ms, 2.0f, 80.0f);
    }
    float TraceFollowWindowPx() const {
        return std::clamp(trace_follow_window_px, 20.0f, 260.0f);
    }
    float TraceSupportDivisor() const {
        return std::clamp(trace_support_divisor, 8.0f, 60.0f);
    }
    int TraceSupportMinCount() const {
        return std::clamp(static_cast<int>(std::round(trace_support_min)), 1, 20);
    }
    int TraceSupportMaxCount() const {
        return std::max(
            TraceSupportMinCount(),
            std::clamp(static_cast<int>(std::round(trace_support_max)), 2, 30));
    }
    float TraceSupportRadiusPx() const {
        return std::clamp(trace_support_radius_px, 0.5f, 4.0f);
    }
    float TraceBorderRatio() const {
        return std::clamp(trace_border_percent / 100.0f, 0.0f, 0.10f);
    }
    float TraceBorderPercent() const { return TraceBorderRatio() * 100.0f; }
    int TracePolarityMode() const { return std::clamp(trace_polarity_mode, 0, 2); }
    double TraceMemorySeconds() const {
        return static_cast<double>(std::clamp(trace_memory_ms, 40.0f, 3000.0f)) * 1.0e-3;
    }
    bool TraceUseRawInput() const { return trace_use_raw_input; }
    bool TraceUseRadiusGate() const { return trace_radius_gate_enabled; }
    bool CircleFittingEnabled() const { return circle_fitting_enabled; }
    bool WeightedRegressionEnabled() const { return weighted_regression_enabled; }
    int SliceMode() const { return std::clamp(slice_mode, 0, 2); }
    bool Show2D() const { return view_mode == 0; }
    bool Show3D() const { return view_mode == 1; }
    bool ShowTopView() const { return view_mode == 2; }
    bool ShowQualityView() const { return view_mode == 3; }
    bool ShowTraceView() const { return view_mode == 4; }
    void NextView() {
        constexpr int kViewModeCount = 5;
        if (save_f || read_f) return;
        view_dropdown_edit_mode = false;
        view_mode = (std::clamp(view_mode, 0, kViewModeCount - 1) + 1) % kViewModeCount;
    }
    void PreviousView() {
        constexpr int kViewModeCount = 5;
        if (save_f || read_f) return;
        view_dropdown_edit_mode = false;
        view_mode = (std::clamp(view_mode, 0, kViewModeCount - 1) + kViewModeCount - 1) % kViewModeCount;
    }

    bool ConsumeResetRequested() {
        const bool requested = reset_requested;
        reset_requested = false;
        return requested;
    }

    float lector = 0.0f;
    int save = 0;
    int read = 0;
    char save_file[200] = "";
    char read_file[200] = "";
    float rayon_cote = 0.0f;
    bool positive_only = false;

private:
    float bandwidth = 50.0f;
    float maxevent = 1000.0f;
    float minNb = 40.0f;
    float coef = 0.45f;
    float filterSize = 115.0f;
    float maxResidual = 19.0f;
    float alpha = 50.0f;
    float sym_coef = 29.0f;
    float sym_coef2 = 157.0f;
    float timeslice = 484.32f;
    float trace_width_step_px = 8.0f;
    float trace_line_window_px = 65.69f;
    float trace_memory_ms = 40.0f;
    float trace_line_bin_width_px = 4.0f;
    float trace_line_order = 2.0f;
    float trace_pca_period_ms = 36.10f;
    float trace_follow_window_px = 98.17f;
    float trace_support_divisor = 28.0f;
    float trace_support_min = 3.0f;
    float trace_support_max = 9.0f;
    float trace_support_radius_px = 1.75f;
    float trace_border_percent = 3.5f;
    bool trace_use_raw_input = false;
    bool trace_radius_gate_enabled = false;
    bool weighted_regression_enabled = false;
    int trace_polarity_mode = 2;
    float temporal_slices = 5.0f;
    float events_per_slice = 100.0f;

    bool color_switch = false;
    bool circle_fitting_enabled = false;
    bool record = false;
    bool reader_source_sequences = true;
    bool show3d = false;
    bool reader_mode = false;
    bool playback_playing = false;
    double reader_duration_seconds = 0.0;
    bool reset_requested = false;
    bool option = false;
    bool save_f = false;
    bool read_f = false;
    bool view_dropdown_edit_mode = false;
    bool recording_dropdown_edit_mode = false;
    std::vector<std::string> recording_files;
    std::string recording_dropdown_text = "No recordings";
    int recording_dropdown_index = 0;
    int slice_mode = 0;
    int view_mode = 0;
};
