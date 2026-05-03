#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <opencv2/core.hpp>
#include <raylib.h>

#include "EventWriter.h"
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
    CoefParabolique l{};
    Ui &ui;
    std::chrono::steady_clock::time_point last_render_time{};

    Camera3D sceneCamera{};
    bool hasCurrentBall3D = false;
    Vector3 currentBallWorld3D{0.0f, 0.0f, 0.0f};
    float currentBallRadius3D = 0.04f;
    std::string currentBallText3D = "Ball pose: unavailable";
    std::vector<Vector3> trajectoryWorld3D;
    bool trajectory3DValid = false;
    LineFit3D fitX3D{};
    LineFit3D fitY3D{};
    QuadFit3D fitZ3D{};
    float fitTMin3D = 0.0f;
    float fitTMax3D = 0.0f;

    Vector3 WorldToScene(const Vector3 &world) const;
    void Draw2DScene();
    void Draw3DScene();
    void DrawHudTexts();

    std::unique_ptr<EventWriter> event_writer_;
    std::unique_ptr<EventReader> event_reader_;
    std::string path_writer_ = "tmp.bin";
    std::string path_reader_ = "events.bin";

public:
    Gui(const dv::EventStore &view, Ui &uii, int screenWidth = 1920, int screenHeight = 1080);
    ~Gui();

    void Draw();
    void Update();
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
    void Parabole(float a, float b, float c) { l = {a, b, c}; }
    Vector2 CamToScreen(float x, float y);
    void DrawOverlays();
    void DrawParabole();

    void SetBall3D(const Vector3 &worldPosition, float radiusMeters, const std::string &label);
    void ClearCurrentBall3D();
    void SetTrajectory3D(const std::vector<Vector3> &worldTrack, const LineFit3D &fitX, const LineFit3D &fitY,
                         const QuadFit3D &fitZ, float tMin, float tMax, bool valid);
    void ClearTrajectory3D();

    void WriteStore(const dv::EventStore &event) {
        if (event_reader_ != nullptr) {
            event_reader_->close();
            event_reader_ = nullptr;
        }

        if (event_writer_ == nullptr) {
            event_writer_ = std::make_unique<EventWriter>("events3.bin");
            std::cerr << "Writer ready with " << event_writer_->count() << " events\n";
        }

        if (event_writer_) {
            event_writer_->writeStore(event);
        }
    }

    void ReadStore(dv::EventStore &event, float start_coef, float slice_coef) {
        if (event_writer_ != nullptr) {
            event_writer_->close();
            std::cerr << "Writer closed, " << event_writer_->count() << " events saved\n";
            event_writer_ = nullptr;
        }

        if (event_reader_ == nullptr) {
            try {
                event_reader_ = std::make_unique<EventReader>("events3.bin");
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
            event_reader_->readStore(event, start_coef, slice_coef);
        }
    }

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
        timeslice = 0.0f;
        show3d = false;
        reset_requested = false;
        option = false;
        save_f = false;
        read_f = false;
        save = 0;
        read = 0;
    }

    void DrawPanel() {
        float px = 300.0f;
        float py = 20.0f;
        const float w = 350.0f;
        const float h = 40.0f;
        const float gapx = 400.0f;
        const float gapy = 70.0f;
        int counter = 0;

        DrawRectangle(
            static_cast<int>(px) - 5,
            static_cast<int>(py) - 20,
            static_cast<int>(gapx * 13.0f) + 10,
            static_cast<int>(gapy * 4.0f) + 10,
            Fade(DARKGRAY, 0.3f)
        );

        auto slider = [&](const char *label, float &val, float lo, float hi) {
            DrawText(label, static_cast<int>(px), static_cast<int>(py) - 18, 17, BLACK);
            GuiSliderBar({px, py, w, h}, nullptr, nullptr, &val, lo, hi);
            DrawText(TextFormat("%.2f", val), static_cast<int>(px + w + 6.0f), static_cast<int>(py), 14, BLACK);
            px += gapx;
            counter++;
            if (counter % 4 == 0) {
                px = 300.0f;
                py += gapy;
            }
        };

        slider("Bandwidth", bandwidth, 1.0f, 300.0f);
        slider("Min events/cluster", minNb, 1.0f, 1000.0f);
        slider("Filtre rayon", coef, 0.0f, 1.0f);
        slider("RMS max", maxResidual, 0.0f, 200.0f);
        slider("Coef time", alpha, 0.0f, 100.0f);
        slider("Symetrie coef", sym_coef, 0.0f, 120.0f);
        slider("Symetrie coef2", sym_coef2, 0.0f, 1000.0f);
        slider("Lecteur", lector, 0.0f, 1.0f);
        slider("Time Slice", timeslice, 0.00001f, 0.001f);
        slider("Max Events", maxevent, 1.0f, 10000.0f);
        slider("Rayon cote", rayon_cote, 0.0f, 2.0f);

        DrawText("Color mode", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        GuiToggle({px, py, 90.0f, h}, color_switch ? "ON" : "OFF", &color_switch);
        px += 120.0f;

        DrawText("Record", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        GuiToggle({px, py, 90.0f, h}, record ? "REC" : "---", &record);
        px += 130.0f;

        DrawText("View", static_cast<int>(px), static_cast<int>(py) - 14, 13, BLACK);
        if (GuiButton({px, py, 170.0f, h}, show3d ? "Switch to 2D" : "Switch to 3D")) {
            show3d = !show3d;
        }

        py += gapy;
        px -= 130.0f;

        if (GuiButton({px, py, 170.0f, h}, "Reset track")) {
            reset_requested = true;
        }

        px += 130.0f;
        GuiToggle({px, py, 90.0f, h}, "Option", &option);
        py += gapy;
        GuiToggle({px, py, 90.0f, h}, "Positif only", &positive_only);

        if (option) {
            const float x = 900.0f;
            const float y = 500.0f;
            const float textbox_width = 200.0f;
            const float button_h = 30.0f;
            const float inner_gap_x = 50.0f;
            const float inner_gap_y = 30.0f;
            const float option_width = textbox_width + 2.0f * inner_gap_x;
            const float row_count = 4.0f;
            const float dy = button_h + 20.0f;
            const float option_height = dy * row_count + 2.0f * inner_gap_y;

            DrawRectangle(x - inner_gap_x, y - inner_gap_y, option_width, option_height, RED);

            Rectangle save_rec = {x, y + dy, textbox_width, button_h};
            Rectangle read_rec = {x, y + 3.0f * dy, textbox_width, button_h};
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
            DrawText("Read file:", static_cast<int>(x), static_cast<int>(y + 2.0f * dy), 14, BLACK);
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
    float Time_Slice() const { return timeslice; }
    bool Show3D() const { return show3d; }

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
    float timeslice = 0.0f;

    bool color_switch = false;
    bool record = false;
    bool show3d = false;
    bool reset_requested = false;
    bool option = false;
    bool save_f = false;
    bool read_f = false;
};
