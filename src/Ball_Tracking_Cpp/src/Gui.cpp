#define RAYGUI_IMPLEMENTATION
#include "Gui.h"
#undef RAYGUI_IMPLEMENTATION

#include <algorithm>
#include <execution>
#include <format>
#include <utility>

#include <raymath.h>

Gui::Gui(const dv::EventStore &view, Ui &uii, int screenWidth, int screenHeight)
    : screenWidth(screenWidth), screenHeight(screenHeight), View(view), ui(uii)
     {
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

void Gui::Draw() {
    std::fill(std::execution::par_unseq, pixelBuffer.begin(), pixelBuffer.end(), BLANK);
    for (int y = 0; y < 480; ++y) {
        for (int x = 640; x < 1280; ++x) {
            pixelBuffer[y * 1280 + x] = BLACK;
        }
    }

    DrawView();
    DrawClusters();
    UpdateTexture(cpuTexture, pixelBuffer.data());

    BeginDrawing();
    ClearBackground(RAYWHITE);

    if (ui.Show3D()) {
        Draw3DScene();
    }
    else {
        Draw2DScene();
    }

    DrawHudTexts();
    DrawPerformance();
    ui.DrawPanel();
    DrawText(TextFormat("Nombre evenment    : %ld", nb_event), 8, 84, 20, ORANGE);
    EndDrawing();
}

void Gui::Draw2DScene() {
    DrawText(TextFormat("Mouse Position: (%.0f, %.0f)", mousePosition.x, mousePosition.y), 190, 200, 20, LIGHTGRAY);
    DrawTextureEx(cpuTexture, {offset.x, offset.y}, 0.0f, scale, WHITE);
    DrawRectangleLinesEx({offset.x, offset.y, 1280 * scale, 480 * scale}, 2, RED);
    DrawOverlays();
    DrawParabole();
}

void Gui::Draw3DScene() {
    static bool wasControlling = false;
    const bool controlCamera = IsMouseButtonDown(MOUSE_RIGHT_BUTTON);

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

    const double dure = std::chrono::duration<double, std::milli>(now - last_render_time).count();
    if (dure >= RENDER_PERIOD_MS) {

        mousePosition = GetMousePosition();
        Draw();
        clusters.clear();
        if (IsKeyPressed(KEY_C) || ui.ConsumeResetRequested()) {
            ClearPoses = true;
            l = {0, 0, 0};
            ClearTrajectory3D();
            ClearCurrentBall3D();
        }
        if(IsKeyPressed(KEY_Z) && !IsKeyDown(KEY_LEFT_SHIFT)) ui.lector += 0.001;
        if(IsKeyPressed(KEY_X) && !IsKeyDown(KEY_LEFT_SHIFT)) ui.lector += 0.0001;
        if(IsKeyPressed(KEY_Z) && IsKeyDown(KEY_LEFT_SHIFT)) ui.lector -= 0.001;
        if(IsKeyPressed(KEY_X) && IsKeyDown(KEY_LEFT_SHIFT)) ui.lector -= 0.0001;
        last_render_time = now;
        if(ui.read == 1){
            ui.read = 0;
            path_reader_ = std::format("{}.bin",std::string(ui.read_file));
            if (event_reader_ != nullptr) event_reader_->close();
            if(event_writer_ != nullptr){
                event_writer_->close();
                std::cerr << "Writer closed, " << event_writer_->count() << " events saved\n";
            }
            event_reader_ = std::make_unique<EventReader>(path_reader_);
        }
        if(ui.save == 1){
            ui.save = 0;
            path_writer_ = std::format("{}.bin",std::string(ui.save_file));
            if (event_writer_ != nullptr) event_writer_->close();
            if (event_reader_ != nullptr) event_reader_->close();
            event_writer_ = std::make_unique<EventWriter>(path_writer_);
        }
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

void Gui::DrawParabole() {
    Vector2 p1 = CamToScreen(0.0f, l.c);
    for (int i = 0; i < 40; i++) {
        float x = (float)i * 16.0f;
        float y = l.a * x * x + l.b * x + l.c;
        Vector2 p2 = CamToScreen(x, y);
        DrawLineEx(p1, p2, 2.0f, BLUE);
        p1 = p2;
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

void Gui::SetTrajectory3D(const std::vector<Vector3> &worldTrack, const LineFit3D &fitX, const LineFit3D &fitY,
                          const QuadFit3D &fitZ, float tMin, float tMax, bool valid) {
    trajectoryWorld3D = worldTrack;
    fitX3D = fitX;
    fitY3D = fitY;
    fitZ3D = fitZ;
    fitTMin3D = tMin;
    fitTMax3D = tMax;
    trajectory3DValid = valid;
}

void Gui::ClearTrajectory3D() {
    trajectoryWorld3D.clear();
    fitX3D = {};
    fitY3D = {};
    fitZ3D = {};
    fitTMin3D = 0.0f;
    fitTMax3D = 0.0f;
    trajectory3DValid = false;
}