#include "simulator_gdi.hpp"

#include <windows.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
#include <vector>

#include "nogps_core.hpp"

namespace {
constexpr int kLeftPanelWidth = 640;
constexpr float kFrameDt = 0.016f;
float kCoreViewScale = 0.6f;

const int MAP_W = 640;
const int MAP_H = 800;
std::vector<bool> worldMap(MAP_W * MAP_H, false);
uint32_t mapPixels[MAP_H][MAP_W];

Vec2 dronePos(100.0f, 100.0f);
Vec2 droneVel(0.0f, 0.0f);
DroneCore core;

DWORD lastFpsUpdate = 0;
int frameCount = 0;
float currentFps = 0.0f;

void updateMapBuffer() {
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            mapPixels[y][x] = worldMap[y * MAP_W + x] ? 0x00FFFFFF : 0x00000000;
        }
    }
}

void generateCaveMap() {
    for (int i = 0; i < MAP_W * MAP_H; ++i) {
        worldMap[i] = (rand() % 100 < 45);
    }

    for (int iter = 0; iter < 5; ++iter) {
        std::vector<bool> newMap = worldMap;
        for (int y = 1; y < MAP_H - 1; ++y) {
            for (int x = 1; x < MAP_W - 1; ++x) {
                int neighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (worldMap[(y + dy) * MAP_W + (x + dx)]) {
                            ++neighbors;
                        }
                    }
                }
                newMap[y * MAP_W + x] = (neighbors > 4);
            }
        }
        worldMap = newMap;
    }

    updateMapBuffer();
}

void DrawLine(HDC hdc, int x1, int y1, int x2, int y2, COLORREF color, int thickness = 1) {
    HPEN hPen = CreatePen(PS_SOLID, thickness, color);
    HGDIOBJ hOld = SelectObject(hdc, hPen);
    MoveToEx(hdc, x1, y1, nullptr);
    LineTo(hdc, x2, y2);
    SelectObject(hdc, hOld);
    DeleteObject(hPen);
}

void DrawCircle(HDC hdc, int x, int y, int r, COLORREF color, bool filled) {
    HBRUSH hBrush = filled ? CreateSolidBrush(color) : (HBRUSH)GetStockObject(NULL_BRUSH);
    HPEN hPen = CreatePen(PS_SOLID, 1, color);
    HGDIOBJ oldB = SelectObject(hdc, hBrush);
    HGDIOBJ oldP = SelectObject(hdc, hPen);
    Ellipse(hdc, x - r, y - r, x + r, y + r);
    SelectObject(hdc, oldB);
    SelectObject(hdc, oldP);
    if (filled) {
        DeleteObject(hBrush);
    }
    DeleteObject(hPen);
}

float CastRay(Vec2 start, Vec2 dir) {
    for (float d = 0; d < 500; d += 2.0f) {
        int ix = (int)(start.x + dir.x * d);
        int iy = (int)(start.y + dir.y * d);
        if (ix < 0 || ix >= MAP_W || iy < 0 || iy >= MAP_H || worldMap[iy * MAP_W + ix]) {
            return d;
        }
    }
    return 500.0f;
}

POINT toCoreView(Vec2 worldPoint, Vec2 center, int winW, int winH) {
    float panelCenterX = kLeftPanelWidth + (winW - kLeftPanelWidth) / 2.0f;
    float panelCenterY = winH / 2.0f;
    POINT pt;
    pt.x = (LONG)(panelCenterX + (worldPoint.x - center.x) * kCoreViewScale);
    pt.y = (LONG)(panelCenterY + (worldPoint.y - center.y) * kCoreViewScale);
    return pt;
}

void drawUi(HDC hdc, int winW, int winH) {
    SetTextColor(hdc, RGB(0, 255, 0));
    SetBkMode(hdc, TRANSPARENT);

    const int x = kLeftPanelWidth + 10;
    char buf[128];
    TextOutA(hdc, x, 20, "--- CORE REALTIME TUNING ---", 28);
    sprintf(buf, "[1/2] Merge Tol: %.1f", core.params.mergeTolerance);
    TextOutA(hdc, x, 48, buf, (int)strlen(buf));
    sprintf(buf, "[3/4] Split Tol: %.1f", core.params.splitTolerance);
    TextOutA(hdc, x, 68, buf, (int)strlen(buf));
    sprintf(buf, "[5/6] RAM Limit: %zu KB", core.params.ramLimitBytes / 1024);
    TextOutA(hdc, x, 88, buf, (int)strlen(buf));
    sprintf(buf, "[7/8] Reflex Force: %.1f", core.params.reflexForce);
    TextOutA(hdc, x, 108, buf, (int)strlen(buf));
    sprintf(buf, "[9/0] Node Dist: %.1f", core.params.newNodeDist);
    TextOutA(hdc, x, 128, buf, (int)strlen(buf));
    sprintf(buf, "[-/=] Reflex Dist: %.1f", core.params.reflexDist);
    TextOutA(hdc, x, 148, buf, (int)strlen(buf));
    TextOutA(hdc, x, 168, "R: Reset world/core", 19);

    std::string logs = core.getLogs();
    RECT textR = {x, winH - 120, winW - 10, winH - 10};
    DrawTextA(hdc, logs.c_str(), -1, &textR, DT_LEFT);
}

void drawFrame(HDC hdcMem, HWND hwnd, const std::vector<LidarPoint>& scan) {
    RECT clRect;
    GetClientRect(hwnd, &clRect);
    const int winW = clRect.right;
    const int winH = clRect.bottom;

    FillRect(hdcMem, &clRect, (HBRUSH)GetStockObject(BLACK_BRUSH));
    DrawLine(hdcMem, kLeftPanelWidth, 0, kLeftPanelWidth, winH, RGB(100, 100, 100));

    BITMAPINFO bmi = {0};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = MAP_W;
    bmi.bmiHeader.biHeight = -MAP_H;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
    StretchDIBits(hdcMem, 0, 0, MAP_W, MAP_H, 0, 0, MAP_W, MAP_H, mapPixels, &bmi, DIB_RGB_COLORS, SRCCOPY);

    for (const auto& p : scan) {
        Vec2 end = dronePos + p.toCartesian();
        DrawLine(hdcMem, (int)dronePos.x, (int)dronePos.y, (int)end.x, (int)end.y, RGB(50, 50, 50));
    }
    DrawCircle(hdcMem, (int)dronePos.x, (int)dronePos.y, 7, RGB(0, 255, 0), true);

    Vec2 estPos = core.getEstPos();
    for (const auto& l : core.getGlobalLines()) {
        POINT p1 = toCoreView(l.start, estPos, winW, winH);
        POINT p2 = toCoreView(l.end, estPos, winW, winH);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, RGB(150, 150, 0));
    }

    for (const auto& l : core.getDebugLines()) {
        POINT p1 = toCoreView(l.start, estPos, winW, winH);
        POINT p2 = toCoreView(l.end, estPos, winW, winH);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, RGB(0, 255, 255), 2);
    }

    for (const auto& n : core.getGraph()) {
        POINT p = toCoreView(n.position, estPos, winW, winH);
        DrawCircle(hdcMem, p.x, p.y, 3, n.isOffloaded ? RGB(50, 50, 50) : RGB(255, 0, 0), true);
    }

    POINT c = toCoreView(estPos, estPos, winW, winH);
    DrawCircle(hdcMem, c.x, c.y, 5, RGB(0, 255, 0), false);

    drawUi(hdcMem, winW, winH);

    char title[256];
    sprintf(title, "NoGPS Sim | FPS: %.1f | Global Lines: %zu", currentFps, core.getGlobalLines().size());
    SetWindowTextA(hwnd, title);

    HDC hdcWin = GetDC(hwnd);
    BitBlt(hdcWin, 0, 0, winW, winH, hdcMem, 0, 0, SRCCOPY);
    ReleaseDC(hwnd, hdcWin);
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    if (uMsg == WM_DESTROY) {
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}
} // namespace

int runSimulator() {
    CreateDirectoryA("cache", NULL);
    generateCaveMap();

    WNDCLASSA wc = {0};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = "NoGPS_Sim";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(
        0,
        wc.lpszClassName,
        "NoGPS Core Simulator",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        100,
        100,
        1280,
        800,
        NULL,
        NULL,
        wc.hInstance,
        NULL);

    HDC hdcWin = GetDC(hwnd);
    HDC hdcMem = CreateCompatibleDC(hdcWin);
    HBITMAP hbmMem = CreateCompatibleBitmap(hdcWin, 1920, 1080);
    SelectObject(hdcMem, hbmMem);

    while (true) {
        MSG msg;
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                return 0;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        if (GetAsyncKeyState('1') & 0x8000) core.params.mergeTolerance -= 0.5f;
        if (GetAsyncKeyState('2') & 0x8000) core.params.mergeTolerance += 0.5f;
        if (GetAsyncKeyState('3') & 0x8000) core.params.splitTolerance -= 0.1f;
        if (GetAsyncKeyState('4') & 0x8000) core.params.splitTolerance += 0.1f;
        if (GetAsyncKeyState('5') & 0x8000 && core.params.ramLimitBytes > 16 * 1024) core.params.ramLimitBytes -= 1024;
        if (GetAsyncKeyState('6') & 0x8000) core.params.ramLimitBytes += 1024;
        if (GetAsyncKeyState('7') & 0x8000 && core.params.reflexForce > 1.0f) core.params.reflexForce -= 1.0f;
        if (GetAsyncKeyState('8') & 0x8000) core.params.reflexForce += 1.0f;
        if (GetAsyncKeyState('9') & 0x8000 && core.params.newNodeDist > 10.0f) core.params.newNodeDist -= 1.0f;
        if (GetAsyncKeyState('0') & 0x8000) core.params.newNodeDist += 1.0f;
        if (GetAsyncKeyState(VK_OEM_MINUS) & 0x8000 && core.params.reflexDist > 5.0f) core.params.reflexDist -= 1.0f;
        if (GetAsyncKeyState(VK_OEM_PLUS) & 0x8000) core.params.reflexDist += 1.0f;

        if (core.params.mergeTolerance < 1.0f) core.params.mergeTolerance = 1.0f;
        if (core.params.splitTolerance < 0.5f) core.params.splitTolerance = 0.5f;
        if (core.params.reflexForce < 0.0f) core.params.reflexForce = 0.0f;
        if (core.params.newNodeDist < 10.0f) core.params.newNodeDist = 10.0f;
        if (core.params.reflexDist < 5.0f) core.params.reflexDist = 5.0f;
        if (core.params.ramLimitBytes < 16 * 1024) core.params.ramLimitBytes = 16 * 1024;

        Vec2 input(0.0f, 0.0f);
        if (GetAsyncKeyState('W') & 0x8000) input.y -= 120;
        if (GetAsyncKeyState('S') & 0x8000) input.y += 120;
        if (GetAsyncKeyState('A') & 0x8000) input.x -= 120;
        if (GetAsyncKeyState('D') & 0x8000) input.x += 120;
        if (GetAsyncKeyState('R') & 0x8000) {
            generateCaveMap();
            core.reset();
            dronePos = {100.0f, 100.0f};
        }

        std::vector<LidarPoint> scan;
        scan.reserve(120);
        for (int i = 0; i < 120; ++i) {
            const float a = (i * 2.0f * PI) / 120.0f;
            const Vec2 rayDir(std::cos(a), std::sin(a));
            const float noisyDist = CastRay(dronePos, rayDir) + static_cast<float>(rand() % 100) / 50.0f;
            scan.push_back({a, noisyDist});
        }

        droneVel = input + core.velocityCommand;
        dronePos = dronePos + droneVel * kFrameDt;
        core.update(kFrameDt, scan, droneVel);

        drawFrame(hdcMem, hwnd, scan);

        ++frameCount;
        if (GetTickCount() - lastFpsUpdate > 1000) {
            currentFps = frameCount * 1000.0f / (GetTickCount() - lastFpsUpdate);
            frameCount = 0;
            lastFpsUpdate = GetTickCount();
        }

        Sleep(16);
    }

    return 0;
}
