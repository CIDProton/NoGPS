#include "simulator_gdi.hpp"

#include <windows.h>

#include <cstdlib>
#include <string>
#include <vector>

#include "nogps_core.hpp"

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 800;
constexpr int kLeftPanelWidth = 640;
constexpr float kFrameDt = 0.016f;
constexpr float kCoreViewScale = 1.4f;

std::vector<LineSegment> walls;
Vec2 dronePos(400.0f, 300.0f);
Vec2 droneVel(0.0f, 0.0f);
DroneCore core;

void DrawLine(HDC hdc, int x1, int y1, int x2, int y2, COLORREF color, int thickness = 1) {
    HPEN hPen = CreatePen(PS_SOLID, thickness, color);
    HGDIOBJ hOld = SelectObject(hdc, hPen);
    MoveToEx(hdc, x1, y1, nullptr);
    LineTo(hdc, x2, y2);
    SelectObject(hdc, hOld);
    DeleteObject(hPen);
}

void DrawCircle(HDC hdc, int x, int y, int r, COLORREF color, bool filled) {
    HBRUSH hBrush = filled ? CreateSolidBrush(color) : static_cast<HBRUSH>(GetStockObject(NULL_BRUSH));
    HPEN hPen = CreatePen(PS_SOLID, 1, color);
    HGDIOBJ oldBrush = SelectObject(hdc, hBrush);
    HGDIOBJ oldPen = SelectObject(hdc, hPen);
    Ellipse(hdc, x - r, y - r, x + r, y + r);
    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    if (filled) {
        DeleteObject(hBrush);
    }
    DeleteObject(hPen);
}

float CastRay(Vec2 start, Vec2 dir) {
    float minDst = 100.0f;
    const Vec2 end = start + dir * minDst;

    for (const auto& w : walls) {
        const float x1 = w.start.x;
        const float y1 = w.start.y;
        const float x2 = w.end.x;
        const float y2 = w.end.y;
        const float x3 = start.x;
        const float y3 = start.y;
        const float x4 = end.x;
        const float y4 = end.y;

        const float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (den == 0.0f) {
            continue;
        }

        const float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
        const float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
        if (t > 0.0f && t < 1.0f && u > 0.0f && u < 1.0f) {
            const Vec2 pt = {x1 + t * (x2 - x1), y1 + t * (y2 - y1)};
            const float dst = start.dist(pt);
            if (dst < minDst) {
                minDst = dst;
            }
        }
    }

    return minDst;
}

POINT toCoreView(Vec2 worldPoint, Vec2 center) {
    const float localX = (worldPoint.x - center.x) * kCoreViewScale;
    const float localY = (worldPoint.y - center.y) * kCoreViewScale;

    POINT pt;
    pt.x = static_cast<LONG>(kLeftPanelWidth + (kLeftPanelWidth / 2.0f) + localX);
    pt.y = static_cast<LONG>((kWindowHeight / 2.0f) + localY);
    return pt;
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    case WM_ERASEBKGND:
        return 1;
    default:
        return DefWindowProc(hwnd, uMsg, wParam, lParam);
    }
}

void drawFrame(HDC hdcMem, HDC hdcWindow, const std::vector<LidarPoint>& scan) {
    RECT rect = {0, 0, kWindowWidth, kWindowHeight};
    FillRect(hdcMem, &rect, static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));

    DrawLine(hdcMem, kLeftPanelWidth, 0, kLeftPanelWidth, kWindowHeight, RGB(100, 100, 100));

    for (const auto& w : walls) {
        DrawLine(hdcMem, static_cast<int>(w.start.x), static_cast<int>(w.start.y), static_cast<int>(w.end.x), static_cast<int>(w.end.y), RGB(255, 255, 255), 3);
    }
    for (const auto& p : scan) {
        const Vec2 pt = dronePos + p.toCartesian();
        DrawLine(hdcMem, static_cast<int>(dronePos.x), static_cast<int>(dronePos.y), static_cast<int>(pt.x), static_cast<int>(pt.y), RGB(50, 50, 50));
    }
    DrawCircle(hdcMem, static_cast<int>(dronePos.x), static_cast<int>(dronePos.y), 8, RGB(0, 255, 0), true);

    const Vec2 estPos = core.getEstPos();
    const auto lines = core.getDebugLines();
    for (const auto& line : lines) {
        const POINT p1 = toCoreView(line.start, estPos);
        const POINT p2 = toCoreView(line.end, estPos);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, RGB(0, 255, 255), 2);
    }

    const auto graph = core.getGraph();
    for (const auto& node : graph) {
        COLORREF color = node.isOffloaded ? RGB(100, 100, 100) : RGB(255, 0, 0);
        const POINT nPos = toCoreView(node.position, estPos);
        DrawCircle(hdcMem, nPos.x, nPos.y, 4, color, true);

        for (int id : node.connectedNodes) {
            if (id < 0 || id >= static_cast<int>(graph.size())) {
                continue;
            }
            const POINT p2 = toCoreView(graph[id].position, estPos);
            DrawLine(hdcMem, nPos.x, nPos.y, p2.x, p2.y, color);
        }
    }

    const POINT center = toCoreView(estPos, estPos);
    DrawCircle(hdcMem, center.x, center.y, 6, RGB(0, 255, 0), false);

    const std::string logs = core.getLogs();
    SetTextColor(hdcMem, RGB(255, 255, 0));
    SetBkMode(hdcMem, TRANSPARENT);
    RECT textRect = {650, 600, 1200, 800};
    DrawText(hdcMem, logs.c_str(), -1, &textRect, DT_LEFT);

    TextOut(hdcMem, 10, 10, "SIMULATION REALITY", 18);
    TextOut(hdcMem, 650, 10, "CORE MEMORY VIEW", 16);
    TextOut(hdcMem, 650, 30, "Centered on estimated drone position", 35);

    BitBlt(hdcWindow, 0, 0, kWindowWidth, kWindowHeight, hdcMem, 0, 0, SRCCOPY);
}

void initMap() {
    walls.clear();
    walls.push_back({{100, 100}, {1100, 100}});
    walls.push_back({{100, 700}, {1100, 700}});
    walls.push_back({{100, 100}, {100, 700}});
    walls.push_back({{1100, 100}, {1100, 700}});
    walls.push_back({{400, 100}, {400, 500}});
    walls.push_back({{800, 700}, {800, 300}});
}

}  // namespace

int runSimulator() {
    initMap();

    const char CLASS_NAME[] = "NoGPS_Sim_Class";
    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.lpszClassName = CLASS_NAME;
    RegisterClass(&wc);

    HWND hwnd = CreateWindowEx(
        0,
        CLASS_NAME,
        "NoGPS Core Simulator (GDI Version)",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        kWindowWidth,
        kWindowHeight,
        nullptr,
        nullptr,
        GetModuleHandle(nullptr),
        nullptr);

    HDC hdcWindow = GetDC(hwnd);
    HDC hdcMem = CreateCompatibleDC(hdcWindow);
    HBITMAP hbmMem = CreateCompatibleBitmap(hdcWindow, kWindowWidth, kWindowHeight);
    HGDIOBJ oldBmp = SelectObject(hdcMem, hbmMem);

    bool running = true;
    while (running) {
        MSG msg = {};
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running = false;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        Vec2 inputVel(0.0f, 0.0f);
        if (GetAsyncKeyState('W') & 0x8000) {
            inputVel.y = -100.0f;
        }
        if (GetAsyncKeyState('S') & 0x8000) {
            inputVel.y = 100.0f;
        }
        if (GetAsyncKeyState('A') & 0x8000) {
            inputVel.x = -100.0f;
        }
        if (GetAsyncKeyState('D') & 0x8000) {
            inputVel.x = 100.0f;
        }

        std::vector<LidarPoint> scan;
        constexpr int kNumRays = 100;
        scan.reserve(kNumRays);
        for (int i = 0; i < kNumRays; i++) {
            const float angle = (i * 2.0f * PI) / kNumRays;
            const float dst = CastRay(dronePos, {std::cos(angle), std::sin(angle)});
            const float noise = ((std::rand() % 100) / 100.0f) * 3.0f;
            scan.push_back({angle, dst + noise});
        }

        droneVel = inputVel + core.velocityCommand;
        dronePos = dronePos + droneVel * kFrameDt;
        core.update(kFrameDt, scan, droneVel);

        drawFrame(hdcMem, hdcWindow, scan);
        Sleep(16);
    }

    SelectObject(hdcMem, oldBmp);
    DeleteObject(hbmMem);
    DeleteDC(hdcMem);
    ReleaseDC(hwnd, hdcWindow);

    return 0;
}
