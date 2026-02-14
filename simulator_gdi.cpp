#include "simulator_gdi.hpp"

#include <windows.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>   // для std::min, std::max

#include "nogps_core.hpp"

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 800;
constexpr int kLeftPanelWidth = 640;
constexpr float kFrameDt = 0.016f;
constexpr float kCoreViewScale = 1.4f;

// Размеры карты (левая панель)
const int MAP_W = 640;
const int MAP_H = 800;
std::vector<bool> worldMap(MAP_W * MAP_H, false);

Vec2 dronePos(400.0f, 300.0f);
Vec2 droneVel(0.0f, 0.0f);
DroneCore core;

// ---------------------------------------------------------------------
// Генерация пещер клеточным автоматом (из новой версии)
// ---------------------------------------------------------------------
void generateCaveMap() {
    // Инициализация случайным шумом
    for (int i = 0; i < MAP_W * MAP_H; i++) {
        worldMap[i] = (rand() % 100 < 45);   // ~45% стен
    }

    // Несколько итераций клеточного автомата для сглаживания
    for (int iter = 0; iter < 5; iter++) {
        std::vector<bool> newMap = worldMap;
        for (int y = 1; y < MAP_H - 1; y++) {
            for (int x = 1; x < MAP_W - 1; x++) {
                int neighbors = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (worldMap[(y + dy) * MAP_W + (x + dx)]) neighbors++;
                    }
                }
                // Правило: если больше 4 соседей — стена, иначе — пусто
                newMap[y * MAP_W + x] = (neighbors > 4);
            }
        }
        worldMap.swap(newMap);
    }

    // Убедимся, что границы всегда стены
    for (int x = 0; x < MAP_W; x++) {
        worldMap[0 * MAP_W + x] = true;
        worldMap[(MAP_H - 1) * MAP_W + x] = true;
    }
    for (int y = 0; y < MAP_H; y++) {
        worldMap[y * MAP_W + 0] = true;
        worldMap[y * MAP_W + (MAP_W - 1)] = true;
    }
}

// ---------------------------------------------------------------------
// Вспомогательные функции рисования (без изменений)
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// Ray marching по растровой карте (вместо старого пересечения отрезков)
// ---------------------------------------------------------------------
float CastRay(Vec2 start, Vec2 dir) {
    float maxDist = 100.0f;
    float step = 0.5f;
    float dist = 0.0f;
    while (dist < maxDist) {
        float x = start.x + dir.x * dist;
        float y = start.y + dir.y * dist;
        // Проверка границ карты
        if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) {
            return maxDist;   // упёрлись в край мира
        }
        if (worldMap[(int)y * MAP_W + (int)x]) {
            return dist;      // стена
        }
        dist += step;
    }
    return maxDist;
}

// ---------------------------------------------------------------------
// Преобразование мировых координат в экранные для правой панели (без изменений)
// ---------------------------------------------------------------------
POINT toCoreView(Vec2 worldPoint, Vec2 center) {
    const float localX = (worldPoint.x - center.x) * kCoreViewScale;
    const float localY = (worldPoint.y - center.y) * kCoreViewScale;

    POINT pt;
    pt.x = static_cast<LONG>(kLeftPanelWidth + (kLeftPanelWidth / 2.0f) + localX);
    pt.y = static_cast<LONG>((kWindowHeight / 2.0f) + localY);
    return pt;
}

// ---------------------------------------------------------------------
// Оконная процедура (без изменений)
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// Отрисовка кадра (изменён только вывод стен)
// ---------------------------------------------------------------------
void drawFrame(HDC hdcMem, HDC hdcWindow, const std::vector<LidarPoint>& scan) {
    RECT rect = {0, 0, kWindowWidth, kWindowHeight};
    FillRect(hdcMem, &rect, static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));

    // Вертикальная разделительная линия
    DrawLine(hdcMem, kLeftPanelWidth, 0, kLeftPanelWidth, kWindowHeight, RGB(100, 100, 100));

    // ---- Левая панель: стены (растровая карта) ----
    // Рисуем каждый пиксель стены как белый прямоугольник 1x1
    for (int y = 0; y < MAP_H; y++) {
        for (int x = 0; x < MAP_W; x++) {
            if (worldMap[y * MAP_W + x]) {
                DrawLine(hdcMem, x, y, x, y, RGB(255, 255, 255));
            }
        }
    }

    // ---- Лидаровские лучи и точки ----
    for (const auto& p : scan) {
        const Vec2 pt = dronePos + p.toCartesian();
        DrawLine(hdcMem,
                 static_cast<int>(dronePos.x), static_cast<int>(dronePos.y),
                 static_cast<int>(pt.x), static_cast<int>(pt.y),
                 RGB(50, 50, 50));
    }
    // Реальный дрон (зелёный)
    DrawCircle(hdcMem,
               static_cast<int>(dronePos.x), static_cast<int>(dronePos.y),
               8, RGB(0, 255, 0), true);

    // ---- Правая панель: отладочная информация ядра ----
    const Vec2 estPos = core.getEstPos();

    // Линии текущих признаков (голубые)
    const auto lines = core.getDebugLines();
    for (const auto& line : lines) {
        const POINT p1 = toCoreView(line.start, estPos);
        const POINT p2 = toCoreView(line.end, estPos);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, RGB(0, 255, 255), 2);
    }

    // Граф памяти
    const auto graph = core.getGraph();
    for (const auto& node : graph) {
        COLORREF color = node.isOffloaded ? RGB(100, 100, 100) : RGB(255, 0, 0);
        const POINT nPos = toCoreView(node.position, estPos);
        DrawCircle(hdcMem, nPos.x, nPos.y, 4, color, true);

        for (int id : node.connectedNodes) {
            if (id < 0 || id >= static_cast<int>(graph.size())) continue;
            const POINT p2 = toCoreView(graph[id].position, estPos);
            DrawLine(hdcMem, nPos.x, nPos.y, p2.x, p2.y, color);
        }
    }

    // Центр (оценённое положение дрона)
    const POINT center = toCoreView(estPos, estPos);
    DrawCircle(hdcMem, center.x, center.y, 6, RGB(0, 255, 0), false);

    // Логи (жёлтый текст)
    const std::string logs = core.getLogs();
    SetTextColor(hdcMem, RGB(255, 255, 0));
    SetBkMode(hdcMem, TRANSPARENT);
    RECT textRect = {650, 600, 1200, 800};
    DrawText(hdcMem, logs.c_str(), -1, &textRect, DT_LEFT);

    // Пояснительные надписи
    TextOut(hdcMem, 10, 10, "SIMULATION REALITY", 18);
    TextOut(hdcMem, 650, 10, "CORE MEMORY VIEW", 16);
    TextOut(hdcMem, 650, 30, "Centered on estimated drone position", 35);

    // Перенос буфера на экран
    BitBlt(hdcWindow, 0, 0, kWindowWidth, kWindowHeight, hdcMem, 0, 0, SRCCOPY);
}

// ---------------------------------------------------------------------
// Инициализация мира (генерация пещер и поиск свободного места)
// ---------------------------------------------------------------------
void initWorld() {
    generateCaveMap();

    // Ищем первую свободную клетку (не стену)
    for (int y = 1; y < MAP_H - 1; y++) {
        for (int x = 1; x < MAP_W - 1; x++) {
            if (!worldMap[y * MAP_W + x]) {
                dronePos = Vec2((float)x, (float)y);
                return;
            }
        }
    }
    // Запасной вариант (если вся карта забита)
    dronePos = Vec2(100.0f, 100.0f);
}

} // namespace

// ---------------------------------------------------------------------
// Главная функция симулятора
// ---------------------------------------------------------------------
int runSimulator() {
    // Инициализация мира (вместо старой initMap)
    initWorld();

    const char CLASS_NAME[] = "NoGPS_Sim_Class";
    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.lpszClassName = CLASS_NAME;
    RegisterClass(&wc);

    HWND hwnd = CreateWindowEx(
        0,
        CLASS_NAME,
        "NoGPS Core Simulator (GDI Version) - Cave World",
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

        // Управление (WASD)
        Vec2 inputVel(0.0f, 0.0f);
        if (GetAsyncKeyState('W') & 0x8000) inputVel.y = -100.0f;
        if (GetAsyncKeyState('S') & 0x8000) inputVel.y = 100.0f;
        if (GetAsyncKeyState('A') & 0x8000) inputVel.x = -100.0f;
        if (GetAsyncKeyState('D') & 0x8000) inputVel.x = 100.0f;

        // Сканирование лидаром
        std::vector<LidarPoint> scan;
        constexpr int kNumRays = 100;
        scan.reserve(kNumRays);
        for (int i = 0; i < kNumRays; i++) {
            float angle = (i * 2.0f * PI) / kNumRays;
            float dst = CastRay(dronePos, {std::cos(angle), std::sin(angle)});
            float noise = ((std::rand() % 100) / 100.0f) * 3.0f;
            scan.push_back({angle, dst + noise});
        }

        // Обновление физики
        droneVel = inputVel + core.velocityCommand;
        dronePos = dronePos + droneVel * kFrameDt;

        // Простое удержание дрона в пределах карты (чтобы не улететь за границы)
        dronePos.x = std::max(1.0f, std::min((float)(MAP_W - 2), dronePos.x));
        dronePos.y = std::max(1.0f, std::min((float)(MAP_H - 2), dronePos.y));

        // Обновление ядра
        core.update(kFrameDt, scan, droneVel);

        // Отрисовка
        drawFrame(hdcMem, hdcWindow, scan);
        Sleep(16);
    }

    SelectObject(hdcMem, oldBmp);
    DeleteObject(hbmMem);
    DeleteDC(hdcMem);
    ReleaseDC(hwnd, hdcWindow);

    return 0;
}