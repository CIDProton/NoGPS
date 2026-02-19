#include "simulator_gdi.hpp"

#include <windows.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include "nogps_core.hpp"

namespace {
// ============================================================================
// КОНФИГУРАЦИЯ И КОНСТАНТЫ
// ============================================================================
constexpr int kLeftPanelWidth = 700;
constexpr int kRightPanelWidth = 350;
constexpr float kFrameDt = 0.016f;
constexpr float kCoreViewScale = 0.7f;
constexpr int kTargetFPS = 60;

// Размеры карты
constexpr int MAP_W = 800;
constexpr int MAP_H = 900;

// Цветовая схема (профессиональная палитра)
namespace Colors {
    constexpr COLORREF Black = RGB(0, 0, 0);
    constexpr COLORREF White = RGB(255, 255, 255);
    constexpr COLORREF DarkGray = RGB(30, 30, 30);
    constexpr COLORREF MediumGray = RGB(60, 60, 60);
    constexpr COLORREF LightGray = RGB(120, 120, 120);
    
    // Стены и мир
    constexpr COLORREF Wall = RGB(200, 200, 200);
    constexpr COLORREF WallEdge = RGB(100, 100, 100);
    
    // Дрон
    constexpr COLORREF DroneBody = RGB(0, 200, 100);
    constexpr COLORREF DroneBodyStuck = RGB(255, 50, 50);
    constexpr COLORREF DroneOrientation = RGB(0, 255, 255);
    constexpr COLORREF VelocityVector = RGB(100, 100, 255);
    
    // Лидар
    constexpr COLORREF LidarRay = RGB(80, 80, 80);
    constexpr COLORREF LidarHit = RGB(0, 255, 100);
    constexpr COLORREF LidarMiss = RGB(255, 100, 100);
    
    // Граф и карта
    constexpr COLORREF GlobalLineHighConf = RGB(255, 200, 0);
    constexpr COLORREF GlobalLineMedConf = RGB(255, 150, 50);
    constexpr COLORREF GlobalLineLowConf = RGB(150, 100, 50);
    constexpr COLORREF GraphNodeActive = RGB(0, 255, 0);
    constexpr COLORREF GraphNodeOffloaded = RGB(80, 80, 80);
    constexpr COLORREF GraphEdge = RGB(0, 150, 150);
    
    // UI элементы
    constexpr COLORREF PanelBg = RGB(20, 25, 30);
    constexpr COLORREF PanelBorder = RGB(60, 80, 100);
    constexpr COLORREF TextNormal = RGB(180, 220, 255);
    constexpr COLORREF TextHighlight = RGB(100, 255, 150);
    constexpr COLORREF TextWarning = RGB(255, 200, 50);
    constexpr COLORREF TextCritical = RGB(255, 80, 80);
    
    // Индикаторы доверия
    constexpr COLORREF ConfHigh = RGB(0, 200, 100);
    constexpr COLORREF ConfMedium = RGB(200, 150, 0);
    constexpr COLORREF ConfLow = RGB(200, 50, 50);
}

// ============================================================================
// ФИЗИКА ДРОНА (реалистичная модель)
// ============================================================================
struct DronePhysics {
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
    Vec2 localAcceleration;  // Ускорение в локальных координатах
    float orientation = 0.0f;
    float angularVelocity = 0.0f;
    float angularAcceleration = 0.0f;
    
    // Параметры физики
    float mass = 1.0f;
    float drag = 0.92f;           // Сопротивление воздуха
    float angularDrag = 0.85f;    // Угловое затухание
    float maxThrust = 200.0f;
    float maxTorque = 5.0f;
    float maxVelocity = 300.0f;
    float bodyRadius = 6.0f;
    
    // Состояние
    bool isStuck = false;
    float stuckTime = 0.0f;
    Vec2 lastValidPosition;
    
    void update(float dt) {
        lastValidPosition = position;
        
        // Интеграция скорости с сопротивлением
        velocity = velocity + acceleration * dt;
        
        // Ограничение максимальной скорости
        if (velocity.length() > maxVelocity) {
            velocity = velocity.normalized() * maxVelocity;
        }
        
        // Применение сопротивления воздуха
        velocity = velocity * drag;
        
        // Интеграция позиции
        position = position + velocity * dt;
        
        // Интеграция вращения
        angularVelocity = angularVelocity + angularAcceleration * dt;
        angularVelocity = angularVelocity * angularDrag;
        orientation = orientation + angularVelocity * dt;
        
        // Нормализация угла
        while (orientation > PI) orientation -= 2.0f * PI;
        while (orientation < -PI) orientation += 2.0f * PI;
        
        // Сброс ускорений (задаются каждый кадр)
        acceleration = Vec2(0, 0);
        angularAcceleration = 0.0f;
        
        // Детекция застревания
        float speed = velocity.length();
        if (speed < 5.0f && localAcceleration.length() > 20.0f) {
            isStuck = true;
            stuckTime += dt;
        } else {
            isStuck = false;
            stuckTime = 0.0f;
        }
    }
    
    void applyThrust(Vec2 localThrust) {
        // Ограничение максимальной тяги
        if (localThrust.length() > maxThrust) {
            localThrust = localThrust.normalized() * maxThrust;
        }
        
        localAcceleration = localThrust / mass;
        
        // Преобразование в глобальные координаты
        Vec2 globalThrust = localThrust.rotated(orientation);
        acceleration = acceleration + globalThrust / mass;
    }
    
    void applyTorque(float torque) {
        if (std::fabs(torque) > maxTorque) {
            torque = (torque > 0 ? 1 : -1) * maxTorque;
        }
        angularAcceleration += torque / mass;
    }
    
    void handleCollision(const Vec2& collisionNormal, float penetration) {
        // Отталкивание от стены с физикой
        Vec2 pushOut = collisionNormal * (penetration + 1.0f);
        position = position + pushOut;
        
        // Отражение скорости с потерей энергии
        float normalVel = velocity.dot(collisionNormal);
        if (normalVel < 0) {
            velocity = velocity - collisionNormal * normalVel * 1.5f;
            velocity = velocity * 0.3f;  // Потеря энергии при ударе
        }
    }
};

// ============================================================================
// ГЛОБАЛЬНОЕ СОСТОЯНИЕ
// ============================================================================
std::vector<bool> worldMap(MAP_W * MAP_H, false);
uint32_t mapPixels[MAP_H][MAP_W];
DronePhysics drone;
DroneCore core;
Vec2 mapSpawn(100.0f, 100.0f);

// FPS и статистика
DWORD lastFpsUpdate = 0;
int frameCount = 0;
float currentFps = 0.0f;
float avgFrameTime = 0.0f;

// Ввод
struct InputState {
    bool forward = false;
    bool backward = false;
    bool left = false;
    bool right = false;
    bool rotateLeft = false;
    bool rotateRight = false;
    bool strafeLeft = false;
    bool strafeRight = false;
    bool reset = false;
} input;

// Настройки отображения
struct DisplaySettings {
    bool showLidarRays = true;
    bool showGraphEdges = true;
    bool showConfidenceColors = true;
    bool showVelocityVector = true;
    bool showOrientationIndicator = true;
    float viewZoom = 1.0f;
} display;

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================
inline bool inBounds(int x, int y) {
    return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H;
}

bool isWallCell(int x, int y) {
    if (!inBounds(x, y)) return true;
    return worldMap[y * MAP_W + x];
}

void setWallCell(int x, int y, bool isWall) {
    if (inBounds(x, y)) {
        worldMap[y * MAP_W + x] = isWall;
    }
}

void carveDisk(int cx, int cy, int radius) {
    for (int y = cy - radius; y <= cy + radius; ++y) {
        for (int x = cx - radius; x <= cx + radius; ++x) {
            if (!inBounds(x, y)) continue;
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius) {
                setWallCell(x, y, false);
            }
        }
    }
}

void carveEllipseRoom(int cx, int cy, int rx, int ry) {
    for (int y = cy - ry; y <= cy + ry; ++y) {
        for (int x = cx - rx; x <= cx + rx; ++x) {
            if (!inBounds(x, y)) continue;
            const float nx = static_cast<float>(x - cx) / static_cast<float>(rx);
            const float ny = static_cast<float>(y - cy) / static_cast<float>(ry);
            if (nx * nx + ny * ny <= 1.0f) {
                setWallCell(x, y, false);
            }
        }
    }
}

void carveTunnel(Vec2 a, Vec2 b, int radius) {
    Vec2 d = b - a;
    const float len = d.length();
    if (len < 1.0f) {
        carveDisk(static_cast<int>(a.x), static_cast<int>(a.y), radius);
        return;
    }

    const int steps = static_cast<int>(len / 2.0f);
    Vec2 dir = d * (1.0f / len);
    Vec2 p = a;
    for (int i = 0; i <= steps; ++i) {
        carveDisk(static_cast<int>(p.x), static_cast<int>(p.y), radius);
        const float sideJitter = static_cast<float>((rand() % 100) - 50) * 0.02f;
        Vec2 jitter(-dir.y * sideJitter, dir.x * sideJitter);
        p = p + dir * 2.0f + jitter;
    }
}

bool isAreaFree(Vec2 p, int radius) {
    int minX = std::max(0, static_cast<int>(p.x) - radius);
    int maxX = std::min(MAP_W - 1, static_cast<int>(p.x) + radius);
    int minY = std::max(0, static_cast<int>(p.y) - radius);
    int maxY = std::min(MAP_H - 1, static_cast<int>(p.y) + radius);
    
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            if (isWallCell(x, y)) return false;
        }
    }
    return true;
}

Vec2 pickSafeSpawn(const std::vector<Vec2>& roomCenters) {
    for (const Vec2& c : roomCenters) {
        if (isAreaFree(c, 15)) return c;
    }
    for (int i = 0; i < 5000; ++i) {
        Vec2 p(static_cast<float>(rand() % (MAP_W - 60) + 30),
               static_cast<float>(rand() % (MAP_H - 60) + 30));
        if (isAreaFree(p, 15)) return p;
    }
    return Vec2(100.0f, 100.0f);
}

void updateMapBuffer() {
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            mapPixels[y][x] = worldMap[y * MAP_W + x] ? 0x00C8C8C8 : 0x001A1A1A;
        }
    }
}

// ============================================================================
// ГЕНЕРАЦИЯ КАРТЫ (улучшенная)
// ============================================================================
void generateCaveMap() {
    std::fill(worldMap.begin(), worldMap.end(), true);

    std::vector<Vec2> roomCenters;
    const int roomCount = 12 + rand() % 8;
    roomCenters.reserve(roomCount);

    // Создаём комнаты разных размеров
    for (int i = 0; i < roomCount; ++i) {
        const int cx = 60 + rand() % (MAP_W - 120);
        const int cy = 60 + rand() % (MAP_H - 120);
        const int rx = 25 + rand() % 80;
        const int ry = 22 + rand() % 70;
        carveEllipseRoom(cx, cy, rx, ry);
        roomCenters.push_back(Vec2(static_cast<float>(cx), static_cast<float>(cy)));
    }

    // Соединяем комнаты туннелями
    for (size_t i = 1; i < roomCenters.size(); ++i) {
        const int r = 12 + rand() % 8;
        carveTunnel(roomCenters[i - 1], roomCenters[i], r);
    }

    // Случайные соединения для циклов
    for (int i = 0; i < roomCount / 2; ++i) {
        int a = rand() % roomCount;
        int b = rand() % roomCount;
        if (a == b) b = (b + 1) % roomCount;
        carveTunnel(roomCenters[a], roomCenters[b], 10 + rand() % 6);
    }

    // Боковые пещеры
    const int sideCaves = 20 + rand() % 16;
    for (int i = 0; i < sideCaves; ++i) {
        const int cx = 40 + rand() % (MAP_W - 80);
        const int cy = 40 + rand() % (MAP_H - 80);
        carveEllipseRoom(cx, cy, 10 + rand() % 25, 10 + rand() % 22);
    }

    // Сглаживание (cellular automata)
    for (int iter = 0; iter < 4; ++iter) {
        std::vector<bool> next = worldMap;
        for (int y = 1; y < MAP_H - 1; ++y) {
            for (int x = 1; x < MAP_W - 1; ++x) {
                int wallNeighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        if (isWallCell(x + dx, y + dy)) ++wallNeighbors;
                    }
                }
                if (worldMap[y * MAP_W + x]) {
                    next[y * MAP_W + x] = (wallNeighbors >= 4);
                } else {
                    next[y * MAP_W + x] = (wallNeighbors >= 6);
                }
            }
        }
        worldMap.swap(next);
    }

    // Границы карты
    for (int x = 0; x < MAP_W; ++x) {
        setWallCell(x, 0, true);
        setWallCell(x, MAP_H - 1, true);
    }
    for (int y = 0; y < MAP_H; ++y) {
        setWallCell(0, y, true);
        setWallCell(MAP_W - 1, y, true);
    }

    mapSpawn = pickSafeSpawn(roomCenters);
    carveDisk(static_cast<int>(mapSpawn.x), static_cast<int>(mapSpawn.y), 18);

    // Сброс дрона
    drone.position = mapSpawn;
    drone.velocity = Vec2(0, 0);
    drone.acceleration = Vec2(0, 0);
    drone.orientation = 0.0f;
    drone.angularVelocity = 0.0f;
    drone.isStuck = false;
    drone.stuckTime = 0.0f;

    core.reset(mapSpawn);  // 🆕 Передаём позицию спавна
    updateMapBuffer();
}

// ============================================================================
// ОТРИСОВКА (улучшенная)
// ============================================================================
void DrawLine(HDC hdc, int x1, int y1, int x2, int y2, COLORREF color, int thickness = 1) {
    HPEN hPen = CreatePen(PS_SOLID, thickness, color);
    HGDIOBJ hOld = SelectObject(hdc, hPen);
    MoveToEx(hdc, x1, y1, nullptr);
    LineTo(hdc, x2, y2);
    SelectObject(hdc, hOld);
    DeleteObject(hPen);
}

void DrawLineAlpha(HDC hdc, int x1, int y1, int x2, int y2, COLORREF color, int thickness = 1, float alpha = 1.0f) {
    // Упрощённая альфа-смесь (для GDI)
    if (alpha < 1.0f) {
        int r = GetRValue(color);
        int g = GetGValue(color);
        int b = GetBValue(color);
        r = static_cast<int>(r * alpha);
        g = static_cast<int>(g * alpha);
        b = static_cast<int>(b * alpha);
        color = RGB(r, g, b);
    }
    DrawLine(hdc, x1, y1, x2, y2, color, thickness);
}

void DrawCircle(HDC hdc, int x, int y, int r, COLORREF color, bool filled) {
    HBRUSH hBrush = filled ? CreateSolidBrush(color) : (HBRUSH)GetStockObject(NULL_BRUSH);
    HPEN hPen = CreatePen(PS_SOLID, 1, color);
    HGDIOBJ oldB = SelectObject(hdc, hBrush);
    HGDIOBJ oldP = SelectObject(hdc, hPen);
    Ellipse(hdc, x - r, y - r, x + r, y + r);
    SelectObject(hdc, oldB);
    SelectObject(hdc, oldP);
    if (filled) DeleteObject(hBrush);
    DeleteObject(hPen);
}

void DrawRect(HDC hdc, int x, int y, int w, int h, COLORREF borderColor, COLORREF fillColor) {
    HBRUSH hBrush = CreateSolidBrush(fillColor);
    HPEN hPen = CreatePen(PS_SOLID, 1, borderColor);
    HGDIOBJ oldB = SelectObject(hdc, hBrush);
    HGDIOBJ oldP = SelectObject(hdc, hPen);
    Rectangle(hdc, x, y, x + w, y + h);
    SelectObject(hdc, oldB);
    SelectObject(hdc, oldP);
    DeleteObject(hBrush);
    DeleteObject(hPen);
}

void DrawPanel(HDC hdc, int x, int y, int w, int h, const char* title) {
    // Фон панели
    DrawRect(hdc, x, y, w, h, Colors::PanelBorder, Colors::PanelBg);
    
    // Заголовок
    if (title && title[0]) {
        SetTextColor(hdc, Colors::TextHighlight);
        SetBkMode(hdc, TRANSPARENT);
        RECT titleRect = {x + 10, y + 5, x + w - 10, y + 22};
        DrawTextA(hdc, title, -1, &titleRect, DT_LEFT | DT_SINGLELINE);
        
        // Линия под заголовком
        DrawLine(hdc, x + 5, y + 25, x + w - 5, y + 25, Colors::PanelBorder, 1);
    }
}

void DrawProgressBar(HDC hdc, int x, int y, int w, int h, float value, 
                     COLORREF lowColor, COLORREF highColor) {
    // Фон
    DrawRect(hdc, x, y, w, h, Colors::MediumGray, Colors::DarkGray);
    
    // Заполнение
    value = std::clamp(value, 0.0f, 1.0f);
    int fillW = static_cast<int>(w * value);
    
    // Интерполяция цвета
    int r1 = GetRValue(lowColor), g1 = GetGValue(lowColor), b1 = GetBValue(lowColor);
    int r2 = GetRValue(highColor), g2 = GetGValue(highColor), b2 = GetBValue(highColor);
    int r = static_cast<int>(r1 + (r2 - r1) * value);
    int g = static_cast<int>(g1 + (g2 - g1) * value);
    int b = static_cast<int>(b1 + (b2 - b1) * value);
    
    if (fillW > 0) {
        DrawRect(hdc, x + 1, y + 1, fillW - 2, h - 2, Colors::Black, RGB(r, g, b));
    }
}

COLORREF getConfidenceColor(float confidence) {
    if (confidence > 0.7f) return Colors::ConfHigh;
    if (confidence > 0.4f) return Colors::ConfMedium;
    return Colors::ConfLow;
}

float CastRay(Vec2 start, Vec2 dir) {
    for (float d = 0; d < 500; d += 2.0f) {
        int ix = static_cast<int>(start.x + dir.x * d);
        int iy = static_cast<int>(start.y + dir.y * d);
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
    pt.x = static_cast<LONG>(panelCenterX + (worldPoint.x - center.x) * kCoreViewScale * display.viewZoom);
    pt.y = static_cast<LONG>(panelCenterY + (worldPoint.y - center.y) * kCoreViewScale * display.viewZoom);
    return pt;
}

// ============================================================================
// ОТРИСОВКА UI ПАНЕЛЕЙ
// ============================================================================
void drawStatsPanel(HDC hdc, int winW, int winH) {
    int panelX = kLeftPanelWidth + 10;
    int panelY = 10;
    int panelW = kRightPanelWidth - 20;
    int lineHeight = 20;
    int y = panelY + 30;
    
    // Панель производительности
    DrawPanel(hdc, panelX, panelY, panelW, 140, "PERFORMANCE");
    
    char buf[128];
    SetTextColor(hdc, Colors::TextNormal);
    SetBkMode(hdc, TRANSPARENT);
    
    sprintf_s(buf, "FPS: %.1f", currentFps);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "Frame: %.2f ms", avgFrameTime);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "Lines: %zu", core.getGlobalLines().size());
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    auto stats = core.getStats();
    sprintf_s(buf, "Nodes: %zu (off: %zu, merged: %zu)", 
              stats.totalNodes, stats.offloadedNodes, stats.mergedNodes);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "RAM: %zu KB", stats.ramUsage / 1024);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    
    // Панель доверия
    panelY += 150;
    DrawPanel(hdc, panelX, panelY, panelW, 180, "POSE CONFIDENCE");
    y = panelY + 30;
    
    auto conf = core.getPoseConfidence();
    
    sprintf_s(buf, "Overall: %.0f%%", conf.value * 100.0f);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    DrawProgressBar(hdc, panelX + 15, y + 15, panelW - 30, 12, conf.value, 
                    Colors::ConfLow, Colors::ConfHigh);
    y += 35;
    
    sprintf_s(buf, "Lidar Match: %.0f%%", conf.lidarAlignmentScore * 100.0f);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "IMU Drift: %.2f", conf.imuDriftEstimate);
    SetTextColor(hdc, conf.imuDriftEstimate > 0.5f ? Colors::TextWarning : Colors::TextNormal);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    SetTextColor(hdc, Colors::TextNormal);
    y += lineHeight;
    
    sprintf_s(buf, "Lidar Weight: %.0f%%", conf.getLidarWeight() * 100.0f);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    DrawProgressBar(hdc, panelX + 15, y + 15, panelW - 30, 12, conf.getLidarWeight(),
                    Colors::ConfLow, Colors::ConfHigh);
    y += 35;
    
    // Статус застревания
    if (drone.isStuck) {
        SetTextColor(hdc, Colors::TextCritical);
        sprintf_s(buf, "STATUS: STUCK (%.1fs)", drone.stuckTime);
    } else {
        SetTextColor(hdc, Colors::TextHighlight);
        sprintf_s(buf, "STATUS: NORMAL");
    }
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    
    // Панель управления
    panelY += 190;
    DrawPanel(hdc, panelX, panelY, panelW, 200, "CONTROLS");
    y = panelY + 30;
    
    SetTextColor(hdc, Colors::TextNormal);
    const char* controls[] = {
        "W/S - Thrust Forward/Back",
        "Q/E - Rotate Left/Right",
        "A/D - Strafe Left/Right",
        "R - Reset Map & Core",
        "Z/X - Zoom In/Out",
        "1-5 - Toggle Display",
        "+/- - Adjust Params"
    };
    
    for (const char* ctrl : controls) {
        TextOutA(hdc, panelX + 15, y, ctrl, static_cast<int>(strlen(ctrl)));
        y += lineHeight;
    }
    
    // Панель параметров ядра
    panelY += 210;
    DrawPanel(hdc, panelX, panelY, panelW, 180, "CORE PARAMETERS");
    y = panelY + 30;
    
    sprintf_s(buf, "Merge Tol: %.1f", core.params.mergeTolerance);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "Split Tol: %.1f", core.params.splitTolerance);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "Node Dist: %.1f", core.params.newNodeDist);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "Reflex Dist: %.1f", core.params.reflexDist);
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
    y += lineHeight;
    
    sprintf_s(buf, "RAM Limit: %zu MB", core.params.ramLimitBytes / (1024 * 1024));
    TextOutA(hdc, panelX + 15, y, buf, static_cast<int>(strlen(buf)));
}

void drawLogsPanel(HDC hdc, int winW, int winH) {
    int panelX = kLeftPanelWidth + 10;
    int panelY = winH - 150;
    int panelW = kRightPanelWidth - 20;
    int panelH = 140;
    
    DrawPanel(hdc, panelX, panelY, panelW, panelH, "SYSTEM LOGS");
    
    std::string logs = core.getLogs();
    RECT textRect = {panelX + 10, panelY + 30, panelX + panelW - 10, panelY + panelH - 10};
    
    SetTextColor(hdc, Colors::TextNormal);
    SetBkMode(hdc, TRANSPARENT);
    DrawTextA(hdc, logs.c_str(), -1, &textRect, DT_LEFT | DT_WORDBREAK);
}

// ============================================================================
// ОТРИСОВКА ОСНОВНОГО ВИДА
// ============================================================================
void drawMainView(HDC hdcMem, HWND hwnd, const std::vector<LidarPoint>& scan) {
    RECT clRect;
    GetClientRect(hwnd, &clRect);
    const int winW = clRect.right;
    const int winH = clRect.bottom;

    // Очистка фона
    FillRect(hdcMem, &clRect, (HBRUSH)GetStockObject(BLACK_BRUSH));

    // Разделительная линия
    DrawLine(hdcMem, kLeftPanelWidth, 0, kLeftPanelWidth, winH, Colors::PanelBorder, 2);

    // Отрисовка карты мира
    BITMAPINFO bmi = {0};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = MAP_W;
    bmi.bmiHeader.biHeight = -MAP_H;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
    StretchDIBits(hdcMem, 0, 0, MAP_W, MAP_H, 0, 0, MAP_W, MAP_H, 
                  mapPixels, &bmi, DIB_RGB_COLORS, SRCCOPY);

    Vec2 estPos = core.getEstPos();

    // Лидар лучи
    if (display.showLidarRays) {
        for (const auto& p : scan) {
            Vec2 end = drone.position + p.toCartesian();
            bool hit = p.dist < 490.0f;
            COLORREF rayColor = hit ? Colors::LidarHit : Colors::LidarMiss;
            DrawLineAlpha(hdcMem, static_cast<int>(drone.position.x), 
                         static_cast<int>(drone.position.y),
                         static_cast<int>(end.x), static_cast<int>(end.y),
                         rayColor, 1, 0.4f);
        }
    }

    // Глобальные линии карты (с цветом по доверию)
    for (const auto& l : core.getGlobalLines()) {
        COLORREF lineColor;
        if (display.showConfidenceColors) {
            // Получаем доверие из глобальных линий (нужно добавить в core)
            lineColor = Colors::GlobalLineMedConf;
        } else {
            lineColor = Colors::GlobalLineHighConf;
        }
        
        POINT p1 = toCoreView(l.start, estPos, winW, winH);
        POINT p2 = toCoreView(l.end, estPos, winW, winH);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, lineColor, 1);
    }

    // Рёбра графа
    if (display.showGraphEdges) {
        const auto& edges = core.getGraphEdges();
        const auto& nodes = core.getGraph();
        for (const auto& edge : edges) {
            if (edge.fromId < 0 || edge.toId < 0 ||
                edge.fromId >= static_cast<int>(nodes.size()) || 
                edge.toId >= static_cast<int>(nodes.size())) continue;
            
            POINT p1 = toCoreView(nodes[edge.fromId].position, estPos, winW, winH);
            POINT p2 = toCoreView(nodes[edge.toId].position, estPos, winW, winH);
            DrawLineAlpha(hdcMem, p1.x, p1.y, p2.x, p2.y, Colors::GraphEdge, 1, 0.6f);
        }
    }

    // Узлы графа
    for (const auto& n : core.getGraph()) {
        POINT p = toCoreView(n.position, estPos, winW, winH);
        COLORREF nodeColor = n.isOffloaded ? Colors::GraphNodeOffloaded : Colors::GraphNodeActive;
        int radius = n.isOffloaded ? 2 : 4;
        DrawCircle(hdcMem, p.x, p.y, radius, nodeColor, true);
    }

    // Дрон (реальная позиция)
    COLORREF droneColor = drone.isStuck ? Colors::DroneBodyStuck : Colors::DroneBody;
    DrawCircle(hdcMem, static_cast<int>(drone.position.x), 
               static_cast<int>(drone.position.y), 5, droneColor, true);

    // Индикатор ориентации
    if (display.showOrientationIndicator) {
        Vec2 orientEnd = drone.position + Vec2(std::cos(drone.orientation), 
                                                std::sin(drone.orientation)) * 15.0f;
        DrawLine(hdcMem, static_cast<int>(drone.position.x), 
                static_cast<int>(drone.position.y),
                static_cast<int>(orientEnd.x), static_cast<int>(orientEnd.y),
                Colors::DroneOrientation, 2);
    }

    // Вектор скорости
    if (display.showVelocityVector) {
        Vec2 velEnd = drone.position + drone.velocity * 0.5f;
        DrawLine(hdcMem, static_cast<int>(drone.position.x),
                static_cast<int>(drone.position.y),
                static_cast<int>(velEnd.x), static_cast<int>(velEnd.y),
                Colors::VelocityVector, 2);
    }

    // Оценка позиции ядра (отличается от реальной при дрифте)
    POINT c = toCoreView(estPos, estPos, winW, winH);
    DrawCircle(hdcMem, c.x, c.y, 7, Colors::DroneBody, false);
    DrawCircle(hdcMem, c.x, c.y, 3, Colors::DroneOrientation, true);
}

void drawFrame(HDC hdcMem, HWND hwnd, const std::vector<LidarPoint>& scan) {
    drawMainView(hdcMem, hwnd, scan);
    
    RECT clRect;
    GetClientRect(hwnd, &clRect);
    drawStatsPanel(hdcMem, clRect.right, clRect.bottom);
    drawLogsPanel(hdcMem, clRect.right, clRect.bottom);

    // Заголовок окна
    char title[256];
    auto stats = core.getStats();
    sprintf_s(title, "NoGPS Sim v2.0 | FPS: %.1f | Lines: %zu | Nodes: %zu | RAM: %zuKB | %s",
              currentFps, core.getGlobalLines().size(), stats.totalNodes, 
              stats.ramUsage / 1024, drone.isStuck ? "STUCK!" : "OK");
    SetWindowTextA(hwnd, title);

    // Копирование на экран
    HDC hdcWin = GetDC(hwnd);
    BitBlt(hdcWin, 0, 0, clRect.right, clRect.bottom, hdcMem, 0, 0, SRCCOPY);
    ReleaseDC(hwnd, hdcWin);
}

// ============================================================================
// ОБРАБОТКА ВВОДА
// ============================================================================
void processInput() {
    input.forward = (GetAsyncKeyState('W') & 0x8000) != 0;
    input.backward = (GetAsyncKeyState('S') & 0x8000) != 0;
    input.rotateLeft = (GetAsyncKeyState('Q') & 0x8000) != 0;
    input.rotateRight = (GetAsyncKeyState('E') & 0x8000) != 0;
    input.strafeLeft = (GetAsyncKeyState('A') & 0x8000) != 0;
    input.strafeRight = (GetAsyncKeyState('D') & 0x8000) != 0;
    input.reset = (GetAsyncKeyState('R') & 0x8000) != 0;

    // Настройка отображения
    if (GetAsyncKeyState('Z') & 0x8000) display.viewZoom = std::min(2.0f, display.viewZoom + 0.05f);
    if (GetAsyncKeyState('X') & 0x8000) display.viewZoom = std::max(0.3f, display.viewZoom - 0.05f);
    if (GetAsyncKeyState('1') & 0x8000) display.showLidarRays = !display.showLidarRays;
    if (GetAsyncKeyState('2') & 0x8000) display.showGraphEdges = !display.showGraphEdges;
    if (GetAsyncKeyState('3') & 0x8000) display.showConfidenceColors = !display.showConfidenceColors;
    if (GetAsyncKeyState('4') & 0x8000) display.showVelocityVector = !display.showVelocityVector;
    if (GetAsyncKeyState('5') & 0x8000) display.showOrientationIndicator = !display.showOrientationIndicator;

    // Параметры ядра
    if (GetAsyncKeyState(VK_OEM_PLUS) & 0x8000) core.params.mergeTolerance += 0.2f;
    if (GetAsyncKeyState(VK_OEM_MINUS) & 0x8000) core.params.mergeTolerance = std::max(1.0f, core.params.mergeTolerance - 0.2f);
}

void applyInputToDrone() {
    Vec2 localThrust(0.0f, 0.0f);
    
    if (input.forward) localThrust.y -= 1.0f;
    if (input.backward) localThrust.y += 0.5f;  // Торможение слабее
    if (input.strafeLeft) localThrust.x -= 0.5f;
    if (input.strafeRight) localThrust.x += 0.5f;

    // Нормализация ввода
    if (localThrust.length() > 1.0f) {
        localThrust = localThrust.normalized();
    }

    // Применение тяги
    drone.applyThrust(localThrust * 150.0f);

    // Вращение
    if (input.rotateLeft) drone.applyTorque(-3.0f);
    if (input.rotateRight) drone.applyTorque(3.0f);
}

void handleCollisions() {
    const float bodyRadius = drone.bodyRadius;
    
    // Проверка коллизий по осям с отталкиванием
    Vec2 target = drone.position + drone.velocity * kFrameDt;

    // Проверка по X
    Vec2 tryX(target.x, drone.position.y);
    if (!isAreaFree(tryX, static_cast<int>(bodyRadius))) {
        // Коллизия по X - определяем нормаль
        Vec2 normal(drone.velocity.x > 0 ? -1.0f : 1.0f, 0.0f);
        drone.handleCollision(normal, bodyRadius);
    } else {
        drone.position.x = tryX.x;
    }

    // Проверка по Y
    Vec2 tryY(drone.position.x, target.y);
    if (!isAreaFree(tryY, static_cast<int>(bodyRadius))) {
        Vec2 normal(drone.velocity.y > 0 ? -1.0f : 1.0f, 0.0f);
        drone.handleCollision(normal, bodyRadius);
    } else {
        drone.position.y = tryY.y;
    }

    // Ограничение границами карты
    drone.position.x = std::clamp(drone.position.x, 10.0f, static_cast<float>(MAP_W - 10));
    drone.position.y = std::clamp(drone.position.y, 10.0f, static_cast<float>(MAP_H - 10));
}

// ============================================================================
// ОКНО И ЦИКЛ
// ============================================================================
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    if (uMsg == WM_DESTROY) {
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

} // namespace

// ============================================================================
// ТОЧКА ВХОДА
// ============================================================================
int runSimulator() {
    // Инициализация
    CreateDirectoryA("cache", NULL);
    srand(static_cast<unsigned>(time(nullptr)));
    generateCaveMap();

    // Регистрация окна
    WNDCLASSA wc = {0};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandleA(NULL);
    wc.lpszClassName = "NoGPS_Sim_v2";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(
        0,
        wc.lpszClassName,
        "NoGPS Core Simulator v2.0",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        100, 100, 1100, 850,
        NULL, NULL, wc.hInstance, NULL);

    // Создание DC для двойной буферизации
    HDC hdcWin = GetDC(hwnd);
    HDC hdcMem = CreateCompatibleDC(hdcWin);
    HBITMAP hbmMem = CreateCompatibleBitmap(hdcWin, 1920, 1080);
    SelectObject(hdcMem, hbmMem);

    // Основной цикл
    DWORD lastTime = GetTickCount();
    
    while (true) {
        DWORD currentTime = GetTickCount();
        float dt = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;
        
        // Ограничение dt для стабильности
        dt = std::min(dt, 0.05f);

        // Обработка сообщений
        MSG msg;
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                DeleteObject(hbmMem);
                DeleteDC(hdcMem);
                ReleaseDC(hwnd, hdcWin);
                return 0;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // Сброс
        if (input.reset) {
            generateCaveMap();
            input.reset = false;
        }

        // Обработка ввода
        processInput();
        applyInputToDrone();

        // Генерация скана лидара
        std::vector<LidarPoint> scan;
        scan.reserve(120);
        for (int i = 0; i < 120; ++i) {
            const float a = (i * 2.0f * PI) / 120.0f;
            const Vec2 rayDir(std::cos(a), std::sin(a));
            const float noisyDist = CastRay(drone.position, rayDir) + 
                                    static_cast<float>(rand() % 100) / 50.0f;
            scan.push_back({a, noisyDist});
        }

        // Физическое обновление дрона
        drone.update(kFrameDt);
        handleCollisions();

        // Создание инерциальных данных для ядра
        InertialData inertial;
        inertial.acceleration = drone.localAcceleration;
        inertial.angularVelocity = drone.angularVelocity;
        inertial.orientation = drone.orientation;
        inertial.deltaTime = kFrameDt;

        // Обновление ядра
        core.update(currentTime / 1000.0f, scan, inertial);

        // Отрисовка
        drawFrame(hdcMem, hwnd, scan);

        // FPS counter
        ++frameCount;
        if (currentTime - lastFpsUpdate > 1000) {
            currentFps = frameCount * 1000.0f / (currentTime - lastFpsUpdate);
            avgFrameTime = 1000.0f / currentFps;
            frameCount = 0;
            lastFpsUpdate = currentTime;
        }

        // Ограничение FPS
        DWORD frameTime = GetTickCount() - currentTime;
        if (frameTime < (1000 / kTargetFPS)) {
            Sleep((1000 / kTargetFPS) - frameTime);
        }
    }
}