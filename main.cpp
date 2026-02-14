#include <windows.h>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>
#include <iostream>

// ==========================================
// ЧАСТЬ 0: ОБЩАЯ МАТЕМАТИКА
// ==========================================

const float PI = 3.14159265359f;

struct Vec2 {
    float x, y;
    Vec2(float _x = 0, float _y = 0) : x(_x), y(_y) {}
    Vec2 operator+(const Vec2& v) const { return { x + v.x, y + v.y }; }
    Vec2 operator-(const Vec2& v) const { return { x - v.x, y - v.y }; }
    Vec2 operator*(float s) const { return { x * s, y * s }; }
    float length() const { return std::sqrt(x * x + y * y); }
    Vec2 normalized() const {
        float l = length();
        return (l > 0) ? Vec2(x / l, y / l) : Vec2(0, 0);
    }
    float dist(const Vec2& v) const { return (*this - v).length(); }
};

struct LineSegment {
    Vec2 start;
    Vec2 end;
};

struct LidarPoint {
    float angle;
    float dist;
    Vec2 toCartesian() const {
        return { std::cos(angle) * dist, std::sin(angle) * dist };
    }
};

// ==========================================
// ЧАСТЬ 1: ЯДРО ДРОНА (NoGPS_Core)
// Логика идентична предыдущей, только без зависимостей
// ==========================================

struct GraphNode {
    int id;
    Vec2 position;
    std::vector<LineSegment> localFeatures;
    std::vector<int> connectedNodes;
    bool isOffloaded;
};

class DroneCore {
private:
    float SPLIT_MERGE_TOLERANCE = 15.0f; 
    float NEW_NODE_DIST = 120.0f;       
    float REFLEX_DIST = 50.0f;           

    Vec2 estimatedPos;
    std::vector<LineSegment> currentFeatures;
    std::vector<GraphNode> mapGraph;
    int currentNodeIndex = -1;
    std::stringstream memoryLog;

public:
    Vec2 velocityCommand;

    DroneCore() : estimatedPos(400, 300) {
        GraphNode startNode;
        startNode.id = 0;
        startNode.position = estimatedPos;
        startNode.isOffloaded = false;
        mapGraph.push_back(startNode);
        currentNodeIndex = 0;
    }

    void recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result) {
        if (start >= end) return;
        Vec2 p1 = points[start];
        Vec2 p2 = points[end];
        
        float maxDist = 0;
        int splitIndex = -1;
        float lineLen = p1.dist(p2);
        
        if (lineLen > 0.1f) {
            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            for (int i = start + 1; i < end; i++) {
                float num = std::abs(dy * points[i].x - dx * points[i].y + p2.x * p1.y - p2.y * p1.x);
                float d = num / lineLen;
                if (d > maxDist) {
                    maxDist = d;
                    splitIndex = i;
                }
            }
        }

        if (maxDist > SPLIT_MERGE_TOLERANCE) {
            recursiveSplit(points, start, splitIndex, result);
            recursiveSplit(points, splitIndex, end, result);
        } else {
            result.push_back({ p1, p2 });
        }
    }

    void update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity) {
        memoryLog.str("");
        estimatedPos = estimatedPos + (imuVelocity * dt);

        std::vector<Vec2> cloud;
        for (auto& p : scan) {
            if (p.dist == 0) continue;
            cloud.push_back(estimatedPos + p.toCartesian());
        }

        currentFeatures.clear();
        if (cloud.size() > 2) {
            recursiveSplit(cloud, 0, (int)cloud.size() - 1, currentFeatures);
        }

        velocityCommand = { 0, 0 };
        Vec2 reflexForce = { 0, 0 };
        for (auto& p : scan) {
            if (p.dist > 0 && p.dist < REFLEX_DIST) {
                Vec2 dir = p.toCartesian().normalized();
                reflexForce = reflexForce - (dir * (REFLEX_DIST - p.dist) * 8.0f);
            }
        }
        velocityCommand = reflexForce; 

        GraphNode& currNode = mapGraph[currentNodeIndex];
        if (estimatedPos.dist(currNode.position) > NEW_NODE_DIST) {
            GraphNode newNode;
            newNode.id = (int)mapGraph.size();
            newNode.position = estimatedPos;
            newNode.localFeatures = currentFeatures;
            newNode.isOffloaded = false;
            newNode.connectedNodes.push_back(currNode.id);
            currNode.connectedNodes.push_back(newNode.id);
            mapGraph.push_back(newNode);
            currentNodeIndex = newNode.id;
        }

        int activeCount = 0;
        for (auto& node : mapGraph) {
            if (std::abs(node.id - currentNodeIndex) > 4 && !node.isOffloaded) {
                node.localFeatures.clear();
                node.isOffloaded = true;
            }
            if (!node.isOffloaded) activeCount++;
        }
        
        memoryLog << "RAM OPTIMIZER:\n";
        memoryLog << "Nodes Total: " << mapGraph.size() << "\n";
        memoryLog << "Active (RAM): " << activeCount << "\n";
        memoryLog << "Offloaded (Disk): " << mapGraph.size() - activeCount << "\n";
        memoryLog << "Vectors: " << currentFeatures.size();
    }

    std::vector<LineSegment> getDebugLines() { return currentFeatures; }
    std::vector<GraphNode> getGraph() { return mapGraph; }
    Vec2 getEstPos() { return estimatedPos; }
    std::string getLogs() { return memoryLog.str(); }
};

// ==========================================
// ЧАСТЬ 2: СИМУЛЯТОР НА WINDOWS GDI (БЕЗ SFML)
// ==========================================

std::vector<LineSegment> walls;
Vec2 dronePos(400, 300);
Vec2 droneVel(0, 0);
DroneCore core;

// Функция для рисования линии
void DrawLine(HDC hdc, int x1, int y1, int x2, int y2, COLORREF color, int thickness = 1) {
    HPEN hPen = CreatePen(PS_SOLID, thickness, color);
    HGDIOBJ hOld = SelectObject(hdc, hPen);
    MoveToEx(hdc, x1, y1, NULL);
    LineTo(hdc, x2, y2);
    SelectObject(hdc, hOld);
    DeleteObject(hPen);
}

// Функция для рисования круга
void DrawCircle(HDC hdc, int x, int y, int r, COLORREF color, bool filled) {
    HBRUSH hBrush = filled ? CreateSolidBrush(color) : (HBRUSH)GetStockObject(NULL_BRUSH);
    HPEN hPen = CreatePen(PS_SOLID, 1, color);
    HGDIOBJ oldBrush = SelectObject(hdc, hBrush);
    HGDIOBJ oldPen = SelectObject(hdc, hPen);
    Ellipse(hdc, x - r, y - r, x + r, y + r);
    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    if(filled) DeleteObject(hBrush);
    DeleteObject(hPen);
}

// Raycast
float CastRay(Vec2 start, Vec2 dir) {
    float minDst = 100.0f;
    Vec2 end = start + dir * minDst;
    for (auto& w : walls) {
        float x1 = w.start.x, y1 = w.start.y;
        float x2 = w.end.x, y2 = w.end.y;
        float x3 = start.x, y3 = start.y;
        float x4 = end.x, y4 = end.y;
        float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (den == 0) continue;
        float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
        float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
        if (t > 0 && t < 1 && u > 0 && u < 1) {
            Vec2 pt = { x1 + t * (x2 - x1), y1 + t * (y2 - y1) };
            float dst = start.dist(pt);
            if (dst < minDst) minDst = dst;
        }
    }
    return minDst;
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    case WM_ERASEBKGND:
        return 1; // Чтобы не мерцало
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

int main() {
    // 1. Инициализация карты
    walls.push_back({ {100, 100}, {1100, 100} });
    walls.push_back({ {100, 700}, {1100, 700} });
    walls.push_back({ {100, 100}, {100, 700} });
    walls.push_back({ {1100, 100}, {1100, 700} });
    walls.push_back({ {400, 100}, {400, 500} });
    walls.push_back({ {800, 700}, {800, 300} });

    // 2. Создание окна
    const char CLASS_NAME[] = "NoGPS_Sim_Class";
    WNDCLASS wc = { };
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = CLASS_NAME;
    RegisterClass(&wc);

    HWND hwnd = CreateWindowEx(0, CLASS_NAME, "NoGPS Core Simulator (GDI Version)", 
        WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT, 1280, 800, 
        NULL, NULL, GetModuleHandle(NULL), NULL);

    // Буфер для рисования (чтобы не мерцало)
    HDC hdcWindow = GetDC(hwnd);
    HDC hdcMem = CreateCompatibleDC(hdcWindow);
    HBITMAP hbmMem = CreateCompatibleBitmap(hdcWindow, 1280, 800);
    SelectObject(hdcMem, hbmMem);

    bool running = true;
    while (running) {
        MSG msg = { };
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) running = false;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // --- Управление ---
        Vec2 inputVel(0, 0);
        if (GetAsyncKeyState('W') & 0x8000) inputVel.y = -100;
        if (GetAsyncKeyState('S') & 0x8000) inputVel.y = 100;
        if (GetAsyncKeyState('A') & 0x8000) inputVel.x = -100;
        if (GetAsyncKeyState('D') & 0x8000) inputVel.x = 100;

        // --- Сенсоры ---
        std::vector<LidarPoint> scan;
        int numRays = 100;
        for (int i = 0; i < numRays; i++) {
            float angle = (i * 2 * PI) / numRays;
            float dst = CastRay(dronePos, { std::cos(angle), std::sin(angle) });
            float noise = ((rand() % 100) / 100.0f) * 3.0f; // Шум
            scan.push_back({ angle, dst + noise });
        }

        // --- Update Ядра ---
        core.update(0.016f, scan, droneVel);
        
        // --- Физика ---
        droneVel = inputVel + core.velocityCommand;
        dronePos = dronePos + droneVel * 0.016f;

        // ================= ОТРИСОВКА (GDI) =================
        
        // Очистка фона
        RECT rect = { 0, 0, 1280, 800 };
        FillRect(hdcMem, &rect, (HBRUSH)GetStockObject(BLACK_BRUSH));

        // Разделительная линия
        DrawLine(hdcMem, 640, 0, 640, 800, RGB(100, 100, 100));

        // --- ОКНО 1: РЕАЛЬНОСТЬ (Слева) ---
        // Стены
        for (auto& w : walls) DrawLine(hdcMem, w.start.x, w.start.y, w.end.x, w.end.y, RGB(255, 255, 255), 3);
        // Лидар
        for (auto& p : scan) {
            Vec2 pt = dronePos + p.toCartesian();
            DrawLine(hdcMem, dronePos.x, dronePos.y, pt.x, pt.y, RGB(50, 50, 50));
        }
        // Дрон
        DrawCircle(hdcMem, dronePos.x, dronePos.y, 8, RGB(0, 255, 0), true);

        // --- ОКНО 2: МОЗГИ ЯДРА (Справа) ---
        // Чтобы отобразить справа, добавляем смещение +640 по X
        int offsetX = 640;
        
        // Векторы (синие линии)
        auto lines = core.getDebugLines();
        for (auto& l : lines) {
            DrawLine(hdcMem, l.start.x + offsetX, l.start.y, l.end.x + offsetX, l.end.y, RGB(0, 255, 255), 2);
        }

        // Граф (красные точки)
        auto graph = core.getGraph();
        for (auto& n : graph) {
            COLORREF c = n.isOffloaded ? RGB(100, 100, 100) : RGB(255, 0, 0);
            DrawCircle(hdcMem, n.position.x + offsetX, n.position.y, 4, c, true);
            
            for(int id : n.connectedNodes) {
                Vec2 p2 = graph[id].position;
                DrawLine(hdcMem, n.position.x + offsetX, n.position.y, p2.x + offsetX, p2.y, c);
            }
        }
        // Дрон (фантом)
        Vec2 estPos = core.getEstPos();
        DrawCircle(hdcMem, estPos.x + offsetX, estPos.y, 6, RGB(0, 255, 0), false);

        // --- ТЕКСТ (Логи) ---
        std::string logs = core.getLogs();
        SetTextColor(hdcMem, RGB(255, 255, 0));
        SetBkMode(hdcMem, TRANSPARENT);
        RECT textRect = { 650, 600, 1200, 800 };
        DrawText(hdcMem, logs.c_str(), -1, &textRect, DT_LEFT);
        
        TextOut(hdcMem, 10, 10, "SIMULATION REALITY", 18);
        TextOut(hdcMem, 650, 10, "CORE MEMORY VIEW", 16);

        // Копируем буфер на экран
        BitBlt(hdcWindow, 0, 0, 1280, 800, hdcMem, 0, 0, SRCCOPY);

        Sleep(16); // ~60 FPS
    }

    return 0;
}