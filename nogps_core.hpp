#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

constexpr float PI = 3.14159265359f;

// --- Конфигурация ---
struct CoreParams {
    float splitTolerance = 4.0f;    
    float mergeTolerance = 15.0f;   
    float newNodeDist = 40.0f;      // Уменьшено для более плотного графа
    float reflexDist = 45.0f;
    float reflexForce = 15.0f;
    size_t ramLimitBytes = 64 * 1024 * 1024; // 64MB для активного набора (реалистично для 256MB total)
    float gridCellSize = 50.0f;     // Размер ячейки пространственной сетки
    float pruneDistance = 300.0f;   // Удалять линии дальше этого расстояния от ЛЮБОГО узла
};

// --- Математика ---
struct Vec2 {
    float x, y;
    Vec2(float xValue = 0.0f, float yValue = 0.0f) : x(xValue), y(yValue) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2 operator/(float s) const { return {x / s, y / s}; }
    float lengthSq() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSq()); }
    float dist(const Vec2& o) const { return (*this - o).length(); }
    float dot(const Vec2& o) const { return x * o.x + y * o.y; }
    Vec2 normalized() const {
        float l = length();
        return l > 0.0f ? *this * (1.0f / l) : Vec2(0, 0);
    }
};

struct LineSegment {
    Vec2 start, end;
    float lengthSq() const { return (end - start).lengthSq(); }
};

struct LidarPoint {
    float angle, dist;
    Vec2 toCartesian() const { return {std::cos(angle) * dist, std::sin(angle) * dist}; }
};

// --- Графовая система ---
struct GraphEdge {
    int fromId;
    int toId;
    float weight; // Обратная дисперсия (чем точнее связь, тем больше вес)
    Vec2 relativeTransform; // Позиция to относительно from
};

struct GraphNode {
    int id = -1;
    Vec2 position;
    std::vector<LineSegment> localFeatures;
    std::vector<int> connectedNodes; // IDs соседей
    bool isOffloaded = false;
    
    // Для оптимизации памяти
    size_t getMemoryUsage() const {
        return sizeof(GraphNode) + (localFeatures.capacity() * sizeof(LineSegment)) + (connectedNodes.capacity() * sizeof(int));
    }
};

// --- Пространственный индекс (Spatial Hash) ---
class SpatialGrid {
public:
    void clear() { cells.clear(); }
    
    void addLine(int lineIndex, const Vec2& start, const Vec2& end, float cellSize) {
        // Добавляем линию во все ячейки, которые она пересекает (упрощенно: по концам и середине)
        addPointToCell(lineIndex, start, cellSize);
        addPointToCell(lineIndex, end, cellSize);
        addPointToCell(lineIndex, (start + end) * 0.5f, cellSize);
    }

    std::vector<int> query(const Vec2& pos, float radius, float cellSize) const {
        std::vector<int> result;
        int cx = (int)(pos.x / cellSize);
        int cy = (int)(pos.y / cellSize);
        int rCells = (int)(radius / cellSize) + 1;

        for (int y = -rCells; y <= rCells; ++y) {
            for (int x = -rCells; x <= rCells; ++x) {
                int key = (cx + x) * 10000 + (cy + y); // Простой хеш
                auto it = cells.find(key);
                if (it != cells.end()) {
                    for (int idx : it->second) {
                        // Проверка дубликатов проста, для продакшена нужен set или проверка
                        result.push_back(idx);
                    }
                }
            }
        }
        return result;
    }

private:
    std::unordered_map<int, std::vector<int>> cells;
    void addPointToCell(int lineIndex, const Vec2& p, float cellSize) {
        int key = (int)(p.x / cellSize) * 10000 + (int)(p.y / cellSize);
        cells[key].push_back(lineIndex);
    }
};

// --- Ядро ---
class DroneCore {
public:
    DroneCore();
    ~DroneCore();

    void update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity);
    void reset();

    // Getters
    const std::vector<GraphNode>& getGraph() const { return mapGraph; }
    const std::vector<LineSegment>& getGlobalLines() const { return globalLines; }
    const std::vector<GraphEdge>& getGraphEdges() const { return graphEdges; }
    Vec2 getEstPos() const { return estimatedPos; }
    std::string getLogs() const { return memoryLog; }
    std::vector<LineSegment> getDebugLines() const { return currentFeatures; }

    CoreParams params;
    Vec2 velocityCommand;

private:
    // Logic
    void buildSpatialGrid();
    void alignScanToMap(const std::vector<LidarPoint>& cleanScan);
    void mergeIntoGlobal(const std::vector<LineSegment>& newLines);
    void pruneMap();
    void processMemory();
    void createGraphNode();
    void updateGraphTopology();
    
    // Helpers
    std::vector<LidarPoint> preprocessScan(const std::vector<LidarPoint>& scan) const;
    void recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result);
    float distToLineSegment(Vec2 p, Vec2 a, Vec2 b, Vec2& outClosest);
    
    // State
    Vec2 estimatedPos;
    std::vector<LineSegment> currentFeatures;
    std::vector<LineSegment> globalLines;
    std::vector<GraphNode> mapGraph;
    std::vector<GraphEdge> graphEdges;
    SpatialGrid spatialGrid;
    
    int currentNodeIndex = -1;
    std::string memoryLog;
    size_t currentRamUsage = 0;
    Vec2 smoothedVelocityCommand;
    int pruneTimer = 0;
    int gridRebuildTimer = 0;
};