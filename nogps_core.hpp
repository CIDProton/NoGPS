#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <queue>
#include <memory>

constexpr float PI = 3.14159265359f;

// --- Конфигурация с расширенными параметрами ---
struct CoreParams {
    // Сегментация линий
    float splitTolerance = 4.0f;
    float mergeTolerance = 15.0f;
    float newNodeDist = 40.0f;
    
    // Рефлекторное избегание
    float reflexDist = 45.0f;
    float reflexForce = 15.0f;
    
    // Память
    size_t ramLimitBytes = 64 * 1024 * 1024;
    float gridCellSize = 50.0f;
    float pruneDistance = 300.0f;
    
    // 🆕 Оптимизация графа
    float nodeMergeDistance = 8.0f;        // Дистанция для слияния узлов
    float nodeFeatureSimilarity = 0.7f;    // Порог схожести признаков для слияния
    int minObservationsForKeep = 3;        // Мин. наблюдений для сохранения линии
    
    // 🆕 Доверие и коррекция
    float imuTrustDecay = 0.95f;           // Затухание доверия к IMU со временем
    float lidarCorrectionMax = 5.0f;       // Макс. коррекция за кадр от лидара
    float stuckVelocityThreshold = 5.0f;   // Порог скорости для детекции застревания
    float stuckConfidenceBoost = 3.0f;     // Множитель доверия к лидару при застревании
    
    // 🆕 Физика дрона
    float droneMass = 1.0f;
    float droneDrag = 0.92f;               // Сопротивление воздуха (0-1)
    float maxThrust = 200.0f;
    float maxAngularVelocity = 3.0f;       // рад/с
    float angularDrag = 0.85f;
};

// --- Математика ---
struct Vec2 {
    float x, y;
    Vec2(float xValue = 0.0f, float yValue = 0.0f) : x(xValue), y(yValue) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2 operator/(float s) const { return {x / s, y + s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    float lengthSq() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSq()); }
    float dist(const Vec2& o) const { return (*this - o).length(); }
    float dot(const Vec2& o) const { return x * o.x + y * o.y; }
    float cross(const Vec2& o) const { return x * o.y - y * o.x; }
    Vec2 normalized() const {
        float l = length();
        return l > 0.0f ? *this * (1.0f / l) : Vec2(0, 0);
    }
    Vec2 rotated(float angle) const {
        float c = std::cos(angle), s = std::sin(angle);
        return {x * c - y * s, x * s + y * c};
    }
    float angle() const { return std::atan2(y, x); }
};

struct LineSegment {
    Vec2 start, end;
    int observationCount = 1;          // 🆕 Сколько раз наблюдалась
    float lastObservationTime = 0.0f;  // 🆕 Время последнего наблюдения
    float confidence = 1.0f;           // 🆕 Уверенность в линии
    
    float lengthSq() const { return (end - start).lengthSq(); }
    float length() const { return (end - start).length(); }
    Vec2 center() const { return (start + end) * 0.5f; }
    Vec2 direction() const { return (end - start).normalized(); }
};

struct LidarPoint {
    float angle, dist;
    float intensity = 1.0f;  // 🆕 Интенсивность отражения (для будущего)
    Vec2 toCartesian() const { return {std::cos(angle) * dist, std::sin(angle) * dist}; }
};

// 🆕 Инерциальные данные (вместо абсолютной скорости)
struct InertialData {
    Vec2 acceleration;      // Ускорение в локальных координатах дрона
    float angularVelocity;  // Угловая скорость (рад/с)
    float orientation;      // Текущий угол ориентации (рад)
    float deltaTime;        // Время с последнего кадра
};

// --- Графовая система с доверием ---
struct GraphEdge {
    int fromId;
    int toId;
    float weight;
    Vec2 relativeTransform;
    float confidence = 1.0f;  // 🆕 Доверие к ребру
    int traversalCount = 0;   // 🆕 Сколько раз проходили
};

struct GraphNode {
    int id = -1;
    Vec2 position;
    float orientation = 0.0f;          // 🆕 Ориентация узла
    std::vector<LineSegment> localFeatures;
    std::vector<int> connectedNodes;
    bool isOffloaded = false;
    
    // 🆕 Метрики доверия
    float positionConfidence = 1.0f;
    int visitCount = 0;
    float lastVisitTime = 0.0f;
    Vec2 accumulatedCorrection;        // 🆕 Накопленная коррекция позиции
    
    size_t getMemoryUsage() const {
        return sizeof(GraphNode) + 
               (localFeatures.capacity() * sizeof(LineSegment)) + 
               (connectedNodes.capacity() * sizeof(int));
    }
    
    // 🆕 Схожесть признаков с другим узлом (0-1)
    float featureSimilarity(const GraphNode& other) const {
        if (localFeatures.empty() || other.localFeatures.empty()) return 0.0f;
        
        int matches = 0;
        for (const auto& lf : localFeatures) {
            for (const auto& of : other.localFeatures) {
                Vec2 dir1 = lf.direction();
                Vec2 dir2 = of.direction();
                float dot = std::fabs(dir1.dot(dir2));
                float dist = lf.center().dist(of.center());
                
                if (dot > 0.9f && dist < 20.0f) {
                    matches++;
                    break;
                }
            }
        }
        
        return static_cast<float>(matches) / 
               std::max(localFeatures.size(), other.localFeatures.size());
    }
};

// --- Пространственный индекс ---
class SpatialGrid {
public:
    void clear() { cells.clear(); }
    
    void addLine(int lineIndex, const Vec2& start, const Vec2& end, float cellSize) {
        addPointToCell(lineIndex, start, cellSize);
        addPointToCell(lineIndex, end, cellSize);
        addPointToCell(lineIndex, (start + end) * 0.5f, cellSize);
    }
    
    void removeLine(int lineIndex, const Vec2& start, const Vec2& end, float cellSize) {
        // 🆕 Для удаления нужно знать ячейки (упрощённо - перестройка)
    }

    std::vector<int> query(const Vec2& pos, float radius, float cellSize) const {
        std::vector<int> result;
        int cx = static_cast<int>(pos.x / cellSize);
        int cy = static_cast<int>(pos.y / cellSize);
        int rCells = static_cast<int>(radius / cellSize) + 1;

        for (int y = -rCells; y <= rCells; ++y) {
            for (int x = -rCells; x <= rCells; ++x) {
                int key = (cx + x) * 10000 + (cy + y);
                auto it = cells.find(key);
                if (it != cells.end()) {
                    for (int idx : it->second) {
                        result.push_back(idx);
                    }
                }
            }
        }
        // 🆕 Удаление дубликатов
        std::sort(result.begin(), result.end());
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }
    
    // 🆕 Query по прямоугольнику для оптимизации
    std::vector<int> queryBox(const Vec2& min, const Vec2& max, float cellSize) const {
        std::vector<int> result;
        int minX = static_cast<int>(min.x / cellSize);
        int maxX = static_cast<int>(max.x / cellSize);
        int minY = static_cast<int>(min.y / cellSize);
        int maxY = static_cast<int>(max.y / cellSize);
        
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                int key = x * 10000 + y;
                auto it = cells.find(key);
                if (it != cells.end()) {
                    for (int idx : it->second) {
                        result.push_back(idx);
                    }
                }
            }
        }
        std::sort(result.begin(), result.end());
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }

private:
    std::unordered_map<int, std::vector<int>> cells;
    
    void addPointToCell(int lineIndex, const Vec2& p, float cellSize) {
        int key = static_cast<int>(p.x / cellSize) * 10000 + 
                  static_cast<int>(p.y / cellSize);
        cells[key].push_back(lineIndex);
    }
};

// 🆕 Менеджер доверия к позиции
struct PoseConfidence {
    float value = 1.0f;
    float lidarAlignmentScore = 0.0f;
    int consecutiveLidarMatches = 0;
    float imuDriftEstimate = 0.0f;
    
    void update(float lidarMatchQuality, bool isStuck, float dt) {
        // Доверие растёт при хороших совпадениях с лидаром
        if (lidarMatchQuality > 0.7f) {
            value = std::min(1.0f, value + 0.05f);
            consecutiveLidarMatches++;
        } else {
            value = std::max(0.1f, value - 0.02f);
            consecutiveLidarMatches = 0;
        }
        
        // При застревании доверие к IMU падает
        if (isStuck) {
            imuDriftEstimate += dt * 0.1f;
        } else {
            imuDriftEstimate = std::max(0.0f, imuDriftEstimate - dt * 0.05f);
        }
        
        lidarAlignmentScore = lidarMatchQuality;
    }
    
    float getLidarWeight() const {
        // Чем меньше доверие к IMU, тем больше вес лидара
        return 0.3f + (1.0f - value) * 0.5f + imuDriftEstimate * 0.2f;
    }
};

// --- Ядро ---
class DroneCore { 
public:
    DroneCore();
    ~DroneCore();

    // 🆕 Обновление с инерциальными данными вместо абсолютной скорости
    void update(float currentTime, const std::vector<LidarPoint>& scan,
                const InertialData& inertial);
    void reset(const Vec2& spawnPos = Vec2(400.0f, 300.0f));

    // Getters
    const std::vector<GraphNode>& getGraph() const { return mapGraph; }
    const std::vector<LineSegment>& getGlobalLines() const { return globalLines; }
    const std::vector<GraphEdge>& getGraphEdges() const { return graphEdges; }
    Vec2 getEstPos() const { return estimatedPos; }
    float getEstOrientation() const { return estimatedOrientation; }
    std::string getLogs() const { return memoryLog; }
    std::vector<LineSegment> getDebugLines() const { return currentFeatures; }
    PoseConfidence getPoseConfidence() const { return poseConfidence; }
    
    // 🆕 Статистика для отладки
    struct CoreStats {
        size_t totalNodes;
        size_t offloadedNodes;
        size_t mergedNodes;
        size_t phantomLinesRemoved;
        float avgConfidence;
        size_t ramUsage;
    };
    CoreStats getStats() const { return currentStats; }

    CoreParams params;
    Vec2 velocityCommand;  // Остаётся для совместимости

private:
    // Logic
    void buildSpatialGrid();
    void alignScanToMap(const std::vector<LidarPoint>& cleanScan, float currentTime);
    void mergeIntoGlobal(const std::vector<LineSegment>& newLines, float currentTime);
    void pruneMap();
    void processMemory();
    void createGraphNode();
    void updateGraphTopology();
    
    // 🆕 Оптимизация графа
    void optimizeGraphNodes();
    void mergeNearbyNodes();
    
    // 🆕 Удаление фантомных стен
    void removePhantomLines(float currentTime);
    
    // 🆕 Детекция застревания
    bool detectStuck(const InertialData& inertial) const;
    
    // Helpers
    std::vector<LidarPoint> preprocessScan(const std::vector<LidarPoint>& scan) const;
    void recursiveSplit(const std::vector<Vec2>& points, int start, int end, 
                       std::vector<LineSegment>& result);
    float distToLineSegmentStatic(Vec2 p, Vec2 a, Vec2 b, Vec2& outClosest);
    
    // State
    Vec2 estimatedPos;
    float estimatedOrientation = 0.0f;
    Vec2 estimatedVelocity;          // 🆕 Оценённая скорость
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
    
    // 🆕 Новые состояния
    PoseConfidence poseConfidence;
    float currentTime = 0.0f;
    Vec2 lastEstimatedPos;
    float lastStuckTime = 0.0f;
    CoreStats currentStats;
    
    // 🆕 Для детекции застревания
    std::vector<Vec2> recentVelocities;
    static constexpr int RECENT_VELOCITY_WINDOW = 10;
};