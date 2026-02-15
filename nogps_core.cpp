#include "nogps_core.hpp"
#include <cmath>
#include <sstream>
#include <algorithm>

// =============================================================================
// БАЗОВАЯ МАТЕМАТИКА (Vec2 и LidarPoint)
// =============================================================================

Vec2::Vec2(float xValue, float yValue) : x(xValue), y(yValue) {}

Vec2 Vec2::operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
Vec2 Vec2::operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
Vec2 Vec2::operator*(float scalar) const { return {x * scalar, y * scalar}; }

float Vec2::length() const { return std::sqrt(x * x + y * y); }

Vec2 Vec2::normalized() const {
    const float l = length();
    return (l > 0.0f) ? Vec2(x / l, y / l) : Vec2(0.0f, 0.0f);
}

float Vec2::dist(const Vec2& other) const {
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
}

Vec2 LidarPoint::toCartesian() const {
    return {std::cos(angle) * dist, std::sin(angle) * dist};
}

// =============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ЯДРА
// =============================================================================

// Угол линии для слияния параллельных стен
float getLineAngle(const LineSegment& l) {
    return std::atan2(l.end.y - l.start.y, l.end.x - l.start.x);
}

// Расстояние от точки до отрезка (основа качественного Split-Merge)
float distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    float l2 = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
    if (l2 == 0.0) return p.dist(a);
    float t = std::max(0.0f, std::min(1.0f, ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2));
    Vec2 projection = a + (b - a) * t;
    return p.dist(projection);
}

// =============================================================================
// РЕАЛИЗАЦИЯ DRONE CORE
// =============================================================================

DroneCore::DroneCore() {
    reset();
}

void DroneCore::reset() {
    estimatedPos = Vec2(400.0f, 300.0f);
    mapGraph.clear();
    currentFeatures.clear();
    globalLines.clear();
    
    // Настройки чувствительности
    splitMergeTolerance = 6.0f;   // Порог отклонения точек от прямой
    mergeTolerance = 25.0f;       // Расстояние для склейки линий
    newNodeDist = 60.0f;         // Шаг графа
    reflexDist = 5.0f;           // Дистанция отталкивания от стен
    
    GraphNode startNode;
    startNode.id = 0;
    startNode.position = estimatedPos;
    startNode.isOffloaded = false;
    mapGraph.push_back(startNode);
    currentNodeIndex = 0;
    velocityCommand = {0.0f, 0.0f};
    memoryLog = "System Start";
}

void DroneCore::recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result) {
    if (end - start < 2) return;

    const Vec2 p1 = points[start];
    const Vec2 p2 = points[end];
    float maxDist = 0.0f;
    int splitIndex = -1;

    for (int i = start + 1; i < end; i++) {
        float d = distToSegment(points[i], p1, p2);
        if (d > maxDist) {
            maxDist = d;
            splitIndex = i;
        }
    }

    if (maxDist > splitMergeTolerance && splitIndex != -1) {
        recursiveSplit(points, start, splitIndex, result);
        recursiveSplit(points, splitIndex, end, result);
    } else {
        // Фильтруем слишком короткие линии (шум лидара)
        if (p1.dist(p2) > 12.0f) {
            result.push_back({p1, p2});
        }
    }
}

void DroneCore::mergeIntoGlobal(const std::vector<LineSegment>& newLines) {
    for (const auto& newLine : newLines) {
        bool merged = false;
        float newAngle = getLineAngle(newLine);
        Vec2 newMid = (newLine.start + newLine.end) * 0.5f;

        for (auto& global : globalLines) {
            float globalAngle = getLineAngle(global);
            
            // Считаем разницу углов (с учетом симметрии линий)
            float angleDiff = std::abs(newAngle - globalAngle);
            if (angleDiff > PI/2.0f) angleDiff = std::abs(angleDiff - PI);

            if (angleDiff < 0.2f) { // если почти параллельны (~11 градусов)
                // Если новая линия находится близко к существующей
                if (distToSegment(newMid, global.start, global.end) < mergeTolerance) {
                    // Просто "подтягиваем" существующую линию к новой (усреднение)
                    global.start = global.start * 0.95f + newLine.start * 0.05f;
                    global.end = global.end * 0.95f + newLine.end * 0.05f;
                    merged = true;
                    break;
                }
            }
        }
        
        if (!merged) {
            globalLines.push_back(newLine);
        }
    }

    // Ограничение памяти глобальной карты (чтобы не тормозило)
    if (globalLines.size() > 800) {
        globalLines.erase(globalLines.begin());
    }
}

void DroneCore::update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity) {
    // 1. Интеграция позиции
    estimatedPos = estimatedPos + (imuVelocity * dt);

    // 2. Сглаживание данных лидара (Медианный/Скользящий фильтр)
    std::vector<LidarPoint> filteredScan = scan;
    if (scan.size() > 2) {
        for (size_t i = 1; i < scan.size() - 1; i++) {
            filteredScan[i].dist = (scan[i-1].dist + scan[i].dist + scan[i+1].dist) / 3.0f;
        }
    }

    // 3. Создание облака точек
    std::vector<Vec2> cloud;
    cloud.reserve(filteredScan.size());
    for (const auto& p : filteredScan) {
        if (p.dist > 5.0f && p.dist < 480.0f) {
            cloud.push_back(estimatedPos + p.toCartesian());
        }
    }

    // 4. Выделение векторов (Split-Merge)
    currentFeatures.clear();
    if (cloud.size() > 5) {
        recursiveSplit(cloud, 0, static_cast<int>(cloud.size()) - 1, currentFeatures);
        mergeIntoGlobal(currentFeatures);
    }

    // 5. Рефлекторное избегание препятствий
    velocityCommand = {0.0f, 0.0f};
    Vec2 reflexForce = {0.0f, 0.0f};
    for (const auto& p : filteredScan) {
        if (p.dist > 0.0f && p.dist < reflexDist) {
            const Vec2 dir = p.toCartesian().normalized();
            reflexForce = reflexForce - (dir * (reflexDist - p.dist) * 15.0f);
        }
    }
    velocityCommand = reflexForce;

    // 6. Обновление графа (Node Mapping)
    if (currentNodeIndex >= 0 && currentNodeIndex < (int)mapGraph.size()) {
        if (estimatedPos.dist(mapGraph[currentNodeIndex].position) > newNodeDist) {
            GraphNode newNode;
            newNode.id = static_cast<int>(mapGraph.size());
            newNode.position = estimatedPos;
            newNode.isOffloaded = false;
            newNode.connectedNodes.push_back(currentNodeIndex);
            mapGraph[currentNodeIndex].connectedNodes.push_back(newNode.id);
            mapGraph.push_back(newNode);
            currentNodeIndex = newNode.id;
        }
    }

    // 7. Оптимизация RAM (Offload)
    int activeNodes = 0;
    for (auto& node : mapGraph) {
        bool near = (std::abs(node.id - currentNodeIndex) < 10);
        node.isOffloaded = !near;
        if (!node.isOffloaded) {
            activeNodes++;
            node.localFeatures = currentFeatures; // Сохраняем "память" в активном узле
        } else {
            node.localFeatures.clear(); // Чистим память старых узлов
        }
    }

    // Логи
    std::stringstream log;
    log << "CORE SYSTEM:\n"
        << "Map Vectors: " << globalLines.size() << "\n"
        << "Active Nodes: " << activeNodes << " / " << mapGraph.size() << "\n"
        << "Pos: " << (int)estimatedPos.x << ", " << (int)estimatedPos.y;
    memoryLog = log.str();
}

std::vector<LineSegment> DroneCore::getDebugLines() const { return currentFeatures; }
std::vector<GraphNode> DroneCore::getGraph() const { return mapGraph; }
Vec2 DroneCore::getEstPos() const { return estimatedPos; }
std::string DroneCore::getLogs() const { return memoryLog; }