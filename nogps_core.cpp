#include "nogps_core.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <sstream>

namespace {
// Улучшенная функция расстояния до отрезка с поиском ближайшей точки
float distToLineSegment(Vec2 p, Vec2 a, Vec2 b, Vec2& outClosest) {
    Vec2 ab = b - a;
    float l2 = ab.lengthSq();
    if (l2 == 0.0f) {
        outClosest = a;
        return p.dist(a);
    }
    float t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2;
    t = std::max(0.0f, std::min(1.0f, t));
    outClosest = a + ab * t;
    return p.dist(outClosest);
}

// Старая обертка для совместимости
float distToLine(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 dummy;
    return distToLineSegment(p, a, b, dummy);
}

std::string cacheFileForNode(int id) {
    return "cache/node_" + std::to_string(id) + ".bin";
}

void clampCoreParams(CoreParams& params) {
    if (params.splitTolerance < 0.5f) params.splitTolerance = 0.5f;
    if (params.mergeTolerance < 1.0f) params.mergeTolerance = 1.0f;
    if (params.newNodeDist < 10.0f) params.newNodeDist = 10.0f;
    if (params.reflexDist < 5.0f) params.reflexDist = 5.0f;
    if (params.reflexForce < 0.0f) params.reflexForce = 0.0f;
    if (params.ramLimitBytes < 16 * 1024) params.ramLimitBytes = 16 * 1024;
}

float wrappedAngleDiff(float a, float b) {
    float d = std::fabs(a - b);
    while (d > 2.0f * PI) {
        d -= 2.0f * PI;
    }
    if (d > PI) {
        d = 2.0f * PI - d;
    }
    return d;
}
} // namespace

Vec2::Vec2(float xValue, float yValue) : x(xValue), y(yValue) {}

Vec2 Vec2::operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
Vec2 Vec2::operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
Vec2 Vec2::operator*(float scalar) const { return {x * scalar, y * scalar}; }

float Vec2::length() const { return std::sqrt(x * x + y * y); }
float Vec2::lengthSq() const { return x * x + y * y; }
float Vec2::dot(const Vec2& other) const { return x * other.x + y * other.y; }

Vec2 Vec2::normalized() const {
    const float l = length();
    return l > 0.0f ? Vec2(x / l, y / l) : Vec2(0.0f, 0.0f);
}

float Vec2::dist(const Vec2& other) const {
    return (*this - other).length();
}

Vec2 LidarPoint::toCartesian() const {
    return {std::cos(angle) * dist, std::sin(angle) * dist};
}

DroneCore::DroneCore() {
    reset();
}

DroneCore::~DroneCore() = default;

void DroneCore::reset() {
    std::filesystem::create_directories("cache");

    estimatedPos = Vec2(400.0f, 300.0f);
    mapGraph.clear();
    globalLines.clear();
    currentFeatures.clear();
    currentRamUsage = 0;
    pruneTimer = 0;

    GraphNode start;
    start.id = 0;
    start.position = estimatedPos;
    mapGraph.push_back(start);
    currentNodeIndex = 0;
    velocityCommand = Vec2(0.0f, 0.0f);
    smoothedVelocityCommand = Vec2(0.0f, 0.0f);
    memoryLog = "Core reset";
}

void DroneCore::recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result) {
    // Базовый случай: мало точек
    if (end - start < 2) {
        if (end - start == 1) {
            // Рисуем линию только если точки близко!
            // Это защита от "микро-фантомов"
            if (points[start].dist(points[end]) < 20.0f) {
                result.push_back({points[start], points[end]});
            }
        }
        return;
    }

    float dMax = 0.0f;
    int idx = -1;
    
    // Ищем самую далекую точку от прямой start-end
    for (int i = start + 1; i < end; ++i) {
        const float d = distToLine(points[i], points[start], points[end]);
        if (d > dMax) {
            dMax = d;
            idx = i;
        }
    }

    if (dMax > params.splitTolerance && idx != -1) {
        recursiveSplit(points, start, idx, result);
        recursiveSplit(points, idx, end, result);
    } else {
        // Если аппроксимация хорошая, добавляем сегмент
        // НО! Проверяем длину сегмента. Если мы соединяем точки через полкомнаты (например, на открытом месте)
        // и между ними ничего нет - возможно это ошибка, но recursiveSplit работает внутри кластера,
        // так что тут уже должны быть только близкие точки.
        result.push_back({points[start], points[end]});
    }
}

void DroneCore::mergeIntoGlobal(const std::vector<LineSegment>& newLines) {
    for (const auto& nl : newLines) {
        bool merged = false;
        
        for (auto& gl : globalLines) {
            Vec2 midNew = (nl.start + nl.end) * 0.5f;
            Vec2 midOld = (gl.start + gl.end) * 0.5f;
            
            // Если центры линий рядом
            if (midNew.dist(midOld) < params.mergeTolerance) {
                Vec2 dirNew = (nl.end - nl.start).normalized();
                Vec2 dirOld = (gl.end - gl.start).normalized();
                float dot = std::fabs(dirNew.dot(dirOld));
                
                // И угол совпадает
                if (dot > 0.9f) {
                    gl.start = gl.start * 0.9f + nl.start * 0.1f;
                    gl.end = gl.end * 0.9f + nl.end * 0.1f;
                    merged = true;
                    break;
                }
            }
        }

        if (!merged) {
            globalLines.push_back(nl);
        }
    }

    if (globalLines.size() > 2000) {
        globalLines.erase(globalLines.begin(), globalLines.begin() + 100);
    }
}

void DroneCore::alignScanToMap(const std::vector<LidarPoint>& cleanScan) {
    if (globalLines.empty() || cleanScan.size() < 10) return;

    Vec2 totalCorrection(0.0f, 0.0f);
    int matchCount = 0;
    const float kSearchRadius = 30.0f; 

    for (const auto& p : cleanScan) {
        Vec2 worldPoint = estimatedPos + p.toCartesian();

        float bestDist = kSearchRadius;
        Vec2 bestClosestPoint;
        bool found = false;

        for (const auto& line : globalLines) {
            // Bounding box optimization
            if (worldPoint.x < std::min(line.start.x, line.end.x) - kSearchRadius) continue;
            if (worldPoint.x > std::max(line.start.x, line.end.x) + kSearchRadius) continue;
            if (worldPoint.y < std::min(line.start.y, line.end.y) - kSearchRadius) continue;
            if (worldPoint.y > std::max(line.start.y, line.end.y) + kSearchRadius) continue;

            Vec2 closest;
            float d = distToLineSegment(worldPoint, line.start, line.end, closest);
            if (d < bestDist) {
                bestDist = d;
                bestClosestPoint = closest;
                found = true;
            }
        }

        if (found) {
            Vec2 error = bestClosestPoint - worldPoint;
            totalCorrection = totalCorrection + error;
            matchCount++;
        }
    }

    if (matchCount > 5) {
        Vec2 avgCorrection = totalCorrection * (1.0f / matchCount);
        if (avgCorrection.length() < 15.0f) {
            estimatedPos = estimatedPos + avgCorrection * 0.3f;
        }
    }
}

void DroneCore::pruneMap(const std::vector<LidarPoint>& scan) {
    if (globalLines.empty()) return;

    for (int i = static_cast<int>(globalLines.size()) - 1; i >= 0; --i) {
        Vec2 mid = (globalLines[i].start + globalLines[i].end) * 0.5f;
        Vec2 toLine = mid - estimatedPos;
        float distToLineCenter = toLine.length();

        if (distToLineCenter > 250.0f) continue; 
        if (distToLineCenter < 15.0f) continue; 

        float angleToLine = std::atan2(toLine.y, toLine.x);
        float minAngleDiff = 100.0f;
        float observedDist = 0.0f;
        
        for (const auto& p : scan) {
            float diff = wrappedAngleDiff(p.angle, angleToLine);
            if (diff < minAngleDiff) {
                minAngleDiff = diff;
                observedDist = p.dist;
            }
        }

        if (minAngleDiff < 0.05f) {
            // Если мы видим "сквозь" стену (дальше на 30 единиц), значит стены нет
            if (observedDist > distToLineCenter + 30.0f) {
                globalLines.erase(globalLines.begin() + i);
            }
        }
    }
}

void DroneCore::saveNodeToDisk(int id) {
    if (id < 0 || id >= static_cast<int>(mapGraph.size())) return;

    std::filesystem::create_directories("cache");
    const std::string fileName = cacheFileForNode(id);
    std::ofstream f(fileName, std::ios::binary | std::ios::trunc);
    if (!f) return;

    const size_t count = mapGraph[id].localFeatures.size();
    f.write(reinterpret_cast<const char*>(&count), sizeof(size_t));
    if (count > 0) {
        f.write(reinterpret_cast<const char*>(mapGraph[id].localFeatures.data()), count * sizeof(LineSegment));
    }
}

void DroneCore::loadNodeFromDisk(int id) {
    if (id < 0 || id >= static_cast<int>(mapGraph.size())) return;

    const std::string fileName = cacheFileForNode(id);
    std::ifstream f(fileName, std::ios::binary);
    if (!f) return;

    size_t count = 0;
    f.read(reinterpret_cast<char*>(&count), sizeof(size_t));
    if (!f) return;

    mapGraph[id].localFeatures.resize(count);
    if (count > 0) {
        f.read(reinterpret_cast<char*>(mapGraph[id].localFeatures.data()), count * sizeof(LineSegment));
    }
}

std::vector<LidarPoint> DroneCore::preprocessScan(const std::vector<LidarPoint>& scan) const {
    std::vector<LidarPoint> filtered;
    filtered.reserve(scan.size());

    // 1. Фильтр дальности - ОТСЕКАЕМ БЕСКОНЕЧНОСТЬ
    // Если луч улетел в "молоко" (490+), мы его не используем для картографии,
    // чтобы не рисовать стены на горизонте.
    for (const auto& p : scan) {
        if (p.dist > 10.0f && p.dist < 480.0f) { // < 480 (было 490), чтобы с запасом от макс рейкаста
            filtered.push_back(p);
        }
    }
    return filtered;
}

size_t DroneCore::estimateRamUsageBytes() const {
    size_t usage = globalLines.capacity() * sizeof(LineSegment);
    for (const auto& node : mapGraph) {
        if (!node.isOffloaded) {
            usage += node.getMemoryUsage();
        }
    }
    return usage;
}

void DroneCore::processMemory() {
    currentRamUsage = estimateRamUsageBytes();

    while (currentRamUsage > params.ramLimitBytes) {
        int bestIdx = -1;
        float bestDist = 0.0f;
        for (int i = 0; i < static_cast<int>(mapGraph.size()); ++i) {
            const auto& node = mapGraph[i];
            if (node.isOffloaded || node.id == currentNodeIndex) continue;
            float dist = node.position.dist(estimatedPos);
            if (dist > bestDist) {
                bestDist = dist;
                bestIdx = i;
            }
        }
        if (bestIdx == -1) break;
        auto& victim = mapGraph[bestIdx];
        saveNodeToDisk(victim.id);
        victim.localFeatures.clear();
        victim.localFeatures.shrink_to_fit();
        victim.isOffloaded = true;
        currentRamUsage = estimateRamUsageBytes();
    }
    
    for (auto& node : mapGraph) {
        if (!node.isOffloaded) continue;
        if (node.position.dist(estimatedPos) < params.newNodeDist) {
            loadNodeFromDisk(node.id);
            node.isOffloaded = false;
        }
    }
}

void DroneCore::update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity) {
    clampCoreParams(params);
    estimatedPos = estimatedPos + (imuVelocity * dt);

    // 1. Фильтруем шум
    const std::vector<LidarPoint> filteredScan = preprocessScan(scan);

    // 2. Scan Matching (по-прежнему нужен)
    alignScanToMap(filteredScan);

    // 3. СЕГМЕНТАЦИЯ СКАНА (Вот тут фикс фантомных стен)
    // Мы не кидаем весь скан в recursiveSplit. Мы режем его на кластеры.
    currentFeatures.clear();
    
    if (!filteredScan.empty()) {
        std::vector<Vec2> currentCluster;
        currentCluster.reserve(filteredScan.size());
        
        currentCluster.push_back(estimatedPos + filteredScan[0].toCartesian());

        for (size_t i = 1; i < filteredScan.size(); ++i) {
            const auto& prevP = filteredScan[i - 1];
            const auto& curP = filteredScan[i];
            
            // Получаем мировые координаты
            Vec2 prevWorld = estimatedPos + prevP.toCartesian();
            Vec2 curWorld = estimatedPos + curP.toCartesian();

            // КРИТЕРИИ РАЗРЫВА ЛИНИИ:
            
            // 1. Разрыв по глубине (Shadow problem).
            // Если соседний луч стал длиннее на 30 единиц - это дыра или угол.
            float depthJump = std::fabs(curP.dist - prevP.dist);
            
            // 2. Разрыв по физической дистанции.
            // Если точки физически далеко друг от друга (например, на горизонте), не соединяем.
            float physDist = prevWorld.dist(curWorld);

            bool isBreak = (depthJump > 20.0f) || (physDist > 25.0f);

            if (isBreak) {
                // Заканчиваем текущий кластер и обрабатываем его
                if (currentCluster.size() >= 2) {
                    recursiveSplit(currentCluster, 0, static_cast<int>(currentCluster.size()) - 1, currentFeatures);
                }
                currentCluster.clear();
            }
            
            currentCluster.push_back(curWorld);
        }

        // Добиваем последний хвост
        if (currentCluster.size() >= 2) {
            recursiveSplit(currentCluster, 0, static_cast<int>(currentCluster.size()) - 1, currentFeatures);
        }
    }

    // 4. Вливаем результат
    mergeIntoGlobal(currentFeatures);

    // 5. Pruning (Очистка)
    pruneTimer++;
    if (pruneTimer > 15) { // Чуть чаще (было 30)
        pruneMap(filteredScan);
        pruneTimer = 0;
    }

    // 6. Управление
    Vec2 instantVelocityCommand(0.0f, 0.0f);
    for (const auto& p : filteredScan) { // Используем фильтрованный, чтобы не шарахаться от глюков
        if (p.dist < params.reflexDist && p.dist > 0.1f) {
            const float gain = (params.reflexDist - p.dist) * params.reflexForce * 0.15f;
            instantVelocityCommand = instantVelocityCommand - (p.toCartesian().normalized() * gain);
        }
    }

    const float smoothing = 0.15f;
    smoothedVelocityCommand = smoothedVelocityCommand * (1.0f - smoothing) + instantVelocityCommand * smoothing;
    velocityCommand = smoothedVelocityCommand;

    // 7. Граф
    if (currentNodeIndex >= 0 && currentNodeIndex < static_cast<int>(mapGraph.size())) {
        float distToCurrent = estimatedPos.dist(mapGraph[currentNodeIndex].position);
        
        if (distToCurrent > params.newNodeDist) {
            GraphNode n;
            n.id = static_cast<int>(mapGraph.size());
            n.position = estimatedPos;
            n.localFeatures = currentFeatures; 
            n.connectedNodes.push_back(currentNodeIndex);
            
            mapGraph[currentNodeIndex].connectedNodes.push_back(n.id);
            mapGraph.push_back(n);
            currentNodeIndex = n.id;
        } else {
            mapGraph[currentNodeIndex].localFeatures = currentFeatures;
            mapGraph[currentNodeIndex].isOffloaded = false;
        }
    }

    processMemory();

    std::stringstream ss;
    const size_t offloaded = std::count_if(mapGraph.begin(), mapGraph.end(), [](const GraphNode& node) {
        return node.isOffloaded;
    });
    ss << "RAM: " << (currentRamUsage / 1024) << "KB / " << (params.ramLimitBytes / 1024) << "KB\n";
    ss << "GLOBAL MAP: " << globalLines.size() << " vectors\n";
    ss << "NODES: " << mapGraph.size() << " (offloaded: " << offloaded << ")\n";
    ss << "POS: " << (int)estimatedPos.x << ", " << (int)estimatedPos.y << "\n";
    ss << "Mode: Segmented & NoSkybox"; 
    memoryLog = ss.str();
}

std::vector<LineSegment> DroneCore::getDebugLines() const {
    return currentFeatures;
}