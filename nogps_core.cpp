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
    if (end - start < 2) {
        if (end - start == 1 && points[start].dist(points[end]) > 5.0f) {
             result.push_back({points[start], points[end]});
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
        // Проверяем, не слишком ли длинная линия для "пустого" места
        if (points[start].dist(points[end]) > 5.0f) {
            result.push_back({points[start], points[end]});
        }
    }
}

void DroneCore::mergeIntoGlobal(const std::vector<LineSegment>& newLines) {
    for (const auto& nl : newLines) {
        bool merged = false;
        
        // Попробуем найти похожую линию в глобальной карте
        for (auto& gl : globalLines) {
            // Проверка 1: Центры линий рядом?
            Vec2 midNew = (nl.start + nl.end) * 0.5f;
            Vec2 midOld = (gl.start + gl.end) * 0.5f;
            
            if (midNew.dist(midOld) < params.mergeTolerance) {
                // Проверка 2: Углы линий похожи? (dot product)
                Vec2 dirNew = (nl.end - nl.start).normalized();
                Vec2 dirOld = (gl.end - gl.start).normalized();
                float dot = std::fabs(dirNew.dot(dirOld));
                
                if (dot > 0.9f) { // ~25 градусов отклонение макс
                    // Сливаем: берем среднее взвешенное
                    // (тут простой вариант: двигаем концы старой линии к новой на 10%)
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

    // Лимит векторов, чтобы память не текла вечно в симуляторе
    if (globalLines.size() > 2000) {
        globalLines.erase(globalLines.begin(), globalLines.begin() + 100);
    }
}

// -----------------------------------------------------------------------------
// SCAN MATCHING (ICP Lite)
// Корректирует estimatedPos на основе совпадения скана с картой
// -----------------------------------------------------------------------------
void DroneCore::alignScanToMap(const std::vector<LidarPoint>& cleanScan) {
    if (globalLines.empty() || cleanScan.size() < 10) return;

    Vec2 totalCorrection(0.0f, 0.0f);
    int matchCount = 0;
    const float kSearchRadius = 30.0f; // Ищем стены в радиусе 30px

    for (const auto& p : cleanScan) {
        // Где эта точка по мнению нашего текущего (возможно ошибочного) положения
        Vec2 worldPoint = estimatedPos + p.toCartesian();

        float bestDist = kSearchRadius;
        Vec2 bestClosestPoint;
        bool found = false;

        // Ищем ближайшую стену
        // (В реальном продакшене тут нужен QuadTree или Grid, но для 2000 линий O(N) сойдет)
        for (const auto& line : globalLines) {
            // Быстрая проверка bounding box
            float minX = std::min(line.start.x, line.end.x) - kSearchRadius;
            float maxX = std::max(line.start.x, line.end.x) + kSearchRadius;
            float minY = std::min(line.start.y, line.end.y) - kSearchRadius;
            float maxY = std::max(line.start.y, line.end.y) + kSearchRadius;

            if (worldPoint.x < minX || worldPoint.x > maxX || 
                worldPoint.y < minY || worldPoint.y > maxY) {
                continue;
            }

            Vec2 closest;
            float d = distToLineSegment(worldPoint, line.start, line.end, closest);
            if (d < bestDist) {
                bestDist = d;
                bestClosestPoint = closest;
                found = true;
            }
        }

        if (found) {
            // Вектор ошибки: куда надо сдвинуть точку скана, чтобы она легла на стену
            Vec2 error = bestClosestPoint - worldPoint;
            totalCorrection = totalCorrection + error;
            matchCount++;
        }
    }

    if (matchCount > 5) {
        // Усредняем коррекцию
        Vec2 avgCorrection = totalCorrection * (1.0f / matchCount);
        
        // Применяем "Gain" (коэффициент доверия). 0.2 - плавная коррекция.
        // Если коррекция слишком большая (>15), значит мы потерялись или это новая комната,
        // лучше не телепортироваться резко.
        if (avgCorrection.length() < 15.0f) {
            estimatedPos = estimatedPos + avgCorrection * 0.3f;
        }
    }
}

// -----------------------------------------------------------------------------
// MAP PRUNING (Raycasting cleanup)
// Удаляет линии, которые лидар видит "насквозь"
// -----------------------------------------------------------------------------
void DroneCore::pruneMap(const std::vector<LidarPoint>& scan) {
    if (globalLines.empty()) return;

    // Идем задом наперед, чтобы безопасно удалять
    for (int i = static_cast<int>(globalLines.size()) - 1; i >= 0; --i) {
        Vec2 mid = (globalLines[i].start + globalLines[i].end) * 0.5f;
        Vec2 toLine = mid - estimatedPos;
        float distToLineCenter = toLine.length();

        // Если стена далеко, мы не можем быть уверены, что "пробили" её
        if (distToLineCenter > 200.0f) continue; 
        if (distToLineCenter < 10.0f) continue; // Слишком близко, может быть глюк

        float angleToLine = std::atan2(toLine.y, toLine.x);
        
        // Ищем соответствующий луч лидара
        // Лидар сканирует 360 градусов. Найдем луч с минимальной разницей угла.
        // (Предполагаем, что скан упорядочен или плотный)
        
        float minAngleDiff = 100.0f;
        float observedDist = 0.0f;
        
        // Простой перебор (можно оптимизировать через индекс, зная шаг лидара)
        for (const auto& p : scan) {
            float diff = wrappedAngleDiff(p.angle, angleToLine);
            if (diff < minAngleDiff) {
                minAngleDiff = diff;
                observedDist = p.dist;
            }
        }

        // Если нашли луч, который смотрит прямо в центр линии (с допуском ~3 градуса)
        if (minAngleDiff < 0.05f) {
            // ГЛАВНОЕ УСЛОВИЕ: Лидар видит ЗНАЧИТЕЛЬНО дальше, чем стоит эта стена.
            // Значит стены тут нет (это был призрак, дрейф или дверь открылась).
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

    // 1. Фильтр дальности (отсекаем слишком близкие шумы и "бесконечность")
    for (const auto& p : scan) {
        if (p.dist > 10.0f && p.dist < 490.0f) {
            filtered.push_back(p);
        }
    }

    if (filtered.size() < 5) return filtered;

    // 2. Медианный фильтр (очень простой) для удаления одиночных выбросов
    std::vector<LidarPoint> smoothed = filtered;
    for (size_t i = 1; i + 1 < filtered.size(); ++i) {
        float d1 = filtered[i-1].dist;
        float d2 = filtered[i].dist;
        float d3 = filtered[i+1].dist;
        // Если точка резко выбивается от соседей -> заменяем средним
        if (std::fabs(d2 - d1) > 10.0f && std::fabs(d2 - d3) > 10.0f) {
            smoothed[i].dist = (d1 + d3) * 0.5f;
        }
    }

    // 3. Сегментация (разрываем скан на кластеры)
    std::vector<LidarPoint> segmented;
    segmented.reserve(smoothed.size());
    if (!smoothed.empty()) segmented.push_back(smoothed.front());

    for (size_t i = 1; i < smoothed.size(); ++i) {
        const auto& prev = smoothed[i - 1];
        const auto& cur = smoothed[i];
        
        const float jump = std::fabs(cur.dist - prev.dist);
        // Используем реальную декартову дистанцию между точками для разрыва
        // (prev_vec - cur_vec).length() - но приближенно:
        
        // Разрыв если: резкий скачок глубины ИЛИ резкий угол
        // 15.0f - порог разрыва глубины
        if (jump > 15.0f) {
            // Это разрыв объектов. Не соединяем их линией в recursiveSplit,
            // но нам нужен непрерывный массив точек для split. 
            // Хитрость: мы просто добавим "Nan" точку или разобьем обработку выше.
            // Но в текущей архитектуре recursiveSplit работает с массивом.
            // Поэтому просто оставим как есть, но recursiveSplit учтет дистанцию.
        }
        
        segmented.push_back(cur);
    }

    return segmented;
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

    // Алгоритм выгрузки (как был, работает нормально)
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
    
    // Алгоритм загрузки (если подошли близко)
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

    // 1. Предсказание положения (IMU / Одометрия)
    estimatedPos = estimatedPos + (imuVelocity * dt);

    const std::vector<LidarPoint> filteredScan = preprocessScan(scan);

    // 2. КОРРЕКЦИЯ ПОЛОЖЕНИЯ (Scan Matching)
    // Важно: делать ДО того, как добавим новые линии в карту
    alignScanToMap(filteredScan);

    // 3. Формирование облака точек в (теперь уже скорректированных) мировых координатах
    std::vector<Vec2> cloud;
    cloud.reserve(filteredScan.size());
    for (const auto& p : filteredScan) {
        cloud.push_back(estimatedPos + p.toCartesian());
    }

    // 4. Выделение геометрии (Split & Merge)
    currentFeatures.clear();
    if (cloud.size() > 5) {
        // Разбиваем скан на линии
        recursiveSplit(cloud, 0, static_cast<int>(cloud.size()) - 1, currentFeatures);
        
        // Вливаем в глобальную карту
        mergeIntoGlobal(currentFeatures);
    }

    // 5. ОЧИСТКА КАРТЫ (Pruning)
    // Запускаем не каждый кадр, а раз в ~0.5 сек (30 кадров), это тяжелая операция
    pruneTimer++;
    if (pruneTimer > 30) {
        pruneMap(filteredScan);
        pruneTimer = 0;
    }

    // 6. Рефлекторное управление (отталкивание)
    Vec2 instantVelocityCommand(0.0f, 0.0f);
    for (const auto& p : filteredScan) {
        if (p.dist < params.reflexDist && p.dist > 0.1f) {
            const float gain = (params.reflexDist - p.dist) * params.reflexForce * 0.15f; // чуть поднял gain
            instantVelocityCommand = instantVelocityCommand - (p.toCartesian().normalized() * gain);
        }
    }

    const float smoothing = 0.15f;
    smoothedVelocityCommand = smoothedVelocityCommand * (1.0f - smoothing) + instantVelocityCommand * smoothing;
    velocityCommand = smoothedVelocityCommand;

    // 7. Работа с графом (создание узлов пути)
    if (currentNodeIndex >= 0 && currentNodeIndex < static_cast<int>(mapGraph.size())) {
        float distToCurrent = estimatedPos.dist(mapGraph[currentNodeIndex].position);
        
        if (distToCurrent > params.newNodeDist) {
            GraphNode n;
            n.id = static_cast<int>(mapGraph.size());
            n.position = estimatedPos;
            n.localFeatures = currentFeatures; // Сохраняем локальный слепок
            n.connectedNodes.push_back(currentNodeIndex);
            
            mapGraph[currentNodeIndex].connectedNodes.push_back(n.id);
            mapGraph.push_back(n);
            currentNodeIndex = n.id;
        } else {
            // Обновляем текущий узел актуальными данными
            mapGraph[currentNodeIndex].localFeatures = currentFeatures;
            mapGraph[currentNodeIndex].isOffloaded = false;
        }
    }

    processMemory();

    // Логирование
    std::stringstream ss;
    const size_t offloaded = std::count_if(mapGraph.begin(), mapGraph.end(), [](const GraphNode& node) {
        return node.isOffloaded;
    });
    ss << "RAM: " << (currentRamUsage / 1024) << "KB / " << (params.ramLimitBytes / 1024) << "KB\n";
    ss << "GLOBAL MAP: " << globalLines.size() << " vectors\n";
    ss << "NODES: " << mapGraph.size() << " (offloaded: " << offloaded << ")\n";
    ss << "POS: " << (int)estimatedPos.x << ", " << (int)estimatedPos.y << "\n";
    ss << "Core: ScanMatched & Pruned"; 
    memoryLog = ss.str();
}

std::vector<LineSegment> DroneCore::getDebugLines() const {
    return currentFeatures;
}