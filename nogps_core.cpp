#include "nogps_core.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <set>
#include <numeric>

namespace {
    void clampParams(CoreParams& p) {
        p.splitTolerance = std::max(0.5f, p.splitTolerance);
        p.mergeTolerance = std::max(1.0f, p.mergeTolerance);
        p.newNodeDist = std::max(10.0f, p.newNodeDist);
        p.ramLimitBytes = std::max(static_cast<size_t>(1024 * 1024), p.ramLimitBytes);
        p.nodeMergeDistance = std::max(2.0f, p.nodeMergeDistance);
        p.droneDrag = std::clamp(p.droneDrag, 0.5f, 0.99f);
    }
}

DroneCore::DroneCore() { reset(); }
DroneCore::~DroneCore() = default;

float DroneCore::distToLineSegmentStatic(Vec2 p, Vec2 a, Vec2 b, Vec2& outClosest) {
    Vec2 ab = b - a;
    float l2 = ab.lengthSq();
    if (l2 == 0.0f) { outClosest = a; return p.dist(a); }
    float t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2;
    t = std::max(0.0f, std::min(1.0f, t));
    outClosest = a + ab * t;
    return p.dist(outClosest);
}

void DroneCore::reset() {
    std::filesystem::create_directories("cache");
    estimatedPos = Vec2(400.0f, 300.0f);
    estimatedOrientation = 0.0f;
    estimatedVelocity = Vec2(0.0f, 0.0f);
    mapGraph.clear();
    globalLines.clear();
    graphEdges.clear();
    currentFeatures.clear();
    spatialGrid.clear();
    currentRamUsage = 0;
    pruneTimer = 0;
    gridRebuildTimer = 0;
    currentTime = 0.0f;
    lastEstimatedPos = estimatedPos;
    poseConfidence = PoseConfidence();
    recentVelocities.clear();
    currentStats = CoreStats();

    GraphNode start;
    start.id = 0;
    start.position = estimatedPos;
    start.positionConfidence = 1.0f;
    mapGraph.push_back(start);
    currentNodeIndex = 0;
    velocityCommand = Vec2(0.0f, 0.0f);
    smoothedVelocityCommand = Vec2(0.0f, 0.0f);
    memoryLog = "Core Reset. Graph System Active. Confidence: 100%";
    buildSpatialGrid();
}

void DroneCore::buildSpatialGrid() {
    spatialGrid.clear();
    for (int i = 0; i < (int)globalLines.size(); ++i) {
        if (globalLines[i].confidence > 0.3f) {  // 🆕 Не добавляем низкодоверительные
            spatialGrid.addLine(i, globalLines[i].start, globalLines[i].end, params.gridCellSize);
        }
    }
    gridRebuildTimer = 0;
}

// 🆕 Оптимизация: слияние близких узлов графа
void DroneCore::mergeNearbyNodes() {
    if (mapGraph.size() < 2) return;
    
    std::vector<bool> toRemove(mapGraph.size(), false);
    int mergedCount = 0;
    
    for (size_t i = 0; i < mapGraph.size(); ++i) {
        if (toRemove[i] || mapGraph[i].isOffloaded) continue;
        
        for (size_t j = i + 1; j < mapGraph.size(); ++j) {
            if (toRemove[j] || mapGraph[j].isOffloaded) continue;
            
            float dist = mapGraph[i].position.dist(mapGraph[j].position);
            if (dist < params.nodeMergeDistance) {
                // Проверка схожести признаков
                float similarity = mapGraph[i].featureSimilarity(mapGraph[j]);
                
                if (similarity > params.nodeFeatureSimilarity) {
                    // Слияние: сохраняем узел с большим visitCount
                    int keepIdx = (mapGraph[i].visitCount > mapGraph[j].visitCount) ? i : j;
                    int removeIdx = (keepIdx == i) ? j : i;
                    
                    // Переносим признаки
                    mapGraph[keepIdx].localFeatures.insert(
                        mapGraph[keepIdx].localFeatures.end(),
                        mapGraph[removeIdx].localFeatures.begin(),
                        mapGraph[removeIdx].localFeatures.end()
                    );
                    
                    // Обновляем рёбра
                    for (int conn : mapGraph[removeIdx].connectedNodes) {
                        if (conn != (int)keepIdx && 
                            std::find(mapGraph[keepIdx].connectedNodes.begin(),
                                     mapGraph[keepIdx].connectedNodes.end(), 
                                     conn) == mapGraph[keepIdx].connectedNodes.end()) {
                            mapGraph[keepIdx].connectedNodes.push_back(conn);
                        }
                    }
                    
                    // Усредняем позицию с весом по доверию
                    float totalConf = mapGraph[i].positionConfidence + mapGraph[j].positionConfidence;
                    if (totalConf > 0) {
                        mapGraph[keepIdx].position = 
                            (mapGraph[i].position * mapGraph[i].positionConfidence +
                             mapGraph[j].position * mapGraph[j].positionConfidence) / totalConf;
                        mapGraph[keepIdx].positionConfidence = 
                            std::max(mapGraph[i].positionConfidence, mapGraph[j].positionConfidence);
                    }
                    
                    toRemove[removeIdx] = true;
                    mergedCount++;
                    break;
                }
            }
        }
    }
    
    // Удаление помеченных узлов (с перенумерацией)
    if (mergedCount > 0) {
        std::vector<int> oldToNew(mapGraph.size(), -1);
        std::vector<GraphNode> newGraph;
        int newId = 0;
        
        for (size_t i = 0; i < mapGraph.size(); ++i) {
            if (!toRemove[i]) {
                oldToNew[i] = newId;
                GraphNode node = mapGraph[i];
                node.id = newId;
                newGraph.push_back(node);
                newId++;
            }
        }
        
        // Обновляем связи
        for (auto& node : newGraph) {
            std::vector<int> newConnections;
            for (int conn : node.connectedNodes) {
                if (conn >= 0 && conn < (int)oldToNew.size() && oldToNew[conn] >= 0) {
                    newConnections.push_back(oldToNew[conn]);
                }
            }
            node.connectedNodes = newConnections;
        }
        
        mapGraph = std::move(newGraph);
        
        // Обновляем текущий индекс
        if (currentNodeIndex >= 0 && currentNodeIndex < (int)oldToNew.size()) {
            currentNodeIndex = oldToNew[currentNodeIndex];
        }
        
        currentStats.mergedNodes += mergedCount;
        memoryLog += " Merged " + std::to_string(mergedCount) + " nodes.";
    }
}

// 🆕 Удаление фантомных стен
void DroneCore::removePhantomLines(float currentTime) {
    if (globalLines.empty()) return;
    
    int removedCount = 0;
    
    for (int i = (int)globalLines.size() - 1; i >= 0; --i) {
        auto& line = globalLines[i];
        
        // Уменьшаем доверие со временем
        float timeSinceObservation = currentTime - line.lastObservationTime;
        line.confidence *= std::pow(0.99f, timeSinceObservation * 10.0f);
        
        // Удаляем если:
        // 1. Мало наблюдений И низкое доверие
        // 2. Доверие упало ниже порога
        // 3. Линия слишком короткая (скорее всего шум)
        bool shouldRemove = false;
        
        if (line.observationCount < params.minObservationsForKeep && 
            line.confidence < 0.3f) {
            shouldRemove = true;
        }
        
        if (line.confidence < 0.1f) {
            shouldRemove = true;
        }
        
        if (line.length() < 3.0f) {
            shouldRemove = true;
        }
        
        if (shouldRemove) {
            globalLines.erase(globalLines.begin() + i);
            removedCount++;
        }
    }
    
    if (removedCount > 0) {
        currentStats.phantomLinesRemoved += removedCount;
        buildSpatialGrid();
        memoryLog += " Removed " + std::to_string(removedCount) + " phantom lines.";
    }
}

// 🆕 Детекция застревания
bool DroneCore::detectStuck(const InertialData& inertial) const {
    // Проверяем недавние скорости
    if (recentVelocities.size() < RECENT_VELOCITY_WINDOW) return false;
    
    float avgSpeed = 0.0f;
    for (const auto& v : recentVelocities) {
        avgSpeed += v.length();
    }
    avgSpeed /= recentVelocities.size();
    
    // Если средняя скорость мала, но есть команда движения - застряли
    bool hasInput = inertial.acceleration.length() > 10.0f;
    bool isSlow = avgSpeed < params.stuckVelocityThreshold;
    
    return hasInput && isSlow;
}

void DroneCore::optimizeGraphNodes() {
    // Вызываем оптимизацию периодически
    static int optimizeTimer = 0;
    optimizeTimer++;
    
    if (optimizeTimer > 120) {  // Каждые ~2 секунды
        mergeNearbyNodes();
        optimizeTimer = 0;
    }
}

void DroneCore::recursiveSplit(const std::vector<Vec2>& points, int start, int end, 
                               std::vector<LineSegment>& result) {
    if (end - start < 1) return;
    if (end - start == 1) {
        if (points[start].dist(points[end]) < 50.0f) {
            result.push_back({points[start], points[end]});
        }
        return;
    }

    float dMax = 0.0f;
    int idx = -1;
    for (int i = start + 1; i < end; ++i) {
        Vec2 dummy;
        float d = distToLineSegmentStatic(points[i], points[start], points[end], dummy);
        if (d > dMax) { dMax = d; idx = i; }
    }

    if (dMax > params.splitTolerance && idx != -1) {
        recursiveSplit(points, start, idx, result);
        recursiveSplit(points, idx, end, result);
    } else {
        result.push_back({points[start], points[end]});
    }
}

void DroneCore::mergeIntoGlobal(const std::vector<LineSegment>& newLines, float currentTime) {
    for (const auto& nl : newLines) {
        if (nl.start.dist(nl.end) < 5.0f) continue;
        
        bool merged = false;
        std::vector<int> candidates = spatialGrid.query(
            nl.center(), params.mergeTolerance * 2, params.gridCellSize
        );
        
        for (int idx : candidates) {
            if (idx < 0 || idx >= (int)globalLines.size()) continue;
            auto& gl = globalLines[idx];
            
            Vec2 midNew = nl.center();
            Vec2 midOld = gl.center();
            
            if (midNew.dist(midOld) < params.mergeTolerance) {
                Vec2 dirNew = nl.direction();
                Vec2 dirOld = gl.direction();
                
                if (std::fabs(dirNew.dot(dirOld)) > 0.9f) {
                    // 🆕 Обновляем существующую линию с накоплением
                    float weight = 0.2f;
                    gl.start = gl.start * (1 - weight) + nl.start * weight;
                    gl.end = gl.end * (1 - weight) + nl.end * weight;
                    gl.observationCount++;
                    gl.lastObservationTime = currentTime;
                    gl.confidence = std::min(1.0f, gl.confidence + 0.1f);
                    merged = true;
                    break;
                }
            }
        }
        
        if (!merged) {
            LineSegment newLine = nl;
            newLine.lastObservationTime = currentTime;
            newLine.confidence = 0.5f;  // 🆕 Новая линия начинается с низким доверием
            globalLines.push_back(newLine);
        }
    }

    gridRebuildTimer++;
    if (gridRebuildTimer > 5) { 
        buildSpatialGrid(); 
    }
}

void DroneCore::alignScanToMap(const std::vector<LidarPoint>& cleanScan, float currentTime) {
    if (globalLines.empty() || cleanScan.size() < 5) return;

    Vec2 totalCorrection(0.0f, 0.0f);
    int matchCount = 0;
    float totalError = 0.0f;
    const float kSearchRadius = 30.0f;

    // 🆕 Детекция застревания для коррекции веса
    bool isStuck = detectStuck(InertialData{Vec2(0,0), 0, 0, 0});
    float lidarWeight = isStuck ? 
        poseConfidence.getLidarWeight() * params.stuckConfidenceBoost :
        poseConfidence.getLidarWeight();
    lidarWeight = std::clamp(lidarWeight, 0.3f, 0.9f);

    for (const auto& p : cleanScan) {
        Vec2 worldPoint = estimatedPos + p.toCartesian();
        
        std::vector<int> candidates = spatialGrid.query(worldPoint, kSearchRadius, params.gridCellSize);
        
        float bestDist = kSearchRadius;
        Vec2 bestClosestPoint;
        bool found = false;

        for (int idx : candidates) {
            if (idx < 0 || idx >= (int)globalLines.size()) continue;
            const auto& line = globalLines[idx];
            
            // 🆕 Игнорируем низкодоверительные линии
            if (line.confidence < 0.3f) continue;
            
            Vec2 closest;
            float d = distToLineSegmentStatic(worldPoint, line.start, line.end, closest);
            if (d < bestDist) {
                bestDist = d;
                bestClosestPoint = closest;
                found = true;
            }
        }

        if (found) {
            Vec2 error = bestClosestPoint - worldPoint;
            totalCorrection = totalCorrection + error;
            totalError += bestDist;
            matchCount++;
        }
    }

    if (matchCount > 5) {
        Vec2 avgCorrection = totalCorrection * (1.0f / matchCount);
        float avgError = totalError / matchCount;
        
        // 🆕 Ограничиваем коррекцию и учитываем доверие
        float correctionScale = lidarWeight * (1.0f - std::min(1.0f, avgError / 10.0f));
        
        if (avgCorrection.length() < params.lidarCorrectionMax) {
            estimatedPos = estimatedPos + avgCorrection * correctionScale * 0.2f;
        }
        
        // 🆕 Обновляем доверие к позиции
        float matchQuality = static_cast<float>(matchCount) / cleanScan.size();
        poseConfidence.update(matchQuality, isStuck, 0.016f);
    }
}

void DroneCore::pruneMap() {
    if (globalLines.empty()) return;

    for (int i = (int)globalLines.size() - 1; i >= 0; --i) {
        Vec2 mid = globalLines[i].center();
        bool keep = false;

        for (const auto& node : mapGraph) {
            if (node.isOffloaded) continue;
            if (mid.dist(node.position) < params.pruneDistance) {
                keep = true;
                break;
            }
        }

        if (!keep) {
            globalLines.erase(globalLines.begin() + i);
        }
    }
    buildSpatialGrid();
}

void DroneCore::updateGraphTopology() {
    if (currentNodeIndex < 0 || currentNodeIndex >= (int)mapGraph.size()) return;
    
    const Vec2& currentPos = mapGraph[currentNodeIndex].position;
    
    for (int i = 0; i < (int)mapGraph.size(); ++i) {
        if (i == currentNodeIndex) continue;
        if (mapGraph[i].isOffloaded) continue;

        float d = currentPos.dist(mapGraph[i].position); 
        if (d < params.newNodeDist * 1.5f) {
            bool exists = false;
            for (int conn : mapGraph[currentNodeIndex].connectedNodes) { 
                if (conn == i) { exists = true; break; }
            }
            
            if (!exists) {
                mapGraph[currentNodeIndex].connectedNodes.push_back(i);
                mapGraph[i].connectedNodes.push_back(currentNodeIndex);
                
                GraphEdge edge;
                edge.fromId = currentNodeIndex;
                edge.toId = i;
                edge.weight = 1.0f / (d * d + 0.1f);
                edge.relativeTransform = mapGraph[i].position - currentPos;
                edge.confidence = poseConfidence.value;  // 🆕 Доверие ребра
                graphEdges.push_back(edge);
            }
        }
    }
}

void DroneCore::createGraphNode() {
    GraphNode n;
    n.id = (int)mapGraph.size();
    n.position = estimatedPos;
    n.orientation = estimatedOrientation;
    n.localFeatures = currentFeatures;
    n.connectedNodes.push_back(currentNodeIndex);
    n.positionConfidence = poseConfidence.value;
    n.visitCount = 1;
    n.lastVisitTime = currentTime;
    
    mapGraph[currentNodeIndex].connectedNodes.push_back(n.id);
    mapGraph.push_back(n);
    currentNodeIndex = n.id;
    
    updateGraphTopology();
}

void DroneCore::processMemory() {
    currentRamUsage = 0;
    for (const auto& node : mapGraph) {
        if (!node.isOffloaded) currentRamUsage += node.getMemoryUsage();
    }
    currentRamUsage += globalLines.size() * sizeof(LineSegment);

    if (currentRamUsage > params.ramLimitBytes) {
        int victimIdx = -1;
        float maxDist = 0.0f;
        
        for (int i = 0; i < (int)mapGraph.size(); ++i) {
            if (i == currentNodeIndex || mapGraph[i].isOffloaded) continue;
            // 🆕 Выбираем узел с низким доверием и далеко
            float score = mapGraph[i].position.dist(estimatedPos) * 
                         (1.0f - mapGraph[i].positionConfidence * 0.5f);
            if (score > maxDist) {
                maxDist = score;
                victimIdx = i;
            }
        }

        if (victimIdx != -1) {
            std::string fileName = "cache/node_" + std::to_string(mapGraph[victimIdx].id) + ".bin";
            std::ofstream f(fileName, std::ios::binary | std::ios::trunc);
            if (f) {
                size_t count = mapGraph[victimIdx].localFeatures.size();
                f.write(reinterpret_cast<const char*>(&count), sizeof(size_t));
                if (count > 0) {
                    f.write(reinterpret_cast<const char*>(mapGraph[victimIdx].localFeatures.data()), 
                           count * sizeof(LineSegment));
                }
            }
            mapGraph[victimIdx].localFeatures.clear();
            mapGraph[victimIdx].localFeatures.shrink_to_fit();
            mapGraph[victimIdx].isOffloaded = true;
        }
    }
    
    for (auto& node : mapGraph) {
        if (!node.isOffloaded) continue;
        if (node.position.dist(estimatedPos) < params.newNodeDist) {
            std::string fileName = "cache/node_" + std::to_string(node.id) + ".bin";
            std::ifstream f(fileName, std::ios::binary);
            if (f) {
                size_t count = 0;
                f.read(reinterpret_cast<char*>(&count), sizeof(size_t));
                if (count > 0) {
                    node.localFeatures.resize(count);
                    f.read(reinterpret_cast<char*>(node.localFeatures.data()), 
                          count * sizeof(LineSegment));
                }
                node.isOffloaded = false;
            }
        }
    }
}

std::vector<LidarPoint> DroneCore::preprocessScan(const std::vector<LidarPoint>& scan) const {
    std::vector<LidarPoint> filtered;
    filtered.reserve(scan.size());
    for (const auto& p : scan) {
        if (p.dist > 5.0f && p.dist < 480.0f) {
            filtered.push_back(p);
        }
    }
    return filtered;
}

void DroneCore::update(float currentTime, const std::vector<LidarPoint>& scan, 
                       const InertialData& inertial) {
    clampParams(params);
    this->currentTime = currentTime;
    
    // 🆕 1. Prediction с инерциальными данными
    Vec2 globalAccel = inertial.acceleration.rotated(inertial.orientation);
    estimatedVelocity = estimatedVelocity + globalAccel * inertial.deltaTime;
    estimatedVelocity = estimatedVelocity * params.droneDrag;  // 🆕 Сопротивление воздуха
    
    Vec2 predictedPos = estimatedPos + estimatedVelocity * inertial.deltaTime;
    estimatedOrientation += inertial.angularVelocity * inertial.deltaTime;
    
    // 🆕 Детекция застревания - если застряли, игнорируем IMU
    bool isStuck = detectStuck(inertial);
    if (isStuck) {
        estimatedVelocity = estimatedVelocity * 0.5f;  // 🆕 Быстрое затухание
        lastStuckTime = currentTime;
    }
    
    // Применяем предсказание с учётом доверия
    float imuTrust = isStuck ? 0.1f : 0.7f;
    estimatedPos = estimatedPos * (1 - imuTrust) + predictedPos * imuTrust;
    
    // 🆕 Сохраняем для окна скоростей
    recentVelocities.push_back(estimatedVelocity);
    if (recentVelocities.size() > RECENT_VELOCITY_WINDOW) {
        recentVelocities.erase(recentVelocities.begin());
    }

    // 2. Preprocess
    const std::vector<LidarPoint> filteredScan = preprocessScan(scan);

    // 3. Correction (Scan Matching) - теперь с учётом доверия
    alignScanToMap(filteredScan, currentTime);

    // 4. Segmentation
    currentFeatures.clear();
    if (!filteredScan.empty()) {
        std::vector<Vec2> cluster;
        cluster.reserve(filteredScan.size());
        cluster.push_back(estimatedPos + filteredScan[0].toCartesian());

        for (size_t i = 1; i < filteredScan.size(); ++i) {
            Vec2 prevW = estimatedPos + filteredScan[i-1].toCartesian();
            Vec2 currW = estimatedPos + filteredScan[i].toCartesian();
             
            float depthJump = std::fabs(filteredScan[i].dist - filteredScan[i-1].dist);
            float physDist = prevW.dist(currW);

            if (depthJump > 30.0f || physDist > 40.0f) {
                if (cluster.size() >= 2) {
                    recursiveSplit(cluster, 0, (int)cluster.size() - 1, currentFeatures);
                }
                cluster.clear();
            }
            cluster.push_back(currW);
        }
        if (cluster.size() >= 2) {
            recursiveSplit(cluster, 0, (int)cluster.size() - 1, currentFeatures);
        }
    }

    // 5. Merge & Graph
    mergeIntoGlobal(currentFeatures, currentTime);
    
    if (currentNodeIndex >= 0 && currentNodeIndex < (int)mapGraph.size()) {
        float distToCurrent = estimatedPos.dist(mapGraph[currentNodeIndex].position);
        if (distToCurrent > params.newNodeDist) {
            createGraphNode();
        } else {
            mapGraph[currentNodeIndex].localFeatures = currentFeatures;
            mapGraph[currentNodeIndex].isOffloaded = false;
            mapGraph[currentNodeIndex].visitCount++;
            mapGraph[currentNodeIndex].lastVisitTime = currentTime;
            mapGraph[currentNodeIndex].positionConfidence = 
                std::max(mapGraph[currentNodeIndex].positionConfidence, poseConfidence.value);
        }
    }

    // 6. Maintenance
    pruneTimer++;
    if (pruneTimer > 60) {
        pruneMap();
        removePhantomLines(currentTime);  // 🆕 Удаление фантомов
        pruneTimer = 0;
    }
    
    optimizeGraphNodes();  // 🆕 Оптимизация графа
    processMemory();

    // 7. Reflex Control
    Vec2 instantVelocityCommand(0.0f, 0.0f);
    for (const auto& p : filteredScan) {
        if (p.dist < params.reflexDist && p.dist > 0.1f) {
            float gain = (params.reflexDist - p.dist) * params.reflexForce * 0.1f;
            instantVelocityCommand = instantVelocityCommand - (p.toCartesian().normalized() * gain);
        }
    }
    smoothedVelocityCommand = smoothedVelocityCommand * 0.85f + instantVelocityCommand * 0.15f;
    velocityCommand = smoothedVelocityCommand;

    // 🆕 Обновление статистики
    currentStats.totalNodes = mapGraph.size();
    currentStats.offloadedNodes = std::count_if(mapGraph.begin(), mapGraph.end(),
        [](const GraphNode& n){ return n.isOffloaded; });
    
    float totalConf = 0.0f;
    for (const auto& n : mapGraph) {
        if (!n.isOffloaded) totalConf += n.positionConfidence;
    }
    currentStats.avgConfidence = mapGraph.empty() ? 0.0f : 
        totalConf / (mapGraph.size() - currentStats.offloadedNodes);
    currentStats.ramUsage = currentRamUsage;

    // Logs
    std::stringstream ss;
    size_t offloaded = currentStats.offloadedNodes;
    ss << "RAM: " << (currentRamUsage / 1024) << "KB\n";
    ss << "LINES: " << globalLines.size() << " (conf: " << currentStats.avgConfidence << ")\n";
    ss << "NODES: " << mapGraph.size() << " (off: " << offloaded << ", merged: " 
       << currentStats.mergedNodes << ")\n";
    ss << "POS: " << (int)estimatedPos.x << ", " << (int)estimatedPos.y << "\n";
    ss << "CONF: " << (poseConfidence.value * 100) << "% | STUCK: " << (isStuck ? "YES" : "NO");
    memoryLog = ss.str();
}