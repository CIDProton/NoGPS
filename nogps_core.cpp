#include "nogps_core.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <set>

namespace {
    float distToLineSegmentStatic(Vec2 p, Vec2 a, Vec2 b, Vec2 &outClosest) {
        Vec2 ab = b - a;
        float l2 = ab.lengthSq();
        if (l2 == 0.0f) { outClosest = a; return p.dist(a); }
        float t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2;
        t = std::max(0.0f, std::min(1.0f, t));
        outClosest = a + ab * t;
        return p.dist(outClosest);
    }

    void clampParams(CoreParams &p) {
        p.splitTolerance = std::max(0.5f, p.splitTolerance);
        p.mergeTolerance = std::max(1.0f, p.mergeTolerance);
        p.newNodeDist = std::max(10.0f, p.newNodeDist);
        // FIX: Явное приведение к size_t для std::max
        p.ramLimitBytes = std::max(static_cast<size_t>(1024 * 1024), p.ramLimitBytes);
    }
}

DroneCore::DroneCore() { reset(); }
DroneCore::~DroneCore() = default;

void DroneCore::reset() {
    std::filesystem::create_directories("cache");
    estimatedPos = Vec2(400.0f, 300.0f);
    mapGraph.clear();
    globalLines.clear();
    graphEdges.clear();
    currentFeatures.clear();
    spatialGrid.clear();
    currentRamUsage = 0;
    pruneTimer = 0;
    gridRebuildTimer = 0;

    GraphNode start;
    start.id = 0;
    start.position = estimatedPos;
    mapGraph.push_back(start);
    currentNodeIndex = 0;
    velocityCommand = Vec2(0.0f, 0.0f);
    smoothedVelocityCommand = Vec2(0.0f, 0.0f);
    memoryLog = "Core Reset. Graph System Active.";
    buildSpatialGrid();
}

void DroneCore::buildSpatialGrid() {
    spatialGrid.clear();
    for (int i = 0; i < (int)globalLines.size(); ++i) {
        spatialGrid.addLine(i, globalLines[i].start, globalLines[i].end, params.gridCellSize);
    }
    gridRebuildTimer = 0;
}

void DroneCore::recursiveSplit(const std::vector<Vec2> &points, int start, int end, std::vector<LineSegment> &result) {
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

void DroneCore::mergeIntoGlobal(const std::vector<LineSegment> &newLines) {
    for (const auto &nl : newLines) {
        if (nl.start.dist(nl.end) < 5.0f) continue;
        
        bool merged = false;
        for (auto &gl : globalLines) {
            Vec2 midNew = (nl.start + nl.end) * 0.5f;
            Vec2 midOld = (gl.start + gl.end) * 0.5f;
            if (midNew.dist(midOld) < params.mergeTolerance) {
                Vec2 dirNew = (nl.end - nl.start).normalized();
                Vec2 dirOld = (gl.end - gl.start).normalized();
                if (std::fabs(dirNew.dot(dirOld)) > 0.9f) {
                    gl.start = gl.start * 0.8f + nl.start * 0.2f;
                    gl.end = gl.end * 0.8f + nl.end * 0.2f;
                    merged = true;
                    break;
                }
            }
        }
        if (!merged) {
            globalLines.push_back(nl);
        }
    }

    gridRebuildTimer++;
    if (gridRebuildTimer > 5) { 
        buildSpatialGrid(); 
    }
}

void DroneCore::alignScanToMap(const std::vector<LidarPoint> &cleanScan) {
    if (globalLines.empty() || cleanScan.size() < 5) return;

    Vec2 totalCorrection(0.0f, 0.0f);
    int matchCount = 0;
    const float kSearchRadius = 30.0f;

    for (const auto &p : cleanScan) {
        Vec2 worldPoint = estimatedPos + p.toCartesian();
        
        std::vector<int> candidates = spatialGrid.query(worldPoint, kSearchRadius, params.gridCellSize);
        
        float bestDist = kSearchRadius;
        Vec2 bestClosestPoint;
        bool found = false;

        for (int idx : candidates) {
            if (idx < 0 || idx >= (int)globalLines.size()) continue;
            const auto &line = globalLines[idx];
            
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
            matchCount++;
        }
    }

    if (matchCount > 5) {
        Vec2 avgCorrection = totalCorrection * (1.0f / matchCount);
        if (avgCorrection.length() < 20.0f) {
            estimatedPos = estimatedPos + avgCorrection * 0.2f;
        }
    }
}

void DroneCore::pruneMap() {
    if (globalLines.empty()) return;

    for (int i = (int)globalLines.size() - 1; i >= 0; --i) {
        Vec2 mid = (globalLines[i].start + globalLines[i].end) * 0.5f;
        bool keep = false;

        for (const auto &node : mapGraph) {
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
    
    const Vec2 &currentPos = mapGraph[currentNodeIndex].position;
    
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
                graphEdges.push_back(edge);
            }
        }
    }
}

void DroneCore::createGraphNode() {
    GraphNode n;
    n.id = (int)mapGraph.size();
    n.position = estimatedPos;
    n.localFeatures = currentFeatures;
    n.connectedNodes.push_back(currentNodeIndex);
    
    mapGraph[currentNodeIndex].connectedNodes.push_back(n.id);
    mapGraph.push_back(n);
    currentNodeIndex = n.id;
    
    updateGraphTopology();
}

void DroneCore::processMemory() {
    currentRamUsage = 0;
    for (const auto &node : mapGraph) {
        if (!node.isOffloaded) currentRamUsage += node.getMemoryUsage();
    }
    currentRamUsage += globalLines.size() * sizeof(LineSegment);

    if (currentRamUsage > params.ramLimitBytes) {
        int victimIdx = -1;
        float maxDist = 0.0f;
        
        for (int i = 0; i < (int)mapGraph.size(); ++i) {
            if (i == currentNodeIndex || mapGraph[i].isOffloaded) continue;
            float d = mapGraph[i].position.dist(estimatedPos);
            if (d > maxDist) {
                maxDist = d;
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
                    f.write(reinterpret_cast<const char*>(mapGraph[victimIdx].localFeatures.data()), count * sizeof(LineSegment));
                }
            }
            mapGraph[victimIdx].localFeatures.clear();
            mapGraph[victimIdx].localFeatures.shrink_to_fit();
            mapGraph[victimIdx].isOffloaded = true;
        }
    }
    
    for (auto &node : mapGraph) {
        if (!node.isOffloaded) continue;
        if (node.position.dist(estimatedPos) < params.newNodeDist) {
            std::string fileName = "cache/node_" + std::to_string(node.id) + ".bin";
            std::ifstream f(fileName, std::ios::binary);
            if (f) {
                size_t count = 0;
                f.read(reinterpret_cast<char*>(&count), sizeof(size_t));
                if (count > 0) {
                    node.localFeatures.resize(count);
                    f.read(reinterpret_cast<char*>(node.localFeatures.data()), count * sizeof(LineSegment));
                }
                node.isOffloaded = false;
            }
        }
    }
}

std::vector<LidarPoint> DroneCore::preprocessScan(const std::vector<LidarPoint> &scan) const {
    std::vector<LidarPoint> filtered;
    filtered.reserve(scan.size());
    for (const auto &p : scan) {
        if (p.dist > 5.0f && p.dist < 480.0f) {
            filtered.push_back(p);
        }
    }
    return filtered;
}

void DroneCore::update(float dt, const std::vector<LidarPoint> &scan, Vec2 imuVelocity) {
    clampParams(params);
    
    estimatedPos = estimatedPos + (imuVelocity * dt);

    const std::vector<LidarPoint> filteredScan = preprocessScan(scan);

    alignScanToMap(filteredScan);

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

    mergeIntoGlobal(currentFeatures);
    
    if (currentNodeIndex >= 0 && currentNodeIndex < (int)mapGraph.size()) {
        float distToCurrent = estimatedPos.dist(mapGraph[currentNodeIndex].position);
        if (distToCurrent > params.newNodeDist) {
            createGraphNode();
        } else {
            mapGraph[currentNodeIndex].localFeatures = currentFeatures;
            mapGraph[currentNodeIndex].isOffloaded = false;
        }
    }

    pruneTimer++;
    if (pruneTimer > 60) {
        pruneMap();
        pruneTimer = 0;
    }

    processMemory();

    Vec2 instantVelocityCommand(0.0f, 0.0f);
    for (const auto &p : filteredScan) {
        if (p.dist < params.reflexDist && p.dist > 0.1f) {
            float gain = (params.reflexDist - p.dist) * params.reflexForce * 0.1f;
            instantVelocityCommand = instantVelocityCommand - (p.toCartesian().normalized() * gain);
        }
    }
    smoothedVelocityCommand = smoothedVelocityCommand * 0.85f + instantVelocityCommand * 0.15f;
    velocityCommand = smoothedVelocityCommand;

    std::stringstream ss;
    size_t offloaded = std::count_if(mapGraph.begin(), mapGraph.end(), [](const GraphNode &n){ return n.isOffloaded; });
    ss << "RAM: " << (currentRamUsage / 1024) << "KB\n";
    ss << "LINES: " << globalLines.size() << "\n";
    ss << "NODES: " << mapGraph.size() << " (off: " << offloaded << ")\n";
    ss << "EDGES: " << graphEdges.size() << "\n";
    ss << "POS: " << (int)estimatedPos.x << ", " << (int)estimatedPos.y;
    memoryLog = ss.str();
}