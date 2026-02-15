#include "nogps_core.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <sstream>

namespace {
float distToLine(Vec2 p, Vec2 a, Vec2 b) {
    float l2 = (a - b).length();
    l2 *= l2;
    if (l2 == 0.0f) {
        return p.dist(a);
    }
    const float dot = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y));
    const float t = std::max(0.0f, std::min(1.0f, dot / l2));
    return p.dist(a + (b - a) * t);
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

float Vec2::length() const {
    return std::sqrt(x * x + y * y);
}

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
        return;
    }

    float dMax = 0.0f;
    int idx = -1;
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
    } else if (points[start].dist(points[end]) > 10.0f) {
        result.push_back({points[start], points[end]});
    }
}

void DroneCore::mergeIntoGlobal(const std::vector<LineSegment>& newLines) {
    for (const auto& nl : newLines) {
        bool merged = false;
        const Vec2 mid = (nl.start + nl.end) * 0.5f;

        for (auto& gl : globalLines) {
            if (distToLine(mid, gl.start, gl.end) < params.mergeTolerance) {
                gl.start = gl.start * 0.98f + nl.start * 0.02f;
                gl.end = gl.end * 0.98f + nl.end * 0.02f;
                merged = true;
                break;
            }
        }

        if (!merged) {
            globalLines.push_back(nl);
        }
    }

    if (globalLines.size() > 1000) {
        globalLines.erase(globalLines.begin(), globalLines.begin() + 1);
    }
}

void DroneCore::saveNodeToDisk(int id) {
    if (id < 0 || id >= static_cast<int>(mapGraph.size())) {
        return;
    }

    std::filesystem::create_directories("cache");
    const std::string fileName = cacheFileForNode(id);
    std::ofstream f(fileName, std::ios::binary | std::ios::trunc);
    if (!f) {
        return;
    }

    const size_t count = mapGraph[id].localFeatures.size();
    f.write(reinterpret_cast<const char*>(&count), sizeof(size_t));
    if (count > 0) {
        f.write(reinterpret_cast<const char*>(mapGraph[id].localFeatures.data()), count * sizeof(LineSegment));
    }
}

void DroneCore::loadNodeFromDisk(int id) {
    if (id < 0 || id >= static_cast<int>(mapGraph.size())) {
        return;
    }

    const std::string fileName = cacheFileForNode(id);
    std::ifstream f(fileName, std::ios::binary);
    if (!f) {
        return;
    }

    size_t count = 0;
    f.read(reinterpret_cast<char*>(&count), sizeof(size_t));
    if (!f) {
        return;
    }

    mapGraph[id].localFeatures.resize(count);
    if (count > 0) {
        f.read(reinterpret_cast<char*>(mapGraph[id].localFeatures.data()), count * sizeof(LineSegment));
        if (!f) {
            mapGraph[id].localFeatures.clear();
            mapGraph[id].localFeatures.shrink_to_fit();
        }
    }
}

std::vector<LidarPoint> DroneCore::preprocessScan(const std::vector<LidarPoint>& scan) const {
    std::vector<LidarPoint> filtered;
    filtered.reserve(scan.size());

    for (const auto& p : scan) {
        if (p.dist > 5.0f && p.dist < 500.0f) {
            filtered.push_back(p);
        }
    }

    if (filtered.size() < 5) {
        return filtered;
    }

    std::vector<LidarPoint> smoothed = filtered;
    for (size_t i = 2; i + 2 < filtered.size(); ++i) {
        float window[5] = {
            filtered[i - 2].dist,
            filtered[i - 1].dist,
            filtered[i].dist,
            filtered[i + 1].dist,
            filtered[i + 2].dist,
        };
        std::sort(window, window + 5);
        smoothed[i].dist = window[2];
    }

    std::vector<LidarPoint> segmented;
    segmented.reserve(smoothed.size());
    segmented.push_back(smoothed.front());
    for (size_t i = 1; i < smoothed.size(); ++i) {
        const auto& prev = smoothed[i - 1];
        const auto& cur = smoothed[i];
        const float jump = std::fabs(cur.dist - prev.dist);
        const float ad = wrappedAngleDiff(cur.angle, prev.angle);
        if (jump > params.mergeTolerance * 2.2f && ad < 0.25f) {
            continue;
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

    while (currentRamUsage > params.ramLimitBytes) {
        int bestIdx = -1;
        float bestDist = 0.0f;

        for (int i = 0; i < static_cast<int>(mapGraph.size()); ++i) {
            const auto& node = mapGraph[i];
            if (node.isOffloaded || node.id == currentNodeIndex) {
                continue;
            }
            const float dist = node.position.dist(estimatedPos);
            if (dist > params.newNodeDist * 1.5f && dist > bestDist) {
                bestDist = dist;
                bestIdx = i;
            }
        }

        if (bestIdx == -1) {
            for (int i = 0; i < static_cast<int>(mapGraph.size()); ++i) {
                const auto& node = mapGraph[i];
                if (node.isOffloaded || node.id == currentNodeIndex) {
                    continue;
                }
                const float dist = node.position.dist(estimatedPos);
                if (dist > bestDist) {
                    bestDist = dist;
                    bestIdx = i;
                }
            }
        }

        if (bestIdx == -1) {
            break;
        }

        auto& victim = mapGraph[bestIdx];
        saveNodeToDisk(victim.id);
        victim.localFeatures.clear();
        victim.localFeatures.shrink_to_fit();
        victim.isOffloaded = true;
        currentRamUsage = estimateRamUsageBytes();
    }

    for (auto& node : mapGraph) {
        if (!node.isOffloaded) {
            continue;
        }
        if (node.position.dist(estimatedPos) >= params.newNodeDist * 1.2f) {
            continue;
        }

        loadNodeFromDisk(node.id);
        node.isOffloaded = false;

        if (estimateRamUsageBytes() > params.ramLimitBytes) {
            saveNodeToDisk(node.id);
            node.localFeatures.clear();
            node.localFeatures.shrink_to_fit();
            node.isOffloaded = true;
        }
    }

    currentRamUsage = estimateRamUsageBytes();
}

void DroneCore::update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity) {
    clampCoreParams(params);
    estimatedPos = estimatedPos + (imuVelocity * dt);

    const std::vector<LidarPoint> filteredScan = preprocessScan(scan);

    std::vector<Vec2> cloud;
    cloud.reserve(filteredScan.size());
    for (const auto& p : filteredScan) {
        cloud.push_back(estimatedPos + p.toCartesian());
    }

    currentFeatures.clear();
    if (cloud.size() > 5) {
        recursiveSplit(cloud, 0, static_cast<int>(cloud.size()) - 1, currentFeatures);
        mergeIntoGlobal(currentFeatures);
    }

    Vec2 instantVelocityCommand(0.0f, 0.0f);
    for (const auto& p : filteredScan) {
        if (p.dist < params.reflexDist && p.dist > 0.1f) {
            const float gain = (params.reflexDist - p.dist) * params.reflexForce * 0.1f;
            instantVelocityCommand = instantVelocityCommand - (p.toCartesian().normalized() * gain);
        }
    }

    const float smoothing = 0.2f;
    smoothedVelocityCommand = smoothedVelocityCommand * (1.0f - smoothing) + instantVelocityCommand * smoothing;
    velocityCommand = smoothedVelocityCommand;

    if (currentNodeIndex >= 0 && currentNodeIndex < static_cast<int>(mapGraph.size())
        && estimatedPos.dist(mapGraph[currentNodeIndex].position) > params.newNodeDist) {
        GraphNode n;
        n.id = static_cast<int>(mapGraph.size());
        n.position = estimatedPos;
        n.localFeatures = currentFeatures;
        n.connectedNodes.push_back(currentNodeIndex);
        mapGraph[currentNodeIndex].connectedNodes.push_back(n.id);
        mapGraph.push_back(n);
        currentNodeIndex = n.id;
    }

    if (currentNodeIndex >= 0 && currentNodeIndex < static_cast<int>(mapGraph.size())) {
        auto& currentNode = mapGraph[currentNodeIndex];
        currentNode.localFeatures = currentFeatures;
        currentNode.isOffloaded = false;
    }

    processMemory();

    std::stringstream ss;
    const size_t offloaded = std::count_if(mapGraph.begin(), mapGraph.end(), [](const GraphNode& node) {
        return node.isOffloaded;
    });
    ss << "RAM: " << (currentRamUsage / 1024) << "KB / " << (params.ramLimitBytes / 1024) << "KB\n";
    ss << "GLOBAL MAP: " << globalLines.size() << " vectors\n";
    ss << "NODES: " << mapGraph.size() << " (offloaded: " << offloaded << ")\n";
    ss << "POS: " << static_cast<int>(estimatedPos.x) << ", " << static_cast<int>(estimatedPos.y);
    memoryLog = ss.str();
}

std::vector<LineSegment> DroneCore::getDebugLines() const {
    return currentFeatures;
}
