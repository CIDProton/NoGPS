#include "nogps_core.hpp"

#include <cmath>
#include <sstream>

Vec2::Vec2(float xValue, float yValue) : x(xValue), y(yValue) {}

Vec2 Vec2::operator+(const Vec2& other) const {
    return {x + other.x, y + other.y};
}

Vec2 Vec2::operator-(const Vec2& other) const {
    return {x - other.x, y - other.y};
}

Vec2 Vec2::operator*(float scalar) const {
    return {x * scalar, y * scalar};
}

float Vec2::length() const {
    return std::sqrt(x * x + y * y);
}

Vec2 Vec2::normalized() const {
    const float l = length();
    return (l > 0.0f) ? Vec2(x / l, y / l) : Vec2(0.0f, 0.0f);
}

float Vec2::dist(const Vec2& other) const {
    return (*this - other).length();
}

Vec2 LidarPoint::toCartesian() const {
    return {std::cos(angle) * dist, std::sin(angle) * dist};
}

DroneCore::DroneCore() : estimatedPos(400.0f, 300.0f) {
    GraphNode startNode;
    startNode.id = 0;
    startNode.position = estimatedPos;
    startNode.isOffloaded = false;
    mapGraph.push_back(startNode);
    currentNodeIndex = 0;
    velocityCommand = {0.0f, 0.0f};
}

void DroneCore::recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result) {
    if (start >= end) {
        return;
    }

    const Vec2 p1 = points[start];
    const Vec2 p2 = points[end];

    float maxDist = 0.0f;
    int splitIndex = -1;
    const float lineLen = p1.dist(p2);

    if (lineLen > 0.1f) {
        const float dx = p2.x - p1.x;
        const float dy = p2.y - p1.y;
        for (int i = start + 1; i < end; i++) {
            const float num = std::abs(dy * points[i].x - dx * points[i].y + p2.x * p1.y - p2.y * p1.x);
            const float d = num / lineLen;
            if (d > maxDist) {
                maxDist = d;
                splitIndex = i;
            }
        }
    }

    if (maxDist > splitMergeTolerance && splitIndex > start && splitIndex < end) {
        recursiveSplit(points, start, splitIndex, result);
        recursiveSplit(points, splitIndex, end, result);
    } else {
        result.push_back({p1, p2});
    }
}

void DroneCore::update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity) {
    estimatedPos = estimatedPos + (imuVelocity * dt);

    std::vector<Vec2> cloud;
    cloud.reserve(scan.size());
    for (const auto& p : scan) {
        if (p.dist == 0.0f) {
            continue;
        }
        cloud.push_back(estimatedPos + p.toCartesian());
    }

    currentFeatures.clear();
    if (cloud.size() > 2) {
        recursiveSplit(cloud, 0, static_cast<int>(cloud.size()) - 1, currentFeatures);
    }

    velocityCommand = {0.0f, 0.0f};
    Vec2 reflexForce = {0.0f, 0.0f};
    for (const auto& p : scan) {
        if (p.dist > 0.0f && p.dist < reflexDist) {
            const Vec2 dir = p.toCartesian().normalized();
            reflexForce = reflexForce - (dir * (reflexDist - p.dist) * 8.0f);
        }
    }
    velocityCommand = reflexForce;

    GraphNode& currNode = mapGraph[currentNodeIndex];
    if (estimatedPos.dist(currNode.position) > newNodeDist) {
        GraphNode newNode;
        newNode.id = static_cast<int>(mapGraph.size());
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
        if (std::abs(node.id - currentNodeIndex) > 14 && !node.isOffloaded) {
            node.localFeatures.clear();
            node.isOffloaded = true;
        }
        if (!node.isOffloaded) {
            activeCount++;
        }
    }

    std::stringstream log;
    log << "RAM OPTIMIZER:\n";
    log << "Nodes Total: " << mapGraph.size() << "\n";
    log << "Active (RAM): " << activeCount << "\n";
    log << "Offloaded (Disk): " << mapGraph.size() - activeCount << "\n";
    log << "Vectors: " << currentFeatures.size();
    memoryLog = log.str();
}

std::vector<LineSegment> DroneCore::getDebugLines() const {
    return currentFeatures;
}

std::vector<GraphNode> DroneCore::getGraph() const {
    return mapGraph;
}

Vec2 DroneCore::getEstPos() const {
    return estimatedPos;
}

std::string DroneCore::getLogs() const {
    return memoryLog;
}
