#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

constexpr float PI = 3.14159265359f;

struct CoreParams {
    float splitTolerance = 6.0f;
    float mergeTolerance = 25.0f;
    float newNodeDist = 120.0f;
    float reflexDist = 45.0f;
    float reflexForce = 15.0f;
    size_t ramLimitBytes = 256 * 1024;
};

struct Vec2 {
    float x;
    float y;

    Vec2(float xValue = 0.0f, float yValue = 0.0f);
    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(float scalar) const;
    float length() const;
    Vec2 normalized() const;
    float dist(const Vec2& other) const;
};

struct LineSegment {
    Vec2 start;
    Vec2 end;
};

struct LidarPoint {
    float angle;
    float dist;

    Vec2 toCartesian() const;
};

struct GraphNode {
    int id = -1;
    Vec2 position;
    std::vector<LineSegment> localFeatures;
    std::vector<int> connectedNodes;
    bool isOffloaded = false;

    size_t getMemoryUsage() const {
        return sizeof(GraphNode)
            + (localFeatures.capacity() * sizeof(LineSegment))
            + (connectedNodes.capacity() * sizeof(int));
    }
};

class DroneCore {
public:
    DroneCore();
    ~DroneCore();

    void update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity);
    void reset();

    std::vector<LineSegment> getDebugLines() const;
    const std::vector<GraphNode>& getGraph() const { return mapGraph; }
    const std::vector<LineSegment>& getGlobalLines() const { return globalLines; }
    Vec2 getEstPos() const { return estimatedPos; }
    std::string getLogs() const { return memoryLog; }

    CoreParams params;
    Vec2 velocityCommand;

private:
    size_t estimateRamUsageBytes() const;
    void processMemory();
    void saveNodeToDisk(int id);
    void loadNodeFromDisk(int id);
    std::vector<LidarPoint> preprocessScan(const std::vector<LidarPoint>& scan) const;

    void recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result);
    void mergeIntoGlobal(const std::vector<LineSegment>& newLines);

    Vec2 estimatedPos;
    std::vector<LineSegment> currentFeatures;
    std::vector<LineSegment> globalLines;
    std::vector<GraphNode> mapGraph;
    int currentNodeIndex = -1;
    std::string memoryLog;
    size_t currentRamUsage = 0;
    Vec2 smoothedVelocityCommand;
};
