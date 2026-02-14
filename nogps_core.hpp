#pragma once

#include <string>
#include <vector>

constexpr float PI = 3.14159265359f;

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
    int id;
    Vec2 position;
    std::vector<LineSegment> localFeatures;
    std::vector<int> connectedNodes;
    bool isOffloaded;
};

class DroneCore {
public:
    DroneCore();

    void update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity);

    std::vector<LineSegment> getDebugLines() const;
    std::vector<GraphNode> getGraph() const;
    Vec2 getEstPos() const;
    std::string getLogs() const;

    Vec2 velocityCommand;

private:
    void recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result);

    float splitMergeTolerance = 15.0f;
    float newNodeDist = 120.0f;
    float reflexDist = 50.0f;

    Vec2 estimatedPos;
    std::vector<LineSegment> currentFeatures;
    std::vector<GraphNode> mapGraph;
    int currentNodeIndex = -1;
    std::string memoryLog;
};
