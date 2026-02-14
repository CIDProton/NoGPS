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
    std::vector<LineSegment> localFeatures; // линии, увиденные в этом узле
    std::vector<int> connectedNodes;
    bool isOffloaded;
};

class DroneCore {
public:
    DroneCore();

    void update(float dt, const std::vector<LidarPoint>& scan, Vec2 imuVelocity);

    std::vector<LineSegment> getDebugLines() const;      // текущие линии (из последнего скана)
    std::vector<GraphNode> getGraph() const;             // граф узлов
    Vec2 getEstPos() const;
    std::string getLogs() const;

    // Новые методы для доступа к глобальной карте
    const std::vector<LineSegment>& getGlobalLines() const { return globalLines; }
    void reset();  // сброс состояния (новая карта)

    Vec2 velocityCommand; // рефлекторная скорость (заполняется ядром)

private:
    void recursiveSplit(const std::vector<Vec2>& points, int start, int end, std::vector<LineSegment>& result);
    void mergeIntoGlobal(const std::vector<LineSegment>& newLines);

    float splitMergeTolerance = 15.0f;   // порог для split-merge лидара
    float newNodeDist = 600.0f;           // расстояние до нового узла
    float reflexDist = 5.0f;             // дистанция рефлекса
    float mergeTolerance = 5.0f;          // порог слияния глобальных линий

    Vec2 estimatedPos;
    std::vector<LineSegment> currentFeatures;    // линии из последнего скана
    std::vector<LineSegment> globalLines;         // накопленная векторная карта
    std::vector<GraphNode> mapGraph;              // граф узлов
    int currentNodeIndex = -1;
    std::string memoryLog;
};