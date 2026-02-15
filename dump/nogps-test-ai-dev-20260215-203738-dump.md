# PROJECT DUMP
# Directory: C:\cidproect\NoGPS\main
# Generated: 2026-02-15 20:37:38
================================================================================


================================================================================
FILE: ARCHITECTURE.md
================================================================================

```md
# NoGPS Architecture

## Цель разделения
Проект разделён на два независимых слоя:
- **Core (ядро)**: алгоритмы позиционирования, карты признаков и ограничение памяти.
- **Simulator (симулятор)**: визуализация, управление, генерация лидара и карта стен.

Это нужно, чтобы позже портировать только `core` на ARM/Linux, а симулятор оставить как dev-инструмент под Windows.

## Текущая структура
- `nogps_core.hpp`, `nogps_core.cpp` — чистая логика ядра без WinAPI.
- `simulator_gdi.hpp`, `simulator_gdi.cpp` — WinAPI/GDI визуализация и цикл симуляции.
- `main.cpp` — тонкая точка входа, запускающая симулятор.

## Контракт между слоями
Симулятор передаёт в ядро:
- `dt`
- `scan` (`std::vector<LidarPoint>`)
- `imuVelocity`

Ядро возвращает:
- `velocityCommand` (рефлекторный отталкивающий вектор)
- текущие debug-линии (`getDebugLines`)
- граф памяти (`getGraph`)
- оценку положения (`getEstPos`)
- текстовые логи (`getLogs`)

## Ключевой фикс отображения
Ранее правая панель просто отрисовывала мировые координаты с X-сдвигом `+640`. Из-за этого граф быстро уезжал за границы правой панели.

Теперь введено преобразование `toCoreView(...)`:
- центр правой панели всегда привязан к `core.getEstPos()`;
- всё отображение ядра рисуется в локальной системе координат относительно дрона;
- добавлен масштаб `kCoreViewScale` для лучшей читаемости.

## Ограничения
- Симулятор использует WinAPI/GDI и компилируется под Windows.
- Ядро пока работает в 2D (ось `z=0`).
- Нет физики столкновений по геометрии корпуса — только рефлекс от лидара.
- Нет постоянного хранилища offload-данных: только логический флаг и очистка RAM-полей.
```

================================================================================
FILE: BUGS.md
================================================================================

```md
# Known bugs / remaining issues

- Нет детектора выхода дрона за границы мира (можно улететь за стенки по интеграции скорости).
- `CastRay` имеет жёсткий лимит дальности 100, что ограничивает дальнюю геометрию.
- Нет проверки столкновения корпуса дрона со стеной (только реактивный вектор от лидара).
- Рефлекторная модель может создавать дрожание в узких проходах.
- Шум лидара не моделирует реальные артефакты (dropout, систематический сдвиг, мультипас).
- Отсутствуют unit/integration тесты.
- Нет явного shutdown-хука для сохранения состояния графа перед выходом.
```

================================================================================
FILE: FUTURE_IDEAS.md
================================================================================

```md
# Future ideas

- Перейти от fixed timestep к accumulator-схеме с точным `dt`.
- Добавить seed для шума и повторяемые сценарии тестов.
- Сделать сериализацию графа/векторов в файл для офлайн-аналитики.
- Реализовать настоящий offload в файл/БД вместо очистки вектора в RAM.
- Добавить сценарии карт (лабиринт, большие холлы, узкие коридоры).
- Поддержать 3D-ready API в ядре (Vec3 + интерфейс сенсоров), сохранив 2D-симулятор.
- Вынести параметры ядра (пороги, дистанции) в конфиг-файл.
- Добавить автотесты core-математики и split/merge.
```

================================================================================
FILE: NoGPS_core.txt
================================================================================

```txt
NoGPS_core
Ну что йоууу погнали

Значит нужно сделать графы...

Задача сделать две программы, первая это ядро дрона на c++ а второе это симуляция с отображением для тестирования и развития ядра.

В чём прикол, память дрона это максимум 256мб оперативной
И всего лишь 4гб основной.
Идея в том что бы оптимизировать хранилище да и все данные в принципе.

Идея в том чтобы работать не с облаком точек лидара а с графами, векторами и сетками. Таким образом планируется достигать точности определения местоположения и устранения дрифта и неточностей.

То есть лидар будет сканировать все вокруг, создавая облако точек, пока не придумал но нужно как-то придумать сделать алгоритм что будет множество точек одной плоскости объединять в одну линию но поскольку в реальной жизни идеально гладких поверхностей не существует поэтому надо как-то придумать коэффициент допустимых отклонений реальных объектов данных с лидара от плоскости в памяти, после сделать чтобы поскольку лидаи не точен, сделать как-то типа повышение точности местоположения объекта за счёт большого количества точек на них которые в ходе полета будут делаться ещё точнее с разных углов сторон. 

Далее по мимо определению стен и т.д. в общем препятствий надо сделать графы доступности (пути)

Граф твоего пути графы делаются в несильно большом количестве и часто оптимизируются в ходе полёта, чтобы определять размер территории по которой пролетел в общем пустоту что точно пустота. При этом эти графы должны объединятся в группы пространств создавая удобную сетку что будет показывать границы зон по мимо этого нельзя слишком сильно объединять (ну я имею ввиду что вот в прям в край оптимизировать) это нужно чтобы создавать большую вариацию путей, потребуется в больших витвистых коридорах пещер или зданий, чтобы обеспечивать что даже если будет повреждены разные пути всегда будет путь возврата на точку старта, при этом как следствие таких технологий сразу будут заметны изменения территорий при полёте.


Для оптимизации было предложено сделать разные уровни обработки данных лидар, самый первый это условный рефлекс всегда держать расстояние от объектов вокруг чтобы не врезаться ничего. 
Так же по сокльку не все обекты вокруг уникальные а в некоторых коридорах просто впринципе малова-то опопрных уникальных точек... Такая бы система бы сильно сбивалась а потом для дополнительной корректировки если же долго небудет неровностей или достаточной кривизны окружающего пространства чтобы сделать опорной точкой.





Периуд - разработки...

На данный момент по скольку данный проект являеться лишь теоритической частью идеи и требуеться проверка целесообразности алгоритмов и концепцуальной новизны проект будет состоять из двух программ первая это симулятор второе это ядро. ядро по сути это и есть все те мозги что в будущем будут на дроне и т.д. симулятор это специальное по на момент разработки чтобы чётко видит работу ядра и симулировать разные обстоятельства, разные окружения разные уровни помех ошибок ключевые моменты алгоритма и т.д.
На данный момент хоть и в будущем будет 3д разработка введёться на 2д (ось z=0) (просьба учитывать чтобы потом если концепт окажеться рабочим быстро портировать и доработать на 3д с нуля не переписывая всё ядро) 


Симулятор:
Симулятор это програмнный комплекс на c++ (дабы обеспечить высокую скорость). Оно имеет ввид виде окна состоящего из трёх подокон, первое основное занимающее почти всё пространства окна это отображение симулированной реальности для ядра, это все препядствия окружающий мир лучи лидара вокруг, реально местоположение то местоположение что считает ядро. то что уже увидело обработало те точки что ядро увидело вообщем всё что думает ядро. 2 часть окна это уже карта памяти дрона где центр это местоположение дрона и уже вокруг него двигаеться память дрона графы препядствия точки и всё остальное... 3 окно уже отображает заполнение использование памяти те состояния процессы ядра тоесть евсли он думает об области, определяет её геометрию то он это туда пишет пишет его местоположение (рассстояние до ближайшего графа) в общем по сути консоль логов чтобы можно было потом эти данные выгружать и анализировать чтобы коректировать алгоритм.

Данное ПО разрабатываеться именно на платформу разработки виндовс.
(ядро же хоть сейчас и тут в будущем будет использоваться на бортовом компьютере дрона а это арм архитектура и скореевсего линукс)


Ядро:
Ядро это весь программный комплекс дрона его базы данных его алгоритмы оптимизации распознавания и т.д. это конфиг что он читает чтобы правильно всё обрабатывать распознавать (в конфиге указываеться ограничения на использование памяи и т.д. режимы отладки или нет) и сами алгоритмы
Язык программмирования ядра это c++ (ради максимальной скорости) и спец библиотек.
Выше планируемая логика ядра уже была описана. Можно лишь добавить что память что используеться должна распределяться динамически а некоторые данные что бесполезны например обалако точек что было сохраннено комнаты тридцать комнат назад ну или около 70+ графов назад просто выгружаеться из оперативной в обычную ради экномии электроэнергии и оптимизации (т.к. планируеться значительную часть рессурсов бортового пк отдать на расспознавание обектов по камерам, для спасателей и т.д.)


Потому эта система агентного ориентирования (поизиционирования) должна быть максимально опотимизирована.
```

================================================================================
FILE: SIMULATOR_LIMITATIONS.md
================================================================================

```md
# Simulator нюансы, ограничения и фишки

## Что важно для разработчиков
- Частота кадра зафиксирована через `Sleep(16)` (~60 FPS), но это не строгий real-time.
- Лидар: 100 лучей по кругу, дальность ограничена `CastRay` (100 единиц).
- Шум лидара — псевдослучайный, равномерный, до 3 единиц.
- Управление: `WASD` задаёт желаемую скорость, поверх накладывается `core.velocityCommand`.

## Почему так сделано
- Минимальная зависимость: никаких внешних графических библиотек, только WinAPI.
- Ядро изолировано от рендера, чтобы проще портировать/тестировать.
- Текущий рендер удобен для отладки логики, а не для финальной UX-визуализации.

## Что можно безопасно менять
- Геометрию карты в `initMap()`.
- Параметры лучей (`kNumRays`, шум, дальность).
- Масштаб и центр в `toCoreView(...)`.
- Цвета/толщины линий без влияния на логику ядра.

## Что менять осторожно
- Порядок update в симуляционном цикле: сначала пересчёт физики дрона, затем `core.update(...)`.
- Структуры `LidarPoint`, `Vec2`, `GraphNode`: это контракт между core и simulator.
```

================================================================================
FILE: main.cpp
================================================================================

```cpp
#include "simulator_gdi.hpp"

int main() {
    return runSimulator();
}
```

================================================================================
FILE: nogps_core.cpp
================================================================================

```cpp
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
```

================================================================================
FILE: nogps_core.hpp
================================================================================

```hpp
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
```

================================================================================
FILE: simulator_gdi.cpp
================================================================================

```cpp
#include "simulator_gdi.hpp"

#include <windows.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
#include <vector>

#include "nogps_core.hpp"

namespace {
constexpr int kLeftPanelWidth = 640;
constexpr float kFrameDt = 0.016f;
float kCoreViewScale = 0.6f;

const int MAP_W = 640;
const int MAP_H = 800;
std::vector<bool> worldMap(MAP_W * MAP_H, false); // true = wall
uint32_t mapPixels[MAP_H][MAP_W];

Vec2 dronePos(100.0f, 100.0f);
Vec2 droneVel(0.0f, 0.0f);
Vec2 mapSpawn(100.0f, 100.0f);
DroneCore core;

DWORD lastFpsUpdate = 0;
int frameCount = 0;
float currentFps = 0.0f;

inline bool inBounds(int x, int y) {
    return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H;
}

bool isWallCell(int x, int y) {
    if (!inBounds(x, y)) {
        return true;
    }
    return worldMap[y * MAP_W + x];
}

void setWallCell(int x, int y, bool isWall) {
    if (inBounds(x, y)) {
        worldMap[y * MAP_W + x] = isWall;
    }
}

void carveDisk(int cx, int cy, int radius) {
    for (int y = cy - radius; y <= cy + radius; ++y) {
        for (int x = cx - radius; x <= cx + radius; ++x) {
            if (!inBounds(x, y)) {
                continue;
            }
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius) {
                setWallCell(x, y, false);
            }
        }
    }
}

void carveEllipseRoom(int cx, int cy, int rx, int ry) {
    for (int y = cy - ry; y <= cy + ry; ++y) {
        for (int x = cx - rx; x <= cx + rx; ++x) {
            if (!inBounds(x, y)) {
                continue;
            }
            const float nx = static_cast<float>(x - cx) / static_cast<float>(rx);
            const float ny = static_cast<float>(y - cy) / static_cast<float>(ry);
            if (nx * nx + ny * ny <= 1.0f) {
                setWallCell(x, y, false);
            }
        }
    }
}

void carveTunnel(Vec2 a, Vec2 b, int radius) {
    Vec2 d = b - a;
    const float len = d.length();
    if (len < 1.0f) {
        carveDisk(static_cast<int>(a.x), static_cast<int>(a.y), radius);
        return;
    }

    const int steps = static_cast<int>(len / 2.0f);
    Vec2 dir = d * (1.0f / len);
    Vec2 p = a;
    for (int i = 0; i <= steps; ++i) {
        carveDisk(static_cast<int>(p.x), static_cast<int>(p.y), radius);

        const float sideJitter = static_cast<float>((rand() % 100) - 50) * 0.02f;
        Vec2 jitter(-dir.y * sideJitter, dir.x * sideJitter);
        p = p + dir * 2.0f + jitter;
    }
}

bool isAreaFree(Vec2 p, int radius) {
    for (int y = static_cast<int>(p.y) - radius; y <= static_cast<int>(p.y) + radius; ++y) {
        for (int x = static_cast<int>(p.x) - radius; x <= static_cast<int>(p.x) + radius; ++x) {
            if (isWallCell(x, y)) {
                return false;
            }
        }
    }
    return true;
}

Vec2 pickSafeSpawn(const std::vector<Vec2>& roomCenters) {
    for (const Vec2& c : roomCenters) {
        if (isAreaFree(c, 10)) {
            return c;
        }
    }

    for (int i = 0; i < 5000; ++i) {
        Vec2 p(static_cast<float>(rand() % (MAP_W - 40) + 20), static_cast<float>(rand() % (MAP_H - 40) + 20));
        if (isAreaFree(p, 10)) {
            return p;
        }
    }

    return Vec2(100.0f, 100.0f);
}

void updateMapBuffer() {
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            mapPixels[y][x] = worldMap[y * MAP_W + x] ? 0x00FFFFFF : 0x00000000;
        }
    }
}

void generateCaveMap() {
    // 1) Start from solid walls
    std::fill(worldMap.begin(), worldMap.end(), true);

    // 2) Create large rooms with different sizes
    std::vector<Vec2> roomCenters;
    const int roomCount = 11 + rand() % 6;
    roomCenters.reserve(roomCount);

    for (int i = 0; i < roomCount; ++i) {
        const int cx = 50 + rand() % (MAP_W - 100);
        const int cy = 50 + rand() % (MAP_H - 100);
        const int rx = 22 + rand() % 70;
        const int ry = 20 + rand() % 60;
        carveEllipseRoom(cx, cy, rx, ry);
        roomCenters.push_back(Vec2(static_cast<float>(cx), static_cast<float>(cy)));
    }

    // 3) Connect rooms with tunnels (chain + random cross-links)
    for (size_t i = 1; i < roomCenters.size(); ++i) {
        const int r = 4 + rand() % 4;
        carveTunnel(roomCenters[i - 1], roomCenters[i], r);
    }
    for (int i = 0; i < roomCount / 2; ++i) {
        int a = rand() % roomCount;
        int b = rand() % roomCount;
        if (a == b) {
            b = (b + 1) % roomCount;
        }
        carveTunnel(roomCenters[a], roomCenters[b], 3 + rand() % 4);
    }

    // 4) Add smaller side caverns
    const int sideCaves = 18 + rand() % 14;
    for (int i = 0; i < sideCaves; ++i) {
        const int cx = 30 + rand() % (MAP_W - 60);
        const int cy = 30 + rand() % (MAP_H - 60);
        carveEllipseRoom(cx, cy, 8 + rand() % 22, 8 + rand() % 20);
    }

    // 5) Smooth shape with cave cellular automata
    for (int iter = 0; iter < 3; ++iter) {
        std::vector<bool> next = worldMap;
        for (int y = 1; y < MAP_H - 1; ++y) {
            for (int x = 1; x < MAP_W - 1; ++x) {
                int wallNeighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        if (isWallCell(x + dx, y + dy)) {
                            ++wallNeighbors;
                        }
                    }
                }
                if (worldMap[y * MAP_W + x]) {
                    next[y * MAP_W + x] = (wallNeighbors >= 4);
                } else {
                    next[y * MAP_W + x] = (wallNeighbors >= 6);
                }
            }
        }
        worldMap.swap(next);
    }

    // 6) Keep map borders solid
    for (int x = 0; x < MAP_W; ++x) {
        setWallCell(x, 0, true);
        setWallCell(x, MAP_H - 1, true);
    }
    for (int y = 0; y < MAP_H; ++y) {
        setWallCell(0, y, true);
        setWallCell(MAP_W - 1, y, true);
    }

    // 7) Pick safe spawn and ensure local area free
    mapSpawn = pickSafeSpawn(roomCenters);
    carveDisk(static_cast<int>(mapSpawn.x), static_cast<int>(mapSpawn.y), 14);
    dronePos = mapSpawn;
    droneVel = Vec2(0.0f, 0.0f);

    updateMapBuffer();
}

void DrawLine(HDC hdc, int x1, int y1, int x2, int y2, COLORREF color, int thickness = 1) {
    HPEN hPen = CreatePen(PS_SOLID, thickness, color);
    HGDIOBJ hOld = SelectObject(hdc, hPen);
    MoveToEx(hdc, x1, y1, nullptr);
    LineTo(hdc, x2, y2);
    SelectObject(hdc, hOld);
    DeleteObject(hPen);
}

void DrawCircle(HDC hdc, int x, int y, int r, COLORREF color, bool filled) {
    HBRUSH hBrush = filled ? CreateSolidBrush(color) : (HBRUSH)GetStockObject(NULL_BRUSH);
    HPEN hPen = CreatePen(PS_SOLID, 1, color);
    HGDIOBJ oldB = SelectObject(hdc, hBrush);
    HGDIOBJ oldP = SelectObject(hdc, hPen);
    Ellipse(hdc, x - r, y - r, x + r, y + r);
    SelectObject(hdc, oldB);
    SelectObject(hdc, oldP);
    if (filled) {
        DeleteObject(hBrush);
    }
    DeleteObject(hPen);
}

float CastRay(Vec2 start, Vec2 dir) {
    for (float d = 0; d < 500; d += 2.0f) {
        int ix = (int)(start.x + dir.x * d);
        int iy = (int)(start.y + dir.y * d);
        if (ix < 0 || ix >= MAP_W || iy < 0 || iy >= MAP_H || worldMap[iy * MAP_W + ix]) {
            return d;
        }
    }
    return 500.0f;
}

POINT toCoreView(Vec2 worldPoint, Vec2 center, int winW, int winH) {
    float panelCenterX = kLeftPanelWidth + (winW - kLeftPanelWidth) / 2.0f;
    float panelCenterY = winH / 2.0f;
    POINT pt;
    pt.x = (LONG)(panelCenterX + (worldPoint.x - center.x) * kCoreViewScale);
    pt.y = (LONG)(panelCenterY + (worldPoint.y - center.y) * kCoreViewScale);
    return pt;
}

void drawUi(HDC hdc, int winW, int winH) {
    SetTextColor(hdc, RGB(0, 255, 0));
    SetBkMode(hdc, TRANSPARENT);

    const int x = kLeftPanelWidth + 10;
    char buf[128];
    TextOutA(hdc, x, 20, "--- CORE REALTIME TUNING ---", 28);
    sprintf(buf, "[1/2] Merge Tol: %.1f", core.params.mergeTolerance);
    TextOutA(hdc, x, 48, buf, (int)strlen(buf));
    sprintf(buf, "[3/4] Split Tol: %.1f", core.params.splitTolerance);
    TextOutA(hdc, x, 68, buf, (int)strlen(buf));
    sprintf(buf, "[5/6] RAM Limit: %zu KB", core.params.ramLimitBytes / 1024);
    TextOutA(hdc, x, 88, buf, (int)strlen(buf));
    sprintf(buf, "[7/8] Reflex Force: %.1f", core.params.reflexForce);
    TextOutA(hdc, x, 108, buf, (int)strlen(buf));
    sprintf(buf, "[9/0] Node Dist: %.1f", core.params.newNodeDist);
    TextOutA(hdc, x, 128, buf, (int)strlen(buf));
    sprintf(buf, "[-/=] Reflex Dist: %.1f", core.params.reflexDist);
    TextOutA(hdc, x, 148, buf, (int)strlen(buf));
    TextOutA(hdc, x, 168, "R: Reset world/core", 19);

    std::string logs = core.getLogs();
    RECT textR = {x, winH - 120, winW - 10, winH - 10};
    DrawTextA(hdc, logs.c_str(), -1, &textR, DT_LEFT);
}

void drawFrame(HDC hdcMem, HWND hwnd, const std::vector<LidarPoint>& scan) {
    RECT clRect;
    GetClientRect(hwnd, &clRect);
    const int winW = clRect.right;
    const int winH = clRect.bottom;

    FillRect(hdcMem, &clRect, (HBRUSH)GetStockObject(BLACK_BRUSH));
    DrawLine(hdcMem, kLeftPanelWidth, 0, kLeftPanelWidth, winH, RGB(100, 100, 100));

    BITMAPINFO bmi = {0};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = MAP_W;
    bmi.bmiHeader.biHeight = -MAP_H;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
    StretchDIBits(hdcMem, 0, 0, MAP_W, MAP_H, 0, 0, MAP_W, MAP_H, mapPixels, &bmi, DIB_RGB_COLORS, SRCCOPY);

    for (const auto& p : scan) {
        Vec2 end = dronePos + p.toCartesian();
        DrawLine(hdcMem, (int)dronePos.x, (int)dronePos.y, (int)end.x, (int)end.y, RGB(50, 50, 50));
    }
    DrawCircle(hdcMem, (int)dronePos.x, (int)dronePos.y, 7, RGB(0, 255, 0), true);

    Vec2 estPos = core.getEstPos();
    for (const auto& l : core.getGlobalLines()) {
        POINT p1 = toCoreView(l.start, estPos, winW, winH);
        POINT p2 = toCoreView(l.end, estPos, winW, winH);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, RGB(150, 150, 0));
    }

    for (const auto& l : core.getDebugLines()) {
        POINT p1 = toCoreView(l.start, estPos, winW, winH);
        POINT p2 = toCoreView(l.end, estPos, winW, winH);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, RGB(0, 255, 255), 2);
    }

    for (const auto& n : core.getGraph()) {
        POINT p = toCoreView(n.position, estPos, winW, winH);
        DrawCircle(hdcMem, p.x, p.y, 3, n.isOffloaded ? RGB(50, 50, 50) : RGB(255, 0, 0), true);
    }

    POINT c = toCoreView(estPos, estPos, winW, winH);
    DrawCircle(hdcMem, c.x, c.y, 5, RGB(0, 255, 0), false);

    drawUi(hdcMem, winW, winH);

    char title[256];
    sprintf(title, "NoGPS Sim | FPS: %.1f | Global Lines: %zu", currentFps, core.getGlobalLines().size());
    SetWindowTextA(hwnd, title);

    HDC hdcWin = GetDC(hwnd);
    BitBlt(hdcWin, 0, 0, winW, winH, hdcMem, 0, 0, SRCCOPY);
    ReleaseDC(hwnd, hdcWin);
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    if (uMsg == WM_DESTROY) {
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}
} // namespace

int runSimulator() {
    CreateDirectoryA("cache", NULL);
    generateCaveMap();

    WNDCLASSA wc = {0};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = "NoGPS_Sim";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(
        0,
        wc.lpszClassName,
        "NoGPS Core Simulator",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        100,
        100,
        1280,
        800,
        NULL,
        NULL,
        wc.hInstance,
        NULL);

    HDC hdcWin = GetDC(hwnd);
    HDC hdcMem = CreateCompatibleDC(hdcWin);
    HBITMAP hbmMem = CreateCompatibleBitmap(hdcWin, 1920, 1080);
    SelectObject(hdcMem, hbmMem);

    while (true) {
        MSG msg;
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                return 0;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        if (GetAsyncKeyState('1') & 0x8000) core.params.mergeTolerance -= 0.5f;
        if (GetAsyncKeyState('2') & 0x8000) core.params.mergeTolerance += 0.5f;
        if (GetAsyncKeyState('3') & 0x8000) core.params.splitTolerance -= 0.1f;
        if (GetAsyncKeyState('4') & 0x8000) core.params.splitTolerance += 0.1f;
        if (GetAsyncKeyState('5') & 0x8000 && core.params.ramLimitBytes > 16 * 1024) core.params.ramLimitBytes -= 1024;
        if (GetAsyncKeyState('6') & 0x8000) core.params.ramLimitBytes += 1024;
        if (GetAsyncKeyState('7') & 0x8000 && core.params.reflexForce > 1.0f) core.params.reflexForce -= 1.0f;
        if (GetAsyncKeyState('8') & 0x8000) core.params.reflexForce += 1.0f;
        if (GetAsyncKeyState('9') & 0x8000 && core.params.newNodeDist > 10.0f) core.params.newNodeDist -= 1.0f;
        if (GetAsyncKeyState('0') & 0x8000) core.params.newNodeDist += 1.0f;
        if (GetAsyncKeyState(VK_OEM_MINUS) & 0x8000 && core.params.reflexDist > 5.0f) core.params.reflexDist -= 1.0f;
        if (GetAsyncKeyState(VK_OEM_PLUS) & 0x8000) core.params.reflexDist += 1.0f;

        if (core.params.mergeTolerance < 1.0f) core.params.mergeTolerance = 1.0f;
        if (core.params.splitTolerance < 0.5f) core.params.splitTolerance = 0.5f;
        if (core.params.reflexForce < 0.0f) core.params.reflexForce = 0.0f;
        if (core.params.newNodeDist < 10.0f) core.params.newNodeDist = 10.0f;
        if (core.params.reflexDist < 5.0f) core.params.reflexDist = 5.0f;
        if (core.params.ramLimitBytes < 16 * 1024) core.params.ramLimitBytes = 16 * 1024;

        Vec2 input(0.0f, 0.0f);
        if (GetAsyncKeyState('W') & 0x8000) input.y -= 120;
        if (GetAsyncKeyState('S') & 0x8000) input.y += 120;
        if (GetAsyncKeyState('A') & 0x8000) input.x -= 120;
        if (GetAsyncKeyState('D') & 0x8000) input.x += 120;
        if (GetAsyncKeyState('R') & 0x8000) {
            generateCaveMap();
            core.reset();
            dronePos = mapSpawn;
        }

        std::vector<LidarPoint> scan;
        scan.reserve(120);
        for (int i = 0; i < 120; ++i) {
            const float a = (i * 2.0f * PI) / 120.0f;
            const Vec2 rayDir(std::cos(a), std::sin(a));
            const float noisyDist = CastRay(dronePos, rayDir) + static_cast<float>(rand() % 100) / 50.0f;
            scan.push_back({a, noisyDist});
        }

        droneVel = input + core.velocityCommand;

        // collision-safe move with axis separation and radius check
        const float bodyRadius = 6.0f;
        Vec2 target = dronePos + droneVel * kFrameDt;

        Vec2 tryX(target.x, dronePos.y);
        if (isAreaFree(tryX, static_cast<int>(bodyRadius))) {
            dronePos.x = tryX.x;
        }

        Vec2 tryY(dronePos.x, target.y);
        if (isAreaFree(tryY, static_cast<int>(bodyRadius))) {
            dronePos.y = tryY.y;
        }

        core.update(kFrameDt, scan, droneVel);

        drawFrame(hdcMem, hwnd, scan);

        ++frameCount;
        if (GetTickCount() - lastFpsUpdate > 1000) {
            currentFps = frameCount * 1000.0f / (GetTickCount() - lastFpsUpdate);
            frameCount = 0;
            lastFpsUpdate = GetTickCount();
        }

        Sleep(16);
    }

    return 0;
}
```

================================================================================
FILE: simulator_gdi.hpp
================================================================================

```hpp
#pragma once

int runSimulator();
```

================================================================================
STATS
Files: 10
Total text size: 38664 bytes
================================================================================
