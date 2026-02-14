# PROJECT DUMP
# Directory: C:\cidproect\NoGPS\main
# Generated: 2026-02-14 17:36:09
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
        if (std::abs(node.id - currentNodeIndex) > 4 && !node.isOffloaded) {
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
```

================================================================================
FILE: nogps_core.hpp
================================================================================

```hpp
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
```

================================================================================
FILE: simulator_gdi.cpp
================================================================================

```cpp
#include "simulator_gdi.hpp"

#include <windows.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>   // для std::min, std::max
#include <cstdint>     // для uint32_t

#include "nogps_core.hpp"

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 800;
constexpr int kLeftPanelWidth = 640;
constexpr float kFrameDt = 0.016f;
constexpr float kCoreViewScale = 1.4f;

// Размеры карты (левая панель)
const int MAP_W = 640;
const int MAP_H = 800;
std::vector<bool> worldMap(MAP_W * MAP_H, false);

// Буфер пикселей для левой панели (формат 32-bit ABGR, но под Windows BI_RGB обычно BGR)
uint32_t mapPixels[MAP_H][MAP_W]; // [y][x] – каждый uint32_t хранит цвет 0x00BBGGRR (или 0x00RRGGBB в зависимости от порядка)
// Для простоты используем RGB: белый 0x00FFFFFF, чёрный 0x00000000.

Vec2 dronePos(400.0f, 300.0f);
Vec2 droneVel(0.0f, 0.0f);
DroneCore core;

// ---------------------------------------------------------------------
// Генерация пещер клеточным автоматом
// ---------------------------------------------------------------------
void generateCaveMap() {
    // Инициализация случайным шумом
    for (int i = 0; i < MAP_W * MAP_H; i++) {
        worldMap[i] = (rand() % 100 < 45);   // ~45% стен
    }

    // Несколько итераций клеточного автомата для сглаживания
    for (int iter = 0; iter < 5; iter++) {
        std::vector<bool> newMap = worldMap;
        for (int y = 1; y < MAP_H - 1; y++) {
            for (int x = 1; x < MAP_W - 1; x++) {
                int neighbors = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (worldMap[(y + dy) * MAP_W + (x + dx)]) neighbors++;
                    }
                }
                // Правило: если больше 4 соседей — стена, иначе — пусто
                newMap[y * MAP_W + x] = (neighbors > 4);
            }
        }
        worldMap.swap(newMap);
    }

    // Убедимся, что границы всегда стены
    for (int x = 0; x < MAP_W; x++) {
        worldMap[0 * MAP_W + x] = true;
        worldMap[(MAP_H - 1) * MAP_W + x] = true;
    }
    for (int y = 0; y < MAP_H; y++) {
        worldMap[y * MAP_W + 0] = true;
        worldMap[y * MAP_W + (MAP_W - 1)] = true;
    }
}

// ---------------------------------------------------------------------
// Обновление пиксельного буфера левой панели по текущей карте
// ---------------------------------------------------------------------
void updateMapBuffer() {
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            // worldMap хранится линейно, обращаемся по индексу y*MAP_W + x
            // Белый цвет для стен, чёрный для пустоты.
            // Формат: 0x00BBGGRR (little-endian, в памяти BGR). Установим все компоненты в 0xFF для белого.
            mapPixels[y][x] = worldMap[y * MAP_W + x] ? 0x00FFFFFF : 0x00000000;
        }
    }
}

// ---------------------------------------------------------------------
// Вспомогательные функции рисования
// ---------------------------------------------------------------------
void DrawLine(HDC hdc, int x1, int y1, int x2, int y2, COLORREF color, int thickness = 1) {
    HPEN hPen = CreatePen(PS_SOLID, thickness, color);
    HGDIOBJ hOld = SelectObject(hdc, hPen);
    MoveToEx(hdc, x1, y1, nullptr);
    LineTo(hdc, x2, y2);
    SelectObject(hdc, hOld);
    DeleteObject(hPen);
}

void DrawCircle(HDC hdc, int x, int y, int r, COLORREF color, bool filled) {
    HBRUSH hBrush = filled ? CreateSolidBrush(color) : static_cast<HBRUSH>(GetStockObject(NULL_BRUSH));
    HPEN hPen = CreatePen(PS_SOLID, 1, color);
    HGDIOBJ oldBrush = SelectObject(hdc, hBrush);
    HGDIOBJ oldPen = SelectObject(hdc, hPen);
    Ellipse(hdc, x - r, y - r, x + r, y + r);
    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    if (filled) {
        DeleteObject(hBrush);
    }
    DeleteObject(hPen);
}

// ---------------------------------------------------------------------
// DDA Ray marching по растровой карте
// ---------------------------------------------------------------------
float CastRay(Vec2 start, Vec2 dir) {
    // Начальная клетка
    int x = static_cast<int>(start.x);
    int y = static_cast<int>(start.y);

    // Направления шага
    int stepX = (dir.x > 0) ? 1 : -1;
    int stepY = (dir.y > 0) ? 1 : -1;

    // Расстояния до ближайших границ клетки по x и y
    float tMaxX, tMaxY;
    if (dir.x != 0.0f) {
        float nextX = (stepX > 0) ? (x + 1 - start.x) : (start.x - x);
        tMaxX = nextX / fabs(dir.x);
    } else {
        tMaxX = INFINITY;
    }

    if (dir.y != 0.0f) {
        float nextY = (stepY > 0) ? (y + 1 - start.y) : (start.y - y);
        tMaxY = nextY / fabs(dir.y);
    } else {
        tMaxY = INFINITY;
    }

    // Шаг по сетке (приращение t при переходе на следующую клетку)
    float tDeltaX = (dir.x != 0.0f) ? 1.0f / fabs(dir.x) : INFINITY;
    float tDeltaY = (dir.y != 0.0f) ? 1.0f / fabs(dir.y) : INFINITY;

    const float maxDist = 100.0f;

    while (true) {
        // Проверка текущей клетки (если она в пределах карты и является стеной)
        if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H) {
            if (worldMap[y * MAP_W + x]) {
                // Нашли стену. Возвращаем расстояние до точки входа в клетку.
                // Точное расстояние до пересечения: минимальное из tMaxX, tMaxY.
                return std::min(tMaxX, tMaxY);
            }
        } else {
            // Выход за пределы карты – считаем дальней стеной.
            return maxDist;
        }

        // Переходим к следующей клетке
        if (tMaxX < tMaxY) {
            if (tMaxX > maxDist) break;
            x += stepX;
            tMaxX += tDeltaX;
        } else {
            if (tMaxY > maxDist) break;
            y += stepY;
            tMaxY += tDeltaY;
        }
    }
    return maxDist;
}

// ---------------------------------------------------------------------
// Преобразование мировых координат в экранные для правой панели
// ---------------------------------------------------------------------
POINT toCoreView(Vec2 worldPoint, Vec2 center) {
    const float localX = (worldPoint.x - center.x) * kCoreViewScale;
    const float localY = (worldPoint.y - center.y) * kCoreViewScale;

    POINT pt;
    pt.x = static_cast<LONG>(kLeftPanelWidth + (kLeftPanelWidth / 2.0f) + localX);
    pt.y = static_cast<LONG>((kWindowHeight / 2.0f) + localY);
    return pt;
}

// ---------------------------------------------------------------------
// Оконная процедура
// ---------------------------------------------------------------------
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    case WM_ERASEBKGND:
        return 1;
    default:
        return DefWindowProc(hwnd, uMsg, wParam, lParam);
    }
}

// ---------------------------------------------------------------------
// Отрисовка кадра (оптимизированная)
// ---------------------------------------------------------------------
void drawFrame(HDC hdcMem, HDC hdcWindow, const std::vector<LidarPoint>& scan) {
    RECT rect = {0, 0, kWindowWidth, kWindowHeight};
    FillRect(hdcMem, &rect, static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));

    // Вертикальная разделительная линия
    DrawLine(hdcMem, kLeftPanelWidth, 0, kLeftPanelWidth, kWindowHeight, RGB(100, 100, 100));

    // ---- Левая панель: стены (растровая карта) - БЫСТРАЯ ОТРИСОВКА ЧЕРЕЗ ПИКСЕЛЬНЫЙ БУФЕР ----
    BITMAPINFO bmi = {};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = MAP_W;
    bmi.bmiHeader.biHeight = -MAP_H; // отрицательная высота, чтобы первый ряд пикселей был верхним
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;

    StretchDIBits(hdcMem,
                  0, 0, MAP_W, MAP_H,
                  0, 0, MAP_W, MAP_H,
                  mapPixels,                // указатель на пиксели
                  &bmi,
                  DIB_RGB_COLORS,
                  SRCCOPY);

    // ---- Лидаровские лучи и точки ----
    for (const auto& p : scan) {
        const Vec2 pt = dronePos + p.toCartesian();
        DrawLine(hdcMem,
                 static_cast<int>(dronePos.x), static_cast<int>(dronePos.y),
                 static_cast<int>(pt.x), static_cast<int>(pt.y),
                 RGB(50, 50, 50));
    }
    // Реальный дрон (зелёный)
    DrawCircle(hdcMem,
               static_cast<int>(dronePos.x), static_cast<int>(dronePos.y),
               8, RGB(0, 255, 0), true);

    // ---- Правая панель: отладочная информация ядра ----
    const Vec2 estPos = core.getEstPos();

    // Линии текущих признаков (голубые)
    const auto lines = core.getDebugLines();
    for (const auto& line : lines) {
        const POINT p1 = toCoreView(line.start, estPos);
        const POINT p2 = toCoreView(line.end, estPos);
        DrawLine(hdcMem, p1.x, p1.y, p2.x, p2.y, RGB(0, 255, 255), 2);
    }

    // Граф памяти
    const auto graph = core.getGraph();
    for (const auto& node : graph) {
        COLORREF color = node.isOffloaded ? RGB(100, 100, 100) : RGB(255, 0, 0);
        const POINT nPos = toCoreView(node.position, estPos);
        DrawCircle(hdcMem, nPos.x, nPos.y, 4, color, true);

        for (int id : node.connectedNodes) {
            if (id < 0 || id >= static_cast<int>(graph.size())) continue;
            const POINT p2 = toCoreView(graph[id].position, estPos);
            DrawLine(hdcMem, nPos.x, nPos.y, p2.x, p2.y, color);
        }
    }

    // Центр (оценённое положение дрона)
    const POINT center = toCoreView(estPos, estPos);
    DrawCircle(hdcMem, center.x, center.y, 6, RGB(0, 255, 0), false);

    // Логи (жёлтый текст)
    const std::string logs = core.getLogs();
    SetTextColor(hdcMem, RGB(255, 255, 0));
    SetBkMode(hdcMem, TRANSPARENT);
    RECT textRect = {650, 600, 1200, 800};
    DrawText(hdcMem, logs.c_str(), -1, &textRect, DT_LEFT);

    // Пояснительные надписи
    TextOut(hdcMem, 10, 10, "SIMULATION REALITY", 18);
    TextOut(hdcMem, 650, 10, "CORE MEMORY VIEW", 16);
    TextOut(hdcMem, 650, 30, "Centered on estimated drone position", 35);

    // Перенос буфера на экран
    BitBlt(hdcWindow, 0, 0, kWindowWidth, kWindowHeight, hdcMem, 0, 0, SRCCOPY);
}

// ---------------------------------------------------------------------
// Инициализация мира (генерация пещер и поиск свободного места)
// ---------------------------------------------------------------------
void initWorld() {
    generateCaveMap();

    // Ищем первую свободную клетку (не стену)
    for (int y = 1; y < MAP_H - 1; y++) {
        for (int x = 1; x < MAP_W - 1; x++) {
            if (!worldMap[y * MAP_W + x]) {
                dronePos = Vec2((float)x, (float)y);
                return;
            }
        }
    }
    // Запасной вариант (если вся карта забита)
    dronePos = Vec2(100.0f, 100.0f);
}

} // namespace

// ---------------------------------------------------------------------
// Главная функция симулятора
// ---------------------------------------------------------------------
int runSimulator() {
    // Инициализация мира
    initWorld();
    // После генерации карты обновляем пиксельный буфер
    updateMapBuffer();

    const char CLASS_NAME[] = "NoGPS_Sim_Class";
    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.lpszClassName = CLASS_NAME;
    RegisterClass(&wc);

    HWND hwnd = CreateWindowEx(
        0,
        CLASS_NAME,
        "NoGPS Core Simulator (GDI Version) - Cave World",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        kWindowWidth,
        kWindowHeight,
        nullptr,
        nullptr,
        GetModuleHandle(nullptr),
        nullptr);

    HDC hdcWindow = GetDC(hwnd);
    HDC hdcMem = CreateCompatibleDC(hdcWindow);
    HBITMAP hbmMem = CreateCompatibleBitmap(hdcWindow, kWindowWidth, kWindowHeight);
    HGDIOBJ oldBmp = SelectObject(hdcMem, hbmMem);

    bool running = true;
    while (running) {
        MSG msg = {};
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running = false;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // Управление (WASD)
        Vec2 inputVel(0.0f, 0.0f);
        if (GetAsyncKeyState('W') & 0x8000) inputVel.y = -100.0f;
        if (GetAsyncKeyState('S') & 0x8000) inputVel.y = 100.0f;
        if (GetAsyncKeyState('A') & 0x8000) inputVel.x = -100.0f;
        if (GetAsyncKeyState('D') & 0x8000) inputVel.x = 100.0f;

        // Сканирование лидаром (DDA ray marching)
        std::vector<LidarPoint> scan;
        constexpr int kNumRays = 100;
        scan.reserve(kNumRays);
        for (int i = 0; i < kNumRays; i++) {
            float angle = (i * 2.0f * PI) / kNumRays;
            float dst = CastRay(dronePos, {std::cos(angle), std::sin(angle)});
            float noise = ((std::rand() % 100) / 100.0f) * 3.0f;
            scan.push_back({angle, dst + noise});
        }

        // Обновление физики
        droneVel = inputVel + core.velocityCommand;
        dronePos = dronePos + droneVel * kFrameDt;

        // Простое удержание дрона в пределах карты (чтобы не улететь за границы)
        dronePos.x = std::max(1.0f, std::min((float)(MAP_W - 2), dronePos.x));
        dronePos.y = std::max(1.0f, std::min((float)(MAP_H - 2), dronePos.y));

        // Обновление ядра
        core.update(kFrameDt, scan, droneVel);

        // Отрисовка
        drawFrame(hdcMem, hdcWindow, scan);
        Sleep(16);
    }

    SelectObject(hdcMem, oldBmp);
    DeleteObject(hbmMem);
    DeleteDC(hdcMem);
    ReleaseDC(hwnd, hdcWindow);

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
Total text size: 28885 bytes
================================================================================
