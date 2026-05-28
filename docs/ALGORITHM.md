# 算法实现说明

## 数据流

```
VehicleInfo + VehicleHistory + MultimodalGraph
        │
        ├─ Map Matching → GraphPosition（边 + 沿边距离 / 节点）
        ├─ 速度融合 → speedMs（历史 + GPS，km/h → m/s）
        ├─ 时间对齐 → t0 = max(tA, tB)，较早车辆沿最短路推进到 t0
        ├─ Dijkstra → 各车到全图节点的最短时间场 T(·)
        └─ 会合求解 → min_x max(T_A(x), T_B(x))（节点 + 边采样/求根）
```

## 会合定义

- 空间：两车在图上同一点（节点或边上的同一 `alongMeters`），等价距离 < `meetDistance`（默认 300m）。
- 时间：`T_meet = max(T_A, T_B)`，取所有候选点中最小者。
- 输出绝对时间：`meetTime = t0 + T_meet`（Unix 秒）。

## 货车休息

边权/沿边旅行时间：`travelTime(d) = d/v + floor((d/v)/truckCycle) * truckRest`。

## 接口

| 函数 | 用途 |
|------|------|
| `predictMeetings` | 全车队两两最早会合 |
| `predictBestMeetingFor` | 指定当前车，找最快会合的一辆车 |
| `predictBestMeetingForCurrent` | `fleet[0]` 为当前车 |

## 模块

| 文件 | 职责 |
|------|------|
| `geo.cpp` | 经纬度 ↔ 局部平面、点到线段距离 |
| `matching.cpp` | GPS 匹配最近公路/铁路边 |
| `motion.cpp` | 历史速度、休息模型 |
| `routing.cpp` | Dijkstra、时间对齐推进 |
| `meeting.cpp` | 两车会合点搜索 |
| `predict.cpp` | 对外 API |

## 限制（当前版本）

- 图需调用方提供（`MultimodalGraph`）；全国 OSM 导入见阶段 1。
- 火车与货车类型不同则不计算会合；公铁枢纽 `HUB` 边扩展待 OSM 建图时接入。
- 航向暂未用于匹配有向边。
