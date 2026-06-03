# 设计约定（阶段 0 补充）

本文档记录技术方案未写明、但实现采用的约定。

## 服务范围

- **全国**：路网与预测逻辑覆盖中国全境（同一套 OSM 中国 extract + 过滤规则）。
- **重点地区**：新疆等西部干线用于典型场景验证、精度与性能压测；不改变全国 API 与数据模型。
- 技术方案中的「新疆跨区域」为 **示例场景**，非功能边界。

## 单位与坐标

| 量 | 单位 |
|----|------|
| 距离 | 米 (m) |
| 时间 | 秒 (s)；API 输入/输出使用 **ISO UTC 字符串**（如 `2026-06-01T10:00:00Z`） |
| 速度（输入/历史） | 千米/小时 (km/h) |
| 航向 | 度，正北为 0，顺时针 |
| 经纬度 | WGS84 |

## 预测起点时间

两车 `time`（观测时刻）不一致时，预测起点取 **较晚** 的观测时刻：

`t0 = max(timeA, timeB)`

较早到达的一方在会合模型中视为在 `t0` 时刻已沿最短路前进到对应位置（阶段 4 实现时展开）。

## 多模态图

- 公路边 `EdgeType::ROAD`，铁路边 `EdgeType::RAIL`。
- `NodeKind::HUB` 表示公铁联运枢纽；首版加载 OSM 时由 `railway=station` 且邻近公路节点启发式标记，后续可人工表维护。
- 火车仅在 RAIL 子图（及 HUB）上匹配与路由；货车仅在 ROAD 子图（及 HUB）上匹配与路由。

## 无会合输出

在 `maxTime` 内不可达或子图不连通的车辆对 **不写入** 结果列表。

## 速度融合

`v = α * v_hist + (1 - α) * v_gps`，默认 `α = 0.85`（`constants.hpp`）。无历史记录时 `v = v_gps`；GPS 速度无效（≤0）时用该车型默认限速的 80%。

## 货车休息（首版）

Dijkstra 边权使用 **含休息的等效旅行时间**：

`travelTime(d, v) = (d/v) + floor((d/v) / truckCycle) * truckRest`

（`d` 为米，`v` 为 m/s）。后续可升级为带 `(node, cycle_phase)` 的状态图。

## 当前车 API

- `predictBestMeetingFor(focalId, fleet, ...)`：在 `fleet` 中为 `focalId` 找 `meetTime` 最小的伙伴车。
- `predictBestMeetingForCurrent`：`fleet[0]` 为当前车。
- 详见 [ALGORITHM.md](ALGORITHM.md)。
