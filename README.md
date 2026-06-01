# 多模态物流车辆会合预测系统 (MMLP)

基于 [技术方案](多模态物流车辆会合预测系统技术方案.md) 的实现仓库。

**覆盖范围**：全国公铁多模态路网，见 [docs/OSM_DATA.md](docs/OSM_DATA.md)、[docs/DESIGN.md](docs/DESIGN.md)。

## 状态

| 阶段 | 状态 |
|------|------|
| 0 契约与参数 | 完成 |
| 1 路网图（OSM 下载 + 建图） | 完成，见下方部署 |
| 2–6 匹配 / 路由 / 会合算法 | 完成 |

## 快速开始（两步）

### 1) 初始化（依赖 + 编译 + 图 + 索引）

```bash
bash tools/bootstrap_service.sh
```

### 2) 启动服务

```bash
bash tools/start_http_server.sh
```

启动后在浏览器打开 **http://127.0.0.1:8080/map**，可实时查看 POST 上报的车辆位置、会合点及两车路径（点击标记查看详情，页面每 2 秒自动刷新）。

> **地图底图**：**街道图**可在离线环境由路网**自动生成**（无需外网）；**卫星图**无法从路网生成，需自备影像 mbtiles。详见 [tools/prepare_offline_map.md](tools/prepare_offline_map.md)。

> `bootstrap_service.sh` 已包含初始化所需步骤，通常不需要再手动分步执行。

## 校验路网 + 网页预览（可选）

```bash
python3 tools/verify_graph.py data/graph/china.mmlp.bin
bash tools/export_preview_regions.sh    # 导出若干城市 GeoJSON
bash tools/serve_map_viewer.sh          # http://127.0.0.1:8765/web/index.html
```

在网页里将 **蓝/红路网** 与 **OSM 底图** 对照，即可目视检查偏移与缺失。

## 运行会合预测

不把 5.7GB 全图载入内存，只加载车辆附近区域：

```bash
export MMLP_GRAPH_PATH=data/graph/china.mmlp.bin
./build/mmlp_predict \
  --graph data/graph/china.mmlp.bin \
  --focal truck_A \
  --vehicle truck_A,43.9055,87.4561,72,1700000000 \
  --vehicle truck_B,43.9132,87.4920,72,1700000000 \
  --padding-m 80000
```

输出 JSON 示例：

```json
{
  "found": true,
  "focal": "A",
  "partner": "B",
  "meetTimeUnix": 1700000075,
  "meetTimeUtc": "2023-11-14T22:13:55Z",
  "meetDurationSec": 74.93,
  "lat": 43.9094,
  "lon": 87.4741,
  "locationId": "edge:14457364360006"
}
```

`meetTimeUnix` 为整数 Unix 秒；`meetTimeUtc` 为 UTC 的 ISO 8601 时间。

## 常驻服务（逐车接入）

每来一辆车就与**已接入车队**计算最快会合并返回结果。

```bash
cmake --build build --target mmlp_service

# 方式一：HTTP（推荐，须保持终端运行）
bash tools/start_http_server.sh
# 或: python3 tools/mmlp_http_server.py --port 8080

curl -X POST http://127.0.0.1:8080/api/vehicle \
  -H 'Content-Type: application/json' \
  -d '{"id":"t1","lat":43.9055,"lon":87.4561,"speed":72,"timestamp":1700000100}'

curl -X POST http://127.0.0.1:8080/api/vehicle \
  -H 'Content-Type: application/json' \
  -d '{"id":"t2","lat":43.9132,"lon":87.4920,"speed":72,"timestamp":1700000100}'
```

第二辆车 POST 后，返回的 `partner` 为与之最快会合的已接入车辆；第一辆车时 `found:false`（车队不足 2 辆）。返回含路径 `routeSelf` / `routePartner` 及路程 `routeDistanceSelfM` / `routeDistancePartnerM`，详见下方 [HTTP 接口](#http-接口)。

```bash
# 方式二：JSON 行协议（stdin/stdout）
./build/mmlp_service --graph data/graph/china.mmlp.bin
# 每行一辆车的 JSON，stdout 一行结果
```

请求字段：`id`, `lat`, `lon`, `speed`(km/h), `timestamp`(Unix 秒)；可选 `type`:`train`、`history`:[72,70,...]。

全部两两组合加 `--all-pairs`。车辆 GPS 须能匹配到路网且在同一连通分量上。

## HTTP 接口

通用说明：

- 默认端口 `8080`；服务未就绪时返回 `503`
- 请求体 `Content-Type: application/json`
- 车辆字段：`id`, `lat`, `lon`, `speed`（km/h）, `timestamp`（Unix 秒）；可选 `type`（`truck` / `train`）、`history`（速度样本数组，km/h）
- 时间对齐：两车会合前，先将各车状态对齐到 `max(timestamp)`；`timestamp` 较早的车会沿路网先「追平」时间差，再计算会合
- 会合优化目标：**最早会合时间**（`meetDurationSec` 最小），不是地理中点
- 路径字段：`routeSelf` 为基准车（或本次 POST 车辆）到会合点的路网折线；`routePartner` 为对方车辆到会合点的折线；坐标为 `[[lat, lon], ...]`，首点尽量与车辆上报 GPS 衔接

### 健康检查

- `GET /health`

```bash
curl http://127.0.0.1:8080/health
```

返回示例（就绪）：

```json
{"status": "ok"}
```

返回示例（启动中）：

```json
{"status": "loading", "message": "service starting, retry shortly or watch server log"}
```

### 1) 单车接入（与已接入车队比较）

- `POST /api/vehicle`
- 输入：单辆车 JSON
- 输出：该车（`focal`）与**已接入车队**中会合最快的一辆车之结果；车队不足 2 辆时 `found:false`

```bash
curl -X POST http://127.0.0.1:8080/api/vehicle \
  -H 'Content-Type: application/json' \
  -d '{"id":"t1","lat":43.9055,"lon":87.4561,"speed":72,"timestamp":1700000100}'
```

返回示例（已找到会合）：

```json
{
  "found": true,
  "focal": "t1",
  "partner": "t2",
  "meetTimeUnix": 1700000176,
  "meetTimeUtc": "2023-11-14T22:16:16Z",
  "meetDurationSec": 76.09,
  "lat": 43.909409,
  "lon": 87.474346,
  "locationId": "edge:14457364360006",
  "routeSelf": [
    [43.905536, 87.456125],
    [43.909409, 87.474346]
  ],
  "routePartner": [
    [43.9132, 87.491999],
    [43.913164, 87.492012],
    [43.909409, 87.474346]
  ],
  "routeDistanceSelfM": 1521.9,
  "routeDistancePartnerM": 1479.6
}
```

返回示例（未找到会合）：

```json
{
  "found": false,
  "focal": "t1"
}
```

字段说明：

| 字段 | 说明 |
|------|------|
| `found` | 是否找到可达会合点 |
| `focal` | 本次请求车辆 ID |
| `partner` | 与 `focal` 会合最快的已接入车辆 ID（仅 `found:true`） |
| `meetTimeUnix` | 会合时刻，Unix 秒（整数） |
| `meetTimeUtc` | 会合时刻，UTC ISO 8601 字符串 |
| `meetDurationSec` | 从两车时间对齐后起算，到会合的耗时（秒） |
| `lat` / `lon` | 会合点 WGS84 坐标 |
| `locationId` | 会合点在路网上的位置标识（节点 ID 或 `edge:<id>`） |
| `routeSelf` | 基准车（即 `focal`）沿路网到会合点的坐标序列 |
| `routePartner` | 对方车（即 `partner`）沿路网到会合点的坐标序列 |
| `routeDistanceSelfM` | `routeSelf` 折线长度（米，路网近似） |
| `routeDistancePartnerM` | `routePartner` 折线长度（米，路网近似） |

### 2) 多车批量（第一辆车 vs 其余车辆，返回列表并排序）

- `POST /api/meetings/lead`
- 输入：`vehicles` 数组，`vehicles[0]` 作为基准车（`focal`）
- 输出：基准车与其余每辆车的会合结果；`meetings` 按 `meetDurationSec` **升序**（耗时短的在前）；无法会合的项 `found:false`，排在后面

```bash
curl -X POST http://127.0.0.1:8080/api/meetings/lead \
  -H 'Content-Type: application/json' \
  -d '{
    "vehicles": [
      {"id":"t1","lat":43.9055,"lon":87.4561,"speed":72,"timestamp":1700000100},
      {"id":"t2","lat":43.9132,"lon":87.4920,"speed":70,"timestamp":1700000100},
      {"id":"t3","lat":43.9200,"lon":87.5000,"speed":68,"timestamp":1700000100}
    ]
  }'
```

返回示例：

```json
{
  "focal": "t1",
  "meetings": [
    {
      "found": true,
      "partner": "t2",
      "meetTimeUnix": 1700000176,
      "meetTimeUtc": "2023-11-14T22:16:16Z",
      "meetDurationSec": 76.09,
      "lat": 43.909409,
      "lon": 87.474346,
      "locationId": "edge:14457364360006",
      "routeSelf": [
        [43.905536, 87.456125],
        [43.909409, 87.474346]
      ],
      "routePartner": [
        [43.9132, 87.491999],
        [43.913164, 87.492012],
        [43.909409, 87.474346]
      ],
      "routeDistanceSelfM": 1521.9,
      "routeDistancePartnerM": 1479.6
    },
    {
      "found": true,
      "partner": "t3",
      "meetTimeUnix": 1700000257,
      "meetTimeUtc": "2023-11-14T22:17:37Z",
      "meetDurationSec": 157.02,
      "lat": 43.914401,
      "lon": 87.491518,
      "locationId": "edge:14456442370004",
      "routeSelf": [
        [43.905536, 87.456125],
        [43.913164, 87.492012],
        [43.914401, 87.491518]
      ],
      "routePartner": [
        [43.92, 87.5],
        [43.920162, 87.500773],
        [43.914401, 87.491518]
      ],
      "routeDistanceSelfM": 3124.5,
      "routeDistancePartnerM": 985.3
    }
  ]
}
```

字段说明：

| 字段 | 说明 |
|------|------|
| `focal` | 基准车辆 ID，等于 `vehicles[0].id` |
| `meetings` | 基准车与其余各车的会合结果数组（**不是**其余车之间的两两会合） |
| `meetings[].partner` | 对方车辆 ID |
| 其余字段 | 同「单车接入」表中各 meeting 项字段 |

排序规则：`found:true` 的项按 `meetDurationSec` 升序；`found:false` 的项排在后面（按 `partner` 字典序）。

### 3) 实时地图

- `GET /map` — 浏览器打开实时地图（车辆位置、会合点、路径；点击标记弹出详情）
- `GET /api/map/state` — 最近一次 POST 后的车辆与会合快照（地图页每 2 秒轮询）

**底图说明**：优先本地 MBTiles（`/api/map/tiles/...`）或在线街道瓦片（Carto/OSM/Esri）；都不可用时才回退为离线路网线条。Leaflet 从 `/web/vendor/leaflet/` 本地加载（`bash tools/fetch_web_vendor.sh`）。须通过 `http://host:8080/map` 访问。

```bash
curl http://127.0.0.1:8080/api/map/state
```

返回示例：

```json
{
  "vehicles": [
    {"id": "t1", "lat": 43.9055, "lon": 87.4561, "speed": 72, "timestamp": 1700000100, "type": "truck"},
    {"id": "t2", "lat": 43.9132, "lon": 87.492, "speed": 70, "timestamp": 1700000100, "type": "truck"},
    {"id": "t3", "lat": 43.92, "lon": 87.5, "speed": 68, "timestamp": 1700000100, "type": "truck"}
  ],
  "focal": "t1",
  "meetings": [
    {
      "found": true,
      "partner": "t2",
      "meetTimeUnix": 1700000176,
      "meetTimeUtc": "2023-11-14T22:16:16Z",
      "meetDurationSec": 76.09,
      "lat": 43.909409,
      "lon": 87.474346,
      "locationId": "edge:14457364360006",
      "routeSelf": [[43.905536, 87.456125], [43.909409, 87.474346]],
      "routePartner": [[43.9132, 87.491999], [43.909409, 87.474346]],
      "routeDistanceSelfM": 1521.9,
      "routeDistancePartnerM": 1479.6
    }
  ],
  "mode": "batch",
  "updatedAt": 1717123456.789
}
```

字段说明：

| 字段 | 说明 |
|------|------|
| `vehicles` | 最近一次 POST 涉及的全部车辆（上报 GPS） |
| `focal` | 基准车 ID（批量接口为 `vehicles[0]`；单车接口为本次 POST 车辆） |
| `meetings` | 最近一次 POST 返回的会合结果（结构与上文相同） |
| `mode` | `batch`（`/api/meetings/lead`）或 `single`（`/api/vehicle`） |
| `updatedAt` | 快照更新时间，Unix 秒（浮点） |

## 验证地图（网页预览）

按区域导出 GeoJSON 后在网页叠加底图查看：

```bash
bash tools/preview_map.sh
# 浏览器打开 http://127.0.0.1:8765/
```

蓝线=公路，红线=铁路，橙点=枢纽。

自定义区域示例：

```bash
python3 tools/graph_to_geojson.py -i data/graph/china.mmlp.bin \
  --bbox 104.0,30.5,104.2,30.7 -o web/data/preview.json --max-edges 12000
```

需要按区域建图时，可设置 `BBOX=minLon,minLat,maxLon,maxLat` 后执行 `bash tools/deploy_graph.sh`。

## 加载路网并预测

```cpp
#include "mmlp/graph_load.hpp"
#include "mmlp/predict.hpp"

mmlp::MultimodalGraph graph;
std::string err;
if (!mmlp::loadDefaultGraph(graph, "", &err)) { /* handle err */ }

auto best = mmlp::predictBestMeetingFor("current_id", fleet, histories, graph);
```

## 目录

```
include/mmlp/   公共类型与 API
src/            图与预测实现
tests/          单元测试与 tiny 图 fixture
docs/           设计约定、OSM 数据说明
tools/          OSM 导入脚本（阶段 1）
```

## 主 API

```cpp
#include "mmlp/predict.hpp"

// 全部车辆对的最早会合
std::vector<mmlp::MeetingResult> predictMeetings(
    const std::vector<mmlp::VehicleInfo>& fleet,
    const std::vector<mmlp::VehicleHistory>& histories,
    const mmlp::MultimodalGraph& graph,
    const mmlp::PredictParam& param = {});

// 指定当前车：最快碰头的一辆车 + 时间 + 地点
mmlp::FocalBestMeeting predictBestMeetingFor(
    const std::string& focalVehicleId,
    const std::vector<mmlp::VehicleInfo>& fleet,
    const std::vector<mmlp::VehicleHistory>& histories,
    const mmlp::MultimodalGraph& graph,
    const mmlp::PredictParam& param = {});

// 约定 fleet[0] 为当前车
mmlp::FocalBestMeeting predictBestMeetingForCurrent(fleet, histories, graph);
```

`FocalBestMeeting` 字段：`partnerVehicleId`、`meetTime`（Unix 秒）、`meetDuration`（对齐起点后的等待秒数）、`lat`/`lon`/`locationId`。

设计补充见 [docs/DESIGN.md](docs/DESIGN.md)。
