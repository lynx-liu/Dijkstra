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
  -d '{"id":"t1","lat":43.9055,"lon":87.4561,"speed":72,"timestamp":1700000000}'

curl -X POST http://127.0.0.1:8080/api/vehicle \
  -H 'Content-Type: application/json' \
  -d '{"id":"t2","lat":43.9132,"lon":87.4920,"speed":72,"timestamp":1700000000}'
```

第二辆车 POST 后，返回的 `partner` 为与之最快会合的已接入车辆；第一辆车时 `found:false`（车队不足 2 辆）。

```bash
# 方式二：JSON 行协议（stdin/stdout）
./build/mmlp_service --graph data/graph/china.mmlp.bin
# 每行一辆车的 JSON，stdout 一行结果
```

请求字段：`id`, `lat`, `lon`, `speed`(km/h), `timestamp`(Unix 秒)；可选 `type`:`train`、`history`:[72,70,...]。

全部两两组合加 `--all-pairs`。车辆 GPS 须能匹配到路网且在同一连通分量上。

## HTTP 接口

### 1) 单车接入（与已接入车队比较）

- `POST /api/vehicle`
- 输入：单辆车 JSON
- 输出：该车与车队最快会合结果（或 `found:false`）

```bash
curl -X POST http://127.0.0.1:8080/api/vehicle \
  -H 'Content-Type: application/json' \
  -d '{"id":"t1","lat":43.9055,"lon":87.4561,"speed":72,"timestamp":1700000000}'
```

返回示例（已找到会合）：

```json
{
  "found": true,
  "focal": "t1",
  "partner": "t2",
  "meetTimeUnix": 1700000250,
  "meetTimeUtc": "2023-11-14T22:17:29Z",
  "meetDurationSec": 149.54,
  "lat": 43.913413,
  "lon": 87.491920,
  "locationId": "edge:14456442370006"
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
- `found`：是否找到可达会合点
- `focal`：本次请求车辆 ID
- `partner`：与 `focal` 会合最快的车辆 ID
- `meetTimeUnix` / `meetTimeUtc`：会合时间（Unix 秒 / UTC 字符串）
- `meetDurationSec`：从两车对齐起算到会合的耗时（秒）
- `lat` / `lon`：会合点坐标
- `locationId`：会合位置对应的路网位置标识

### 2) 多车批量（第一辆车 vs 其余车辆，返回列表并排序）

- `POST /api/meetings/lead`
- 输入：`vehicles` 数组，`vehicles[0]` 作为基准车
- 输出：`meetings` 列表，按 `meetDurationSec` 升序（时间短的在前）

```bash
curl -X POST http://127.0.0.1:8080/api/meetings/lead \
  -H 'Content-Type: application/json' \
  -d '{
    "vehicles": [
      {"id":"t1","lat":43.9055,"lon":87.4561,"speed":72,"timestamp":1700000000},
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
      "partner": "t3",
      "meetTimeUnix": 1700000232,
      "meetTimeUtc": "2023-11-14T22:17:12Z",
      "meetDurationSec": 132.15,
      "lat": 43.921756,
      "lon": 87.491721,
      "locationId": "edge:13611128940001"
    },
    {
      "found": true,
      "partner": "t2",
      "meetTimeUnix": 1700000250,
      "meetTimeUtc": "2023-11-14T22:17:29Z",
      "meetDurationSec": 149.54,
      "lat": 43.913413,
      "lon": 87.491920,
      "locationId": "edge:14456442370006"
    },
    {
      "found": false,
      "partner": "t4"
    }
  ]
}
```

字段与排序说明：
- `focal`：基准车辆 ID（即输入 `vehicles[0].id`）
- `meetings`：其余车辆与 `focal` 的会合结果列表
- 列表按 `meetDurationSec` 升序排序（时间短的在前）
- 无法会合的项为 `found:false`，排在已找到结果之后

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
