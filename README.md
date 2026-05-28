# 多模态物流车辆会合预测系统 (MMLP)

基于 [技术方案](多模态物流车辆会合预测系统技术方案.md) 的实现仓库。

**覆盖范围**：中国全境公铁多模态路网；新疆等为重点验证区域，见 [docs/OSM_DATA.md](docs/OSM_DATA.md)、[docs/DESIGN.md](docs/DESIGN.md)。

## 状态

| 阶段 | 状态 |
|------|------|
| 0 契约与参数 | 完成 |
| 1 路网图（OSM 下载 + 建图） | 完成，见下方部署 |
| 2–6 匹配 / 路由 / 会合算法 | 完成 |

## 构建

```bash
bash tools/install_deps.sh    # pip install osmium
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## 部署全国 OSM 路网图（真实数据在 `data/`，不在 Git 里）

仓库里**只有**测试用 `tests/fixtures/sample.osm`（3 个节点）。全国图需本机生成：

```bash
bash tools/install_deps.sh
bash tools/deploy_graph_nationwide.sh   # 或 deploy_graph.sh（勿设置 BBOX）
export MMLP_GRAPH_PATH=data/graph/china.mmlp.bin
bash tools/graph_status.sh            # 查看下载/建图进度
```

| 生成文件 | 说明 |
|----------|------|
| `data/osm/china-latest.osm.pbf` | Geofabrik 中国原始数据 (~1.4 GB) |
| `data/graph/china.mmlp.bin` | 全国公铁路网（程序读取此文件） |

下载约 10～30 分钟，建图可能 **1～3 小时**（视 CPU/内存）。日志：`data/deploy.log`。

## 校验路网 + 网页预览

```bash
python3 tools/verify_graph.py data/graph/china.mmlp.bin
bash tools/export_preview_regions.sh    # 导出若干城市 GeoJSON
bash tools/serve_map_viewer.sh          # http://127.0.0.1:8765/web/index.html
```

在网页里将 **蓝/红路网** 与 **OSM 底图** 对照，即可目视检查偏移与缺失。

## 运行会合预测（全国图，按区域加载）

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

适合持续上报 GPS：每来一辆车就与**已接入的全车队**算最快碰头，立即返回。

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

## 验证地图是否正确（网页预览）

全国 `.mmlp.bin` 太大，浏览器无法整图加载。按城市导出一条 GeoJSON 后在网页上叠加 OSM 底图查看：

```bash
bash tools/preview_map.sh
# 浏览器打开 http://127.0.0.1:8765/
```

蓝线=公路，红线=铁路，橙点=枢纽。可与底图道路是否重合、铁路走向是否一致作人工核对。

自定义区域：

```bash
python3 tools/graph_to_geojson.py -i data/graph/china.mmlp.bin \
  --bbox 104.0,30.5,104.2,30.7 -o web/data/chengdu.json --max-edges 12000
```

可选：仅导入某一区域（仍从全国 PBF 读取，Python 按 bbox 过滤）：

```bash
export BBOX=73.5,34.3,96.4,49.2   # minLon,minLat,maxLon,maxLat
bash tools/deploy_graph.sh
```

更快裁剪（需 `osmium-tool`）：`export USE_OSMIUM_EXTRACT=1` 后再执行 `deploy_graph.sh`。

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
