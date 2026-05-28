# 阶段 1：OpenStreetMap 数据说明

## 服务范围

- **生产范围**：中国全境（公铁多模态路网）
- **重点验证区域**：新疆及西部干线（长距离、公铁联运典型场景）；东部/中部同样适用同一套图与算法
- **数据主源**：全国一份 OSM 中国 extract，而非仅新疆局部图

## 结论：是否合适？

**适合作为算法级多模态底图**，与方案 §4.3 一致。OSM 在全球范围内提供：

- **公路**：`highway=*`（motorway、trunk、primary、secondary 等）
- **铁路**：`railway=rail`（干线）、以及 `light_rail`、`subway` 等（可按需过滤）

中国全境数据在 Geofabrik 持续更新（约 **1.4 GB** PBF，日更），**直接对应全国部署**。省级 Geofabrik 子包不完整，区域试验可用 **bbox 裁剪** 或 **Overpass** 从全国包派生。

### 优势

- 免费、ODbL 许可，可离线处理
- 公路网在东部/干线较完整；铁路主干 `railway=rail` 普遍有线位
- 与方案中的 `Edge`/`Node` 结构自然对应

### 局限（需知）

| 问题 | 说明 | 应对 |
|------|------|------|
| 中国道路精度 | 部分区域道路偏移、等级不全 | 方案中的高德纠偏（阶段 2+），或局部校正 |
| 铁路属性 | `maxspeed`、单双线、货运线属性不完整 | 用默认铁路限速 + 业务历史速度 |
| 枢纽 | 公铁换乘关系 OSM 不统一 | `HUB` 节点规则 + 可选人工表 |
| 全国 PBF 体积 | 1.4GB+，解析与建图耗时 | 全国离线预处理一次；运行时可按省/网格分片加载（见下） |
| 区域质量差异 | 东部道路较密，西部稀疏/偏移 | 全国统一规则；重点区（新疆等）加强测试与高德纠偏 |

**不建议**把 Geofabrik 的 `china-latest-free.shp.zip` 作为唯一数据源（全国 shapefile 不提供）；应使用 **`.osm.pbf` + 自研/工具链过滤**。

## 推荐数据源

### 1. Geofabrik — 全国主数据（生产）

- 中国：[https://download.geofabrik.de/asia/china.html](https://download.geofabrik.de/asia/china.html)
- 文件：`china-latest.osm.pbf`（约 1.4 GB，每日更新）
- 流程：下载全国 PBF → 过滤公/铁路 → 建 `MultimodalGraph` → 可选按省或空间网格 **分片存储**（运行时只加载车辆所在片）

### 2. 区域裁剪 — 开发 / 重点区回归（可选）

从同一 `china-latest.osm.pbf` 派生，**不替代**全国图：

**A. osmium extract（推荐）**

```bash
# 示例：新疆重点区 bbox（WGS84）73.5,34.3,96.4,49.2
osmium extract -b 73.5,34.3,96.4,49.2 china-latest.osm.pbf -o data/osm/xinjiang.osm.pbf

# 示例：华东某省试验（按实际 bbox 替换）
# osmium extract -b ... china-latest.osm.pbf -o data/osm/region.osm.pbf
```

**B. Overpass API**（小范围、快速试标签规则）

```
[out:json][timeout:180];
(
  way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|service)$"]({{bbox}});
  way["railway"="rail"]({{bbox}});
);
out body;
>;
out skel qt;
```

### 3. 开发 / CI 用小样本

- [BBBike extract](https://extract.bbbike.org/) 按城市 bbox 导出
- 或仓库内 `tests/fixtures/` 手工小图（不依赖 OSM）

## 全国部署时的图组织（阶段 1 架构）

| 模式 | 说明 |
|------|------|
| **单图全国** | 内存/磁盘允许时，一次加载全国 `MultimodalGraph`；实现简单，适合离线批算 |
| **分片全国** | 预处理为省界或固定网格（如 1°×1°）多个图文件；预测时按车辆 GPS 加载 1～N 片，跨片车辆在片边界 HUB 上会合 |
| **重点区加严** | 新疆等片区单独跑回归数据集；底图仍来自全国包裁剪，保证与生产一致 |

Geofabrik 对中国 **不提供完整省级 osm.pbf 列表**；分省文件需自行用 [admin boundary](https://wiki.openstreetmap.org/wiki/Tag:boundary%3Dadministrative) + `osmium extract --polygon` 或国标省界 `.poly` 从全国包切出。

## 本项目的 OSM 过滤规则（阶段 1 实现）

### 公路边

- 保留 `highway` ∈ { motorway, trunk, primary, secondary, tertiary, unclassified, residential, service }
- 排除 `area=yes`、纯步行径（footway、path 等，除非后续需要）
- `length`：由 way 折线 Haversine 累加
- `speedLimit`：`maxspeed` 标签（解析 km/h）；缺失则用 `kDefaultRoadSpeedKmh`

### 铁路边

- 保留 `railway=rail`
- 可选排除 `service=*` 侧线/场内线（首版保留干线）
- `speedLimit`：缺失则用 `kDefaultRailSpeedKmh`

### 节点

- 每条 way 的端点与折线相交点 → `ROAD_JUNCTION` / `RAIL_STATION`
- `railway=station` 且 500m 内有公路节点 → 标记 `HUB`

### 边 ID

- 稳定 ID：`osm_way_id`（正向）；反向行驶用同一 way 的几何反向边（实现时 duplicate 或运行时定向）

## 建议工具链

| 工具 | 用途 |
|------|------|
| [osmium-tool](https://osmcode.org/osmium-tool/) | bbox 裁剪、格式转换 |
| [libosmium](https://github.com/osmcode/libosmium) | C++ 解析 PBF（阶段 1b 可选） |
| [osmconvert](https://wiki.openstreetmap.org/wiki/Osmconvert) | 轻量裁剪（备选） |

已实现 **`tools/osm_to_graph.py`** → `*.mmlp.bin`，C++ 用 `loadGraphFromFile` / `loadDefaultGraph` 加载。一键部署：`bash tools/deploy_graph.sh`。

## 许可

使用 OSM 数据需遵守 [ODbL 1.0](https://opendatacommons.org/licenses/odbl/)，产出需注明 © OpenStreetMap contributors。
