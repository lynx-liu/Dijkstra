# 地图数据目录（全国）

真实数据**不会提交到 Git**（见根目录 `.gitignore`），需在本机生成。

## 全国一键生成

```bash
bash tools/install_deps.sh
bash tools/deploy_graph_nationwide.sh
```

完成后应有：

| 文件 | 约大小 | 说明 |
|------|--------|------|
| `osm/china-latest.osm.pbf` | ~1.4 GB | Geofabrik 中国原始 OSM |
| `graph/china.mmlp.bin` | 数百 MB～数 GB | 本项目路网（公铁边+节点） |

## 使用

```bash
export MMLP_GRAPH_PATH=/data/Dijkstra/data/graph/china.mmlp.bin
```

```cpp
mmlp::MultimodalGraph graph;
mmlp::loadDefaultGraph(graph);
```

## 查看进度

```bash
tail -f data/deploy.log
ls -lh data/osm/china-latest.osm.pbf
ls -lh data/graph/china.mmlp.bin
```

## 校验地图是否正确

**1. 结构检查（不加载进 C++）**

```bash
python3 tools/verify_graph.py data/graph/china.mmlp.bin
```

**2. 网页叠加预览（推荐）**

从全国 bin 按区域导出 GeoJSON，在浏览器里与 OSM 底图对照：

```bash
bash tools/export_preview_regions.sh   # 或只导出单城，见 tools/export_graph_geojson.py
bash tools/serve_map_viewer.sh
# 浏览器打开 http://127.0.0.1:8765/web/index.html
```

- 蓝线 = 公路，红线 = 铁路  
- 底图是官方 OSM 瓦片，路网应大致重合  
- 已预导乌鲁木齐：`data/graph/preview/urumqi.geojson`

## 仅测试

`tests/fixtures/sample.osm` 为 3 节点假数据，**不是**全国图。
