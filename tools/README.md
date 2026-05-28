# Tools

## 依赖

```bash
bash tools/install_deps.sh   # pyosmium
# 可选: apt install osmium-tool wget
```

## 全国路网一键部署

```bash
bash tools/deploy_graph.sh
```

产物：`data/graph/china.mmlp.bin`  
配置：`config/osm.defaults.env`（URL、路径、`BBOX`）

## 分步执行

```bash
bash tools/download_osm.sh                              # data/osm/china-latest.osm.pbf
python3 tools/osm_to_graph.py -i data/osm/china-latest.osm.pbf -o data/graph/china.mmlp.bin
```

区域过滤示例：

```bash
python3 tools/osm_to_graph.py -i data/osm/china-latest.osm.pbf -o data/graph/xj.mmlp.bin \
  --bbox 73.5,34.3,96.4,49.2
```

## 二进制图格式

| 字段 | 类型 |
|------|------|
| magic | 8 bytes `MMLPGRPH` |
| version | uint32 |
| nodeCount / edgeCount | uint64 ×2 |
| 节点 | id i64, lat f64, lon f64, kind i32 |
| 边 | id, from, to i64; type i32; length, speedLimit f64 |

C++ 加载：`mmlp::loadGraphFromFile(path, graph)` 或 `loadDefaultGraph(graph)`。
