# Tools

## 依赖

```bash
bash tools/install_deps.sh   # pyosmium
# 可选: apt install osmium-tool wget
```

## 路网部署

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

## 服务初始化与启动（推荐）

```bash
# 一次完成依赖、编译、图与索引准备
bash tools/bootstrap_service.sh

# 启动服务
bash tools/start_http_server.sh
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
