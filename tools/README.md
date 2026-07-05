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
# 一次完成：依赖、编译、全国图索引、区域图(PRD hwy overlay)
bash tools/bootstrap_service.sh

# 启动服务（默认 MMLP_WORKERS=12，端口占用时自动重启）
bash tools/start_http_server.sh
# 浏览器: http://127.0.0.1:8080/map
# 接口说明见仓库根目录 README.md「HTTP 接口」
```

**目的地到达 benchmark（98 车广州用例，需服务已启动）：**

```bash
python3 tools/bench_dest_arrive.py --label verify --runs 3
```

环境变量（可选）：

| 变量 | 默认 | 说明 |
|------|------|------|
| `MMLP_BUILD_JOBS` | 12 | bootstrap 编译并行度 |
| `MMLP_WORKERS` | 12 | 路由线程池大小 |
| `RESTART` | 1 | 8080 已被占用时先停再起 |
| `SKIP_EXISTING` | 1 | 区域图已存在则跳过重建 |
| `START_AFTER_BOOTSTRAP` | 0 | 设为 1 则 bootstrap 后直接启动 |

## 二进制图格式

| 字段 | 类型 |
|------|------|
| magic | 8 bytes `MMLPGRPH` |
| version | uint32 |
| nodeCount / edgeCount | uint64 ×2 |
| 节点 | id i64, lat f64, lon f64, kind i32 |
| 边 | id, from, to i64; type i32; length, speedLimit f64 |

C++ 加载：`mmlp::loadGraphFromFile(path, graph)` 或 `loadDefaultGraph(graph)`。
