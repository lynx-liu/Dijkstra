# 离线地图说明

## 推荐方案：Shortbread 矢量 mbtiles（道路 + 地名，无外网 CDN）

| 项目 | 说明 |
|------|------|
| 数据源 | BBBike 全国 Shortbread 矢量包 |
| 安装位置 | `data/map/china.mbtiles`（约 2 GB，**不进 git**） |
| 渲染 | MapLibre GL 3.6.2 + 本地字形（`web/vendor/map/glyphs/`） |
| 瓦片 API | `/api/map/tiles/{z}/{x}/{y}.pbf` |
| 未安装时 | 回退为路网实时渲染 PNG（慢，无地名） |

### 新机器一键复现（与当前效果相同）

```bash
git pull
bash tools/bootstrap_service.sh          # 依赖、编译、Leaflet + MapLibre + 字形
bash tools/download_mbtiles.sh           # 约 2 GB，需有网，只需一次
RESTART=1 bash tools/start_http_server.sh
```

或 bootstrap 时自动下载 mbtiles：

```bash
AUTO_DOWNLOAD_MBTILES=1 bash tools/bootstrap_service.sh
bash tools/start_http_server.sh
```

浏览器打开 `http://<主机>:8080/map`，**Ctrl+F5** 强刷。

### 验证

```bash
ls -lh data/map/china.mbtiles
curl -s http://127.0.0.1:8080/api/map/tiles/meta | python3 -m json.tool
```

已装好时应看到：

```json
"source": "mbtiles",
"format": "pbf",
"vector": true
```

状态栏应显示 **「Shortbread · 矢量底图（Shortbread，含地名）」**。

### 自定义 mbtiles 路径

```bash
export MMLP_MBTILES=/path/to/your.mbtiles
RESTART=1 bash tools/start_http_server.sh
```

## 回退：路网实时渲染（无需 mbtiles）

若未安装 mbtiles，服务从 `china.mmlp.bin` 实时渲染 PNG：

```bash
curl http://127.0.0.1:8080/api/map/tiles/meta
# "source": "graph_render"
```

## 卫星图

卫星/航拍影像无法从路网生成，需自备影像 mbtiles 并设置 `MMLP_MBTILES`。

## 依赖

- Pillow（bootstrap 已通过 `pip3 install -r tools/requirements.txt` 安装）
- `unzip`（下载 mbtiles 时解压 zip）
