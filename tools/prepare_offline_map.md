# 离线地图说明

## 能自动做什么

| 类型 | 离线能否自动生成 | 说明 |
|------|------------------|------|
| **街道图** | **能** | 服务从已有 `china.mmlp.bin` + 索引**实时渲染** PNG 瓦片（`/api/map/tiles/{z}/{x}/{y}.png`），无需外网、无需手工准备 mbtiles |
| **卫星图** | **不能** | 卫星/航拍是真实影像，**无法**从路网数据「画」出来，只能一次性下载影像包（mbtiles）后本地读取 |

另一台设备之前看不到街道图，是因为当时还在拉外网瓦片（被墙/断网）。现在默认会走**本地生成的街道瓦片**。

## 验证本地街道瓦片

```bash
curl http://127.0.0.1:8080/api/map/tiles/meta
# 应看到 "source": "graph_render", "available": true

curl -o /tmp/t.png "http://127.0.0.1:8080/api/map/tiles/11/1234/567.png"
file /tmp/t.png   # PNG image
```

浏览器打开 `http://<主机>:8080/map`，状态栏应显示 **「本地街道图（离线生成）」** 或在线 Carto（有外网时优先尝试在线，本地始终可用作回退）。

## 可选：更高质量的离线街道图 / 卫星图

若需要**带建筑、地名**的完整 OSM 风格，或**卫星影像**：

1. 在有网机器用 MapTiler / tilemaker 等从 OSM 或影像源导出 `.mbtiles`
2. 放到 `data/map/region.mbtiles` 或设置 `MMLP_MBTILES=/path/to/file.mbtiles`
3. 重启服务（会优先使用 mbtiles，比实时渲染更精细）

```bash
export MMLP_MBTILES=/path/to/satellite_or_street.mbtiles
bash tools/start_http_server.sh
```

## 依赖

本地渲染需要 Pillow（bootstrap 已通过 `pip3 install -r tools/requirements.txt` 安装）：

```bash
pip3 install Pillow
```
