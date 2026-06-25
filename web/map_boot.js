(function (global) {
  'use strict';

  const LEAFLET_CSS = ['/web/vendor/leaflet/leaflet.css'];
  const LEAFLET_JS = ['/web/vendor/leaflet/leaflet.js'];
  const MAPLIBRE_CSS = ['/web/vendor/map/maplibre-gl.css'];
  const MAPLIBRE_JS = ['/web/vendor/map/maplibre-gl.js'];
  const MAPLIBRE_LEAFLET_JS = ['/web/vendor/map/leaflet-maplibre-gl.js'];

  const LOCAL_RASTER_URL = '/api/map/tiles/{z}/{x}/{y}.png';

  function loadStylesheet(urls, index) {
    return new Promise((resolve, reject) => {
      if (index >= urls.length) {
        reject(new Error(urls[0] + ' missing'));
        return;
      }
      const link = document.createElement('link');
      link.rel = 'stylesheet';
      link.href = urls[index];
      link.onload = () => resolve();
      link.onerror = () => loadStylesheet(urls, index + 1).then(resolve, reject);
      document.head.appendChild(link);
    });
  }

  function loadScript(urls, index) {
    return new Promise((resolve, reject) => {
      if (index >= urls.length) {
        reject(new Error(urls[0] + ' missing'));
        return;
      }
      const script = document.createElement('script');
      script.src = urls[index];
      script.onload = () => resolve();
      script.onerror = () => loadScript(urls, index + 1).then(resolve, reject);
      document.head.appendChild(script);
    });
  }

  let bootPromise = null;
  let mapLibreBoot = null;

  function ready() {
    if (!bootPromise) {
      bootPromise = loadStylesheet(LEAFLET_CSS, 0)
        .then(() => loadScript(LEAFLET_JS, 0))
        .then(() => {
          if (!global.L) throw new Error('Leaflet failed to load');
        });
    }
    return bootPromise;
  }

  function ensureMapLibre() {
    if (!mapLibreBoot) {
      mapLibreBoot = loadStylesheet(MAPLIBRE_CSS, 0)
        .then(() => loadScript(MAPLIBRE_JS, 0))
        .then(() => loadScript(MAPLIBRE_LEAFLET_JS, 0))
        .then(() => {
          if (!global.maplibregl || !global.L.maplibreGL) {
            throw new Error('MapLibre missing');
          }
        });
    }
    return mapLibreBoot;
  }

  function roadStyle(feature) {
    const t = feature.properties && feature.properties.type;
    if (t === 'rail') {
      return { color: '#dc2626', weight: 2, opacity: 0.75, dashArray: '4 3' };
    }
    return { color: '#2563eb', weight: 1.2, opacity: 0.7 };
  }

  function boundsQuery(bounds, padFrac) {
    const pad = padFrac == null ? 0.12 : padFrac;
    const sw = bounds.getSouthWest();
    const ne = bounds.getNorthEast();
    const latSpan = Math.max(ne.lat - sw.lat, 0.008);
    const lonSpan = Math.max(ne.lng - sw.lng, 0.008);
    return {
      minLon: sw.lng - lonSpan * pad,
      minLat: sw.lat - latSpan * pad,
      maxLon: ne.lng + lonSpan * pad,
      maxLat: ne.lat + latSpan * pad,
    };
  }

  async function fetchTileMeta() {
    try {
      const res = await fetch('/api/map/tiles/meta');
      if (!res.ok) return null;
      return await res.json();
    } catch (e) {
      return null;
    }
  }

  function tryRasterLayer(map, url, options) {
    return new Promise((resolve, reject) => {
      const layer = global.L.tileLayer(url, options || {});
      let errors = 0;
      let ok = false;
      const fail = () => {
        if (ok) return;
        try {
          map.removeLayer(layer);
        } catch (e) {
          /* ignore */
        }
        reject(new Error('tile load failed'));
      };
      const timer = setTimeout(fail, 12000);
      layer.on('tileload', () => {
        if (!ok) {
          ok = true;
          clearTimeout(timer);
          resolve(layer);
        }
      });
      layer.on('tileerror', () => {
        errors += 1;
        if (!ok && errors >= 6) {
          clearTimeout(timer);
          fail();
        }
      });
      layer.addTo(map);
    });
  }

  async function initRasterBasemap(map, meta) {
    const minZoom = meta && meta.minzoom != null ? meta.minzoom : 8;
    const maxZoom = meta && meta.maxzoom != null ? meta.maxzoom : 16;
    const layer = await tryRasterLayer(map, LOCAL_RASTER_URL, {
      minZoom,
      maxZoom,
      attribution: '本机离线瓦片',
    });
    map.setMinZoom(minZoom);
    map.setMaxZoom(maxZoom);
    return {
      mode: 'raster',
      layer,
      label: (meta && meta.name) || '本地街道图（离线生成）',
      hint: '本机离线瓦片',
    };
  }

  function patchStyle(style, meta) {
    const origin = global.location.origin || '';
    const minZoom = meta && meta.minzoom != null ? meta.minzoom : 0;
    const maxZoom = meta && meta.maxzoom != null ? meta.maxzoom : 14;
    if (style.glyphs && style.glyphs.indexOf('http') !== 0) {
      style.glyphs = origin + style.glyphs;
    }
    if (style.sources && style.sources.shortbread) {
      const tiles = style.sources.shortbread.tiles || [];
      style.sources.shortbread.tiles = tiles.map((u) => {
        if (u.indexOf('http') === 0) return u;
        return origin + u;
      });
      style.sources.shortbread.minzoom = minZoom;
      style.sources.shortbread.maxzoom = maxZoom;
    }
    return style;
  }

  function waitMapLibreLayer(map, layer) {
    return new Promise((resolve, reject) => {
      const glMap = layer.getMaplibreMap();
      if (!glMap) {
        reject(new Error('MapLibre map not created'));
        return;
      }
      const timer = setTimeout(() => reject(new Error('MapLibre load timeout')), 45000);
      glMap.once('load', () => {
        clearTimeout(timer);
        map.invalidateSize();
        resolve(layer);
      });
      glMap.on('error', (ev) => {
        clearTimeout(timer);
        const msg = (ev && ev.error && ev.error.message) ? ev.error.message : 'MapLibre error';
        reject(new Error(msg));
      });
    });
  }

  async function initVectorBasemap(map, meta) {
    await ensureMapLibre();
    const styleRes = await fetch('/web/map_style_shortbread.json');
    if (!styleRes.ok) throw new Error('map_style_shortbread missing');
    let style;
    try {
      style = patchStyle(await styleRes.json(), meta);
    } catch (e) {
      throw new Error('map style JSON invalid: ' + e.message);
    }
    const layer = global.L.maplibreGL({
      style,
      interactive: false,
      padding: 0.05,
      localIdeographFontFamily: 'sans-serif',
      attribution: '离线矢量 mbtiles',
    });
    layer.addTo(map);
    const minZoom = meta && meta.minzoom != null ? meta.minzoom : 0;
    const maxZoom = meta && meta.maxzoom != null ? meta.maxzoom : 14;
    map.setMinZoom(minZoom);
    map.setMaxZoom(Math.max(maxZoom, 16));
    await waitMapLibreLayer(map, layer);
    return {
      mode: 'mbtiles',
      layer,
      label: (meta && meta.name) || '离线矢量 mbtiles',
      hint: '矢量底图（Shortbread，含地名）',
    };
  }

  async function loadRoadFallback(map, roadLayer, options) {
    const opts = options || {};
    let url = '/api/map/roads?';
    if (opts.bbox) {
      const b = opts.bbox;
      url += `minLon=${b.minLon}&minLat=${b.minLat}&maxLon=${b.maxLon}&maxLat=${b.maxLat}`;
    } else if (opts.auto) {
      url += 'auto=1';
    } else if (map && map.getBounds) {
      const b = boundsQuery(map.getBounds(), opts.padFrac || 0.15);
      url += `minLon=${b.minLon}&minLat=${b.minLat}&maxLon=${b.maxLon}&maxLat=${b.maxLat}`;
    } else {
      url += 'auto=1';
    }

    const res = await fetch(url);
    if (!res.ok) throw new Error('roads ' + res.status);
    const geojson = await res.json();
    roadLayer.clearLayers();
    global.L.geoJSON(geojson, { style: roadStyle, interactive: false }).addTo(roadLayer);

    const props = geojson.properties || {};
    return {
      mode: 'roads',
      label: '离线路网（无街道底图）',
      features: props.features || (geojson.features ? geojson.features.length : 0),
      truncated: !!props.truncated,
    };
  }

  async function initBasemap(map, roadLayer) {
    const meta = await fetchTileMeta();
    const src = meta && meta.source;
    const fmt = ((meta && meta.format) || '').toLowerCase();
    try {
      if (src === 'mbtiles' && (fmt === 'pbf' || meta.vector)) {
        roadLayer.clearLayers();
        return await initVectorBasemap(map, meta);
      }
      return await initRasterBasemap(map, meta);
    } catch (e) {
      const roads = await loadRoadFallback(map, roadLayer, {
        bbox: { minLon: 87.42, minLat: 43.78, maxLon: 87.68, maxLat: 43.92 },
      });
      return {
        mode: 'roads',
        label: roads.label,
        features: roads.features,
        truncated: roads.truncated,
        hint: '底图失败，仅路网线条 — ' + e.message,
      };
    }
  }

  function addRoadLegend(map) {
    const legend = global.L.control({ position: 'bottomright' });
    legend.onAdd = () => {
      const d = global.L.DomUtil.create('div', 'legend');
      d.style.cssText =
        'background:#fff;padding:8px 10px;border-radius:6px;font-size:12px;line-height:1.6;box-shadow:0 1px 4px rgba(0,0,0,.15)';
      d.innerHTML =
        '<div><span style="display:inline-block;width:14px;height:3px;background:#2563eb;margin-right:6px;vertical-align:middle"></span>公路</div>' +
        '<div><span style="display:inline-block;width:14px;height:3px;background:#dc2626;margin-right:6px;vertical-align:middle"></span>铁路</div>';
      return d;
    };
    legend.addTo(map);
    return legend;
  }

  global.MapBoot = {
    ready,
    createMap(containerId, mapOptions) {
      return global.L.map(containerId, Object.assign({ preferCanvas: false }, mapOptions || {}));
    },
    addScale(map) {
      global.L.control.scale({ metric: true }).addTo(map);
    },
    boundsQuery,
    initBasemap,
    loadRoadFallback,
    addRoadLegend,
  };
})(window);
