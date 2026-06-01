(function (global) {
  'use strict';

  const LEAFLET_CSS = ['/web/vendor/leaflet/leaflet.css'];
  const LEAFLET_JS = ['/web/vendor/leaflet/leaflet.js'];

  // 1) local mbtiles  2) online street tiles  3) local road graph fallback
  const RASTER_SOURCES = [
    {
      name: '本地街道图（离线生成）',
      url: '/api/map/tiles/{z}/{x}/{y}.png',
      options: { maxZoom: 16, minZoom: 8, attribution: '本地路网渲染' },
      local: true,
    },
    {
      name: 'Carto 街道图',
      url: 'https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png',
      options: {
        subdomains: 'abcd',
        maxZoom: 20,
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a> &copy; CARTO',
      },
    },
    {
      name: 'OpenStreetMap',
      url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
      options: { maxZoom: 19, attribution: '&copy; OpenStreetMap' },
    },
    {
      name: 'Esri 街道图',
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}',
      options: { maxZoom: 19, attribution: '&copy; Esri' },
    },
  ];

  function loadStylesheet(urls, index) {
    return new Promise((resolve, reject) => {
      if (index >= urls.length) {
        reject(new Error('leaflet.css missing — run: bash tools/fetch_web_vendor.sh'));
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
        reject(new Error('leaflet.js missing — run: bash tools/fetch_web_vendor.sh'));
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

  async function localTilesAvailable() {
    try {
      const res = await fetch('/api/map/tiles/meta');
      if (!res.ok) return false;
      const meta = await res.json();
      return !!meta.available;
    } catch (e) {
      return false;
    }
  }

  function tryRasterLayer(map, src) {
    return new Promise((resolve, reject) => {
      const layer = global.L.tileLayer(src.url, src.options);
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
      const timer = setTimeout(fail, src.local ? 4000 : 6000);
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

  async function initRasterBasemap(map) {
    const online = RASTER_SOURCES.filter((s) => !s.local);
    const offline = RASTER_SOURCES.filter((s) => s.local);
    const order = [...online, ...offline];

    let lastErr = null;
    for (const src of order) {
      try {
        const layer = await tryRasterLayer(map, src);
        return { mode: 'raster', layer, label: src.name, source: src };
      } catch (e) {
        lastErr = e;
      }
    }
    throw lastErr || new Error('no raster basemap');
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
    try {
      const raster = await initRasterBasemap(map);
      return {
        mode: 'raster',
        layer: raster.layer,
        label: raster.label,
        hint: '标准街道底图；车辆/会合标记叠在上方',
      };
    } catch (e) {
      const roads = await loadRoadFallback(map, roadLayer, {
        bbox: { minLon: 87.42, minLat: 43.78, maxLon: 87.68, maxLat: 43.92 },
      });
      return {
        mode: 'roads',
        label: roads.label,
        features: roads.features,
        truncated: roads.truncated,
        hint: '当前无街道底图，仅显示离线路网线条。可配置 data/map/region.mbtiles 或开放外网',
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
      return global.L.map(containerId, mapOptions || {});
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
