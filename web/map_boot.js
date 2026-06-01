(function (global) {
  'use strict';

  const LEAFLET_CSS = [
    '/web/vendor/leaflet/leaflet.css',
    'https://cdn.bootcdn.net/ajax/libs/leaflet/1.9.4/leaflet.css',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
  ];
  const LEAFLET_JS = [
    '/web/vendor/leaflet/leaflet.js',
    'https://cdn.bootcdn.net/ajax/libs/leaflet/1.9.4/leaflet.js',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
  ];

  // WGS84 底图，按顺序尝试（国内网络常无法访问 tile.openstreetmap.org）
  const TILE_SOURCES = [
    {
      label: 'OpenStreetMap',
      url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
      options: { maxZoom: 19, attribution: '&copy; OpenStreetMap' },
    },
    {
      label: 'Carto',
      url: 'https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png',
      options: { subdomains: 'abcd', maxZoom: 20, attribution: '&copy; CARTO &copy; OSM' },
    },
    {
      label: 'Esri',
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}',
      options: { maxZoom: 19, attribution: '&copy; Esri' },
    },
    {
      label: 'OSM DE',
      url: 'https://{s}.tile.openstreetmap.de/{z}/{x}/{y}.png',
      options: { subdomains: 'abc', maxZoom: 19, attribution: '&copy; OSM' },
    },
  ];

  function loadStylesheet(urls, index) {
    return new Promise((resolve, reject) => {
      if (index >= urls.length) {
        reject(new Error('leaflet css load failed'));
        return;
      }
      const link = document.createElement('link');
      link.rel = 'stylesheet';
      link.href = urls[index];
      link.crossOrigin = '';
      link.onload = () => resolve();
      link.onerror = () => loadStylesheet(urls, index + 1).then(resolve, reject);
      document.head.appendChild(link);
    });
  }

  function loadScript(urls, index) {
    return new Promise((resolve, reject) => {
      if (index >= urls.length) {
        reject(new Error('leaflet js load failed'));
        return;
      }
      const script = document.createElement('script');
      script.src = urls[index];
      script.crossOrigin = '';
      script.onload = () => resolve();
      script.onerror = () => loadScript(urls, index + 1).then(resolve, reject);
      document.head.appendChild(script);
    });
  }

  function addTileLayerWithFallback(map, onSourceChange) {
    let sourceIndex = 0;
    let activeLayer = null;
    let errorCount = 0;
    let loaded = false;
    const maxErrors = 4;

    function activateNext() {
      if (activeLayer) {
        map.removeLayer(activeLayer);
        activeLayer = null;
      }
      if (sourceIndex >= TILE_SOURCES.length) {
        if (onSourceChange) {
          onSourceChange(null, '底图不可用（请检查外网或运行 bash tools/fetch_web_vendor.sh）');
        }
        return;
      }
      const source = TILE_SOURCES[sourceIndex++];
      errorCount = 0;
      loaded = false;
      activeLayer = global.L.tileLayer(source.url, source.options);
      activeLayer.on('tileerror', () => {
        errorCount += 1;
        if (!loaded && errorCount >= maxErrors) {
          activateNext();
        }
      });
      activeLayer.on('load', () => {
        loaded = true;
        if (onSourceChange) {
          onSourceChange(source.label);
        }
      });
      activeLayer.addTo(map);
      if (onSourceChange) {
        onSourceChange(source.label, null, true);
      }
    }

    activateNext();
  }

  let bootPromise = null;

  function ready() {
    if (!bootPromise) {
      bootPromise = loadStylesheet(LEAFLET_CSS, 0)
        .then(() => loadScript(LEAFLET_JS, 0))
        .then(() => {
          if (!global.L) {
            throw new Error('Leaflet missing after load');
          }
        });
    }
    return bootPromise;
  }

  global.MapBoot = {
    ready,
    createMap(containerId, mapOptions) {
      return global.L.map(containerId, mapOptions || {});
    },
    addBaseTiles: addTileLayerWithFallback,
    addScale(map) {
      global.L.control.scale({ metric: true }).addTo(map);
    },
  };
})(window);
