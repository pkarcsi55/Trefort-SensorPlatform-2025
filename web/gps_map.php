<?php
// gps_map.php
$flight_id = isset($_GET["flight_id"]) ? trim($_GET["flight_id"]) : "";
?>
<!DOCTYPE html>
<html lang="hu">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>CanSat GPS térkép (TX1/M1)</title>

  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet@latest/dist/leaflet.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 0; }
    header { display:flex; align-items:center; justify-content:center; gap:10px; margin: 10px 0 0; padding: 0 10px; }
    header img { width: 56px; height: auto; }
    h3 { text-align:center; margin: 0; }

    #map { margin: 10px auto; width: 95%; height: 70vh; border: 1px solid #ccc; }
    .status-message { text-align:center; padding: 6px 10px; font-family: monospace; }

    #control-panel {
      position: absolute; top: 10px; right: 14px;
      background-color: rgba(200,200,200,0.65);
      padding: 10px; border: 1px solid #ccc;
      z-index: 1000; border-radius: 8px;
      display: flex; gap: 10px; align-items: center; flex-wrap: wrap;
    }

    #data-container {
      position: absolute; top: 80px; right: 14px;
      width: min(460px, 44vw);
      background-color: rgba(0, 128, 0, 0.20);
      padding: 10px; border: 1px solid #ccc;
      max-height: 220px; overflow-y: auto;
      font-family: monospace; z-index: 1000;
      border-radius: 8px; white-space: pre;
    }

    #chart-container {
      position: absolute; right: 14px; top: 320px;
      width: min(460px, 44vw);
      background-color: rgba(255, 216, 0, 0.22);
      padding: 10px; border: 1px solid #ccc;
      max-height: 320px; overflow-y: auto;
      z-index: 1000; border-radius: 8px;
    }

    button {
      padding: 6px 10px;
      border: 1px solid #999;
      border-radius: 6px;
      background: rgba(255,255,255,0.75);
      cursor: pointer;
    }
    button:hover { background: rgba(255,255,255,0.95); }

    @media (max-width: 700px){
      #control-panel, #data-container, #chart-container{
        position: static; width: 95%; margin: 10px auto;
      }
      #map{ height: 55vh; }
    }
  </style>

  <script>
    let map, chart;
    const markers = [];
    const polylines = [];
    const points = []; // {id, lat, lon, alt, received_at, seq}

    let dataStep = 5;
    let sinceId = 0;

    const POLL_MS = 500;
    const MAX_POINTS = 2500;
    const ALT_SCALE = 100000;

    // csak egyszer rázummolunk automatikusan az adatokra
    let autoFitted = false;

    // localStorage kulcs (flight_id szerint is tudjuk külön tárolni)
    const flightId = <?php echo json_encode($flight_id, JSON_UNESCAPED_UNICODE); ?>;
    const VIEW_KEY = "cansat_last_view_" + (flightId ? flightId : "global");

    function saveLastView(){
      try{
        if (!map) return;
        const c = map.getCenter();
        const z = map.getZoom();
        const payload = { lat: c.lat, lon: c.lng, zoom: z, ts: Date.now() };
        localStorage.setItem(VIEW_KEY, JSON.stringify(payload));
      } catch(e){
        // ignore
      }
    }

    function loadLastView(){
      try{
        const raw = localStorage.getItem(VIEW_KEY);
        if (!raw) return null;
        const obj = JSON.parse(raw);
        if (typeof obj.lat !== "number" || typeof obj.lon !== "number" || typeof obj.zoom !== "number") return null;
        return obj;
      } catch(e){
        return null;
      }
    }

    function initializeMap(){
      // demó: ha nincs élő adat, is ugorjon a legutóbbi nézetre
      const last = loadLastView();
      const defaultCenter = [47.11677, 19.35366];
      const defaultZoom = 13;

      if (last){
        map = L.map('map').setView([last.lat, last.lon], last.zoom);
      } else {
        map = L.map('map').setView(defaultCenter, defaultZoom);
      }

      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 25 }).addTo(map);

      // ha a user mozgatja/zoomolja: mentsük
      map.on("moveend", saveLastView);
      map.on("zoomend", saveLastView);
    }

    function initializeChart(){
      const ctx = document.getElementById('chart').getContext('2d');
      chart = new Chart(ctx, {
        type: 'line',
        data: { labels: [], datasets: [{ label: 'Magasság (m)', data: [], borderColor: 'red', fill: false }] },
        options: { responsive: true, plugins: { legend: { display: false } }, scales: { x: { ticks: { display: false } } } }
      });
    }

    function fmtLine(p){
      const alt = (p.alt === null || p.alt === undefined) ? "–" : p.alt.toFixed(1);
      return `${p.received_at}  id=${p.id}  seq=${p.seq}  lat=${p.lat.toFixed(6)}  lon=${p.lon.toFixed(6)}  alt=${alt}`;
    }

    function updateLast10Lines(){
      const last = points.slice(-10);
      document.getElementById('data-container').textContent =
        "Az utolsó 10 adatsor (TX1/M1)\n" + last.map(fmtLine).join("\n");
    }

    function updateChart(){
      const last = points.slice(-10);
      const altitudes = last.map(p => (p.alt ?? null)).filter(a => a !== null);
      chart.data.labels = altitudes.map((_, i) => `Adatsor ${i+1}`);
      chart.data.datasets[0].data = altitudes;
      chart.update();
    }

    function clearMapOverlays(){
      markers.forEach(m => map.removeLayer(m));
      markers.length = 0;
      polylines.forEach(l => map.removeLayer(l));
      polylines.length = 0;
    }

    function updateRoute(){
      clearMapOverlays();
      const filtered = points.filter((_, idx) => (idx % dataStep) === 0);

      filtered.forEach((p) => {
        const coord = [p.lat, p.lon];

        markers.push(
          L.circleMarker(coord, { radius:5, color:'blue', fillColor:'blue', fillOpacity:0.5 })
           .addTo(map)
        );

        const alt = (p.alt ?? 0);
        const elevatedCoord = [p.lat + (alt / ALT_SCALE), p.lon];

        markers.push(
          L.circleMarker(elevatedCoord, { radius:1, color:'red', fillColor:'red', fillOpacity:0.5 })
           .addTo(map)
        );

        polylines.push(
          L.polyline([coord, elevatedCoord], { color:'red', weight:1 }).addTo(map)
        );
      });
    }

   function autoFitToData(){
  if (autoFitted) return;
  if (points.length < 2) return;

  const latlngs = points.map(p => [p.lat, p.lon]);
  const bounds = L.latLngBounds(latlngs);

  // 1) először rázummolunk a pontokra
  map.fitBounds(bounds, { padding: [30, 30] });

  // 2) majd egy lépcsővel kijjebb zoomolunk (biztosan távolabb)
  map.once("moveend", () => {
    map.setZoom(map.getZoom() - 3);   // ha még távolabb kell: -2
    saveLastView();                  // demóhoz: utolsó jó nézet mentése
  });

  autoFitted = true;
}
   
    function setStatus(msg, ok=null){
      const el = document.getElementById("status");
      el.textContent = msg;
      if (ok === true) el.style.color = "green";
      else if (ok === false) el.style.color = "red";
      else el.style.color = "#333";
    }

    async function poll(){
      try{
        const params = new URLSearchParams();
        params.set("since_id", String(sinceId));
        params.set("limit", "800");
        if (flightId) params.set("flight_id", flightId);

        const url = "gps_points_api.php?" + params.toString();
        const r = await fetch(url, { cache: "no-store" });
        if (!r.ok) throw new Error("HTTP " + r.status);
        const j = await r.json();
        if (!j.ok) throw new Error(j.error || "API ok=false");

        const newRows = j.rows || [];
        if (newRows.length === 0){
          // Demó mód: ha nincs repülés, a last_view már úgyis be lett húzva induláskor.
          setStatus(`Nincs új GPS pont. since_id=${sinceId}`, null);
          return;
        }

        for (const row of newRows) points.push(row);
        if (points.length > MAX_POINTS) points.splice(0, points.length - MAX_POINTS);

        sinceId = j.last_id || sinceId;

        updateLast10Lines();
        updateChart();
        updateRoute();
        autoFitToData();

        setStatus(`OK: +${newRows.length} pont, last_id=${sinceId}`, true);

      } catch(e){
        setStatus("Hiba: " + (e.message || String(e)), false);
      }
    }

    function recenterToLastOrData(){
      // kézi gomb: ha van adat -> fitBounds, ha nincs -> last view (vagy default)
      if (points.length >= 2){
        autoFitted = false;
        autoFitToData();
        return;
      }

      const last = loadLastView();
      if (last){
        map.setView([last.lat, last.lon], last.zoom);
        return;
      }

      map.setView([47.11677, 19.35366], 13);
    }

    window.onload = () => {
      document.getElementById('data-step-input').value = dataStep;
      document.getElementById('data-step-input').addEventListener('change', (event) => {
        const v = parseInt(event.target.value, 10);
        if (!isNaN(v) && v > 0) dataStep = v;
      });

      initializeMap();
      initializeChart();

      poll();
      setInterval(poll, POLL_MS);
    };
  </script>
</head>

<body>
  <header>
    <img src="logo.png" alt="Logo">
    <h3>CanSat GPS térkép (TX1 / M1)</h3>
  </header>

  <div id="control-panel">
    <label for="data-step-input">Megjelenítési lépés: </label>
    <input type="number" id="data-step-input" min="1" value="5" style="width:80px;">
    <button type="button" onclick="recenterToLastOrData()">Középre</button>
  </div>

  <div id="data-container">Az utolsó 10 adatsor (TX1/M1)</div>

  <div id="chart-container">
    <h5>Utolsó 10 magasság</h5>
    <canvas id="chart"></canvas>
  </div>

  <div id="map"></div>
  <div id="status" class="status-message">indul…</div>
</body>
</html>