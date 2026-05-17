<?php
// telemetry_live.php
$flight_id = isset($_GET["flight_id"]) ? trim($_GET["flight_id"]) : "";
?>
<!doctype html>
<html lang="hu">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <meta name="color-scheme" content="light dark">
  <title>CanSat – Live Telemetry</title>
  <style>
    :root{
      --bg: #0b0f14;
      --card: rgba(255,255,255,0.06);
      --card2: rgba(255,255,255,0.10);
      --text: rgba(255,255,255,0.92);
      --muted: rgba(255,255,255,0.70);
      --ok: #46e38a;
      --warn: #ffcc66;
      --bad: #ff5c5c;
      --line: rgba(255,255,255,0.10);
      --mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
      --sans: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial, "Noto Sans", "Liberation Sans", sans-serif;
    }
    @media (prefers-color-scheme: light){
      :root{
        --bg: #f6f7fb;
        --card: rgba(0,0,0,0.05);
        --card2: rgba(0,0,0,0.08);
        --text: rgba(0,0,0,0.90);
        --muted: rgba(0,0,0,0.62);
        --line: rgba(0,0,0,0.10);
      }
    }
    body{
      margin:0;
      font-family: var(--sans);
      background: radial-gradient(1200px 700px at 20% 10%, rgba(70,227,138,0.15), transparent 60%),
                  radial-gradient(1000px 600px at 80% 0%, rgba(100,140,255,0.18), transparent 60%),
                  var(--bg);
      color: var(--text);
    }
    .wrap{ max-width: 720px; margin: 0 auto; padding: 14px 14px 32px; }

    /* --- LOGO --- */
    .logoWrap{
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 10px;
      margin-bottom: 10px;
      backdrop-filter: blur(8px);
    }
    .logoWrap img{
      width: 100%;
      height: auto;            /* arányt tart */
      display: block;
      border-radius: 12px;
    }

    .top{ display:flex; align-items:flex-end; justify-content:space-between; gap:12px; padding: 8px 4px 12px; }
    .title{ line-height:1.15; }
    .title h1{ font-size: 20px; margin: 0 0 4px; letter-spacing: .2px; }
    .title .sub{ font-family: var(--mono); font-size: 12px; color: var(--muted); word-break: break-all; }
    .status{ text-align:right; font-family: var(--mono); font-size: 12px; color: var(--muted); min-width: 160px; }
    .pill{ display:inline-flex; align-items:center; gap:8px; padding: 6px 10px; border-radius: 999px; background: var(--card); border: 1px solid var(--line); margin-top: 6px; }
    .dot{ width:10px; height:10px; border-radius:50%; background: var(--warn); box-shadow: 0 0 0 4px rgba(255,204,102,0.18); }

    .grid{ display:grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    @media (max-width: 420px){ .grid{ grid-template-columns: 1fr; } .status{ min-width:auto; } }

    .card{ background: var(--card); border: 1px solid var(--line); border-radius: 16px; padding: 12px 12px 14px; backdrop-filter: blur(8px); }
    .card .label{ font-size: 12px; color: var(--muted); letter-spacing: .3px; text-transform: uppercase; }
    .card .value{ margin-top: 6px; font-size: 28px; font-weight: 700; letter-spacing: .2px; display:flex; align-items: baseline; gap: 8px; }
    .unit{ font-size: 14px; color: var(--muted); font-weight: 600; }

    .smallrow{ display:grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 12px; }
    .mini{ background: var(--card2); border: 1px solid var(--line); border-radius: 14px; padding: 10px 10px 12px; }
    .mini .k{ font-size: 12px; color: var(--muted); }
    .mini .v{ font-family: var(--mono); margin-top: 6px; font-size: 13px; word-break: break-word; }

    .footer{ margin-top: 14px; font-family: var(--mono); font-size: 12px; color: var(--muted); display:flex; justify-content: space-between; gap: 10px; flex-wrap: wrap; padding: 0 4px; }

    .err{ margin-top: 12px; background: rgba(255,92,92,0.12); border: 1px solid rgba(255,92,92,0.35); color: var(--text); border-radius: 14px; padding: 10px 12px; font-family: var(--mono); font-size: 12px; display:none; }
  </style>
</head>
<body>
  <div class="wrap">

    <!-- LOGO a legtetején -->
    <div class="logoWrap">
      <img src="logo.png" alt="CanSat logo">
    </div>

    <div class="top">
      <div class="title">
        <h1>CanSat – Live Telemetry</h1>
        <div class="sub" id="subLine">last packet: –</div>
      </div>
      <div class="status">
        <div class="pill">
          <span class="dot" id="dot"></span>
          <span id="stText">connecting…</span>
        </div>
        <div style="margin-top:6px;" id="tsLine">–</div>
      </div>
    </div>

    <div class="grid">
      <div class="card"><div class="label">T külső</div><div class="value"><span id="temp_out">–</span> <span class="unit">°C</span></div></div>
      <div class="card"><div class="label">T belső</div><div class="value"><span id="temp_in">–</span> <span class="unit">°C</span></div></div>
      <div class="card"><div class="label">Nyomás (P)</div><div class="value"><span id="p">–</span> <span class="unit">hPa</span></div></div>
      <div class="card"><div class="label">Rel. páratartalom</div><div class="value"><span id="rh">–</span> <span class="unit">%</span></div></div>

      <div class="card">
        <div class="label">h barométer</div>
        <div class="value"><span id="h_baro">–</span> <span class="unit">m</span></div>
      </div>

      <div class="card"><div class="label">h GPS</div><div class="value"><span id="h_gps">–</span> <span class="unit">m</span></div></div>
    </div>

    <div class="smallrow">
      <div class="mini">
        <div class="k">GPS</div>
        <div class="v" id="gps">lat: – / lon: –</div>
      </div>
      <div class="mini">
        <div class="k">Rádió / Akku</div>
        <div class="v" id="radio">RSSI: – dBm | SNR: – dB</div>
        <div class="v" id="bat">Ubat: – V (–%)</div>
      </div>
    </div>

    <div class="err" id="errBox">error: –</div>

    <div class="footer">
      <div>Auto-refresh: <span id="refreshMs">1000</span> ms</div>
      <div>API: telemetry_latest_api.php<?php if ($flight_id !== "") echo "?flight_id=" . htmlspecialchars($flight_id, ENT_QUOTES, "UTF-8"); ?></div>
    </div>
  </div>

<script>
  const REFRESH_MS = 1000;
  const H_BARO_OFFSET_M = 120.0; // <-- barométer magasság korrekció

  document.getElementById("refreshMs").textContent = String(REFRESH_MS);
  const flightId = <?php echo json_encode($flight_id, JSON_UNESCAPED_UNICODE); ?>;

  function fmt(x, digits=1){
    if (x === null || x === undefined || Number.isNaN(x)) return "–";
    const n = Number(x);
    if (!Number.isFinite(n)) return "–";
    return n.toFixed(digits);
  }

  function setStatus(ok, text){
    const dot = document.getElementById("dot");
    const st = document.getElementById("stText");
    st.textContent = text;

    if (ok === true){
      dot.style.background = "var(--ok)";
      dot.style.boxShadow = "0 0 0 4px rgba(70,227,138,0.18)";
    } else if (ok === false){
      dot.style.background = "var(--bad)";
      dot.style.boxShadow = "0 0 0 4px rgba(255,92,92,0.18)";
    } else {
      dot.style.background = "var(--warn)";
      dot.style.boxShadow = "0 0 0 4px rgba(255,204,102,0.18)";
    }
  }

  function showError(msg){
    const b = document.getElementById("errBox");
    b.style.display = "block";
    b.textContent = "error: " + msg;
  }
  function hideError(){
    const b = document.getElementById("errBox");
    b.style.display = "none";
  }

  let lastSeq = null;

  async function tick(){
    try{
      setStatus(null, "updating…");
      const url = "telemetry_latest_api.php" + (flightId ? ("?flight_id=" + encodeURIComponent(flightId)) : "");
      const r = await fetch(url, { cache: "no-store" });
      if (!r.ok) throw new Error("HTTP " + r.status);
      const j = await r.json();
      if (!j.ok) throw new Error(j.error || "API ok=false");
      if (!j.data){
        setStatus(false, "no data");
        return;
      }
      hideError();

      const d = j.data;

      document.getElementById("temp_out").textContent = fmt(d.temp_out_c, 1);
      document.getElementById("temp_in").textContent  = fmt(d.temp_in_c, 1);
      document.getElementById("p").textContent        = fmt(d.pressure_hpa, 2);
      document.getElementById("rh").textContent       = fmt(d.rh_pct, 1);

      // h barométer: +130 m korrekció
      const hBaro = (d.h_bme_m === null || d.h_bme_m === undefined) ? null : (Number(d.h_bme_m) + H_BARO_OFFSET_M);
      document.getElementById("h_baro").textContent   = fmt(hBaro, 1);

      document.getElementById("h_gps").textContent    = fmt(d.gps_alt_m, 1);

      document.getElementById("gps").textContent =
        `lat: ${d.gps_lat ?? "–"} / lon: ${d.gps_lon ?? "–"}`;

      document.getElementById("radio").textContent =
        `RSSI: ${d.rssi_dbm ?? "–"} dBm | SNR: ${d.snr_db ?? "–"} dB`;

      document.getElementById("bat").textContent =
        `Ubat: ${d.ubat_v ?? "–"} V (${d.ubat_percent ?? "–"}%)`;

      document.getElementById("subLine").textContent =
        `flight=${d.flight_id} | pkt=${d.pkt} | mission=${d.mission} | seq=${d.seq} | ms=${d.ms}`;

      document.getElementById("tsLine").textContent =
        `received_at=${d.received_at} | ts_utc=${d.ts_utc ?? "–"}`;

      if (lastSeq === null || d.seq !== lastSeq) setStatus(true, "LIVE");
      else setStatus(false, "STALE");

      lastSeq = d.seq;

    } catch(e){
      setStatus(false, "ERROR");
      showError(e.message || String(e));
    }
  }

  tick();
  setInterval(tick, REFRESH_MS);
</script>
</body>
</html>