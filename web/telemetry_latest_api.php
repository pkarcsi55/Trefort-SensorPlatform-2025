<?php
// telemetry_latest_api.php
header("Content-Type: application/json; charset=utf-8");
header("Cache-Control: no-store, no-cache, must-revalidate, max-age=0");
header("Pragma: no-cache");

require_once __DIR__ . "/cansat_connect.php";

function out($arr, int $code = 200): void {
    http_response_code($code);
    echo json_encode($arr, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES);
    exit;
}

$db = ConnMe();

// Opcionális: flight_id szűrés (?flight_id=...)
$flight_id = isset($_GET["flight_id"]) ? trim($_GET["flight_id"]) : "";
if ($flight_id !== "" && !preg_match('/^[A-Za-z0-9_\-\.]{1,64}$/', $flight_id)) {
    out(["ok" => false, "error" => "Invalid flight_id"], 400);
}

$sql = "
SELECT
  id, flight_id, pkt, mission, seq, ms, ts_utc, received_at,
  pressure_hpa, temp_in_c, rh_pct, temp_out_c, h_bme_m,
  gps_lat, gps_lon, gps_alt_m, rssi_dbm, snr_db, ubat_v, ubat_percent
FROM telemetry
" . ($flight_id !== "" ? "WHERE flight_id = ? " : "") . "
ORDER BY received_at DESC, id DESC
LIMIT 1
";

if ($flight_id !== "") {
    $stmt = mysqli_prepare($db, $sql);
    if (!$stmt) out(["ok" => false, "error" => "DB prepare failed"], 500);

    mysqli_stmt_bind_param($stmt, "s", $flight_id);
    if (!mysqli_stmt_execute($stmt)) out(["ok" => false, "error" => "DB execute failed"], 500);

    $res = mysqli_stmt_get_result($stmt);
} else {
    $res = mysqli_query($db, $sql);
}

if (!$res) out(["ok" => false, "error" => "DB query failed"], 500);

$row = mysqli_fetch_assoc($res);
if (!$row) out(["ok" => true, "data" => null]); // nincs még adat

// numerikus mezők normalizálása (JSON-ban szám legyen, ne string)
$intFields = ["id","seq","ms","rssi_dbm","ubat_percent"];
$floatFields = ["pressure_hpa","temp_in_c","rh_pct","temp_out_c","h_bme_m","gps_lat","gps_lon","gps_alt_m","snr_db","ubat_v"];

foreach ($intFields as $f) {
    if (array_key_exists($f, $row) && $row[$f] !== null) $row[$f] = (int)$row[$f];
}
foreach ($floatFields as $f) {
    if (array_key_exists($f, $row) && $row[$f] !== null) $row[$f] = (float)$row[$f];
}

out(["ok" => true, "data" => $row]);