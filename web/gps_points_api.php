<?php
// gps_points_api.php  (PRIMARY = TX1 / M1)
// A térképhez használt magasság mostantól BME/nyomásalapú: h_bme_m.
// A GPS-magasságot külön mezőben továbbra is visszaadjuk ellenőrzéshez: gps_alt_m.
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

$flight_id = isset($_GET["flight_id"]) ? trim($_GET["flight_id"]) : "";
$since_id  = isset($_GET["since_id"]) ? (int)$_GET["since_id"] : 0;
$limit     = isset($_GET["limit"]) ? (int)$_GET["limit"] : 600;

if ($limit < 10) $limit = 10;
if ($limit > 2000) $limit = 2000;

if ($flight_id !== "" && !preg_match('/^[A-Za-z0-9_\-\.]{1,64}$/', $flight_id)) {
    out(["ok" => false, "error" => "Invalid flight_id"], 400);
}

// Elsődleges küldetés
$pkt = "TX1";
$mission = "M1";

$where = [];
$where[] = "id > " . (int)$since_id;

// A helyzet továbbra is GPS-alapú, ezért csak értelmes GPS-koordinátákat rajzolunk.
// A magasság viszont már nem gps_alt_m, hanem h_bme_m.
$where[] = "gps_lat IS NOT NULL AND gps_lon IS NOT NULL";
$where[] = "gps_lat BETWEEN 45.7 AND 48.6";
$where[] = "gps_lon BETWEEN 16.1 AND 22.9";

$where[] = "pkt = '" . mysqli_real_escape_string($db, $pkt) . "'";
$where[] = "mission = '" . mysqli_real_escape_string($db, $mission) . "'";

if ($flight_id !== "") {
    $where[] = "flight_id = '" . mysqli_real_escape_string($db, $flight_id) . "'";
}

$sql = "
SELECT
  id, flight_id, seq, ms, received_at,
  gps_lat, gps_lon,
  h_bme_m, gps_alt_m
FROM telemetry
WHERE " . implode(" AND ", $where) . "
ORDER BY id ASC
LIMIT " . (int)$limit;

$res = mysqli_query($db, $sql);
if (!$res) out(["ok" => false, "error" => "DB query failed: " . mysqli_error($db)], 500);

$rows = [];
while ($r = mysqli_fetch_assoc($res)) {
    $h_bme_m = ($r["h_bme_m"] === null ? null : (float)$r["h_bme_m"]);
    $gps_alt_m = ($r["gps_alt_m"] === null ? null : (float)$r["gps_alt_m"]);

    $rows[] = [
        "id"          => (int)$r["id"],
        "flight_id"   => $r["flight_id"],
        "seq"         => (int)$r["seq"],
        "ms"          => (int)$r["ms"],
        "received_at" => $r["received_at"],
        "lat"         => (float)$r["gps_lat"],
        "lon"         => (float)$r["gps_lon"],

        // FONTOS: a gps_map.php ezt az "alt" mezőt használja a magassági kijelzéshez.
        // Eddig gps_alt_m volt, mostantól BME/nyomásalapú h_bme_m.
        "alt"         => $h_bme_m,
        "alt_source"  => "BME",

        // Kontrollként meghagyjuk az eredeti GPS-magasságot is, de a térkép nem ezt használja.
        "gps_alt_m"   => $gps_alt_m,
        "h_bme_m"     => $h_bme_m,
    ];
}

$last_id = $since_id;
if (count($rows) > 0) $last_id = $rows[count($rows)-1]["id"];

out([
    "ok" => true,
    "filter" => [
        "pkt" => $pkt,
        "mission" => $mission,
        "flight_id" => ($flight_id === "" ? null : $flight_id),
        "since_id" => $since_id,
        "limit" => $limit,
        "alt_source" => "BME/h_bme_m"
    ],
    "last_id" => $last_id,
    "rows" => $rows
]);
