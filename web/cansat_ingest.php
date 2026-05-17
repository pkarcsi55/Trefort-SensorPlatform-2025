<?php
header("Content-Type: application/json; charset=utf-8");

$LOG = __DIR__ . "/ingest_error.log";
function logline($s){
    global $LOG;
    @file_put_contents($LOG, "[".gmdate("Y-m-d H:i:s")."Z] ".$s."\n", FILE_APPEND);
}

$DEBUG = (isset($_GET["debug"]) && $_GET["debug"] == "1");

// Fatal catcher -> always JSON
register_shutdown_function(function() use ($DEBUG) {
    $e = error_get_last();
    if ($e && in_array($e["type"], [E_ERROR, E_PARSE, E_CORE_ERROR, E_COMPILE_ERROR], true)) {
        logline("FATAL: ".$e["message"]." @ ".$e["file"].":".$e["line"]);
        http_response_code(500);
        $out = ["ok"=>false, "error"=>"FATAL", "message"=>$e["message"]];
        if ($DEBUG) { $out["file"]=$e["file"]; $out["line"]=$e["line"]; }
        echo json_encode($out);
    }
});

function bad($code, $msg, $detail=null){
    logline("ERROR $code: $msg ".($detail!==null?json_encode($detail):""));
    http_response_code($code);
    $out=["ok"=>false,"error"=>$msg];
    if($detail!==null) $out["detail"]=$detail;
    echo json_encode($out);
    exit;
}

// --- include connect ---
$connectPath = __DIR__ . "/cansat_connect.php";
if (!file_exists($connectPath)) bad(500, "Missing cansat_connect.php", $connectPath);
require_once($connectPath);

if ($_SERVER["REQUEST_METHOD"] !== "POST") bad(405, "POST required");

// --- read JSON ---
$raw = file_get_contents("php://input");
$data = json_decode($raw, true);
if (!is_array($data)) bad(400, "Invalid JSON", $raw);

// ---- helpers: NA/"" -> NULL; parse typed ----
function isNA($v){
    if ($v === null) return true;
    if (is_string($v)) {
        $t = trim($v);
        return ($t === "" || strcasecmp($t, "NA") === 0);
    }
    return false;
}

function toStrOrNull($v){
    if (isNA($v)) return null;
    $s = trim((string)$v);
    return $s === "" ? null : $s;
}

function toFloatOrNull($v){
    if (isNA($v)) return null;
    if (is_numeric($v)) return (float)$v;
    return null;
}

function toIntOrNull($v){
    if (isNA($v)) return null;
    if (is_numeric($v)) return (int)$v;
    return null;
}

// Parse TX UTC "yyyyMMdd-HHmmss" -> "YYYY-mm-dd HH:ii:ss" (UTC)
function parseTsUtcOrNull($s){
    if ($s === null) return null;
    $s = trim((string)$s);
    if ($s === "" || strcasecmp($s, "NA") === 0) return null;

    // accept MySQL style directly
    if (preg_match('/^\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}$/', $s)) {
        return $s;
    }

    // accept TX style
    $dt = DateTime::createFromFormat("Ymd-His", $s, new DateTimeZone("UTC"));
    if (!$dt) return null;
    return $dt->format("Y-m-d H:i:s");
}

// --- REQUIRED ---
$flight_id = trim($data["flight_id"] ?? "");
$seq       = (int)($data["seq"] ?? 0);

if ($flight_id === "" || $seq <= 0) {
    bad(400, "Missing flight_id/seq", ["flight_id"=>$flight_id,"seq"=>$seq]);
}

// --- OPTIONAL / V1 FIELDS ---
$pkt      = toStrOrNull($data["pkt"] ?? null);       // "TX1"
$mission  = toStrOrNull($data["mission"] ?? null);   // "M1"
$ms       = toIntOrNull($data["ms"] ?? null);        // millis()

$ts_utc   = parseTsUtcOrNull($data["ts_utc"] ?? null);

$pressure_hpa = toFloatOrNull($data["pressure_hpa"] ?? null);
$temp_in_c    = toFloatOrNull($data["temp_in_c"] ?? null);
$rh_pct       = toFloatOrNull($data["rh_pct"] ?? null);
$temp_out_c   = toFloatOrNull($data["temp_out_c"] ?? null);
$h_bme_m      = toFloatOrNull($data["h_bme_m"] ?? null);

$gps_lat   = toFloatOrNull($data["gps_lat"] ?? null);
$gps_lon   = toFloatOrNull($data["gps_lon"] ?? null);
$gps_alt_m = toFloatOrNull($data["gps_alt_m"] ?? null);

$rssi_dbm  = toIntOrNull($data["rssi_dbm"] ?? null);
$snr_db    = toFloatOrNull($data["snr_db"] ?? null);

$ubat_v       = toFloatOrNull($data["ubat_v"] ?? null);
$ubat_percent = toIntOrNull($data["ubat_percent"] ?? null);

// raw_line + src_ip (helpful terepen)
$raw_line = toStrOrNull($data["raw_line"] ?? null);
$src_ip = $_SERVER["REMOTE_ADDR"] ?? null;

// Extra sanity: if raw_line present, keep it short
if ($raw_line !== null && strlen($raw_line) > 512) {
    $raw_line = substr($raw_line, 0, 512);
}

if ($DEBUG) {
    echo json_encode([
        "ok"=>true,
        "debug"=>"pre-insert-v1",
        "flight_id"=>$flight_id,
        "seq"=>$seq,
        "pkt"=>$pkt,
        "mission"=>$mission,
        "ms"=>$ms,
        "ts_utc"=>$ts_utc,
        "pressure_hpa"=>$pressure_hpa,
        "temp_in_c"=>$temp_in_c,
        "rh_pct"=>$rh_pct,
        "temp_out_c"=>$temp_out_c,
        "h_bme_m"=>$h_bme_m,
        "gps_lat"=>$gps_lat,
        "gps_lon"=>$gps_lon,
        "gps_alt_m"=>$gps_alt_m,
        "rssi_dbm"=>$rssi_dbm,
        "snr_db"=>$snr_db,
        "ubat_v"=>$ubat_v,
        "ubat_percent"=>$ubat_percent,
        "src_ip"=>$src_ip
    ]);
    exit;
}

// --- DB insert ---
$db = ConnMe();
@mysqli_set_charset($db, "utf8mb4");

$sql = "INSERT INTO telemetry
(flight_id, pkt, mission, seq, ms, ts_utc,
 pressure_hpa, temp_in_c, rh_pct, temp_out_c, h_bme_m,
 gps_lat, gps_lon, gps_alt_m,
 rssi_dbm, snr_db,
 ubat_v, ubat_percent,
 raw_line, src_ip)
VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)";

$stmt = mysqli_prepare($db, $sql);
if (!$stmt) bad(500, "Prepare failed", mysqli_error($db));

/*
20 params:
1  flight_id    s
2  pkt          s
3  mission      s
4  seq          i
5  ms           i
6  ts_utc       s

7  pressure_hpa d
8  temp_in_c    d
9  rh_pct       d
10 temp_out_c   d
11 h_bme_m      d

12 gps_lat      d
13 gps_lon      d
14 gps_alt_m    d

15 rssi_dbm     i
16 snr_db       d

17 ubat_v       d
18 ubat_percent i

19 raw_line     s
20 src_ip       s
*/

$types = "sssiisddddddddiddiss";

$okBind = mysqli_stmt_bind_param(
    $stmt, $types,
    $flight_id, $pkt, $mission, $seq, $ms, $ts_utc,
    $pressure_hpa, $temp_in_c, $rh_pct, $temp_out_c, $h_bme_m,
    $gps_lat, $gps_lon, $gps_alt_m,
    $rssi_dbm, $snr_db,
    $ubat_v, $ubat_percent,
    $raw_line, $src_ip
);
if (!$okBind) bad(500, "bind_param failed", mysqli_stmt_error($stmt));

$ok = mysqli_stmt_execute($stmt);
if (!$ok) {
    $err = mysqli_stmt_error($stmt);
    if (stripos($err, "Duplicate") !== false) {
        echo json_encode(["ok"=>true, "duplicate"=>true, "seq"=>$seq]);
    } else {
        bad(500, "Insert failed", $err);
    }
} else {
    $lastId = mysqli_insert_id($db);
    echo json_encode(["ok"=>true, "seq"=>$seq, "insert_id"=>$lastId]);
}

mysqli_stmt_close($stmt);
mysqli_close($db);