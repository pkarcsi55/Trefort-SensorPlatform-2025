//control.php
<?php
require_once("cansat_connect.php");
$db = ConnMe();
$msg = "";
$err = "";

if ($_SERVER["REQUEST_METHOD"] === "POST" && isset($_POST["backup_truncate"])) {
    mysqli_begin_transaction($db);

    try {
        // Megkeressük a következő backup tábla nevét: telemetry01, telemetry02, ...
        $sql = "SHOW TABLES LIKE 'telemetry__'";
        $res = mysqli_query($db, $sql);
        if (!$res) {
            throw new Exception("SHOW TABLES hiba: " . mysqli_error($db));
        }

        $maxNum = 0;
        while ($row = mysqli_fetch_row($res)) {
            $tableName = $row[0];
            if (preg_match('/^telemetry(\d{2})$/', $tableName, $m)) {
                $n = (int)$m[1];
                if ($n > $maxNum) $maxNum = $n;
            }
        }

        $nextNum = $maxNum + 1;
        if ($nextNum > 99) {
            throw new Exception("Elfogytak a kétjegyű backup nevek (telemetry01..telemetry99).");
        }

        $backupTable = "telemetry" . str_pad((string)$nextNum, 2, "0", STR_PAD_LEFT);

        // Backup tábla létrehozása
        $sql = "CREATE TABLE `$backupTable` LIKE `telemetry`";
        if (!mysqli_query($db, $sql)) {
            throw new Exception("CREATE TABLE hiba: " . mysqli_error($db));
        }

        // Adatok másolása
        $sql = "INSERT INTO `$backupTable` SELECT * FROM `telemetry`";
        if (!mysqli_query($db, $sql)) {
            throw new Exception("INSERT INTO backup hiba: " . mysqli_error($db));
        }

        // Eredeti tábla ürítése
        $sql = "TRUNCATE TABLE `telemetry`";
        if (!mysqli_query($db, $sql)) {
            throw new Exception("TRUNCATE hiba: " . mysqli_error($db));
        }

        mysqli_commit($db);
        $msg = "Sikeres mentés és ürítés. Létrejött: " . $backupTable;
    } catch (Exception $e) {
        mysqli_rollback($db);
        $err = $e->getMessage();
    }
}
?>
<!DOCTYPE html>
<html lang="hu">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>CanSat Control Panel</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 760px;
            margin: 30px auto;
            padding: 0 16px;
            background: #f6f6f6;
            color: #222;
        }
        .card {
            background: #fff;
            border: 1px solid #ccc;
            border-radius: 12px;
            padding: 18px;
            margin-bottom: 18px;
        }
        h1 {
            margin-top: 0;
        }
        a.linkbtn {
            display: block;
            margin: 10px 0;
            padding: 12px 14px;
            background: #e9eef7;
            border: 1px solid #b8c7e6;
            border-radius: 8px;
            text-decoration: none;
            color: #123;
            font-weight: bold;
        }
        a.linkbtn:hover {
            background: #dfe8f7;
        }
        button {
            padding: 12px 16px;
            font-size: 16px;
            border-radius: 8px;
            border: 1px solid #a33;
            background: #c62828;
            color: white;
            cursor: pointer;
        }
        button:hover {
            background: #b71c1c;
        }
        .msg {
            margin-top: 12px;
            padding: 10px 12px;
            border-radius: 8px;
            background: #e8f5e9;
            border: 1px solid #81c784;
            color: #1b5e20;
        }
        .err {
            margin-top: 12px;
            padding: 10px 12px;
            border-radius: 8px;
            background: #ffebee;
            border: 1px solid #e57373;
            color: #b71c1c;
        }
        .small {
            color: #666;
            font-size: 14px;
        }
    </style>
</head>
<body>

    <div class="card">
        <h1>CanSat vezérlőfelület</h1>
        <div class="small">Gyors elérés a nézetekhez és a telemetry tábla kezelése.</div>
    </div>

    <div class="card">
        <h2>Nézetek</h2>
        <a class="linkbtn" href="gps_map.php">GPS térkép</a>
        <a class="linkbtn" href="telemetry_live.php">Élő telemetria</a>
    </div>

    <div class="card">
        <h2>Adatbázis művelet</h2>
        <p>
            A gomb megnyomásakor először készül egy teljes másolat a
            <strong>telemetry</strong> tábláról, pl. <strong>telemetry01</strong>,
            majd ezután történik a <strong>TRUNCATE</strong>.
        </p>

        <form method="post" onsubmit="return confirm('Biztosan mentsük és ürítsük a telemetry táblát?');">
            <button type="submit" name="backup_truncate" value="1">
                Backup + TRUNCATE telemetry
            </button>
        </form>

        <?php if ($msg !== ""): ?>
            <div class="msg"><?php echo htmlspecialchars($msg, ENT_QUOTES, 'UTF-8'); ?></div>
        <?php endif; ?>

        <?php if ($err !== ""): ?>
            <div class="err"><?php echo htmlspecialchars($err, ENT_QUOTES, 'UTF-8'); ?></div>
        <?php endif; ?>
    </div>

</body>
</html>