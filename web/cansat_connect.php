<?php
// cansat_connect.php

function ConnMe(){
    $Host = "localhost";
    $UsrName = "root";
    $DbPas = "";
    $DbName = "slinky";
	
	
    $db = mysqli_connect($Host, $UsrName, $DbPas, $DbName);

    if (!$db) {
        die("MySQL connection failed: " . mysqli_connect_error());
    }

    if (!mysqli_set_charset($db, "utf8mb4")) {
        die("Charset error: " . mysqli_error($db));
    }

    return $db;
}
?>






