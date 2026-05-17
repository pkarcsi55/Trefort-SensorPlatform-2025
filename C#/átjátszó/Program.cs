namespace atjatszo2026
{

    using Newtonsoft.Json;
    using System;
    using System.Collections.Concurrent;
    using System.Collections.Generic;
    using System.Globalization;
    using System.IO;
    using System.IO.Ports;
    using System.Linq;
    using System.Net.Http;
    using System.Text;
    using System.Threading;
    using System.Xml;

    internal static class Program
    {
        private static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        private static readonly HttpClient Http = new HttpClient();

        // ====== SETTINGS ======
        private const int BaudRate = 115200;
        private static readonly bool EnableHttpUpload = true;
        private static readonly TimeSpan HttpTimeout = TimeSpan.FromSeconds(4);
        private const int MaxUploadRetries = 1; // 1 extra retry after the first failed attempt
        private const int UploadRetryDelayMs = 250;
        private const int MaxUploadQueueLength = 1000;

        private static string IngestUrl = "https://www.trefort.elte.hu/cansat2026/cansat_ingest.php";
        private const string FlightId = "CANSAT2026_GS1";

        private static string _urlSourceHint = "default";
        private static volatile bool _stopRequested = false;

        private static readonly ConcurrentQueue<UploadJob> UploadQueue = new ConcurrentQueue<UploadJob>();
        private static readonly AutoResetEvent UploadSignal = new AutoResetEvent(false);

        private static readonly object ConsoleSync = new object();
        private static readonly object LogSync = new object();
        private static readonly object WebLogSync = new object();

        private static StreamWriter _ingestLogWriter;
        private static StreamWriter _webLogWriter;

        private static int _queueLength = 0;
        private static int _queueDropped = 0;
        private static int _uploadOk = 0;
        private static int _uploadFail = 0;
        private static int _uploadDuplicate = 0;
        private static int _parseFail = 0;
        private static int _linesSeen = 0;

        // ====== GPS CACHE (utolsó ismert GPS adatok) ======
        private static double? _lastGpsLat = null;
        private static double? _lastGpsLon = null;
        private static double? _lastGpsAlt = null;

        // ====== CURRENT 17-FIELD RX CSV FORMAT ======
        // PKT,MISSION,SEQ,MS,UTC,P_hPa,T_in_C,RH_pct,T_out_C,H_bme_m,LAT,LON,ALT_gps_m,BAT_V,BAT_PCT,RSSI,SNR
        private const int IDX_PKT = 0;
        private const int IDX_MISSION = 1;
        private const int IDX_SEQ = 2;
        private const int IDX_MS = 3;
        private const int IDX_UTC = 4;
        private const int IDX_P = 5;
        private const int IDX_TIN = 6;
        private const int IDX_RH = 7;
        private const int IDX_TOUT = 8;
        private const int IDX_H_BME = 9;
        private const int IDX_LAT = 10;
        private const int IDX_LON = 11;
        private const int IDX_ALT_GPS = 12;
        private const int IDX_BAT_V = 13;
        private const int IDX_BAT_PCT = 14;
        private const int IDX_RSSI = 15;
        private const int IDX_SNR = 16;

        private static void Main()
        {
            Http.Timeout = HttpTimeout;
            IngestUrl = LoadIngestUrlFromFile(IngestUrl);

            string portName = SelectPortInteractive();

            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CI);
            string webLogFile = "web_log_" + stamp + ".csv";
            string ingestLogFile = "ingest_log_" + stamp + ".txt";

            Console.WriteLine("Web log: " + webLogFile);
            Console.WriteLine("Ingest log: " + ingestLogFile);

            using (_webLogWriter = new StreamWriter(webLogFile, false, Encoding.UTF8))
            using (_ingestLogWriter = new StreamWriter(ingestLogFile, false, Encoding.UTF8))
            using (SerialPort sp = CreateSerialPort(portName))
            {
                _webLogWriter.AutoFlush = true;
                _ingestLogWriter.AutoFlush = true;

                WriteWebLogHeader();

                try
                {
                    sp.Open();
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Nem tudtam megnyitni: " + portName);
                    Console.WriteLine(ex.Message);
                    return;
                }

                Thread uploaderThread = null;
                if (EnableHttpUpload)
                {
                    uploaderThread = new Thread(UploadWorkerLoop);
                    uploaderThread.IsBackground = true;
                    uploaderThread.Name = "Uploader";
                    uploaderThread.Start();
                }

                PrintStartupInfo(portName, webLogFile, ingestLogFile);

                StringBuilder rxBuffer = new StringBuilder();
                DateTime lastStatsPrint = DateTime.Now;

                Console.CancelKeyPress += delegate (object sender, ConsoleCancelEventArgs e)
                {
                    e.Cancel = true;
                    _stopRequested = true;
                    UploadSignal.Set();
                    SafeClose(sp);
                };

                while (!_stopRequested && sp.IsOpen)
                {
                    try
                    {
                        string chunk = sp.ReadExisting();

                        if (string.IsNullOrEmpty(chunk))
                        {
                            PrintPeriodicStats(ref lastStatsPrint);
                            Thread.Sleep(10);
                            continue;
                        }

                        rxBuffer.Append(chunk);

                        string line;
                        while (TryExtractLine(rxBuffer, out line))
                        {
                            line = (line ?? string.Empty).Trim();
                            if (line.Length == 0)
                                continue;

                            ProcessReceivedLine(line);
                            PrintPeriodicStats(ref lastStatsPrint);
                        }
                    }
                    catch
                    {
                        PrintPeriodicStats(ref lastStatsPrint);
                        Thread.Sleep(20);
                    }
                }

                _stopRequested = true;
                UploadSignal.Set();

                if (uploaderThread != null && !uploaderThread.Join(3000))
                    SafeConsoleWriteLine("[WARN] Az uploader szál nem állt le 3 másodpercen belül.");

                PrintFinalStats();
            }
        }

        private static void ProcessReceivedLine(string line)
        {
            if (!LooksLikeTelemetryLine(line))
            {
                SafeConsoleWriteLine(line);
                return;
            }

            TelemetryV1 telemetry;
            if (!TryParseTelemetry(line, out telemetry))
            {
                Interlocked.Increment(ref _parseFail);
                SafeConsoleWriteLine(line + " | PARSEFAIL");
                return;
            }

            // ====== GPS CACHE LOGIKA ======
            if (telemetry.gps_lat.HasValue && telemetry.gps_lon.HasValue)
            {
                _lastGpsLat = telemetry.gps_lat;
                _lastGpsLon = telemetry.gps_lon;
                _lastGpsAlt = telemetry.gps_alt_m;
            }
            else
            {
                telemetry.gps_lat = _lastGpsLat;
                telemetry.gps_lon = _lastGpsLon;
                telemetry.gps_alt_m = _lastGpsAlt;
            }

            Interlocked.Increment(ref _linesSeen);
            WriteWebLogLine(telemetry);

            string pretty = PrettyTelemetry(telemetry);

            if (!EnableHttpUpload)
            {
                SafeConsoleWriteLine(pretty + " | OK OFF");
                return;
            }

            bool enqueued = TryEnqueueUpload(telemetry, pretty);
            if (!enqueued)
                SafeConsoleWriteLine(pretty + " | DROP QFULL");
        }

        private static bool TryEnqueueUpload(TelemetryV1 telemetry, string pretty)
        {
            int newLen = Interlocked.Increment(ref _queueLength);
            if (newLen > MaxUploadQueueLength)
            {
                Interlocked.Decrement(ref _queueLength);
                Interlocked.Increment(ref _queueDropped);
                LogIngest("DROP", telemetry.seq, 0, "Upload queue full");
                return false;
            }

            UploadQueue.Enqueue(new UploadJob
            {
                Telemetry = telemetry,
                PrettyLine = pretty
            });

            UploadSignal.Set();
            return true;
        }

        private static void UploadWorkerLoop()
        {
            while (!_stopRequested || !UploadQueue.IsEmpty)
            {
                UploadJob job;
                if (!UploadQueue.TryDequeue(out job))
                {
                    UploadSignal.WaitOne(250);
                    continue;
                }

                Interlocked.Decrement(ref _queueLength);

                UploadResult result = SendWithRetry(job.Telemetry);
                job.UploadResult = result;
                UpdatePendingStatus(job);

                if (result.Success)
                {
                    Interlocked.Increment(ref _uploadOk);
                    if (result.Duplicate)
                        Interlocked.Increment(ref _uploadDuplicate);
                }
                else
                {
                    Interlocked.Increment(ref _uploadFail);
                }
            }
        }

        private static UploadResult SendWithRetry(TelemetryV1 telemetry)
        {
            UploadResult last = null;

            for (int attempt = 1; attempt <= MaxUploadRetries + 1; attempt++)
            {
                UploadResult current = TryPostTelemetry(telemetry, attempt);
                last = current;

                if (current.Success)
                    return current;

                if (attempt <= MaxUploadRetries)
                    Thread.Sleep(UploadRetryDelayMs);
            }

            return last ?? new UploadResult
            {
                Success = false,
                StatusCode = 0,
                ConsoleTag = "FAIL 0 | unknown"
            };
        }

        private static UploadResult TryPostTelemetry(TelemetryV1 t, int attempt)
        {
            string json = string.Empty;
            try
            {
                json = JsonConvert.SerializeObject(
                    t,
                   Newtonsoft.Json.Formatting.None,
                    new JsonSerializerSettings
                    {
                        NullValueHandling = NullValueHandling.Include
                    });

                using (StringContent content = new StringContent(json, Encoding.UTF8, "application/json"))
                using (HttpResponseMessage response = Http.PostAsync(IngestUrl, content).Result)
                {
                    int statusCode = (int)response.StatusCode;
                    string serverReply = response.Content.ReadAsStringAsync().Result ?? string.Empty;
                    string replyOneLine = NormalizeReply(serverReply);

                    bool isDuplicate = replyOneLine.IndexOf("\"duplicate\":true", StringComparison.OrdinalIgnoreCase) >= 0;
                    bool responseOk = response.IsSuccessStatusCode &&
                                      replyOneLine.IndexOf("\"ok\":false", StringComparison.OrdinalIgnoreCase) < 0;

                    if (responseOk)
                    {
                        string tag = isDuplicate
                            ? "OK DUP " + statusCode + " a" + attempt.ToString(CI)
                            : "OK " + statusCode + " a" + attempt.ToString(CI);

                        LogIngest("OK", t.seq, statusCode, replyOneLine, attempt, isDuplicate);
                        return new UploadResult
                        {
                            Success = true,
                            Duplicate = isDuplicate,
                            StatusCode = statusCode,
                            ServerReply = serverReply,
                            ConsoleTag = tag
                        };
                    }

                    LogIngest("FAIL", t.seq, statusCode, replyOneLine, attempt, false);
                    return new UploadResult
                    {
                        Success = false,
                        Duplicate = false,
                        StatusCode = statusCode,
                        ServerReply = serverReply,
                        ConsoleTag = "FAIL " + statusCode.ToString(CI) + " a" + attempt.ToString(CI) +
                                     (replyOneLine.Length > 0 ? " | " + replyOneLine : string.Empty)
                    };
                }
            }
            catch (Exception ex)
            {
                string msg = ex.GetType().Name + ": " + ex.Message;
                LogIngest("EX", t.seq, 0, msg, attempt, false);
                return new UploadResult
                {
                    Success = false,
                    Duplicate = false,
                    StatusCode = 0,
                    ServerReply = msg,
                    ConsoleTag = "EX a" + attempt.ToString(CI) + " | " + msg
                };
            }
        }

        private static void WriteWebLogHeader()
        {
            string header =
                "flight_id,pkt,mission,seq,ms,ts_utc,tx_utc,pressure_hpa,temp_in_c,rh_pct,temp_out_c,h_bme_m,gps_lat,gps_lon,gps_alt_m,ubat_v,ubat_percent,rssi_dbm,snr_dbm,raw_line";
            lock (WebLogSync)
            {
                _webLogWriter.WriteLine(header);
            }
        }

        private static void WriteWebLogLine(TelemetryV1 t)
        {
            string line = string.Join(",", new string[]
            {
            CsvField(t.flight_id),
            CsvField(t.pkt),
            CsvField(t.mission),
            CsvField(t.seq.ToString(CI)),
            CsvField(NullableLongToString(t.ms)),
            CsvField(t.ts_utc),
            CsvField(t.tx_utc),
            CsvField(NullableDoubleToString(t.pressure_hpa, "0.0")),
            CsvField(NullableDoubleToString(t.temp_in_c, "0.0")),
            CsvField(NullableDoubleToString(t.rh_pct, "0.0")),
            CsvField(NullableDoubleToString(t.temp_out_c, "0.0")),
            CsvField(NullableDoubleToString(t.h_bme_m, "0.0")),
            CsvField(NullableDoubleToString(t.gps_lat, "0.0000")),
            CsvField(NullableDoubleToString(t.gps_lon, "0.0000")),
            CsvField(NullableDoubleToString(t.gps_alt_m, "0.0")),
            CsvField(NullableDoubleToString(t.ubat_v, "0.00")),
            CsvField(NullableIntToString(t.ubat_percent)),
            CsvField(NullableIntToString(t.rssi_dbm)),
            CsvField(NullableDoubleToString(t.snr_db, "0.0")),
            CsvField(t.raw_line)
            });

            lock (WebLogSync)
            {
                _webLogWriter.WriteLine(line);
            }
        }

        private static string CsvField(string s)
        {
            s = s ?? string.Empty;
            if (s.IndexOfAny(new[] { ',', '"', '\r', '\n' }) >= 0)
                return "\"" + s.Replace("\"", "\"\"") + "\"";
            return s;
        }

        private static string NullableDoubleToString(double? value, string fmt)
        {
            return value.HasValue ? value.Value.ToString(fmt, CI) : string.Empty;
        }

        private static string NullableIntToString(int? value)
        {
            return value.HasValue ? value.Value.ToString(CI) : string.Empty;
        }

        private static string NullableLongToString(long? value)
        {
            return value.HasValue ? value.Value.ToString(CI) : string.Empty;
        }

        private static void LogIngest(string kind, int seq, int statusCode, string message)
        {
            LogIngest(kind, seq, statusCode, message, 0, false);
        }

        private static void LogIngest(string kind, int seq, int statusCode, string message, int attempt, bool duplicate)
        {
            if (kind == "OK")
                return;

            string line = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff", CI) +
                          " | " + kind +
                          " | seq=" + seq.ToString(CI) +
                          " | http=" + statusCode.ToString(CI) +
                          " | try=" + attempt.ToString(CI) +
                          " | dup=" + (duplicate ? "1" : "0") +
                          " | " + NormalizeReply(message);

            lock (LogSync)
            {
                if (_ingestLogWriter != null)
                    _ingestLogWriter.WriteLine(line);
            }
        }

        private static string NormalizeReply(string s)
        {
            s = (s ?? string.Empty).Trim().Replace("\r", " ").Replace("\n", " ");
            while (s.Contains("  "))
                s = s.Replace("  ", " ");

            if (s.Length > 220)
                s = s.Substring(0, 220) + "...";

            return s;
        }

        private static void PrintPeriodicStats(ref DateTime lastStatsPrint)
        {
            DateTime now = DateTime.Now;
            if (now - lastStatsPrint < TimeSpan.FromSeconds(10))
                return;

            lastStatsPrint = now;

            string line = "[STAT] RX=" + Volatile.Read(ref _linesSeen).ToString(CI) +
                          " | OK=" + Volatile.Read(ref _uploadOk).ToString(CI) +
                          " | DUP=" + Volatile.Read(ref _uploadDuplicate).ToString(CI) +
                          " | FAIL=" + Volatile.Read(ref _uploadFail).ToString(CI) +
                          " | PARSEFAIL=" + Volatile.Read(ref _parseFail).ToString(CI) +
                          " | Q=" + Volatile.Read(ref _queueLength).ToString(CI) +
                          " | DROP=" + Volatile.Read(ref _queueDropped).ToString(CI);

            SafeConsoleWriteLine(line);
        }

        private static void PrintFinalStats()
        {
            SafeConsoleWriteLine("--------------------------------------------------");
            SafeConsoleWriteLine("Leállás. Összesítés:");
            SafeConsoleWriteLine("RX=" + Volatile.Read(ref _linesSeen).ToString(CI) +
                                 " | OK=" + Volatile.Read(ref _uploadOk).ToString(CI) +
                                 " | DUP=" + Volatile.Read(ref _uploadDuplicate).ToString(CI) +
                                 " | FAIL=" + Volatile.Read(ref _uploadFail).ToString(CI) +
                                 " | PARSEFAIL=" + Volatile.Read(ref _parseFail).ToString(CI) +
                                 " | DROP=" + Volatile.Read(ref _queueDropped).ToString(CI));
        }

        private static void SafeConsoleWriteLine(string text)
        {
            lock (ConsoleSync)
            {
                Console.WriteLine(text);
            }
        }

        private static SerialPort CreateSerialPort(string portName)
        {
            SerialPort sp = new SerialPort(portName, BaudRate);
            sp.ReadTimeout = 200;
            sp.DtrEnable = false;
            sp.RtsEnable = false;
            return sp;
        }

        private static void PrintStartupInfo(string portName, string webLogFile, string ingestLogFile)
        {
            Console.WriteLine("Kapcsolódva: " + portName + " @ " + BaudRate);
            Console.WriteLine("WEB log: " + webLogFile);
            Console.WriteLine("Ingest log: " + ingestLogFile);
            Console.WriteLine("HTTP upload: " + (EnableHttpUpload ? "ON (queue)" : "OFF"));
            Console.WriteLine("HTTP timeout: " + HttpTimeout.TotalSeconds.ToString("0.#", CI) + " s");
            Console.WriteLine("Retry: " + MaxUploadRetries.ToString(CI) + " extra próbálkozás");
            Console.WriteLine("Ingest URL: " + IngestUrl);
            Console.WriteLine("URL forrás: " + _urlSourceHint);
            Console.WriteLine("--------------------------------------------------");
        }

        private sealed class TelemetryV1
        {
            public string flight_id { get; set; }
            public string pkt { get; set; }
            public string mission { get; set; }
            public int seq { get; set; }
            public long? ms { get; set; }
            public string ts_utc { get; set; }
            public string tx_utc { get; set; }
            public double? pressure_hpa { get; set; }
            public double? temp_in_c { get; set; }
            public double? rh_pct { get; set; }
            public double? temp_out_c { get; set; }
            public double? h_bme_m { get; set; }
            public double? gps_lat { get; set; }
            public double? gps_lon { get; set; }
            public double? gps_alt_m { get; set; }
            public double? ubat_v { get; set; }
            public int? ubat_percent { get; set; }
            public int? rssi_dbm { get; set; }
            public double? snr_db { get; set; }
            public string raw_line { get; set; }
        }

        private sealed class UploadJob
        {
            public TelemetryV1 Telemetry { get; set; }
            public string PrettyLine { get; set; }
            public UploadResult UploadResult { get; set; }
        }

        private sealed class UploadResult
        {
            public bool Success { get; set; }
            public bool Duplicate { get; set; }
            public int StatusCode { get; set; }
            public string ServerReply { get; set; }
            public string ConsoleTag { get; set; }
        }

        private static bool TryExtractLine(StringBuilder sb, out string line)
        {
            for (int i = 0; i < sb.Length; i++)
            {
                char c = sb[i];
                if (c == '\n' || c == '\r')
                {
                    line = sb.ToString(0, i);

                    int j = i;
                    while (j < sb.Length && (sb[j] == '\n' || sb[j] == '\r'))
                        j++;

                    sb.Remove(0, j);
                    return true;
                }
            }

            line = string.Empty;
            return false;
        }

        private static bool LooksLikeTelemetryLine(string rawLine)
        {
            if (string.IsNullOrWhiteSpace(rawLine))
                return false;

            string[] f = rawLine.Split(',').Select(x => (x ?? string.Empty).Trim()).ToArray();
            if (f.Length < 17)
                return false;

            string pkt = SafeField(f, IDX_PKT, "");
            string mission = SafeField(f, IDX_MISSION, "");

            int seq;
            if (!pkt.StartsWith("TX", StringComparison.OrdinalIgnoreCase))
                return false;

            if (!mission.StartsWith("M", StringComparison.OrdinalIgnoreCase))
                return false;

            if (!int.TryParse(SafeField(f, IDX_SEQ), NumberStyles.Integer, CI, out seq))
                return false;

            return true;
        }

        private static double? ParseDoubleOrNull(string s)
        {
            if (string.IsNullOrWhiteSpace(s))
                return null;

            s = s.Trim();
            if (s.Equals("NA", StringComparison.OrdinalIgnoreCase))
                return null;

            double v;
            return double.TryParse(s, NumberStyles.Float, CI, out v) ? (double?)v : null;
        }

        private static int? ParseIntOrNull(string s)
        {
            if (string.IsNullOrWhiteSpace(s))
                return null;

            s = s.Trim();
            if (s.Equals("NA", StringComparison.OrdinalIgnoreCase))
                return null;

            int v;
            return int.TryParse(s, NumberStyles.Integer, CI, out v) ? (int?)v : null;
        }

        private static long? ParseLongOrNull(string s)
        {
            if (string.IsNullOrWhiteSpace(s))
                return null;

            s = s.Trim();
            if (s.Equals("NA", StringComparison.OrdinalIgnoreCase))
                return null;

            long v;
            return long.TryParse(s, NumberStyles.Integer, CI, out v) ? (long?)v : null;
        }

        private static string SafeField(string[] fields, int idx, string fallback = "NA")
        {
            if (fields == null || idx < 0 || idx >= fields.Length)
                return fallback;

            string s = (fields[idx] ?? string.Empty).Trim();
            return s.Length == 0 ? fallback : s;
        }

        private static bool TryParseTelemetry(string rawLine, out TelemetryV1 t)
        {
            t = null;

            string[] f = rawLine.Split(',').Select(x => (x ?? string.Empty).Trim()).ToArray();
            if (f.Length < 17)
                return false;

            string pkt = SafeField(f, IDX_PKT, "");
            string mission = SafeField(f, IDX_MISSION, "");

            if (!pkt.StartsWith("TX", StringComparison.OrdinalIgnoreCase))
                return false;

            if (!mission.StartsWith("M", StringComparison.OrdinalIgnoreCase))
                return false;

            int seq;
            if (!int.TryParse(SafeField(f, IDX_SEQ), NumberStyles.Integer, CI, out seq))
                return false;

            long? ms = ParseLongOrNull(SafeField(f, IDX_MS));

            t = new TelemetryV1
            {
                flight_id = FlightId,
                pkt = pkt,
                mission = mission,
                seq = seq,
                ms = ms,
                ts_utc = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss", CI),
                tx_utc = SafeField(f, IDX_UTC),
                pressure_hpa = ParseDoubleOrNull(SafeField(f, IDX_P)),
                temp_in_c = ParseDoubleOrNull(SafeField(f, IDX_TIN)),
                rh_pct = ParseDoubleOrNull(SafeField(f, IDX_RH)),
                temp_out_c = ParseDoubleOrNull(SafeField(f, IDX_TOUT)),
                h_bme_m = ParseDoubleOrNull(SafeField(f, IDX_H_BME)),
                gps_lat = ParseDoubleOrNull(SafeField(f, IDX_LAT)),
                gps_lon = ParseDoubleOrNull(SafeField(f, IDX_LON)),
                gps_alt_m = ParseDoubleOrNull(SafeField(f, IDX_ALT_GPS)),
                ubat_v = ParseDoubleOrNull(SafeField(f, IDX_BAT_V)),
                ubat_percent = ParseIntOrNull(SafeField(f, IDX_BAT_PCT)),
                rssi_dbm = ParseIntOrNull(SafeField(f, IDX_RSSI)),
                snr_db = ParseDoubleOrNull(SafeField(f, IDX_SNR)),
                raw_line = string.Join(",", f.Take(17).ToArray())
            };

            return true;
        }

        private static string PrettyTelemetry(TelemetryV1 t)
        {
            string timeLocal = DateTime.Now.ToString("HH:mm:ss", CI);
            string pktMission = (t.pkt ?? "TX?") + "," + (t.mission ?? "M?");
            string seq = "#" + t.seq.ToString(CI);

            string hbme = t.h_bme_m.HasValue ? "Hbme " + t.h_bme_m.Value.ToString("0.0", CI) + " m" : null;
            string gps = (t.gps_lat.HasValue && t.gps_lon.HasValue)
                ? "GPS " + t.gps_lat.Value.ToString("0.0000", CI) + "," + t.gps_lon.Value.ToString("0.0000", CI)
                : null;
            string gpsAlt = t.gps_alt_m.HasValue ? "ALT " + t.gps_alt_m.Value.ToString("0.0", CI) + " m" : null;
            string ubat = t.ubat_percent.HasValue || t.ubat_v.HasValue
                ? "UBAT " + (t.ubat_percent.HasValue ? t.ubat_percent.Value.ToString(CI) + " %" : "NA") +
                  (t.ubat_v.HasValue ? " (" + t.ubat_v.Value.ToString("0.00", CI) + " V)" : string.Empty)
                : null;
            string radio = (t.rssi_dbm.HasValue || t.snr_db.HasValue)
                ? "RSSI " + (t.rssi_dbm.HasValue ? t.rssi_dbm.Value.ToString(CI) : "NA") +
                  " | SNR " + (t.snr_db.HasValue ? t.snr_db.Value.ToString("0.0", CI) : "NA")
                : null;

            List<string> parts = new List<string>();
            parts.Add(timeLocal);
            parts.Add(pktMission);
            parts.Add(seq);
            if (hbme != null) parts.Add(hbme);
            if (gps != null) parts.Add(gps);
            if (gpsAlt != null) parts.Add(gpsAlt);
            if (ubat != null) parts.Add(ubat);
            if (radio != null) parts.Add(radio);

            return string.Join(" | ", parts.ToArray());
        }

        private static void UpdatePendingStatus(UploadJob job)
        {
            string line = job.PrettyLine + " | " + job.UploadResult.ConsoleTag;
            if (!job.UploadResult.Success || Volatile.Read(ref _queueLength) > 0)
                line += " | Q=" + Volatile.Read(ref _queueLength).ToString(CI);

            SafeConsoleWriteLine(line);
        }

        private static string LoadIngestUrlFromFile(string fallback)
        {
            try
            {
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string path = Path.Combine(baseDir, "url.txt");

                if (!File.Exists(path))
                    return fallback;

                foreach (string raw in File.ReadAllLines(path))
                {
                    string line = (raw ?? string.Empty).Trim();
                    if (line.Length == 0 || line.StartsWith("#"))
                        continue;

                    Uri uri;
                    if (Uri.TryCreate(line, UriKind.Absolute, out uri) &&
                        (uri.Scheme == Uri.UriSchemeHttp || uri.Scheme == Uri.UriSchemeHttps))
                    {
                        _urlSourceHint = "url.txt";
                        return line;
                    }

                    break;
                }
            }
            catch
            {
            }

            return fallback;
        }

        private static string SelectPortInteractive()
        {
            string[] ports = SerialPort.GetPortNames().OrderBy(p => p).ToArray();

            if (ports.Length == 0)
            {
                Console.WriteLine("Nincs COM port.");
                Environment.Exit(1);
            }

            Console.WriteLine("Elérhető COM portok:");
            for (int i = 0; i < ports.Length; i++)
                Console.WriteLine((i + 1).ToString(CI) + ". " + ports[i]);

            while (true)
            {
                Console.Write("Válassz (1..N vagy COMxx): ");
                string input = (Console.ReadLine() ?? string.Empty).Trim();

                int n;
                if (int.TryParse(input, out n))
                {
                    if (n >= 1 && n <= ports.Length)
                        return ports[n - 1];

                    string comFromNumber = "COM" + n.ToString(CI);
                    if (ports.Contains(comFromNumber))
                        return comFromNumber;
                }

                if (!input.StartsWith("COM", StringComparison.OrdinalIgnoreCase))
                    input = "COM" + input;

                input = input.ToUpperInvariant();

                if (ports.Contains(input))
                    return input;

                Console.WriteLine("Érvénytelen választás.");
            }
        }

        private static void SafeClose(IDisposable disposable)
        {
            try
            {
                if (disposable != null)
                    disposable.Dispose();
            }
            catch
            {
            }
        }
    }

}