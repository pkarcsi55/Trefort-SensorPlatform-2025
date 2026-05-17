namespace CANSATCONSOL
{
    using System;
    using System.Collections.Generic;
    using System.Globalization;
    using System.IO;
    using System.IO.Ports;
    using System.Linq;
    using System.Text;
    using System.Text.RegularExpressions;
    using System.Threading;

    internal class Program
    {
        private static SerialPort? _sp;
        private static volatile bool _running = true;
        private static volatile bool _exclusiveMode = false;
        private static readonly object _consoleLock = new object();
        private static readonly object _serialLock = new object();

        private static readonly List<SdFileEntry> _lastSdList = new List<SdFileEntry>();
        private static string _downloadsDir = "";

        static void Main(string[] args)
        {
            Console.OutputEncoding = Encoding.UTF8;
            Console.Title = "CanSat Ground Console";

            _downloadsDir = Path.Combine(AppContext.BaseDirectory, "downloads");
            Directory.CreateDirectory(_downloadsDir);

            PrintBanner();

            string[] ports = SerialPort.GetPortNames();
            Array.Sort(ports);

            if (ports.Length == 0)
            {
                Console.WriteLine("Nem találtam soros portot.");
                return;
            }

            Console.WriteLine("Elérhető portok:");
            for (int i = 0; i < ports.Length; i++)
            {
                Console.WriteLine($"  {i + 1}. {ports[i]}");
            }

            Console.WriteLine();
            Console.Write("Válassz portot szám alapján vagy írd be közvetlenül (pl. COM5): ");
            string? input = Console.ReadLine()?.Trim();

            string portName = ResolvePortName(input, ports);
            if (string.IsNullOrWhiteSpace(portName))
            {
                Console.WriteLine("Érvénytelen port.");
                return;
            }

            _sp = new SerialPort(portName, 115200)
            {
                NewLine = "\n",
                ReadTimeout = 250,
                WriteTimeout = 1000,
                DtrEnable = true,
                RtsEnable = true,
                Encoding = Encoding.ASCII
            };

            try
            {
                Console.WriteLine();
                Console.WriteLine($"Port nyitása: {portName}");
                Console.WriteLine("A port megnyitása resetelheti az ESP32-t...");
                _sp.Open();
            }
            catch (Exception ex)
            {
                Console.WriteLine("Nem sikerült megnyitni a portot:");
                Console.WriteLine(ex.Message);
                return;
            }

            Console.WriteLine("Port megnyitva.");
            Console.WriteLine($"Letöltési mappa: {_downloadsDir}");
            Console.WriteLine();

            BootHandshake(_sp);

            Console.WriteLine();
            Console.WriteLine("=== Interaktív mód ===");
            Console.WriteLine("Normál parancsok közvetlenül a CanSat-nek mennek.");
            Console.WriteLine("Helyi parancsok:");
            Console.WriteLine("  @help              helyi súgó");
            Console.WriteLine("  @ls                SD fájllista lekérése és számozása");
            Console.WriteLine("  @dump <név|sorszám> dump letöltése a downloads mappába");
            Console.WriteLine("  @dir               downloads mappa megnyitása / kiírása");
            Console.WriteLine("  exit               kilépés");
            Console.WriteLine();
            Console.WriteLine("Tipp: előbb lépj SD service módba a 4-es menüponttal, utána használd az @ls és @dump parancsokat.");
            Console.WriteLine();

            Thread rxThread = new Thread(ReadLoop)
            {
                IsBackground = true,
                Name = "SerialRx"
            };
            rxThread.Start();

            while (_running)
            {
                Console.Write("CANSAT> ");
                string? line = Console.ReadLine();
                if (line == null)
                    continue;

                line = line.Trim();
                if (line.Length == 0)
                    continue;

                if (line.Equals("exit", StringComparison.OrdinalIgnoreCase))
                {
                    _running = false;
                    break;
                }

                if (line.Equals("@help", StringComparison.OrdinalIgnoreCase))
                {
                    PrintLocalHelp();
                    continue;
                }

                if (line.Equals("@dir", StringComparison.OrdinalIgnoreCase))
                {
                    Console.WriteLine($"Letöltési mappa: {_downloadsDir}");
                    continue;
                }

                if (line.Equals("@ls", StringComparison.OrdinalIgnoreCase))
                {
                    try
                    {
                        QuerySdList();
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("LS hiba: " + ex.Message);
                    }
                    continue;
                }

                if (line.StartsWith("@dump ", StringComparison.OrdinalIgnoreCase))
                {
                    string target = line.Substring(6).Trim();
                    if (target.Length == 0)
                    {
                        Console.WriteLine("Hiányzó fájlnév vagy sorszám.");
                        continue;
                    }

                    try
                    {
                        DumpByNameOrIndex(target);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Dump hiba: " + ex.Message);
                    }
                    continue;
                }

                try
                {
                    lock (_serialLock)
                    {
                        _sp!.Write(line + "\n");
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Írási hiba: " + ex.Message);
                }
            }

            try
            {
                if (_sp != null && _sp.IsOpen)
                    _sp.Close();
            }
            catch
            {
            }

            Console.WriteLine("Program vége.");
        }

        private static void PrintBanner()
        {
            Console.WriteLine("====================================");
            Console.WriteLine("   CanSat Ground Console v1.0");
            Console.WriteLine("====================================");
            Console.WriteLine();
        }

        private static void PrintLocalHelp()
        {
            Console.WriteLine();
            Console.WriteLine("Helyi parancsok:");
            Console.WriteLine("  @ls");
            Console.WriteLine("      SD service módban lekéri a fájllistát és megszámozza.");
            Console.WriteLine("  @dump M1_0036.csv");
            Console.WriteLine("  @dump 12");
            Console.WriteLine("      Letölti a kiválasztott fájlt a downloads mappába.");
            Console.WriteLine("  @dir");
            Console.WriteLine("      Kiírja a letöltési mappát.");
            Console.WriteLine();
        }

        private static string ResolvePortName(string? input, string[] ports)
        {
            if (string.IsNullOrWhiteSpace(input))
                return "";

            if (int.TryParse(input, out int index) && index >= 1 && index <= ports.Length)
                return ports[index - 1];

            return input.ToUpperInvariant();
        }

        private static void BootHandshake(SerialPort sp)
        {
            Console.WriteLine("=== Boot figyelés ===");
            Console.WriteLine("Ha meglátom a 'Press ENTER' promptot, automatikusan küldök Entert.");
            Console.WriteLine();

            StringBuilder sb = new StringBuilder();
            DateTime end = DateTime.UtcNow.AddSeconds(8);
            bool enterSent = false;

            lock (_serialLock)
            {
                while (DateTime.UtcNow < end)
                {
                    string chunk = sp.ReadExisting();
                    if (!string.IsNullOrEmpty(chunk))
                    {
                        Console.Write(chunk);
                        sb.Append(chunk);
                        string all = sb.ToString();

                        if (!enterSent && all.IndexOf("Press ENTER", StringComparison.OrdinalIgnoreCase) >= 0)
                        {
                            Thread.Sleep(150);
                            sp.Write("\n");
                            enterSent = true;
                            Console.WriteLine();
                            Console.WriteLine("[AUTO] ENTER elküldve a setup menühöz.");
                        }

                        bool inConfigMenu =
                            all.IndexOf("CONFIGURATION MENU", StringComparison.OrdinalIgnoreCase) >= 0 ||
                            all.IndexOf("Choice:", StringComparison.OrdinalIgnoreCase) >= 0;

                        if (inConfigMenu)
                        {
                            Console.WriteLine();
                            Console.WriteLine("[AUTO] Konfigurációs menü felismerve.");
                            return;
                        }
                    }

                    Thread.Sleep(20);
                }
            }

            Console.WriteLine();
            Console.WriteLine("=== Boot figyelés vége ===");
        }

        private static void ReadLoop()
        {
            if (_sp == null)
                return;

            while (_running)
            {
                try
                {
                    if (_exclusiveMode)
                    {
                        Thread.Sleep(20);
                        continue;
                    }

                    string chunk;
                    lock (_serialLock)
                    {
                        chunk = _sp.ReadExisting();
                    }

                    if (!string.IsNullOrEmpty(chunk))
                    {
                        lock (_consoleLock)
                        {
                            Console.Write(chunk);
                        }
                    }
                }
                catch
                {
                }

                Thread.Sleep(20);
            }
        }

        private static void QuerySdList()
        {
            if (_sp == null)
                throw new InvalidOperationException("A soros port nincs nyitva.");

            Console.WriteLine();
            Console.WriteLine("[LOCAL] SD fájllista lekérése...");

            _exclusiveMode = true;
            try
            {
                lock (_serialLock)
                {
                    _sp.DiscardInBuffer();
                    _sp.Write("LS\n");

                    List<string> lines = ReadLinesUntil(_sp, "LS END", TimeSpan.FromSeconds(6));
                    ParseAndStoreLs(lines);
                }
            }
            finally
            {
                _exclusiveMode = false;
            }
        }

        private static void ParseAndStoreLs(List<string> lines)
        {
            _lastSdList.Clear();

            bool inside = false;
            foreach (string raw in lines)
            {
                string line = raw.Trim();
                if (line.Length == 0)
                    continue;

                if (line.Equals("LS BEGIN", StringComparison.OrdinalIgnoreCase))
                {
                    inside = true;
                    continue;
                }

                if (line.Equals("LS END", StringComparison.OrdinalIgnoreCase))
                    break;

                if (!inside)
                    continue;

                string[] parts = line.Split(',');
                if (parts.Length != 2)
                    continue;

                string name = parts[0].Trim();
                if (!long.TryParse(parts[1].Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out long size))
                    size = -1;

                _lastSdList.Add(new SdFileEntry(name, size));
            }

            if (_lastSdList.Count == 0)
            {
                Console.WriteLine("Nem sikerült értelmezhető LS listát kapni.");
                return;
            }

            Console.WriteLine();
            Console.WriteLine("SD fájlok:");
            for (int i = 0; i < _lastSdList.Count; i++)
            {
                var f = _lastSdList[i];
                Console.WriteLine($"  {i + 1,2}. {f.Name,-14} {f.Size,8} B");
            }
            Console.WriteLine();
        }

        private static void DumpByNameOrIndex(string token)
        {
            string remoteFile;

            if (int.TryParse(token, out int idx))
            {
                if (_lastSdList.Count == 0)
                    throw new InvalidOperationException("Nincs gyorslista. Előbb futtasd: @ls");

                if (idx < 1 || idx > _lastSdList.Count)
                    throw new InvalidOperationException("A megadott sorszám nincs a listában.");

                remoteFile = _lastSdList[idx - 1].Name;
            }
            else
            {
                remoteFile = token;
            }

            DumpFile(remoteFile);
        }

        private static void DumpFile(string remoteFile)
        {
            if (_sp == null)
                throw new InvalidOperationException("A soros port nincs nyitva.");

            string localName = Path.GetFileName(remoteFile);
            if (string.IsNullOrWhiteSpace(localName))
                throw new InvalidOperationException("Érvénytelen fájlnév.");

            string defaultPath = Path.Combine(_downloadsDir, localName);
            string localPath = MakeUniquePath(defaultPath);

            Console.WriteLine();
            Console.WriteLine($"[LOCAL] Dump indítása: {remoteFile}");
            Console.WriteLine($"[LOCAL] Mentés ide: {localPath}");

            _exclusiveMode = true;
            try
            {
                lock (_serialLock)
                {
                    _sp.DiscardInBuffer();
                    _sp.Write($"DUMP {remoteFile}\n");

                    string beginLine = ReadLineWithTimeout(_sp, TimeSpan.FromSeconds(4));
                    while (string.IsNullOrWhiteSpace(beginLine))
                        beginLine = ReadLineWithTimeout(_sp, TimeSpan.FromSeconds(2));

                    var m = Regex.Match(beginLine.Trim(), @"^DUMP BEGIN\s+/(.+?)\s+(\d+)$", RegexOptions.IgnoreCase);
                    if (!m.Success)
                        throw new InvalidOperationException("Nem jött érvényes DUMP BEGIN fejléc. Kapott sor: " + beginLine.Trim());

                    long size = long.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture);
                    Console.WriteLine($"[LOCAL] Méret: {size} byte");

                    using FileStream fs = new FileStream(localPath, FileMode.CreateNew, FileAccess.Write, FileShare.None);
                    CopyExactBytes(_sp, fs, size);
                    fs.Flush();

                    string tail = ReadLineWithTimeout(_sp, TimeSpan.FromSeconds(3)).Trim();
                    if (tail.Length == 0)
                        tail = ReadLineWithTimeout(_sp, TimeSpan.FromSeconds(3)).Trim();

                    if (!tail.Equals("DUMP END", StringComparison.OrdinalIgnoreCase))
                        Console.WriteLine($"[LOCAL] Figyelem: várt 'DUMP END' helyett ez jött: {tail}");

                    Console.WriteLine($"[LOCAL] Kész: {localPath}");
                }
            }
            finally
            {
                _exclusiveMode = false;
                Console.WriteLine();
            }
        }

        private static void CopyExactBytes(SerialPort sp, Stream output, long totalBytes)
        {
            byte[] buffer = new byte[4096];
            long remaining = totalBytes;
            long done = 0;
            int lastPercent = -1;
            DateTime lastProgress = DateTime.MinValue;

            while (remaining > 0)
            {
                int toRead = (int)Math.Min(buffer.Length, remaining);
                int read = sp.Read(buffer, 0, toRead);
                if (read <= 0)
                    continue;

                output.Write(buffer, 0, read);
                remaining -= read;
                done += read;

                int percent = totalBytes > 0 ? (int)(done * 100 / totalBytes) : 100;
                if (percent != lastPercent && (percent == 100 || percent % 10 == 0 || (DateTime.UtcNow - lastProgress).TotalSeconds >= 1))
                {
                    Console.Write($"\r[LOCAL] Letöltés: {done}/{totalBytes} B ({percent}%)   ");
                    lastPercent = percent;
                    lastProgress = DateTime.UtcNow;
                }
            }

            Console.WriteLine();
        }

        private static List<string> ReadLinesUntil(SerialPort sp, string terminator, TimeSpan timeout)
        {
            List<string> lines = new List<string>();
            DateTime end = DateTime.UtcNow + timeout;

            while (DateTime.UtcNow < end)
            {
                string line = ReadLineWithTimeout(sp, TimeSpan.FromMilliseconds(700));
                if (line.Length == 0)
                    continue;

                lines.Add(line);
                if (line.Trim().Equals(terminator, StringComparison.OrdinalIgnoreCase))
                    return lines;
            }

            throw new TimeoutException($"Időtúllépés, nem érkezett meg a(z) '{terminator}' lezáró sor.");
        }

        private static string ReadLineWithTimeout(SerialPort sp, TimeSpan timeout)
        {
            DateTime end = DateTime.UtcNow + timeout;
            List<byte> bytes = new List<byte>(128);

            while (DateTime.UtcNow < end)
            {
                int value;
                try
                {
                    value = sp.ReadByte();
                }
                catch (TimeoutException)
                {
                    continue;
                }

                if (value < 0)
                    continue;

                byte b = (byte)value;
                if (b == (byte)'\n')
                    break;

                if (b != (byte)'\r')
                    bytes.Add(b);
            }

            return Encoding.ASCII.GetString(bytes.ToArray());
        }

        private static string MakeUniquePath(string path)
        {
            if (!File.Exists(path))
                return path;

            string dir = Path.GetDirectoryName(path) ?? AppContext.BaseDirectory;
            string stem = Path.GetFileNameWithoutExtension(path);
            string ext = Path.GetExtension(path);

            for (int i = 2; i < 1000; i++)
            {
                string candidate = Path.Combine(dir, $"{stem}_{i}{ext}");
                if (!File.Exists(candidate))
                    return candidate;
            }

            throw new IOException("Nem sikerült egyedi fájlnevet készíteni.");
        }

        private readonly record struct SdFileEntry(string Name, long Size);
    }

}
