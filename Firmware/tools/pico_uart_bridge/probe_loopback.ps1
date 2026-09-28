param(
    [Parameter(Mandatory=$true)][string]$Port,
    [switch]$LoopbackConfirmed,
    [switch]$Quick
)
$ErrorActionPreference = 'Stop'
if (!$LoopbackConfirmed) { throw 'Disconnect motor and jumper GP0 to GP1, then supply -LoopbackConfirmed.' }
$device = Get-CimInstance Win32_PnPEntity | Where-Object {
    $_.Name -match ('\(' + [regex]::Escape($Port) + '\)$')
}
if (@($device).Count -ne 1 -or $device.DeviceID -notlike 'USB\VID_CAFE&PID_4001*') {
    throw 'Selected port is not the Pico PIO UART Bridge. No port was opened.'
}
Add-Type -TypeDefinition @'
using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading;
public static class BridgeLoopback {
    public static void Run(string name, bool quick) {
        using (var port = new SerialPort(name, 115200, Parity.None, 8, StopBits.One)) {
            port.Handshake = Handshake.None;
            port.DtrEnable = false; port.RtsEnable = false;
            port.ReadTimeout = 500; port.WriteTimeout = 30000;
            port.ReadBufferSize = 65536; port.WriteBufferSize = 65536;
            port.Open(); port.DiscardInBuffer();
            int[] bauds = {115200, 921600, 115200, 921600, 921600};
            bool[] dtrs = {false, false, false, true, false};
            for (int test = 0; test < bauds.Length; ++test) {
                int size = quick ? 1024 : (bauds[test] == 921600 ? 1048576 : 32768);
                byte[] sent = new byte[size], received = new byte[size];
                new Random(20260928 + test).NextBytes(sent);
                for (int i = 0; i < Math.Min(size, 256); ++i) sent[i] = (byte)i;
                port.DtrEnable = dtrs[test]; port.BaudRate = bauds[test];
                int count = 0; Exception error = null;
                var watch = Stopwatch.StartNew();
                double deadline = 5 + size * 10.0 / bauds[test] * 2;
                var reader = new Thread(() => {
                    try {
                        while (count < size && watch.Elapsed.TotalSeconds < deadline) {
                            try { count += port.Read(received, count, Math.Min(4096, size-count)); }
                            catch (TimeoutException) { }
                        }
                    } catch (Exception e) { error = e; }
                });
                reader.IsBackground = true; reader.Start();
                try {
                    for (int offset = 0; offset < size; offset += 4096)
                        port.Write(sent, offset, Math.Min(4096, size-offset));
                    if (!reader.Join((int)(deadline * 1000))) throw new Exception("Reader deadline exceeded");
                    if (error != null) throw error;
                    if (count != size) throw new Exception("Short read: " + count + "/" + size);
                    for (int i = 0; i < size; ++i)
                        if (sent[i] != received[i]) throw new Exception("Mismatch at byte " + i);
                    Console.WriteLine("PASS port={0} baud={1} DTR={2} bytes={3} seconds={4:F3}",
                        name, bauds[test], dtrs[test], size, watch.Elapsed.TotalSeconds);
                } catch {
                    port.Close(); reader.Join(1000); throw;
                }
            }
        }
    }
}
'@
[BridgeLoopback]::Run($Port, $Quick.IsPresent)
