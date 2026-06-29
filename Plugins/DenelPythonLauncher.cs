using MissionPlanner.Plugin;
using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows.Forms;

namespace MissionPlanner.Plugin
{
    public class DenelPythonLauncher : Plugin
    {
        private Process _pythonProcess;
        private Thread  _notifThread;
        private volatile bool _notifRunning;

        public override string Name    => "Denel Python Launcher";
        public override string Version => "1.0";
        public override string Author  => "Denel";

        public override bool Init() => true;

        public override bool Loaded()
        {
            // Start notification server first — works independently of the Python script
            _notifRunning = true;
            _notifThread  = new Thread(NotificationServerLoop) { IsBackground = true, Name = "DenelNotif" };
            _notifThread.Start();

            try
            {
                string exeDir    = Path.GetDirectoryName(Application.ExecutablePath);
                string scriptDir = Path.Combine(exeDir, "plugins", "UAV_");
                string scriptPath = Path.Combine(scriptDir, "main.py");

                if (!File.Exists(scriptPath))
                {
                    Console.WriteLine("[DenelPythonLauncher] main.py not found at: " + scriptPath);
                    return true;
                }

                string logPath = Path.Combine(exeDir, "denel_python.log");

                var psi = new ProcessStartInfo
                {
                    FileName               = "python",
                    Arguments              = "-u main.py",
                    WorkingDirectory       = scriptDir,
                    UseShellExecute        = false,
                    CreateNoWindow         = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError  = true,
                };

                var logWriter = new StreamWriter(logPath, append: false) { AutoFlush = true };

                _pythonProcess = new Process { StartInfo = psi };
                _pythonProcess.OutputDataReceived += (s, e) => { if (e.Data != null) logWriter.WriteLine(e.Data); };
                _pythonProcess.ErrorDataReceived  += (s, e) => { if (e.Data != null) logWriter.WriteLine("[ERR] " + e.Data); };
                _pythonProcess.Start();
                _pythonProcess.BeginOutputReadLine();
                _pythonProcess.BeginErrorReadLine();

                Console.WriteLine("[DenelPythonLauncher] main.py started (PID " + _pythonProcess.Id + "), log: " + logPath);

                Application.ApplicationExit += (s, e) => KillScript();
            }
            catch (Exception ex)
            {
                Console.WriteLine("[DenelPythonLauncher] Failed to start main.py: " + ex.Message);
            }

            return true;
        }

        public override bool Exit()
        {
            _notifRunning = false;
            KillScript();
            return true;
        }

        private void NotificationServerLoop()
        {
            var listener = new TcpListener(IPAddress.Loopback, 5764);
            listener.Start();
            Console.WriteLine("[DenelPythonLauncher] Notification server listening on 127.0.0.1:5764");
            try
            {
                while (_notifRunning)
                {
                    if (!listener.Pending()) { Thread.Sleep(200); continue; }
                    TcpClient client = listener.AcceptTcpClient();
                    new Thread(() => HandleNotifClient(client)) { IsBackground = true }.Start();
                }
            }
            finally { listener.Stop(); }
        }

        private void HandleNotifClient(TcpClient client)
        {
            using (client)
            using (var stream = client.GetStream())
            using (var reader = new StreamReader(stream))
            using (var writer = new StreamWriter(stream) { AutoFlush = true })
            {
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    if (line.StartsWith("BUZZER_ALERT:"))
                    {
                        string msg = line.Substring("BUZZER_ALERT:".Length);
                        Console.WriteLine("[DenelPythonLauncher] Buzzer alert: " + msg);
                        MainV2.instance.Invoke(new Action(() =>
                            CustomMessageBox.Show(msg, "GCS Alert",
                                MessageBoxButtons.OK, MessageBoxIcon.Warning)));
                        writer.WriteLine("BUZZER_ACK");
                    }
                }
            }
        }

        private void KillScript()
        {
            try
            {
                if (_pythonProcess != null && !_pythonProcess.HasExited)
                {
                    _pythonProcess.Kill();
                    Console.WriteLine("[DenelPythonLauncher] main.py terminated");
                }
            }
            catch { }
        }
    }
}
