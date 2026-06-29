using MissionPlanner.Plugin;
using System;
using System.Diagnostics;
using System.Drawing;
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
                        MainV2.instance.Invoke(new Action(() => ShowBuzzerAlert(msg)));
                        writer.WriteLine("BUZZER_ACK");
                    }
                }
            }
        }

        private void ShowBuzzerAlert(string msg)
        {
            var cyan = Color.FromArgb(0x00, 0xBF, 0xFF);
            var dark = Color.FromArgb(0x16, 0x18, 0x1A);

            using (var frm = new Form())
            {
                frm.Size            = new Size(500, 280);
                frm.FormBorderStyle = FormBorderStyle.None;
                frm.BackColor       = cyan;   // 3px cyan border shows through
                frm.TopMost         = true;
                frm.StartPosition   = FormStartPosition.Manual;
                frm.KeyPreview      = true;
                frm.KeyDown        += (s, e) => { if (e.KeyCode == Keys.Escape) frm.Close(); };

                Rectangle screen = Screen.PrimaryScreen.Bounds;
                frm.Location = new Point(
                    screen.X + (screen.Width  - frm.Width)  / 2,
                    screen.Y + (screen.Height - frm.Height) / 2);

                // Dark content panel inset 3px to reveal the cyan border all around
                var content = new Panel
                {
                    BackColor = dark,
                    Location  = new Point(3, 3),
                    Size      = new Size(frm.Width - 6, frm.Height - 6)
                };

                // Cyan title strip across the top
                var titleBar = new Panel
                {
                    BackColor = cyan,
                    Location  = new Point(0, 0),
                    Size      = new Size(content.Width, 38)
                };
                var titleLabel = new Label
                {
                    Text      = "GCS ALERT",
                    ForeColor = dark,
                    BackColor = Color.Transparent,
                    Font      = new Font("Segoe UI", 12f, FontStyle.Bold),
                    Location  = new Point(12, 0),
                    Size      = new Size(titleBar.Width - 12, titleBar.Height),
                    TextAlign = ContentAlignment.MiddleLeft
                };
                titleBar.Controls.Add(titleLabel);

                var warnIcon = new PictureBox
                {
                    Image     = SystemIcons.Warning.ToBitmap(),
                    SizeMode  = PictureBoxSizeMode.StretchImage,
                    Size      = new Size(48, 48),
                    Location  = new Point(14, 60),
                    BackColor = Color.Transparent
                };

                var msgLabel = new Label
                {
                    Text      = msg,
                    ForeColor = Color.White,
                    BackColor = Color.Transparent,
                    Font      = new Font("Segoe UI", 12f),
                    Location  = new Point(74, 55),
                    Size      = new Size(content.Width - 94, 140),
                    AutoSize  = false
                };

                var btnOk = new Button
                {
                    Text      = "OK",
                    Size      = new Size(120, 40),
                    Location  = new Point((content.Width - 120) / 2, content.Height - 60),
                    BackColor = cyan,
                    ForeColor = dark,
                    FlatStyle = FlatStyle.Flat,
                    Font      = new Font("Segoe UI", 11f, FontStyle.Bold),
                };
                btnOk.FlatAppearance.BorderColor = cyan;
                btnOk.Click += (s, e) => frm.Close();
                frm.AcceptButton = btnOk;

                // Flash border and title bar cyan ↔ red at 500ms
                bool flashOn = false;
                var alertRed  = Color.FromArgb(0xFF, 0x33, 0x00);
                var flashTimer = new System.Windows.Forms.Timer { Interval = 500 };
                flashTimer.Tick += (s, e) =>
                {
                    flashOn            = !flashOn;
                    var c              = flashOn ? alertRed : cyan;
                    frm.BackColor      = c;
                    titleBar.BackColor = c;
                    titleLabel.ForeColor = flashOn ? Color.White : dark;
                };
                flashTimer.Start();
                frm.FormClosed += (s, e) => { flashTimer.Stop(); flashTimer.Dispose(); };

                content.Controls.AddRange(new Control[] { titleBar, warnIcon, msgLabel, btnOk });
                frm.Controls.Add(content);
                frm.ShowDialog();
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
