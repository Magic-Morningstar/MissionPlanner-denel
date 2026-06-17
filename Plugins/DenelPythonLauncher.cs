using MissionPlanner.Plugin;
using System;
using System.Diagnostics;
using System.IO;
using System.Windows.Forms;

namespace MissionPlanner.Plugin
{
    public class DenelPythonLauncher : Plugin
    {
        private Process _pythonProcess;

        public override string Name    => "Denel Python Launcher";
        public override string Version => "1.0";
        public override string Author  => "Denel";

        public override bool Init() => true;

        public override bool Loaded()
        {
            try
            {
                string exeDir    = Path.GetDirectoryName(Application.ExecutablePath);
                string scriptDir = Path.Combine(exeDir, "plugins", "Misson_PlannerScripts");
                string scriptPath = Path.Combine(scriptDir, "Heart.py");

                if (!File.Exists(scriptPath))
                {
                    Console.WriteLine("[DenelPythonLauncher] Heart.py not found at: " + scriptPath);
                    return true;
                }

                string logPath = Path.Combine(exeDir, "denel_python.log");

                var psi = new ProcessStartInfo
                {
                    FileName               = "python",
                    Arguments              = "-u Heart.py",   // -u = unbuffered so log updates in real time
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

                Console.WriteLine("[DenelPythonLauncher] Heart.py started (PID " + _pythonProcess.Id + "), log: " + logPath);

                Application.ApplicationExit += (s, e) => KillScript();
            }
            catch (Exception ex)
            {
                Console.WriteLine("[DenelPythonLauncher] Failed to start Heart.py: " + ex.Message);
            }

            return true;
        }

        public override bool Exit()
        {
            KillScript();
            return true;
        }

        private void KillScript()
        {
            try
            {
                if (_pythonProcess != null && !_pythonProcess.HasExited)
                {
                    _pythonProcess.Kill();
                    Console.WriteLine("[DenelPythonLauncher] Heart.py terminated");
                }
            }
            catch { }
        }
    }
}
