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
                string scriptDir = Path.Combine(exeDir, "plugins", "UAV_");
                string scriptPath = Path.Combine(scriptDir, "main.py");

                if (!File.Exists(scriptPath))
                {
                    Console.WriteLine("[DenelPythonLauncher] main.py not found at: " + scriptPath);
                    return true;
                }

                string pythonExe = ResolvePythonExe(scriptDir);

                if (!IsPythonAvailable(pythonExe))
                {
                    string msg = "Python was not found or is not runnable on this machine. " +
                                 "The STM32/joystick controller bridge will not start.\n\n" +
                                 "Install Python, make sure it is on PATH, then run:\n" +
                                 "    pip install -r plugins\\UAV_\\requirements.txt";
                    Console.WriteLine("[DenelPythonLauncher] " + msg);
                    MainV2.instance.Invoke(new Action(() =>
                        MessageBox.Show(MainV2.instance, msg, "Denel GCS - Python Bridge Unavailable",
                                         MessageBoxButtons.OK, MessageBoxIcon.Warning)));
                    return true;
                }

                // ProgramData rather than next to the exe: it is always writable no
                // matter where the release ZIP was extracted, and the log survives
                // replacing the app folder on an upgrade. Writing into a read-only
                // location throws UnauthorizedAccessException here and silently kills
                // the bridge before it starts.
                string logDir = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.CommonApplicationData), "Denel GCS");
                Directory.CreateDirectory(logDir);
                string logPath = Path.Combine(logDir, "denel_python.log");

                var psi = new ProcessStartInfo
                {
                    FileName               = pythonExe,
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
                MainV2.instance.Invoke(new Action(() =>
                    MessageBox.Show(MainV2.instance,
                        "Failed to start the Python controller bridge:\n" + ex.Message,
                        "Denel GCS - Python Bridge Error", MessageBoxButtons.OK, MessageBoxIcon.Warning)));
            }

            return true;
        }

        // Normally resolves to "python.exe" on PATH — target machines are expected
        // to have Python and the bridge's dependencies (plugins\UAV_\requirements.txt)
        // already installed.
        //
        // Optional manual override: drop a python_path.txt next to main.py holding the
        // full path to a python.exe, and that interpreter is used instead. That is the
        // escape hatch for a machine with several Pythons installed, where PATH order
        // decides which one wins and the dependencies may only be present in one of them.
        private string ResolvePythonExe(string scriptDir)
        {
            try
            {
                string pathFile = Path.Combine(scriptDir, "python_path.txt");
                if (File.Exists(pathFile))
                {
                    string p = File.ReadAllText(pathFile).Trim();
                    if (!string.IsNullOrEmpty(p) && File.Exists(p))
                        return p;
                }
            }
            catch { }
            return "python.exe";
        }

        private bool IsPythonAvailable(string pythonExe)
        {
            try
            {
                var psi = new ProcessStartInfo
                {
                    FileName               = pythonExe,
                    Arguments              = "--version",
                    UseShellExecute        = false,
                    CreateNoWindow         = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError  = true,
                };
                using (var probe = Process.Start(psi))
                {
                    probe.WaitForExit(3000);
                    return probe.HasExited && probe.ExitCode == 0;
                }
            }
            catch (System.ComponentModel.Win32Exception)
            {
                // Win32 error 2 (ERROR_FILE_NOT_FOUND) — python.exe not on PATH at all.
                return false;
            }
            catch
            {
                return false;
            }
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
                    Console.WriteLine("[DenelPythonLauncher] main.py terminated");
                }
            }
            catch { }
        }
    }
}
