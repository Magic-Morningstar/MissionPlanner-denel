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
        private StreamWriter _logWriter;
        private bool _stopped;

        // How long to wait for main.py to exit on its own after being asked.
        // It needs long enough to send GOODBYE, stop its threads and close
        // the serial port — a fraction of a second normally. If it hasn't
        // gone by then something is wedged, and we fall back to Kill().
        private const int GracefulExitTimeoutMs = 4000;

        public override string Name    => "Denel Python Launcher";
        public override string Version => "1.2";
        public override string Author  => "Denel";

        public override bool Init() => true;

        public override bool Loaded()
        {
            try
            {
                string exeDir     = Path.GetDirectoryName(Application.ExecutablePath);
                string scriptDir  = Path.Combine(exeDir, "plugins", "UAV_");
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
                // replacing the app folder on an upgrade.
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

                    // The shutdown channel. With CreateNoWindow there is no
                    // console, so Python cannot receive Ctrl+C or a console
                    // close event. Instead we write "shutdown" here and close
                    // the pipe; main.py's StdinWatcher treats either as a
                    // request for a clean exit.
                    //
                    // Closing is what matters most. If Mission Planner crashes,
                    // Windows closes this handle for us and Python sees EOF —
                    // so even an abnormal exit gets a clean shutdown on the
                    // Python side.
                    RedirectStandardInput  = true,
                };

                _logWriter = new StreamWriter(logPath, append: false) { AutoFlush = true };

                _pythonProcess = new Process { StartInfo = psi };
                _pythonProcess.OutputDataReceived += (s, e) => { if (e.Data != null) SafeLog(e.Data); };
                _pythonProcess.ErrorDataReceived  += (s, e) => { if (e.Data != null) SafeLog("[ERR] " + e.Data); };
                _pythonProcess.Start();
                _pythonProcess.BeginOutputReadLine();
                _pythonProcess.BeginErrorReadLine();

                Log("main.py started (PID " + _pythonProcess.Id + "), log: " + logPath);

                // Three shutdown hooks, because Mission Planner does not
                // guarantee which of them runs, or in what order.
                //
                // FormClosing is the one to rely on: it fires on the UI
                // thread while the main window and every plugin are still
                // alive. ApplicationExit can fire late or not at all if MP
                // leaves via Environment.Exit, and Exit() depends on MP's
                // plugin loader remembering to call it. StopScript() is
                // idempotent, so whichever arrives first does the work.
                MainV2.instance.FormClosing += (s, e) => { Log("hook: MainV2.FormClosing"); StopScript(); };
                Application.ApplicationExit += (s, e) => { Log("hook: Application.ApplicationExit"); StopScript(); };
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

        // Normally resolves to "python.exe" on PATH. Optional override: a
        // python_path.txt next to main.py holding the full path to a
        // python.exe — the escape hatch for machines with several Pythons,
        // where the dependencies may only be installed in one of them.
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
                return false;   // ERROR_FILE_NOT_FOUND — not on PATH at all
            }
            catch
            {
                return false;
            }
        }

        public override bool Exit()
        {
            Log("hook: Plugin.Exit");
            StopScript();
            return true;
        }

        // Replaces the old KillScript(). Process.Kill() is TerminateProcess:
        // the child is removed instantly and runs no code at all, so main.py
        // never sent GOODBYE and the panel showed "connection lost" (red)
        // every time Mission Planner closed normally.
        //
        // Now: ask politely, wait, and only then kill.
        private void StopScript()
        {
            // Exit() and Application.ApplicationExit both land here, often
            // back to back. Only the first call should do anything.
            if (_stopped) return;
            _stopped = true;

            try
            {
                if (_pythonProcess == null)
                {
                    Log("StopScript: no process to stop");
                    return;
                }
                if (_pythonProcess.HasExited)
                {
                    Log("StopScript: main.py had already exited (code " + _pythonProcess.ExitCode + ")");
                    return;
                }

                Log("StopScript: asking main.py to shut down");
                try
                {
                    _pythonProcess.StandardInput.WriteLine("shutdown");
                    _pythonProcess.StandardInput.Flush();
                    _pythonProcess.StandardInput.Close();   // EOF — the part that really counts
                    Log("StopScript: shutdown sent, stdin closed");
                }
                catch (Exception ex)
                {
                    Log("StopScript: could not write to stdin (" + ex.GetType().Name + ": " + ex.Message + ")");
                }

                if (_pythonProcess.WaitForExit(GracefulExitTimeoutMs))
                {
                    // Parameterless overload drains the async stdout/stderr
                    // readers, so Python's final lines — the GOODBYE — make
                    // it into the log before we close it.
                    _pythonProcess.WaitForExit();
                    Log("StopScript: main.py exited cleanly (code " + _pythonProcess.ExitCode + ")");
                }
                else
                {
                    // Wedged. Kill as a last resort; the panel's heartbeat
                    // timeout will report the link as lost, which in this
                    // case is the honest answer.
                    Log("StopScript: main.py still running after " + GracefulExitTimeoutMs + " ms — killing");
                    _pythonProcess.Kill();
                }
            }
            catch (Exception ex)
            {
                Log("StopScript: unexpected " + ex.GetType().Name + ": " + ex.Message);
            }
            finally
            {
                try { _logWriter?.Dispose(); _logWriter = null; } catch { }
            }
        }

        private void SafeLog(string line)
        {
            try { _logWriter?.WriteLine(line); } catch { }
        }

        // Launcher's own lines, written into the same file as Python's so
        // the shutdown sequence reads top to bottom in one place. Console
        // output goes nowhere inside Mission Planner, which is why the
        // previous version's diagnostics were invisible.
        private void Log(string msg)
        {
            string line = DateTime.Now.ToString("HH:mm:ss.fff") + " [LAUNCHER] " + msg;
            Console.WriteLine(line);
            SafeLog(line);
        }
    }
}