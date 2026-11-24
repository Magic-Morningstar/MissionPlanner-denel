using MissionPlanner.Plugin;
using System;
using System.Diagnostics;
using System.IO;
using System.Windows.Forms;

namespace MissionPlanner.Plugin
{
    /// <summary>
    /// Starts the GCS-side MAVLink-over-MQTT bridge alongside Mission Planner.
    ///
    /// DEFAULT OFF. The MQTT link is a backup path, not the normal one, so an
    /// always-running second Python process and a second broker connection are
    /// not something to impose on every operator. Enable it on a machine by
    /// creating the file:
    ///
    ///     plugins\UAV_\mqtt_bridge\enabled.txt      (contents ignored)
    ///
    /// and disable it by deleting that file. A file rather than a Settings key
    /// for two reasons: the operator need not hand-edit config.xml, and reading
    /// Settings here would pull in MissionPlanner.Utilities, which targets
    /// netstandard2.0 -- plugins are compiled at RUNTIME by CodeGenRoslyn against
    /// whatever assemblies happen to be loaded, so relying on the netstandard
    /// shim being among them is an avoidable risk for a feature flag.
    ///
    /// Deliberately self-contained rather than sharing code with
    /// DenelPythonLauncher: that class is the suspected silent-failure path on
    /// the GCS machine, and its _pythonProcess / _logWriter / _stopped fields are
    /// single-instance by design -- StopScript()'s `if (_stopped) return;` would
    /// orphan whichever child it did not handle first. Two classes, two log
    /// files, two independent stop flags. The duplication is ~40 lines and is
    /// worth it until that class is understood.
    /// </summary>
    public class DenelMqttBridgeLauncher : Plugin
    {
        private Process _process;
        private StreamWriter _logWriter;
        private bool _stopped;

        private const int GracefulExitTimeoutMs = 6000;

        public override string Name    => "Denel MQTT Bridge";
        public override string Version => "1.0";
        public override string Author  => "Denel";

        public override bool Init() => true;

        public override bool Loaded()
        {
            try
            {
                string exeDir    = Path.GetDirectoryName(Application.ExecutablePath);
                string uavDir    = Path.Combine(exeDir, "plugins", "UAV_");
                string bridgeDir = Path.Combine(uavDir, "mqtt_bridge");

                if (!IsEnabled(bridgeDir))
                {
                    Console.WriteLine("[DenelMqttBridgeLauncher] disabled; create " +
                                      Path.Combine(bridgeDir, "enabled.txt") + " to enable");
                    return true;
                }

                if (!File.Exists(Path.Combine(bridgeDir, "gcs_bridge.py")))
                {
                    Console.WriteLine("[DenelMqttBridgeLauncher] mqtt_bridge not found at " + bridgeDir);
                    return true;
                }

                string pythonExe = ResolvePythonExe(uavDir);

                // Checking that python runs is not enough: the Windows GCS has
                // been observed with pymavlink but WITHOUT paho-mqtt, and the
                // bridge would then die on an import with nothing on screen.
                string missing;
                if (!AreDepsAvailable(pythonExe, out missing))
                {
                    string msg = "The MQTT bridge cannot start: " + missing + "\n\n" +
                                 "Install its dependencies with:\n" +
                                 "    pip install -r plugins\\UAV_\\mqtt_bridge\\requirements.txt";
                    Console.WriteLine("[DenelMqttBridgeLauncher] " + msg);
                    MainV2.instance.Invoke(new Action(() =>
                        MessageBox.Show(MainV2.instance, msg, "Denel GCS - MQTT Bridge Unavailable",
                                        MessageBoxButtons.OK, MessageBoxIcon.Warning)));
                    return true;
                }

                // Its own file, never denel_python.log: that one is opened with
                // append:false by the other launcher, so sharing it would mean
                // whichever started second truncated the other's output.
                string logDir = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.CommonApplicationData),
                    "Denel GCS");
                Directory.CreateDirectory(logDir);
                string logPath = Path.Combine(logDir, "denel_mqtt_bridge_launcher.log");
                _logWriter = new StreamWriter(logPath, append: false) { AutoFlush = true };

                var psi = new ProcessStartInfo
                {
                    FileName = pythonExe,
                    // --stdin-shutdown because we hold the pipe open. It is off by
                    // default precisely so that a service launched with /dev/null
                    // on stdin does not read EOF and exit immediately.
                    Arguments = "-u -m mqtt_bridge.gcs_bridge --stdin-shutdown",
                    WorkingDirectory = uavDir,
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    RedirectStandardInput = true,
                };

                _process = new Process { StartInfo = psi };
                _process.OutputDataReceived += (s, e) => { if (e.Data != null) SafeLog(e.Data); };
                _process.ErrorDataReceived  += (s, e) => { if (e.Data != null) SafeLog("[ERR] " + e.Data); };
                _process.Start();
                _process.BeginOutputReadLine();
                _process.BeginErrorReadLine();

                Log("gcs_bridge started (PID " + _process.Id + "), log: " + logPath);
                Log("Mission Planner connects to it with UDPCl on 127.0.0.1:14570");

                MainV2.instance.FormClosing += (s, e) => { Log("hook: FormClosing"); Stop(); };
                Application.ApplicationExit += (s, e) => { Log("hook: ApplicationExit"); Stop(); };
            }
            catch (Exception ex)
            {
                Console.WriteLine("[DenelMqttBridgeLauncher] failed to start: " + ex.Message);
            }

            return true;
        }

        private static bool IsEnabled(string bridgeDir)
        {
            try
            {
                return File.Exists(Path.Combine(bridgeDir, "enabled.txt"));
            }
            catch { return false; }
        }

        private string ResolvePythonExe(string uavDir)
        {
            // Same override file as DenelPythonLauncher, so a machine with several
            // Pythons only has to be told once which one has the dependencies.
            try
            {
                string pathFile = Path.Combine(uavDir, "python_path.txt");
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

        private bool AreDepsAvailable(string pythonExe, out string missing)
        {
            missing = null;
            try
            {
                var psi = new ProcessStartInfo
                {
                    FileName = pythonExe,
                    Arguments = "-c \"import paho.mqtt.client, pymavlink\"",
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                };
                using (var probe = Process.Start(psi))
                {
                    string err = probe.StandardError.ReadToEnd();
                    probe.WaitForExit(8000);
                    if (!probe.HasExited) { missing = "python did not respond"; return false; }
                    if (probe.ExitCode == 0) return true;
                    missing = string.IsNullOrWhiteSpace(err)
                        ? "a required Python package is missing"
                        : err.Trim().Split('\n')[err.Trim().Split('\n').Length - 1];
                    return false;
                }
            }
            catch (System.ComponentModel.Win32Exception)
            {
                missing = "Python was not found on PATH";
                return false;
            }
            catch (Exception ex)
            {
                missing = ex.Message;
                return false;
            }
        }

        public override bool Exit()
        {
            Log("hook: Plugin.Exit");
            Stop();
            return true;
        }

        private void Stop()
        {
            if (_stopped) return;
            _stopped = true;

            try
            {
                if (_process == null) { Log("Stop: nothing to stop"); return; }
                if (_process.HasExited)
                {
                    Log("Stop: already exited (code " + _process.ExitCode + ")");
                    return;
                }

                // Closing stdin is what matters: the bridge treats EOF as a
                // shutdown request, and Windows closes this handle for us even if
                // Mission Planner crashes, so an abnormal exit still shuts the
                // bridge down cleanly -- which is what publishes 'offline' to the
                // retained status topic.
                try
                {
                    _process.StandardInput.WriteLine("shutdown");
                    _process.StandardInput.Flush();
                    _process.StandardInput.Close();
                    Log("Stop: shutdown sent, stdin closed");
                }
                catch (Exception ex)
                {
                    Log("Stop: could not write stdin (" + ex.GetType().Name + ")");
                }

                if (_process.WaitForExit(GracefulExitTimeoutMs))
                {
                    _process.WaitForExit();   // drains the async readers
                    Log("Stop: exited cleanly (code " + _process.ExitCode + ")");
                }
                else
                {
                    Log("Stop: still running after " + GracefulExitTimeoutMs + " ms - killing");
                    _process.Kill();
                }
            }
            catch (Exception ex)
            {
                Log("Stop: unexpected " + ex.GetType().Name + ": " + ex.Message);
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

        private void Log(string msg)
        {
            string line = DateTime.Now.ToString("HH:mm:ss.fff") + " [MQTT-LAUNCHER] " + msg;
            Console.WriteLine(line);
            SafeLog(line);
        }
    }
}
