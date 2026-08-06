using System;
using System.Drawing;
using System.IO;
using System.Linq;
using MissionPlanner.Controls;
using MissionPlanner.GCSViews;
using Newtonsoft.Json;

namespace MissionPlanner.Plugin
{
    // Draws a small Denel-branded status panel directly onto the HUD via the
    // (upstream-provided) HUD.OnCustomPaint hook. Deliberately a separate
    // plugin from DenelPythonLauncher — that one launches/supervises the
    // Python bridge process, this one only draws. Kept apart so a drawing
    // bug can't take down the process-launch path or vice versa.
    public class DenelHudOverlay : Plugin
    {
        public override string Name    => "Denel HUD Overlay";
        public override string Version => "1.0";
        public override string Author  => "Denel";

        // Denel theme palette (denel_cyan.mpsystheme), hardcoded rather than
        // read from ThemeManager — this panel is a fixed Denel brand element,
        // not a themed stock control, so it should look identical regardless
        // of whichever .mpsystheme happens to be active.
        private static readonly Color PanelBg  = Color.FromArgb(200, 0x28, 0x2C, 0x30); // ControlBGColor
        private static readonly Color Cyan     = Color.FromArgb(0x00, 0xBF, 0xFF);      // ButBG
        private static readonly Color BodyText = Color.FromArgb(0xF5, 0xEC, 0xD7);      // HudText
        private static readonly Color Muted    = Color.FromArgb(0x8a, 0x9b, 0xa8);      // ColorNotEnabled

        private static readonly string StatusFilePath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.CommonApplicationData),
            "Denel GCS", "bridge_status.json");

        private static readonly DateTime UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        private static readonly TimeSpan StatusStaleAfter = TimeSpan.FromSeconds(5);

        private bool _subscribed;
        private bool _loggedDrawError;

        // Cached bridge status — written by Loop() (background plugin thread),
        // read by DrawOverlay (HUD paint thread). Plain fields are fine here:
        // each is written wholesale by one thread and only ever read, never
        // partially mutated, by the other.
        private volatile bool _statusFileValid;
        private volatile bool _stm32Connected;
        private volatile string _stm32Port;
        private DateTime _statusFileTimeUtc = DateTime.MinValue;

        public override float loopratehz { get; set; } = 1f;

        public override bool Init() => true;

        public override bool Loaded() => true;

        public override bool Loop()
        {
            if (!_subscribed && FlightData.myhud != null)
            {
                FlightData.myhud.OnCustomPaint += DrawOverlay;
                _subscribed = true;
            }

            PollStatusFile();

            return true;
        }

        public override bool Exit()
        {
            if (_subscribed && FlightData.myhud != null)
            {
                FlightData.myhud.OnCustomPaint -= DrawOverlay;
            }
            return true;
        }

        private class BridgeStatus
        {
            // Names intentionally match the Python side's JSON keys exactly
            // (Plugins/UAV_/status_reporter.py) rather than C# convention.
            public bool stm32_connected { get; set; }
            public string stm32_port { get; set; }
            public double timestamp { get; set; }
        }

        private void PollStatusFile()
        {
            try
            {
                if (!File.Exists(StatusFilePath))
                {
                    _statusFileValid = false;
                    return;
                }

                string json = File.ReadAllText(StatusFilePath);
                var status = JsonConvert.DeserializeObject<BridgeStatus>(json);
                if (status == null)
                {
                    _statusFileValid = false;
                    return;
                }

                _stm32Connected = status.stm32_connected;
                _stm32Port = status.stm32_port;
                _statusFileTimeUtc = UnixEpoch.AddSeconds(status.timestamp);
                _statusFileValid = true;
            }
            catch
            {
                // Bridge may be mid-write (status_reporter.py writes via a
                // temp-file rename, but belt-and-braces here too), not
                // running yet, or the file may simply not exist yet — all
                // read the same as "unknown", not a crash.
                _statusFileValid = false;
            }
        }

        private void DrawOverlay(HUD hud)
        {
            try
            {
                DrawOverlayInner(hud);
            }
            catch (Exception ex)
            {
                // This runs on every HUD repaint (up to ~60x/sec) — never let
                // it throw uncaught (that's exactly the class of bug that
                // crashed the app before, just on a different thread), and
                // don't spam the console at that rate either.
                if (!_loggedDrawError)
                {
                    Console.WriteLine("[DenelHudOverlay] draw error: " + ex.Message);
                    _loggedDrawError = true;
                }
            }
        }

        private void DrawOverlayInner(HUD hud)
        {
            const int width = 190;
            const int margin = 10;
            const int rowHeight = 16;
            const int titleHeight = 20;
            int rows = 2; // STM32 status + control mode always shown
            bool showLaser = TryGetLaserRange(out float laserRangeM);
            if (showLaser)
                rows++;

            int height = titleHeight + rows * rowHeight + 6;
            float x = hud.Width - width - margin;
            float y = margin;

            hud.FillRectangle(new SolidBrush(PanelBg), new RectangleF(x, y, width, height));
            hud.DrawRectangle(new Pen(Cyan, 1.5f), new RectangleF(x, y, width, height));

            float textX = x + 10;
            float rowY = y + 4;
            hud.DrawString("DENEL STATUS", 9, new SolidBrush(Cyan), textX, rowY);
            rowY += titleHeight;

            // STM32 connection row — driven by Plugins/UAV_/status_reporter.py
            // via the polled bridge_status.json file (no live TCP/socket
            // channel — see CLAUDE.md for why that approach was dropped).
            bool statusFresh = _statusFileValid &&
                (DateTime.UtcNow - _statusFileTimeUtc) < StatusStaleAfter;
            string stm32Text;
            Color stm32Color;
            if (!statusFresh)
            {
                stm32Text = "STM32: --";
                stm32Color = Muted;
            }
            else if (_stm32Connected)
            {
                stm32Text = string.IsNullOrEmpty(_stm32Port)
                    ? "STM32: Connected"
                    : $"STM32: {_stm32Port}";
                stm32Color = Cyan;
            }
            else
            {
                stm32Text = "STM32: Offline";
                stm32Color = Muted;
            }
            hud.DrawString(stm32Text, 8, new SolidBrush(stm32Color), textX, rowY);
            rowY += rowHeight;

            // Control-mode row. FBWA/FBWB is what the Python bridge switches
            // ArduPilot into when the STM32 joystick takes over (see
            // command_sender.py) — read here purely via Mission Planner's own,
            // independent MAVLink connection, no new channel needed. NOTE:
            // as of this writing that switch is not yet wired up on the
            // Python side (dead code in translator.py/command_sender.py), so
            // this row is expected to always read "AUTOPILOT" for now.
            string mode = MainV2.comPort?.MAV?.cs?.mode;
            bool manual = mode == "FBWA" || mode == "FBWB";
            string modeText = manual ? "MANUAL — JOYSTICK" : "AUTOPILOT";
            Color modeColor = manual ? Cyan : BodyText;
            hud.DrawString(modeText, 8, new SolidBrush(modeColor), textX, rowY);
            rowY += rowHeight;

            if (showLaser)
            {
                hud.DrawString($"LASER: {laserRangeM:0.#}m", 8, new SolidBrush(Cyan), textX, rowY);
            }
        }

        // Laser range arrives as a MAVLink NAMED_VALUE_FLOAT named "LaserRange"
        // (see mavlink/payload_commands/payload_command_sender.py on the
        // Python side), which CurrentState generically maps onto one of its
        // customfieldN slots. There's no per-field last-updated timestamp
        // available, so "is this fresh" is approximated by "is the MAVLink
        // link currently connected" rather than a true per-message age check.
        private bool TryGetLaserRange(out float rangeMeters)
        {
            rangeMeters = 0;
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null || !cs.connected)
                    return false;

                var slot = CurrentState.custom_field_names?
                    .FirstOrDefault(kv => kv.Value == "MAV_LASERRANGE").Key;
                if (slot == null)
                    return false;

                var prop = cs.GetType().GetProperty(slot);
                if (prop == null)
                    return false;

                rangeMeters = (float)prop.GetValue(cs);
                return true;
            }
            catch
            {
                return false;
            }
        }
    }
}
