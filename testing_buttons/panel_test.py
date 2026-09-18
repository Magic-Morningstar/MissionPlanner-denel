"""
Panel Test Rig — Tkinter version.

Simulates the physical panel (toggle switches, latching push-buttons,
rocker switches, joysticks) and walks a tester through a pass/fail
sequence, logging every event.

Keyboard stands in for the physical wiring for now. To connect real
hardware later, don't touch any of the test/pass-fail/logging logic —
just call `rig.register_event(control_id, action)` directly from your
hardware-reading code whenever a real switch/button/stick fires, e.g.:

    rig.register_event("S1", "up")
    rig.register_event("PB2", "on")
    rig.register_event("JS1", "left")

That's the one integration point. Everything else (sequencing, pass/fail,
the event log, CSV export) already works off of it.
"""

import csv
import datetime
import random
import tkinter as tk
from tkinter import ttk, filedialog

# ---------------------------------------------------------------------------
# Config: every control on the panel. Rename ids/keys here as needed —
# everything else in this file reads from CONTROLS / GROUPS.
# ---------------------------------------------------------------------------

CONTROLS = [
    {"id": "S1", "label": "S1", "type": "switch", "style": "toggle", "actions": [("up", "1"), ("down", "q")]},
    {"id": "S2", "label": "S2", "type": "switch", "style": "toggle", "actions": [("up", "2"), ("down", "w")]},
    {"id": "S3", "label": "S3", "type": "switch", "style": "toggle", "actions": [("up", "3"), ("down", "e")]},

    {"id": "PB1", "label": "PB1", "type": "latch", "actions": [("on", "7"), ("off", "7")]},
    {"id": "PB2", "label": "PB2", "type": "latch", "actions": [("on", "8"), ("off", "8")]},
    {"id": "PB3", "label": "PB3", "type": "latch", "actions": [("on", "9"), ("off", "9")]},

    {"id": "RS1", "label": "RS1", "type": "switch", "style": "rocker", "actions": [("up", "u"), ("down", "j")]},
    {"id": "RS2", "label": "RS2", "type": "switch", "style": "rocker", "actions": [("up", "i"), ("down", "k")]},
    {"id": "RS3", "label": "RS3", "type": "switch", "style": "rocker", "actions": [("up", "4"), ("down", "r")]},
    {"id": "RS4", "label": "RS4", "type": "switch", "style": "rocker", "actions": [("up", "5"), ("down", "t")]},
    {"id": "RS5", "label": "RS5", "type": "switch", "style": "rocker", "actions": [("up", "6"), ("down", "y")]},

    {"id": "JS1", "label": "JS1", "type": "joystick",
     "actions": [("up", "Up"), ("down", "Down"), ("left", "Left"), ("right", "Right")]},
    {"id": "JS2", "label": "JS2", "type": "joystick",
     "actions": [("up", "f"), ("down", "v"), ("left", "c"), ("right", "b")]},
]

GROUPS = [
    ("Toggle switches", ["S1", "S2", "S3"]),
    ("Latching push-buttons", ["PB1", "PB2", "PB3"]),
    ("Rocker switches", ["RS1", "RS2", "RS3", "RS4", "RS5"]),
    ("Joysticks", ["JS1", "JS2"]),
]

ARROW = {"up": "▲", "down": "▼", "left": "◀", "right": "▶", "on": "●", "off": "○"}

# colors
BG = "#121317"
PANEL = "#18191d"
CARD = "#1b1c21"
TEXT = "#ededef"
DIM = "#96979e"
RED = "#c1272d"
ROCKER = "#101114"
BLUE = "#4fb6e8"
GREEN = "#2fa84f"
AMBER = "#e0912a"
YELLOW = "#e7c245"
CHIP_BG = "#232429"


class TestRig(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Panel Test Rig")
        self.configure(bg=BG)
        self.geometry("1060x800")
        self.minsize(860, 640)

        self.by_id = {c["id"]: c for c in CONTROLS}

        # key (normalized) -> (control_id, action or None for latch/dynamic)
        self.by_key = {}
        for c in CONTROLS:
            if c["type"] == "latch":
                self.by_key[c["actions"][0][1].lower()] = (c["id"], None)
            else:
                for action, key in c["actions"]:
                    self.by_key[key.lower()] = (c["id"], action)

        self.results = {}
        self.attempts = {}
        self.position = {}
        self.reset_results()

        self.sequence = []
        self.current_index = -1
        self.running = False
        self.free_mode = tk.BooleanVar(value=False)
        self.randomize = tk.BooleanVar(value=False)
        self.event_log = []  # list of (time, key, control_id, action, outcome)

        self.widgets = {}   # control_id -> dict of widget refs
        self.blink_state = False

        self._build_ui()
        self.bind_all("<KeyPress>", self.on_key)
        self.render_all()
        self._blink_loop()

    # ------------------------------------------------------------ state ---

    def reset_results(self):
        self.results = {}
        self.attempts = {}
        self.position = {}
        for c in CONTROLS:
            self.results[c["id"]] = {a: "pending" for a, _ in c["actions"]}
            self.attempts[c["id"]] = {a: 0 for a, _ in c["actions"]}
            self.position[c["id"]] = "off" if c["type"] == "latch" else None

    def base_sequence(self):
        seq = []
        for c in CONTROLS:
            for action, _ in c["actions"]:
                seq.append((c["id"], action))
        return seq

    def control_status(self, cid):
        vals = list(self.results[cid].values())
        if "flag" in vals:
            return "flag"
        if all(v == "pass" for v in vals):
            return "pass"
        if any(v == "pass" for v in vals):
            return "partial"
        return "pending"

    def action_phrase(self, c, action):
        if c["type"] == "latch":
            return f"Press {c['label']} to latch {action.upper()}"
        if c["type"] == "joystick":
            return f"Push {c['label']} {action.upper()} to max"
        return f"Flip {c['label']} {action.upper()} now"

    # -------------------------------------------------------------- UI ---

    def _build_ui(self):
        header = tk.Frame(self, bg=BG)
        header.pack(fill="x", padx=20, pady=(16, 10))
        tk.Label(header, text="Panel Test Rig", font=("Segoe UI", 16, "bold"), bg=BG, fg=TEXT).pack(anchor="w")
        tk.Label(
            header,
            text="Toggles, latching push-buttons, rockers, and joysticks — "
                 "keyboard stands in for the physical wiring until it's connected.",
            font=("Segoe UI", 10), bg=BG, fg=DIM, wraplength=760, justify="left",
        ).pack(anchor="w")

        self.panel_frame = tk.Frame(self, bg=PANEL, padx=20, pady=16)
        self.panel_frame.pack(fill="x", padx=20, pady=(0, 12))
        self._build_panel()

        self.instruction_var = tk.StringVar(value='Press "Start test" to begin.')
        instr = tk.Frame(self, bg=CARD)
        instr.pack(fill="x", padx=20, pady=(0, 10))
        tk.Label(instr, textvariable=self.instruction_var, font=("Consolas", 12),
                 bg=CARD, fg=TEXT, anchor="w", padx=12, pady=10).pack(side="left", fill="x", expand=True)
        self.instruction_sub = tk.Label(instr, text="", font=("Consolas", 9), bg=CARD, fg=DIM, padx=12)
        self.instruction_sub.pack(side="right")

        controls = tk.Frame(self, bg=BG)
        controls.pack(fill="x", padx=20, pady=(0, 12))
        ttk.Button(controls, text="Start test", command=self.start_test).pack(side="left", padx=(0, 6))
        self.skip_btn = ttk.Button(controls, text="Flag && skip", command=self.skip_step, state="disabled")
        self.skip_btn.pack(side="left", padx=6)
        ttk.Button(controls, text="Reset", command=self.reset_test).pack(side="left", padx=6)
        ttk.Button(controls, text="Export CSV", command=self.export_csv).pack(side="left", padx=6)
        tk.Checkbutton(controls, text="Randomize order", variable=self.randomize, bg=BG, fg=DIM,
                        selectcolor=BG, activebackground=BG, activeforeground=DIM).pack(side="left", padx=(16, 4))
        tk.Checkbutton(controls, text="Free monitor mode", variable=self.free_mode, bg=BG, fg=DIM,
                        selectcolor=BG, activebackground=BG, activeforeground=DIM).pack(side="left", padx=4)

        cols = tk.Frame(self, bg=BG)
        cols.pack(fill="both", expand=True, padx=20, pady=(0, 8))

        seq_card = tk.Frame(cols, bg=CARD)
        seq_card.pack(side="left", fill="both", expand=True, padx=(0, 9))
        tk.Label(seq_card, text="Sequence", font=("Segoe UI", 10, "bold"), bg=CARD, fg=DIM,
                 anchor="w", padx=10, pady=6).pack(fill="x")
        self.seq_list = tk.Listbox(seq_card, bg=CARD, fg=TEXT, bd=0, highlightthickness=0, font=("Consolas", 10))
        self.seq_list.pack(fill="both", expand=True, padx=6, pady=6)

        log_card = tk.Frame(cols, bg=CARD)
        log_card.pack(side="left", fill="both", expand=True, padx=(9, 0))
        tk.Label(log_card, text="Event log", font=("Segoe UI", 10, "bold"), bg=CARD, fg=DIM,
                 anchor="w", padx=10, pady=6).pack(fill="x")
        self.log_list = tk.Listbox(log_card, bg=CARD, fg=TEXT, bd=0, highlightthickness=0, font=("Consolas", 9))
        self.log_list.pack(fill="both", expand=True, padx=6, pady=6)

        self.summary_var = tk.StringVar(value="")
        tk.Label(self, textvariable=self.summary_var, font=("Consolas", 10), bg=BG, fg=DIM,
                 anchor="w").pack(fill="x", padx=22, pady=(0, 14))

    def _build_panel(self):
        for title, ids in GROUPS:
            group_frame = tk.Frame(self.panel_frame, bg=PANEL)
            group_frame.pack(fill="x", pady=(0, 14))
            tk.Label(group_frame, text=title, font=("Consolas", 9), bg=PANEL, fg="#8c8d92").pack(anchor="w", pady=(0, 6))
            row = tk.Frame(group_frame, bg=PANEL)
            row.pack(anchor="w")
            for cid in ids:
                self._build_unit(row, self.by_id[cid])

    def _build_unit(self, parent, c):
        unit = tk.Frame(parent, bg=PANEL, padx=10)
        unit.pack(side="left")
        w = {}

        if c["type"] == "switch":
            color = RED if c["style"] == "toggle" else ROCKER
            cap = tk.Label(unit, width=5, height=2, bg=color, relief="raised", bd=2,
                            highlightthickness=2, highlightbackground=PANEL)
            cap.pack()
            w["cap"] = cap
        elif c["type"] == "latch":
            cap = tk.Canvas(unit, width=46, height=46, bg=PANEL, highlightthickness=0)
            oval = cap.create_oval(4, 4, 42, 42, fill=ROCKER, outline="#000", width=2)
            cap.pack()
            w["cap"] = cap
            w["oval"] = oval
        else:  # joystick
            ball = tk.Label(unit, width=4, height=2, bg="#2b2c31", relief="raised", bd=2)
            ball.pack()
            w["cap"] = ball

        tk.Label(unit, text=c["label"], font=("Consolas", 9, "bold"), bg=PANEL, fg=YELLOW).pack(pady=(4, 2))

        keys_frame = tk.Frame(unit, bg=PANEL)
        keys_frame.pack()
        w["chips"] = {}
        side = "left" if c["type"] == "joystick" else "top"
        for action, key in c["actions"]:
            chip = tk.Label(keys_frame, text=f"{ARROW.get(action, '')} {key.upper()}", font=("Consolas", 8),
                             bg=CHIP_BG, fg=DIM, padx=4, pady=1)
            chip.pack(side=side, padx=1, pady=1)
            w["chips"][action] = chip

        self.widgets[c["id"]] = w

    # --------------------------------------------------------- rendering ---

    def refresh_unit_states(self):
        target = None
        if self.running and not self.free_mode.get() and self.current_index < len(self.sequence):
            target = self.sequence[self.current_index]

        for c in CONTROLS:
            cid = c["id"]
            w = self.widgets[cid]
            status = self.control_status(cid)

            for action, chip in w["chips"].items():
                st = self.results[cid][action]
                fg = GREEN if st == "pass" else AMBER if st == "flag" else DIM
                chip.configure(bg=CHIP_BG, fg=fg)

            if c["type"] == "switch":
                ring = GREEN if status == "pass" else AMBER if status == "flag" else PANEL
                w["cap"].configure(highlightbackground=ring)
            elif c["type"] == "latch":
                fill = BLUE if self.position[cid] == "on" else ROCKER
                outline = GREEN if status == "pass" else AMBER if status == "flag" else "#000"
                w["cap"].itemconfig(w["oval"], fill=fill, outline=outline)

            if target and target[0] == cid:
                chip = w["chips"][target[1]]
                if self.blink_state:
                    chip.configure(bg=AMBER, fg="#111")
                else:
                    chip.configure(bg=CHIP_BG, fg="white")

    def render_seq_list(self):
        self.seq_list.delete(0, "end")
        for i, (cid, action) in enumerate(self.sequence):
            c = self.by_id[cid]
            st = self.results[cid][action]
            mark = "x" if st == "pass" else "!" if st == "flag" else " "
            cur = "-> " if (self.running and not self.free_mode.get() and i == self.current_index) else "   "
            misses = self.attempts[cid][action]
            miss_txt = f"   ({misses} miss{'es' if misses > 1 else ''})" if misses else ""
            self.seq_list.insert("end", f"{cur}[{mark}] {c['label']} {ARROW.get(action, '')} {action}{miss_txt}")
            if st == "pass":
                self.seq_list.itemconfig(i, fg=GREEN)
            elif st == "flag":
                self.seq_list.itemconfig(i, fg=AMBER)
            elif self.running and not self.free_mode.get() and i == self.current_index:
                self.seq_list.itemconfig(i, fg="white", bg="#2a2410")
            else:
                self.seq_list.itemconfig(i, fg=TEXT, bg=CARD)

    def render_log_list(self):
        self.log_list.delete(0, "end")
        for t, key, cid, action, outcome in reversed(self.event_log):
            key_txt = str(key).upper()
            self.log_list.insert(
                "end", f"{t}  {key_txt:<7} {cid or '-':<5} {action or '-':<6} {outcome}"
            )

    def render_instruction(self):
        if not self.running:
            self.instruction_var.set(
                "Free monitor mode — operate any control to log it." if self.free_mode.get()
                else 'Press "Start test" to begin.'
            )
            self.instruction_sub.configure(text="")
            return
        if self.free_mode.get():
            self.instruction_var.set("Monitoring — operate any control to log it.")
            self.instruction_sub.configure(text=f"{len(self.event_log)} event(s) logged")
            return
        if self.current_index >= len(self.sequence):
            passed = sum(1 for cid, a in self.sequence if self.results[cid][a] == "pass")
            self.instruction_var.set(f"Sequence complete — {passed} / {len(self.sequence)} steps passed.")
            self.instruction_sub.configure(text="")
            return
        cid, action = self.sequence[self.current_index]
        c = self.by_id[cid]
        key = dict(c["actions"])[action]
        phrase = self.action_phrase(c, action)
        self.instruction_var.set(f"{phrase} — key [{key.upper()}]")
        self.instruction_sub.configure(text=f"{self.current_index + 1} of {len(self.sequence)}")

    def render_summary(self):
        total = len(self.sequence)
        passed = sum(1 for cid, a in self.sequence if self.results[cid][a] == "pass")
        flagged = sum(1 for cid, a in self.sequence if self.results[cid][a] == "flag")
        pending = total - passed - flagged
        self.summary_var.set(
            f"{passed} steps passed    {flagged} flagged    {pending} pending    {len(self.event_log)} raw events"
        )

    def render_all(self):
        self.refresh_unit_states()
        self.render_seq_list()
        self.render_log_list()
        self.render_instruction()
        self.render_summary()
        can_skip = self.running and not self.free_mode.get() and self.current_index < len(self.sequence)
        self.skip_btn.configure(state="normal" if can_skip else "disabled")

    def _blink_loop(self):
        self.blink_state = not self.blink_state
        self.refresh_unit_states()
        self.after(500, self._blink_loop)

    # -------------------------------------------------------------- log ---

    def log_event(self, key, control_id, action, outcome):
        t = datetime.datetime.now().strftime("%H:%M:%S")
        self.event_log.append((t, key, control_id, action, outcome))

    # --------------------------------------------------------- input hook ---

    def on_key(self, event):
        """Keyboard stand-in. Resolves the key to (control_id, action) and
        forwards to register_event — the same path real hardware will use."""
        key = event.keysym.lower()
        entry = self.by_key.get(key)
        if not entry:
            return
        cid, action = entry
        if action is None:  # latch: toggle from current state
            action = "off" if self.position[cid] == "on" else "on"
        self.register_event(cid, action, source_key=key)

    def register_event(self, control_id, action, source_key="hw"):
        """The one integration point for real hardware. Call this directly
        (bypassing keyboard entirely) once your button-reading code is ready,
        e.g. rig.register_event('RS3', 'down')."""
        c = self.by_id.get(control_id)
        if c is None:
            return
        if c["type"] in ("switch", "latch"):
            self.position[control_id] = action

        if not self.running:
            self.refresh_unit_states()
            return

        if self.free_mode.get():
            self.log_event(source_key, control_id, action, "observed")
            self.render_all()
            return

        if self.current_index >= len(self.sequence):
            return

        target_id, target_action = self.sequence[self.current_index]
        if control_id == target_id and action == target_action:
            self.results[control_id][action] = "pass"
            self.log_event(source_key, control_id, action, "pass")
            self.current_index += 1
        else:
            self.attempts[target_id][target_action] += 1
            self.log_event(
                source_key, control_id, action,
                f"unexpected (target {self.by_id[target_id]['label']} {target_action})",
            )
        self.render_all()

    # ---------------------------------------------------------- controls ---

    def start_test(self):
        self.reset_results()
        self.event_log = []
        self.sequence = self.base_sequence()
        if self.randomize.get():
            random.shuffle(self.sequence)
        self.current_index = 0
        self.running = True
        self.render_all()

    def skip_step(self):
        if not self.running or self.free_mode.get() or self.current_index >= len(self.sequence):
            return
        cid, action = self.sequence[self.current_index]
        self.results[cid][action] = "flag"
        self.log_event("-", cid, action, "flagged / skipped")
        self.current_index += 1
        self.render_all()

    def reset_test(self):
        self.running = False
        self.current_index = -1
        self.reset_results()
        self.event_log = []
        self.sequence = self.base_sequence()
        self.render_all()

    def export_csv(self):
        default_name = f"panel-test-{datetime.datetime.now():%Y-%m-%d-%H%M%S}.csv"
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV files", "*.csv")], initialfile=default_name
        )
        if not path:
            return
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["control", "type", "action", "result", "misses", "key"])
            for c in CONTROLS:
                for action, key in c["actions"]:
                    writer.writerow([
                        c["label"], c["type"], action,
                        self.results[c["id"]][action], self.attempts[c["id"]][action], key,
                    ])
            writer.writerow([])
            writer.writerow(["time", "key", "control", "action", "outcome"])
            for row in self.event_log:
                writer.writerow(row)


if __name__ == "__main__":
    TestRig().mainloop()