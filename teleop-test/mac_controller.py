"""Mac keyboard console for the ARX X7s."""

import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from control_bridge import PREFIX
from robot_session import DRIVE_KEYS, JOG_KEYS


ROOT = Path(__file__).resolve().parent
ROBOT = "arx@10.90.134.126"
REMOTE = 'exec bash "$HOME/Documents/Teleop Test Enviroment/run-controller.sh"'

BG, PANEL, EDGE = "#10171e", "#19242e", "#293947"
INK, MUTED, QUIET = "#f2f6fa", "#b6c6d2", "#8ea1b1"
ACCENT, ACCENT_INK, DANGER = "#84e5c1", "#10271f", "#ffaaa1"
KEY_BG, ACTIVE_BG = "#2b3b49", "#84e5c1"
FONT, MONO = "Helvetica Neue", "Menlo"

CHASSIS = [(("w", "W"), "Forward"), (("s", "S"), "Back"),
           (("a", "A"), "Strafe left"), (("d", "D"), "Strafe right"),
           (("left", "←"), "Turn left"), (("right", "→"), "Turn right")]
ARMS = [(("i", "I"), "Up"), (("k", "K"), "Down"),
        (("j", "J"), "Left"), (("l", "L"), "Right"),
        (("u", "U"), "Forward"), (("o", "O"), "Back")]
BODY = [(("r", "R"), "Raise lift"), (("f", "F"), "Lower lift"),
        (("q", "Q"), "Tilt torso"), (("e", "E"), "Toward upright")]
TOOLS = [(("g", "G"), "Open grippers"), (("h", "H"), "Close grippers"),
         (("1", "1"), "Head pitch +"), (("2", "2"), "Head pitch −")]
STATUS = {
    "offline": ("Disconnected", QUIET), "connecting": ("Connecting", QUIET),
    "idle": ("Standby", MUTED), "homing": ("Homing", "#ffcf83"),
    "resting": ("Moving to rest", "#ffcf83"), "ready": ("Ready", ACCENT),
    "paused": ("Paused", "#ffcf83"), "fault": ("Motion locked", DANGER),
}
CONTROL_KEYS = set(DRIVE_KEYS) | JOG_KEYS
HEARTBEAT_WAIT = 0.6
CONNECT_WAIT = 12.0


class Button(ttk.Button):
    def __init__(self, parent, title, command, style):
        super().__init__(parent, text=title, command=command, style=style, takefocus=True)
        self.bind("<Return>", self.activate)
        widget, widget_class, top, all_widgets = self.bindtags()
        self.bindtags((widget, top, widget_class, all_widgets))
        self.enabled = True

    def activate(self, event):
        self.invoke()
        return "break"

    def set(self, *, title=None, enabled=True):
        self.enabled = enabled
        self.state(["!disabled" if enabled else "disabled"])
        if title is not None and title != self["text"]:
            self.configure(text=title)


class ControllerWindow:
    def __init__(self, root, socket_path, simulation):
        self.root = root
        self.socket_path = socket_path
        self.simulation = simulation
        self.events = queue.SimpleQueue()
        self.process = None
        self.state = {}
        self.keys = set()
        self.release_jobs = {}
        self.pending_action = ""
        self.enable_requested = False
        self.live_started = False
        self.space_blocked = False
        self.last_status = 0.0
        self.keycaps = {}
        self.closing = False
        self.waiting = False

        root.title("ARX X7s Controller")
        root.configure(bg=BG)
        root.minsize(700, 630)
        root.geometry("800x690")
        root.bind("<KeyPress>", self.press)
        root.bind("<KeyRelease>", self.release)
        root.bind("<FocusOut>", lambda event: root.after_idle(self.check_focus))
        root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style(root)
        style.theme_use("clam")
        for name, background, foreground in (
            ("Connect", KEY_BG, INK), ("Enable", ACCENT, ACCENT_INK), ("Stop", "#40282d", DANGER),
        ):
            button = f"{name}.TButton"
            style.configure(button, background=background, foreground=foreground,
                            bordercolor=BG, lightcolor=background, darkcolor=background,
                            focuscolor=foreground, relief="flat", padding=(14, 8),
                            font=(FONT, 13, "bold"))
            style.map(button, background=[("disabled", PANEL), ("pressed", EDGE), ("active", background)],
                      foreground=[("disabled", QUIET), ("pressed", INK)])
        self.build()
        self.render()
        root.after(15, self.poll)

    @staticmethod
    def label(parent, text, size=13, color=INK, bold=False, mono=False):
        return tk.Label(parent, text=text, anchor="w", justify="left", bg=parent["bg"], fg=color,
                        font=(MONO if mono else FONT, size, "bold" if bold else "normal"))

    def build(self):
        content = tk.Frame(self.root, bg=BG)
        content.pack(fill="both", expand=True, padx=26, pady=20)
        header = tk.Frame(content, bg=BG)
        header.pack(fill="x")
        title = tk.Frame(header, bg=BG)
        title.pack(side="left")
        self.label(title, "ARX X7s Controller", 28, bold=True).pack(anchor="w")
        self.label(title, "Local simulator" if self.simulation else "Lab robot over SSH",
                   12, MUTED).pack(anchor="w", pady=(2, 0))
        buttons = tk.Frame(header, bg=BG)
        buttons.pack(side="right")
        self.connect_button = Button(buttons, "Connect", self.toggle_connection, "Connect.TButton")
        self.connect_button.pack(side="left")
        self.enable_button = Button(buttons, "Enable", self.enable, "Enable.TButton")
        self.enable_button.pack(side="left", padx=5)
        self.stop_button = Button(buttons, "Stop · Esc", self.stop, "Stop.TButton")
        self.stop_button.pack(side="left")

        status = tk.Frame(content, bg=PANEL, padx=14, pady=10, highlightthickness=1,
                          highlightbackground=EDGE)
        status.pack(fill="x", pady=(16, 10))
        status_header = tk.Frame(status, bg=PANEL)
        status_header.pack(fill="x")
        self.phase = self.label(status_header, "", 18, ACCENT, bold=True)
        self.phase.pack(side="left")
        self.notice = self.label(status, "", 12, MUTED)
        self.notice.configure(wraplength=600)
        status.bind("<Configure>", lambda event: self.notice.configure(wraplength=max(400, event.width - 28)))
        self.notice.pack(anchor="w", pady=(3, 7))
        self.startup = self.label(status, "", 12, MUTED)
        self.startup.pack(anchor="w")

        deadman_bar = tk.Frame(content, bg=BG)
        deadman_bar.pack(fill="x", pady=(1, 10))
        self.keycap(deadman_bar, "space", "SPACE", width=7).pack(side="left", padx=(0, 8))
        self.deadman = self.label(deadman_bar, "", 13, bold=True)
        self.deadman.pack(side="left")

        grid = tk.Frame(content, bg=BG)
        grid.pack(fill="both", expand=True)
        for column in range(2):
            grid.columnconfigure(column, weight=1, uniform="cards")
        for row in range(2):
            grid.rowconfigure(row, weight=1)
        self.chassis = self.card(grid, 0, 0, "Chassis", "Hold to drive", CHASSIS)
        self.arms = self.card(grid, 0, 1, "Both arms", "Tap for 5 mm steps", ARMS)
        self.body = self.card(grid, 1, 0, "Torso & lift", "Tap to jog", BODY)
        self.tools = self.card(grid, 1, 1, "Grippers & head", "Tap to jog", TOOLS)
        self.footer = self.label(content, "Keep the controller window in focus while driving.", 11, QUIET)
        self.footer.pack(anchor="w", pady=(10, 0))

    def keycap(self, parent, key, text, width=2):
        cap = tk.Label(parent, text=text, width=width, padx=4, pady=3, bg=KEY_BG, fg=INK,
                       font=(MONO, 12, "bold"))
        self.keycaps[key] = cap
        return cap

    def card(self, parent, row, column, title, instruction, bindings):
        panel = tk.Frame(parent, bg=PANEL, padx=13, pady=10, highlightthickness=1,
                         highlightbackground=EDGE)
        panel.grid(row=row, column=column, sticky="nsew", padx=(0, 5) if not column else (5, 0),
                   pady=(0, 5) if not row else (5, 0))
        heading = tk.Frame(panel, bg=PANEL)
        heading.pack(fill="x", pady=(0, 8))
        self.label(heading, title, 16, bold=True).pack(side="left")
        self.label(heading, instruction, 11, QUIET).pack(side="right")
        keys = tk.Frame(panel, bg=PANEL)
        keys.pack(fill="x")
        for index, ((key, symbol), description) in enumerate(bindings):
            cell = tk.Frame(keys, bg=PANEL)
            cell.grid(row=index // 2, column=index % 2, sticky="w", pady=2)
            keys.columnconfigure(index % 2, weight=1, uniform="keys")
            self.keycap(cell, key, symbol).pack(side="left")
            self.label(cell, description, 12, MUTED).pack(side="left", padx=(6, 0))
        measurement = self.label(panel, "—", 11, MUTED, mono=True)
        measurement.pack(anchor="w", side="bottom", pady=(7, 0))
        return measurement

    def clear_input(self):
        self.space_blocked |= "space" in self.keys
        self.keys.clear()
        self.pending_action = ""
        for job in self.release_jobs.values():
            self.root.after_cancel(job)
        self.release_jobs.clear()

    def toggle_connection(self):
        if self.process:
            if self.confirm_release():
                self.disconnect()
        else:
            self.connect()

    def connect(self):
        self.clear_input()
        if self.simulation:
            command = [sys.executable, "-Bu", str(ROOT / "control_bridge.py")]
        else:
            command = ["/usr/bin/ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                       "-o", "StrictHostKeyChecking=yes", "-o", "ServerAliveInterval=2",
                       "-o", "ServerAliveCountMax=2", "-o", "ClearAllForwardings=yes",
                       "-o", "ForwardAgent=no", "-o", "UpdateHostKeys=no"]
            if self.socket_path:
                command += ["-S", self.socket_path, "-o", "ProxyCommand=/usr/bin/false"]
            command += [ROBOT, REMOTE]
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, bufsize=0, start_new_session=True)
        except OSError as exc:
            self.render(str(exc))
            return
        os.set_blocking(self.process.stdin.fileno(), False)
        self.last_status = time.monotonic()
        self.waiting = True
        self.root.focus_set()
        threading.Thread(target=self.read, args=(self.process,), daemon=True).start()
        self.render()

    def read(self, process):
        with process.stdout:
            for line in process.stdout:
                if line.startswith(PREFIX):
                    try:
                        self.events.put((process, "state", json.loads(line[len(PREFIX):])))
                    except ValueError:
                        pass
                elif line.startswith(b"ARX1 "):
                    self.events.put((process, "old", None))
                    break
                elif line.strip():
                    self.events.put((process, "log", line.decode(errors="replace").strip()))
        self.events.put((process, "closed", process.wait()))

    def send(self, frame):
        if not self.process:
            return False
        try:
            os.write(self.process.stdin.fileno(), json.dumps(frame).encode() + b"\n")
        except (OSError, ValueError):
            return False
        return True

    def poll(self):
        latest = None
        detail = None
        while True:
            try:
                process, kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if process is not self.process:
                continue
            if kind == "state":
                latest = value
            elif kind == "old":
                self.disconnect("The robot has the old controller. Run Send Controller Update.command.")
                latest = None
            elif kind == "log":
                detail = value[:175]
            elif kind == "closed":
                self.disconnect(detail or "Controller connection ended. If powered, check actuator support.")
                latest = None
        if latest and self.process:
            old_phase = self.state.get("phase")
            self.state = latest
            self.last_status = time.monotonic()
            self.waiting = False
            phase = latest["phase"]
            self.live_started |= latest["live"] and latest["started"]
            if self.pending_action != "enable":
                self.enable_requested = False
                if phase == "idle":
                    self.live_started = False
            if phase != old_phase and phase != "idle" and self.pending_action != "enable":
                self.clear_input()
            focused = self.root.focus_displayof() is not None
            deadman = focused and not self.space_blocked and "space" in self.keys and phase == "ready"
            action = self.pending_action
            if not focused or action != "enable" and not deadman:
                action = ""
            if self.send({"lease": latest["lease"], "deadman": deadman,
                          "keys": sorted(self.keys & DRIVE_KEYS.keys()) if deadman else [], "action": action}):
                if action == "enable" and not self.simulation:
                    self.live_started = True
                self.pending_action = ""
            self.render()
        elif self.process:
            delay = time.monotonic() - self.last_status
            if detail and not self.state:
                self.render(detail)
            if self.state and delay > HEARTBEAT_WAIT and not self.waiting:
                self.stop()
                self.waiting = True
                self.render("Feedback interrupted. Stop requested.")
            elif not self.state and delay > CONNECT_WAIT:
                self.disconnect("The controller did not respond. Check the lab connection and SDK startup.")
        self.root.after(15, self.poll)

    def render(self, notice=None):
        state = self.state
        phase = state.get("phase", "connecting" if self.process else "offline")
        label, color = STATUS[phase]
        if self.waiting and state:
            label, color = "Connection interrupted", DANGER
        self.phase.configure(text=f"●  {label}", fg=color)
        if notice is None:
            notice = state.get("notice") or ("Opening controller session…" if self.process else
                                            "Connect to open a controller session.")
        self.notice.configure(text=notice)
        self.connect_button.set(title="Disconnect" if self.process else "Connect")
        self.enable_button.set(enabled=phase in ("idle", "paused") and not self.enable_requested and not self.waiting)
        self.stop_button.set(enabled=bool(self.process))
        arms = state.get("arms", {})
        body = state.get("body", {})
        if phase == "homing":
            lift = "Zeroed" if "0" in body else "Homing"
            torso = "Zeroed" if "1" in body else "Homing"
        else:
            body_status = {"ready": "Holding", "paused": "Holding", "resting": "Moving"}.get(phase, "Feedback")
            moving = state.get("body_moving", ())
            lift = ("Moving" if 0 in moving else body_status) if "0" in body else "—"
            torso = ("Moving" if 1 in moving else "Holding" if phase == "resting" else body_status) if "1" in body else "—"
        self.startup.configure(text=f"Left arm  {arms.get('left', '—')}     Right arm  {arms.get('right', '—')}"
                                    f"     Lift  {lift}     Torso  {torso}")
        active = "space" in self.keys and not self.space_blocked and phase == "ready" and not self.waiting
        deadman_text = ("Controls locked" if phase != "ready" or self.waiting else
                        "Release Space to arm" if self.space_blocked else
                        "Keyboard active" if active else "Hold Space while using the keys")
        self.deadman.configure(text=deadman_text, fg=ACCENT if active else MUTED)
        for key, cap in self.keycaps.items():
            pressed = active and key in self.keys
            cap.configure(bg=ACTIVE_BG if pressed else KEY_BG, fg=ACCENT_INK if pressed else INK)

        velocity = state.get("velocity", ())
        wheels = state.get("wheels")
        wheel_values = "  ".join(f"{value:+.1f}" for value in wheels) if wheels is not None else "—"
        drive = "Drive sent" if any(velocity) else "Brake sent" if state.get("started") else "Idle"
        self.chassis.configure(text=f"{drive}   Wheels {wheel_values}", fg=ACCENT if any(velocity) else MUTED)
        poses = state.get("poses", {})
        readings = [f"{side[0].upper()} {poses[side][2]:.3f} m" for side in ("left", "right") if side in poses]
        self.arms.configure(text="Z   " + "    ".join(readings) if readings else "Z   —")
        lift = f"{body['0']:.2f}" if "0" in body else "—"
        torso = f"{body['1']:.2f} rad" if "1" in body else "—"
        pitch = f"{body['3']:.2f} rad" if "3" in body else "—"
        self.body.configure(text=f"Lift {lift}    Torso {torso}")
        self.tools.configure(text=f"Pitch {pitch}    Grip {state.get('gripper', '—').lower()}")

    def enable(self):
        if not self.enable_button.enabled or self.enable_requested:
            return
        self.clear_input()
        if not self.simulation and not messagebox.askokcancel(
            "Enable physical hardware",
            "Enable can move both arms, the lift, and the torso. Confirm the robot is in view, "
            "the area is clear, and the physical stop is within reach.",
            parent=self.root,
        ):
            return
        self.root.focus_set()
        self.enable_requested = True
        self.pending_action = "enable"
        self.render("Enable requested…")

    def press(self, event):
        key = event.keysym.lower()
        if key == "escape":
            self.stop()
            return "break"
        if key not in CONTROL_KEYS and key != "space":
            return
        if key in self.release_jobs:
            self.root.after_cancel(self.release_jobs.pop(key))
        if self.state.get("phase") != "ready" or self.waiting:
            if key == "space":
                self.space_blocked = True
            return "break"
        if key in self.keys:
            return "break"
        if event.state & (0x0004 | 0x0008 | 0x0010):
            if key == "space":
                self.space_blocked = True
            return "break"
        if self.space_blocked:
            return "break"
        if key != "space" and "space" not in self.keys:
            return "break"
        self.keys.add(key)
        if key in JOG_KEYS and not self.pending_action:
            self.pending_action = key
        self.render()
        return "break"

    def release(self, event):
        key = event.keysym.lower()
        if key not in self.keys and key != "space":
            return
        if key in self.release_jobs:
            self.root.after_cancel(self.release_jobs.pop(key))

        def finish():
            self.keys.discard(key)
            self.release_jobs.pop(key, None)
            if key == "space":
                self.space_blocked = False
                self.keys.clear()
                self.pending_action = ""
            self.render()

        self.release_jobs[key] = self.root.after_idle(finish)

    def check_focus(self):
        if not self.closing and self.root.focus_displayof() is None:
            if self.state.get("phase") in ("homing", "resting", "ready") or self.enable_requested:
                self.stop()

    def stop(self):
        self.clear_input()
        self.enable_requested = False
        self.send({"op": "stop"})
        self.render("Stop requested." if self.process else None)

    def confirm_release(self):
        if self.simulation or not (self.live_started or self.enable_requested):
            return True
        return messagebox.askokcancel(
            "Disconnect and release torque?",
            "Disconnecting can release motor torque. Support the arms, lift, and torso first. "
            "Use Stop to brake and request position hold while connected.", parent=self.root,
        )

    def disconnect(self, notice=None):
        self.clear_input()
        self.send({"op": "quit"})
        process, self.process = self.process, None
        if process:
            process.stdin.close()
        self.state = {}
        self.enable_requested = self.live_started = self.waiting = False
        self.render(notice)

    def close(self):
        if self.confirm_release():
            self.closing = True
            self.disconnect()
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default=os.environ.get("ARX_SSH_SOCKET"))
    parser.add_argument("--simulation", action="store_true")
    parser.add_argument("--connect", action="store_true")
    args = parser.parse_args()
    root = tk.Tk()
    ui = ControllerWindow(root, args.socket, args.simulation or not (args.socket or args.connect))
    if args.connect:
        root.after_idle(ui.connect)
    root.mainloop()


if __name__ == "__main__":
    main()
