"""Mac view of the robot's read-only chassis and body feedback."""

import argparse
import json
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

from mac_controller import ROBOT


class EncoderWindow:
    def __init__(self, root, socket_path):
        self.root = root
        self.events = queue.SimpleQueue()
        self.last_update = 0.0
        root.title("ARX X7s Encoder Monitor")
        root.minsize(670, 430)
        root.protocol("WM_DELETE_WINDOW", self.close)
        page = ttk.Frame(root, padding=22)
        page.pack(fill="both", expand=True)
        ttk.Label(page, text="Encoder monitor", font=("Helvetica Neue", 24, "bold")).pack(anchor="w")
        self.status = ttk.Label(page, text="Connecting to can5…")
        self.status.pack(anchor="w", pady=(2, 15))
        self.wheels = ttk.Label(page, font=("Menlo", 13))
        self.wheels.pack(anchor="w")
        self.drive = ttk.Label(page, font=("Menlo", 11))
        self.drive.pack(anchor="w", pady=(9, 15))
        self.table = ttk.Treeview(page, columns=("axis", "angle", "speed", "status", "command"),
                                  show="headings", height=4)
        for column, title, width in zip(self.table["columns"],
                ("Body motor", "Angle (rad)", "Speed (rad/s)", "Status", "Host command"),
                (150, 106, 113, 65, 132)):
            self.table.heading(column, text=title)
            self.table.column(column, width=width, anchor="w")
        for ident, label in ((1, "Lift"), (4, "Waist pivot"), (5, "Head yaw"), (6, "Head pitch")):
            self.table.insert("", "end", iid=label, values=(f"{label} · ID {ident}", "—", "—", "—", "—"))
        self.table.pack(fill="x")
        ttk.Label(page, text="Body angles are raw motor readings, before linkage conversion.").pack(anchor="w", pady=(7, 3))
        self.totals = ttk.Label(page, wraplength=630, justify="left")
        self.totals.pack(anchor="w")
        command = ["/usr/bin/ssh", "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                   "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                   "-o", "ClearAllForwardings=yes", "-o", "ForwardAgent=no", "-o", "UpdateHostKeys=no"]
        if socket_path:
            command += ["-S", socket_path]
        command += [ROBOT, 'exec python3 -Bu "$HOME/Documents/Teleop Test Enviroment/can_monitor.py"']
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        threading.Thread(target=self.read, daemon=True).start()
        root.after(100, self.poll)

    def read(self):
        with self.process.stdout:
            for line in self.process.stdout:
                try:
                    self.events.put(json.loads(line))
                except ValueError:
                    self.events.put(line.decode(errors="replace").strip())
        self.events.put("Monitor disconnected.")

    def poll(self):
        latest = None
        while True:
            try:
                value = self.events.get_nowait()
            except queue.Empty:
                break
            if isinstance(value, dict):
                latest = value
            elif value:
                latest = None
                self.status.configure(text=value)
        if latest:
            self.last_update = time.monotonic()
            self.status.configure(text="Connected · can5 · Read-only")
            self.render(latest)
        elif self.last_update and time.monotonic() - self.last_update > 0.8:
            if self.process.poll() is None:
                self.status.configure(text="Waiting for fresh telemetry…")
            self.wheels.configure(text="Wheel speeds unavailable")
            self.drive.configure(text="Chassis command unavailable")
            for label in self.table.get_children():
                self.table.item(label, values=(self.table.item(label, "values")[0], "—", "—", "—", "—"))
        self.root.after(100, self.poll)

    @staticmethod
    def values(values):
        return "   ".join(f"{value:+.3f}" for value in values) if values is not None else "—"

    def render(self, data):
        self.wheels.configure(text=f"Wheels 1–4   {self.values(data['wheels'])} rad/s\n"
                                   f"Peak |speed| {self.values(data['peak'])} rad/s")
        current = data["drive"]
        last = data["last_drive"]
        def command_text(command):
            if command is None:
                return "—"
            return f"mode {command['mode']}    X {command['x']:+.3f}    Y {command['y']:+.3f}    yaw {command['yaw']:+.3f}"
        self.drive.configure(text=f"Host chassis       {command_text(current)}\n"
                                  f"Last nonzero drive {command_text(last)}")
        for label, motor in data["motors"].items():
            feedback = motor["feedback"]
            values = (f"{feedback['angle']:+.3f}", f"{feedback['speed']:+.3f}", str(feedback["status"])) if feedback else ("—", "—", "—")
            self.table.item(label, values=(f"{label} · ID {motor['id']}", *values, motor["command"] or "—"))
        counts = data["count"]
        zeros = [key.removeprefix("Set zero ") for key in counts if key.startswith("Set zero ")]
        disabled = sum(count for key, count in counts.items() if key.startswith("Disable "))
        self.totals.configure(text=f"Wheel reports {counts.get('wheel_reports', 0):,}   ·   "
            f"Nonzero drive frames {counts.get('nonzero_drive', 0):,}   ·   Motor disables {disabled:,}\n"
            f"Set-zero commands {', '.join(zeros) or '—'}   ·   Switch {data['switch'] or '—'}")

    def close(self):
        self.process.terminate()
        self.root.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket")
    args = parser.parse_args()
    root = tk.Tk()
    EncoderWindow(root, args.socket)
    root.mainloop()
