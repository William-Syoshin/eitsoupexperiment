import tkinter as tk
from tkinter import ttk, messagebox
import threading
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from collections import deque
import csv
import time
import os

MAX_POINTS = 300
NUM_CH = 8
CHANNEL_LABELS = [
    "Ch1 (0→1)", "Ch2 (1→2)", "Ch3 (2→3)", "Ch4 (3→4)",
    "Ch5 (4→5)", "Ch6 (5→6)", "Ch7 (6→7)", "Ch8 (7→0)"
]
COLORS = ["#e74c3c", "#2ecc71", "#3498db", "#f39c12",
          "#9b59b6", "#1abc9c", "#e67e22", "#ecf0f1"]

BTN_FONT  = ("Arial", 12, "bold")
LBL_FONT  = ("Arial", 12)
VAL_FONT  = ("Courier", 13, "bold")
BTN_H     = 2     # button height in text unit
BTN_W     = 12


class EITApp:
    def __init__(self, root):
        self.root = root
        self.root.title("EIT Monitor")
        self.root.resizable(True, True)

        self.serial = None
        self.running = False
        self.recording = False
        self.csv_writer = None
        self.csv_file = None
        self.record_count = 0

        self.data = [deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS) for _ in range(NUM_CH)]
        self.latest = [0.0] * NUM_CH

        self._build_ui()
        self._refresh_ports()

    # ── UI ──────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── top bar ──
        top = tk.Frame(self.root, padx=10, pady=8)
        top.pack(fill=tk.X)

        tk.Label(top, text="Port:", font=LBL_FONT).pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var,
                                    width=10, state="readonly", font=LBL_FONT)
        self.port_cb.pack(side=tk.LEFT, padx=(4, 12))

        tk.Label(top, text="Baud:", font=LBL_FONT).pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top, textvariable=self.baud_var,
                     values=["9600", "57600", "115200", "230400"],
                     width=8, state="readonly", font=LBL_FONT).pack(side=tk.LEFT, padx=(4, 12))

        self.btn_connect = tk.Button(top, text="Connect", width=BTN_W, height=BTN_H,
                                     font=BTN_FONT, command=self._toggle_connect, bg="#2ecc71")
        self.btn_connect.pack(side=tk.LEFT, padx=4)

        tk.Button(top, text="Refresh", width=9, height=BTN_H,
                  font=BTN_FONT, command=self._refresh_ports).pack(side=tk.LEFT, padx=4)

        tk.Button(top, text="Clear", width=9, height=BTN_H,
                  font=BTN_FONT, command=self._clear,
                  bg="#9b59b6", fg="white").pack(side=tk.LEFT, padx=4)

        self.btn_record = tk.Button(top, text="Start Record", width=BTN_W, height=BTN_H,
                                    font=BTN_FONT, command=self._toggle_record,
                                    bg="#3498db", state=tk.DISABLED)
        self.btn_record.pack(side=tk.LEFT, padx=(20, 4))

        self.lbl_file = tk.Label(top, text="", font=("Arial", 11), fg="#555")
        self.lbl_file.pack(side=tk.LEFT, padx=4)

        # ── live value panel ──
        val_frame = tk.Frame(self.root, padx=10, pady=4)
        val_frame.pack(fill=tk.X)
        self.val_labels = []
        for i in range(NUM_CH):
            tk.Label(val_frame, text=CHANNEL_LABELS[i] + ":",
                     fg=COLORS[i], font=VAL_FONT).pack(side=tk.LEFT, padx=(0, 4))
            lbl = tk.Label(val_frame, text="-.----", width=7,
                           font=VAL_FONT, fg=COLORS[i])
            lbl.pack(side=tk.LEFT, padx=(0, 24))
            self.val_labels.append(lbl)

        # ── chart ──
        self.fig, self.ax = plt.subplots(figsize=(10, 4.5), tight_layout=True)
        self.ax.set_facecolor("#1e1e1e")
        self.fig.patch.set_facecolor("#2d2d2d")
        self.ax.tick_params(colors="white", labelsize=10)
        for spine in self.ax.spines.values():
            spine.set_edgecolor("#555")
        self.ax.set_title("EIT — RMS Voltage per Electrode Pair", color="white", fontsize=12)
        self.ax.set_xlabel("Samples", color="white", fontsize=11)
        self.ax.set_ylabel("RMS (V)", color="white", fontsize=11)
        self.ax.set_xlim(0, MAX_POINTS)
        self.ax.set_ylim(0, 3.0)
        self.ax.grid(color="#444", linewidth=0.5)

        self.lines = [
            self.ax.plot([], [], color=COLORS[i], lw=1.5, label=CHANNEL_LABELS[i])[0]
            for i in range(NUM_CH)
        ]
        self.ax.legend(loc="upper right", fontsize=10,
                       facecolor="#333", labelcolor="white", framealpha=0.7)

        canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.canvas = canvas

        self.anim = FuncAnimation(self.fig, self._update_chart, interval=150, blit=True)

        # ── status bar ──
        self.status_var = tk.StringVar(value="Disconnected")
        tk.Label(self.root, textvariable=self.status_var, anchor=tk.W,
                 font=("Arial", 11), relief=tk.SUNKEN, padx=8).pack(fill=tk.X, side=tk.BOTTOM)

    # ── serial ──────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showerror("Error", "Select a port first.")
            return
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            self.running = True
            self.btn_connect.config(text="Disconnect", bg="#e74c3c")
            self.btn_record.config(state=tk.NORMAL)
            self.status_var.set(f"Connected — {port} @ {baud}")
            threading.Thread(target=self._read_loop, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))

    def _disconnect(self):
        self.running = False
        if self.recording:
            self._stop_record()
        if self.serial:
            self.serial.close()
            self.serial = None
        self.btn_connect.config(text="Connect", bg="#2ecc71")
        self.btn_record.config(state=tk.DISABLED)
        self.status_var.set("Disconnected")

    def _read_loop(self):
        while self.running:
            try:
                raw = self.serial.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue
                if raw.startswith("["):
                    self.status_var.set(f"Device: {raw}")
                    continue
                parts = raw.split(",")
                if len(parts) != NUM_CH:
                    continue
                values = [float(p.strip()) for p in parts]
                for i in range(NUM_CH):
                    self.data[i].append(values[i])
                self.latest = values
                if self.recording and self.csv_writer:
                    self.csv_writer.writerow([time.time()] + values)
                    self.record_count += 1
                    self.status_var.set(
                        f"Recording — {self.record_count} rows — {os.path.basename(self.csv_file.name)}")
            except Exception:
                if self.running:
                    self.status_var.set("Read error — reconnect")
                    self.running = False
                break

    # ── chart ───────────────────────────────────────────────────────────────

    def _update_chart(self, _frame):
        for i, line in enumerate(self.lines):
            line.set_data(range(len(self.data[i])), list(self.data[i]))
            self.val_labels[i].config(text=f"{self.latest[i]:.4f}")

        all_vals = [v for ch in self.data for v in ch if v != 0.0]
        if all_vals:
            ymin = max(0, min(all_vals) - 0.05)
            ymax = max(all_vals) + 0.1
            self.ax.set_ylim(ymin, ymax)

        return self.lines

    def _clear(self):
        for ch in self.data:
            ch.clear()
            ch.extend([0.0] * MAX_POINTS)
        self.latest = [0.0] * NUM_CH
        for line in self.lines:
            line.set_data([], [])
        for lbl in self.val_labels:
            lbl.config(text="-.----")
        self.ax.set_ylim(0, 3.0)
        # blit=True caches the background, so force a full redraw
        self.anim.event_source.stop()
        self.canvas.draw()
        self.anim.event_source.start()
        self.status_var.set("Display cleared")

    # ── recording ───────────────────────────────────────────────────────────

    def _toggle_record(self):
        if self.recording:
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(os.path.dirname(__file__), f"eit_{ts}.csv")
        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp"] + CHANNEL_LABELS)
        self.record_count = 0
        self.recording = True
        self.btn_record.config(text="Stop Record", bg="#e74c3c")
        self.lbl_file.config(text=os.path.basename(path))

    def _stop_record(self):
        self.recording = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        self.btn_record.config(text="Start Record", bg="#3498db")
        self.status_var.set(f"Saved {self.record_count} rows — {self.lbl_file.cget('text')}")


if __name__ == "__main__":
    root = tk.Tk()
    app = EITApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app._disconnect(), root.destroy()))
    root.mainloop()
