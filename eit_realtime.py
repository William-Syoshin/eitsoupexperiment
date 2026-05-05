"""
Real-time EIT Reconstruction & Visualization
8-electrode system, adjacent drive/measurement (dist_exc=1, step_meas=1)

Arduino outputs 40 comma-separated RMS values per line @ 115200 baud.

Usage:
    python eit_realtime.py                          # auto-detect port
    python eit_realtime.py --port /dev/cu.usbmodem1234
    python eit_realtime.py --port /dev/cu.usbmodem1234 --save
    python eit_realtime.py --clim 0.10              # tighter color scale floor

Keys:
    r  — reset reference baseline (re-collect v0)
    q  — quit
"""

import sys
import os
import glob
import threading
import queue
import argparse
import time
import csv

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# ── Locate local pyEIT installation ──────────────────────────────────────────
_here = os.path.dirname(os.path.abspath(__file__))
for _candidate in ('pyEIT-master', 'pyEIT', 'pyeit'):
    _p = os.path.join(_here, _candidate)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
        break

import pyeit.mesh as mesh
import pyeit.eit.protocol as protocol
from pyeit.eit.protocol import PyEITProtocol
from pyeit.eit.jac import JAC
from pyeit.eit.bp import BP
from pyeit.eit.interp2d import sim2pts
import serial
import serial.tools.list_ports

# ─── Parameters ───────────────────────────────────────────────────────────────
N_EL        = 8
N_MEAS      = (N_EL // 2) * (N_EL - 4)  # 4 drives x 4 meas = 16
BAUD_RATE   = 115200
H0          = 0.08                  # mesh density
LAMB        = 0.01                  # JAC regularization
N_REF       = 8                     # frames averaged for reference v0
HISTORY_LEN = 120                   # frames shown in trend plot
UPDATE_MS   = 120                   # animation refresh interval (ms)

# ─── Build mesh & solvers ──────────────────────────────────────────────────────
print("[init] Building mesh and solvers...")
mesh_obj = mesh.create(N_EL, h0=H0)

# 対向法・両サイドのみプロトコル（4 drive x 4 meas = 16）
# pyeitの dist_exc=4 は双方向8driveを生成するため、前半4drive（src<sink）だけ使う
_p_full      = protocol.create(N_EL, dist_exc=N_EL//2, step_meas=1, parser_meas="std")
protocol_obj = PyEITProtocol(
    ex_mat   = _p_full.ex_mat[:4],     # drive (0,4),(1,5),(2,6),(3,7) のみ
    meas_mat = _p_full.meas_mat[:4],   # 各driveの4測定ペア
    keep_ba  = _p_full.keep_ba[:32],   # 対応するkeep_ba
)

jac_solver = JAC(mesh_obj, protocol_obj)
jac_solver.setup(p=0.2, lamb=LAMB, method="kotre")

bp_solver = BP(mesh_obj, protocol_obj)
bp_solver.setup(weight="none")

pts = mesh_obj.node      # (n_nodes, 3)  node coordinates (x, y, z)
tri = mesh_obj.element   # (n_elem, 3)   triangle indices
n_nd = pts.shape[0]
n_el = tri.shape[0]

# Electrode positions on the mesh (x, y only)
el_xy = pts[mesh_obj.el_pos, :2]

print(f"[init] {n_nd} nodes, {n_el} elements, {protocol_obj.n_meas_tot} measurements")

# ─── Serial reader thread ──────────────────────────────────────────────────────
data_q      = queue.Queue(maxsize=8)
_stop_evt   = threading.Event()


def _parse(line: str):
    """Parse a comma-separated data line. Returns ndarray or None."""
    line = line.strip()
    if not line or line.startswith('['):
        return None
    parts = line.split(',')
    if len(parts) != N_MEAS:
        return None
    try:
        return np.array([float(p) for p in parts])
    except ValueError:
        return None


def serial_reader(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=2)
        print(f"[serial] Opened {port} @ {baud} baud")
        while not _stop_evt.is_set():
            try:
                line = ser.readline().decode('utf-8', errors='ignore')
            except serial.SerialException:
                break
            v = _parse(line)
            if v is not None:
                if data_q.full():
                    try:
                        data_q.get_nowait()   # drop oldest, keep fresh
                    except queue.Empty:
                        pass
                data_q.put(v)
        ser.close()
        print("[serial] Closed")
    except serial.SerialException as e:
        print(f"[serial] Cannot open {port}: {e}")
        print("  Check USB cable and port name.")


def find_port():
    """Auto-detect Teensy / Arduino serial port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        d = (p.device + ' ' + (p.description or '')).lower()
        if any(k in d for k in ('usbmodem', 'usbserial', 'acm', 'ttyusb',
                                 'teensy', 'arduino', 'ftdi', 'ch340', 'cp210')):
            return p.device
    candidates = (glob.glob('/dev/cu.usbmodem*') +
                  glob.glob('/dev/ttyUSB*') +
                  glob.glob('/dev/ttyACM*'))
    return candidates[0] if candidates else (ports[0].device if ports else None)


# ─── App state ─────────────────────────────────────────────────────────────────
class _S:
    ref_buf    = []
    v0         = None
    frame_cnt  = 0
    n_ref      = N_REF
    clim_min   = 0.05          # minimum color scale limit (set via --clim)
    ds_history = np.zeros(HISTORY_LEN)
    csv_file   = None
    csv_writer = None
    reset_flag = False


S = _S()

# ─── Figure layout ────────────────────────────────────────────────────────────
BG, PAN, GR, FG = '#111827', '#1f2937', '#374151', '#f9fafb'
ACC = '#60a5fa'

fig = plt.figure(figsize=(16, 8.5), facecolor=BG)
fig.canvas.manager.set_window_title('EIT Real-time Monitor')
gs  = GridSpec(2, 3, figure=fig, hspace=0.48, wspace=0.32,
               left=0.04, right=0.97, top=0.91, bottom=0.07)

ax_jac  = fig.add_subplot(gs[0, 0])
ax_bp   = fig.add_subplot(gs[0, 1])
ax_raw  = fig.add_subplot(gs[0, 2])
ax_hist = fig.add_subplot(gs[1, :])

for ax in (ax_jac, ax_bp, ax_raw, ax_hist):
    ax.set_facecolor(PAN)
    ax.tick_params(colors=FG, labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor(GR)

fig.suptitle("EIT Real-time Monitor", fontsize=14, color=FG, fontweight='bold', y=0.965)
status_txt = fig.text(0.5, 0.935, "Initializing…",
                      ha='center', fontsize=9, color='#fbbf24')

# ── EIT mesh — Gouraud shading (smooth gradient across triangles) ──────────────
_zero_nd = np.zeros(n_nd)
tc_jac = ax_jac.tripcolor(pts[:, 0], pts[:, 1], tri, _zero_nd,
                           shading='gouraud', cmap='RdBu_r', vmin=-1, vmax=1)
tc_bp  = ax_bp.tripcolor( pts[:, 0], pts[:, 1], tri, _zero_nd,
                           shading='gouraud', cmap='RdBu_r', vmin=-1, vmax=1)

cb_jac = fig.colorbar(tc_jac, ax=ax_jac, fraction=0.046, pad=0.02)
cb_bp  = fig.colorbar(tc_bp,  ax=ax_bp,  fraction=0.046, pad=0.02)
for cb in (cb_jac, cb_bp):
    cb.ax.tick_params(colors=FG, labelsize=7)
    cb.set_label('Δσ', color=FG, fontsize=8)

_theta = np.linspace(0, 2 * np.pi, 300)
for ax in (ax_jac, ax_bp):
    ax.plot(np.cos(_theta), np.sin(_theta), color=GR, lw=0.8, alpha=0.6)
    ax.plot(el_xy[:, 0], el_xy[:, 1], 'o', ms=7, color='#fde68a',
            zorder=5, markeredgecolor='#111', markeredgewidth=0.5)
    for j, (ex, ey) in enumerate(el_xy):
        ang = np.arctan2(ey, ex)
        ax.text(1.18 * np.cos(ang), 1.18 * np.sin(ang), str(j + 1),
                ha='center', va='center', fontsize=7, color=FG)
    ax.set_aspect('equal')
    ax.set_xlim(-1.35, 1.35)
    ax.set_ylim(-1.35, 1.35)
    ax.axis('off')

ax_jac.set_title('JAC Reconstruction', color=FG, fontsize=10, pad=4)
ax_bp.set_title( 'BP Reconstruction',  color=FG, fontsize=10, pad=4)

# ── Raw voltage bar chart ──────────────────────────────────────────────────────
_x_ch    = np.arange(N_MEAS)
bars_v0  = ax_raw.bar(_x_ch, np.zeros(N_MEAS), width=0.8, alpha=0.45,
                      color='#6366f1', label='v0 (ref)')
bars_v1  = ax_raw.bar(_x_ch, np.zeros(N_MEAS), width=0.45, alpha=0.90,
                      color='#f87171', label='v1 (live)')
ax_raw.set_title('Raw RMS Voltages', color=FG, fontsize=10, pad=4)
ax_raw.set_xlabel('Channel index', color=FG, fontsize=8)
ax_raw.set_ylabel('V_rms  [V]', color=FG, fontsize=8)
ax_raw.set_xlim(-0.6, N_MEAS - 0.4)
ax_raw.legend(fontsize=7, facecolor=PAN, labelcolor=FG, loc='upper right')
for k in range(1, N_EL):
    ax_raw.axvline(k * 5 - 0.5, color=GR, lw=0.6, ls='--', alpha=0.7)
for k in range(N_EL):
    ax_raw.text(k * 5 + 2, 0, f'Tx{k}', ha='center', fontsize=5.5,
                color='#9ca3af', va='bottom')

# ── Trend history plot ─────────────────────────────────────────────────────────
_hx = np.arange(HISTORY_LEN)
hist_line, = ax_hist.plot([], [], lw=1.8, color=ACC, zorder=3)
# fill_between for ±std band — stored as mutable container to allow update
_fill_container = [ax_hist.fill_between(_hx, 0, 0, alpha=0.20, color=ACC, zorder=2)]
ax_hist.set_xlim(0, HISTORY_LEN)
ax_hist.set_ylim(-0.5, 0.5)
ax_hist.set_title(f'Δσ Spatial Mean  (last {HISTORY_LEN} frames)', color=FG, fontsize=10, pad=4)
ax_hist.set_xlabel('Frame', color=FG, fontsize=8)
ax_hist.set_ylabel('mean Δσ', color=FG, fontsize=8)
ax_hist.axhline(0, color=GR, lw=0.8)
ax_hist.grid(True, alpha=0.25, color=GR)

# ─── Keyboard handler ──────────────────────────────────────────────────────────
def on_key(event):
    if event.key == 'r':
        S.reset_flag = True
        status_txt.set_text("Reference reset requested…  collecting new baseline")
        status_txt.set_color('#fbbf24')
        fig.canvas.draw_idle()
    elif event.key in ('q', 'escape'):
        plt.close('all')


fig.canvas.mpl_connect('key_press_event', on_key)

# ─── Animation callback ────────────────────────────────────────────────────────
def animate(_frame):
    # Reference reset
    if S.reset_flag:
        S.ref_buf   = []
        S.v0        = None
        S.reset_flag = False

    try:
        v1 = data_q.get_nowait()
    except queue.Empty:
        return

    S.frame_cnt += 1

    if S.csv_writer is not None:
        S.csv_writer.writerow([time.time()] + v1.tolist())

    # ── Phase 1: collect reference frames ─────────────────────────────────────
    if len(S.ref_buf) < S.n_ref:
        S.ref_buf.append(v1)
        n = len(S.ref_buf)
        status_txt.set_text(
            f"Collecting reference baseline…  ({n}/{S.n_ref})  "
            f"— keep sensor unperturbed")
        status_txt.set_color('#fbbf24')
        ymax = v1.max() * 1.2 or 1.0
        ax_raw.set_ylim(0, ymax)
        for bar, h in zip(bars_v0, v1):
            bar.set_height(h)
        return

    if S.v0 is None:
        S.v0 = np.mean(S.ref_buf, axis=0)
        print(f"[ref] v0 set ({S.n_ref} frames). mean={S.v0.mean():.4f}")

    v0 = S.v0

    # ── Phase 2: reconstruct ───────────────────────────────────────────────────
    try:
        ds_jac_el = np.real(jac_solver.solve(v1, v0, normalize=True))
        ds_bp_raw = np.real(bp_solver.solve(v1, v0, normalize=True))
    except Exception as e:
        status_txt.set_text(f"Solver error: {e}")
        status_txt.set_color('#f87171')
        return

    # JAC: element values (n_elem,) → interpolate to nodes for Gouraud shading
    ds_jac_nd = sim2pts(pts, tri, ds_jac_el)
    # BP: pyeit ≥1.x returns node values directly; older returns element values
    ds_bp_nd  = ds_bp_raw if len(ds_bp_raw) == n_nd else sim2pts(pts, tri, ds_bp_raw)

    # ── Update JAC plot ────────────────────────────────────────────────────────
    lim_jac = max(float(np.percentile(np.abs(ds_jac_nd), 98)), S.clim_min)
    tc_jac.set_array(ds_jac_nd)
    tc_jac.set_clim(-lim_jac, lim_jac)

    # ── Update BP plot ─────────────────────────────────────────────────────────
    lim_bp = max(float(np.abs(ds_bp_nd).max()), S.clim_min)
    tc_bp.set_array(ds_bp_nd)
    tc_bp.set_clim(-lim_bp, lim_bp)

    # ── Update raw bar chart ───────────────────────────────────────────────────
    ymax = max(v0.max(), v1.max()) * 1.2
    if ymax > 0:
        ax_raw.set_ylim(0, ymax)
    for bar, h in zip(bars_v0, v0):
        bar.set_height(h)
    for bar, h in zip(bars_v1, v1):
        bar.set_height(h)

    # ── Update trend history ───────────────────────────────────────────────────
    ds_mean = float(ds_jac_nd.mean())
    ds_std  = float(ds_jac_nd.std())
    S.ds_history = np.roll(S.ds_history, -1)
    S.ds_history[-1] = ds_mean

    hist_line.set_data(_hx, S.ds_history)
    ylim = max(float(np.abs(S.ds_history).max()) * 1.35, 0.01)
    ax_hist.set_ylim(-ylim, ylim)

    _fill_container[0].remove()
    _fill_container[0] = ax_hist.fill_between(
        _hx,
        S.ds_history - ds_std,
        S.ds_history + ds_std,
        alpha=0.20, color=ACC, zorder=2)

    # ── Status ─────────────────────────────────────────────────────────────────
    rec_tag = '  [●REC]' if S.csv_writer else ''
    status_txt.set_text(
        f"Frame {S.frame_cnt}   Δσ mean={ds_mean:+.4f}   std={ds_std:.4f}   "
        f"JAC range=[{-lim_jac:.3f}, {lim_jac:.3f}]{rec_tag}   "
        f"[r] reset ref   [q] quit")
    status_txt.set_color('#86efac')


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Real-time EIT Visualization (8-electrode, adjacent pattern)')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port  e.g. /dev/cu.usbmodem1234')
    parser.add_argument('--baud', type=int, default=BAUD_RATE)
    parser.add_argument('--ref',  type=int, default=N_REF,
                        help=f'Reference frames to average (default {N_REF})')
    parser.add_argument('--save', action='store_true',
                        help='Save all received frames to a timestamped CSV')
    parser.add_argument('--clim', type=float, default=0.05,
                        help='Minimum color scale limit for JAC and BP maps (default 0.05)')
    args = parser.parse_args()

    S.n_ref    = args.ref
    S.clim_min = args.clim

    if args.save:
        ts       = time.strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(_here, f'eit_realtime_{ts}.csv')
        S.csv_file   = open(csv_path, 'w', newline='')
        S.csv_writer = csv.writer(S.csv_file)
        S.csv_writer.writerow(['timestamp'] + [f'ch{i:02d}' for i in range(N_MEAS)])
        print(f"[csv] Recording to {csv_path}")

    port = args.port or find_port()
    if port is None:
        print("[error] No serial port found.")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        print("Re-run with  --port /dev/cu.XXXXXXXX")
        sys.exit(1)

    status_txt.set_text(f"Connecting to {port}…")

    t = threading.Thread(target=serial_reader, args=(port, args.baud), daemon=True)
    t.start()

    ani = animation.FuncAnimation(fig, animate, interval=UPDATE_MS,
                                  cache_frame_data=False, blit=False)

    def on_close(_):
        _stop_evt.set()
        if S.csv_file:
            S.csv_file.close()
            print("[csv] Saved and closed.")

    fig.canvas.mpl_connect('close_event', on_close)

    print("[main] Starting visualization  (press r to reset reference, q to quit)")
    plt.show()


if __name__ == '__main__':
    main()
