"""
eit_realtime2.py  –  Real-time EIT visualization (8-electrode, full-matrix 64ch)
Matches test3.ino: 8 adjacent drive pairs × 8 adjacent meas pairs = 64 values.

Electrode correspondence (Arduino ↔ pyeit):
  Arduino electrode i  ==  pyeit electrode i  (no reversal)

Reconstruction:
  The 64 measurements include some where the meas pair shares an electrode with
  the drive pair (2-wire). These are excluded; the remaining 40 "clean" 4-wire
  measurements are fed to pyeit JAC (standard adjacent protocol, dist_exc=1).

  Mapping:  Arduino index = d*8 + m
            Valid when m ∉ {d, (d+1)%8, (d-1+8)%8}
            → 5 valid m per drive × 8 drives = 40 total

Display:
  Left  – JAC reconstruction (Δσ map)
  Right – 8×8 raw measurement heatmap (drive × meas)
"""

import sys, time, threading, queue, argparse
import numpy as np
import matplotlib
matplotlib.use("macosx")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial, serial.tools.list_ports

import pyeit.mesh as mesh
import pyeit.eit.protocol as protocol
from pyeit.eit.jac import JAC

# ─── Settings ──────────────────────────────────────────────────────────────────
N_EL   = 8
N_MEAS = N_EL * N_EL   # 64
H0     = 0.08
LAMB   = 0.1
N_REF  = 8
CLIM   = 0.15

# ─── CLI ───────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument("--port", default=None)
ap.add_argument("--baud", default=115200, type=int)
ap.add_argument("--ref",  default=N_REF,  type=int)
ap.add_argument("--lamb", default=LAMB,   type=float)
ap.add_argument("--clim", default=CLIM,   type=float)
args = ap.parse_args()

# ─── Index mapping: Arduino 64 → pyeit 40 ─────────────────────────────────────
#
# Arduino full-matrix layout:
#   index = d * N_EL + m
#   drive pair : (d,   (d+1)%N_EL)
#   meas  pair : (m,   (m+1)%N_EL)
#
# A measurement is "valid" (4-wire) when the meas pair shares NO electrode
# with the drive pair.
#   Invalid m values for drive d: { d, (d+1)%N_EL, (d-1+N_EL)%N_EL }
#
# pyeit adjacent protocol orders measurements as:
#   for each drive d=0..7: for m=0..7 in order, skip if invalid
#   → same ordering we compute below.

def build_ard_idx(n_el):
    """Return the 40 Arduino indices that correspond to pyeit's 40-measurement
    adjacent protocol, in the same order pyeit expects them."""
    idx = []
    for d in range(n_el):
        invalid = {d, (d + 1) % n_el, (d - 1 + n_el) % n_el}
        for m in range(n_el):
            if m not in invalid:
                idx.append(d * n_el + m)
    return np.array(idx, dtype=int)

ARD_IDX = build_ard_idx(N_EL)   # shape (40,)

# ─── Build mesh ────────────────────────────────────────────────────────────────
mesh_obj = mesh.create(N_EL, h0=H0)
pts      = mesh_obj.node      # (n_node, 2)
tri      = mesh_obj.element   # (n_elem, 3)
el_pos   = mesh_obj.el_pos    # electrode node indices

# ─── pyeit protocol: adjacent, 40 measurements ─────────────────────────────────
# dist_exc=1  → drive pairs (0,1),(1,2),…,(7,0)
# step_meas=1 → adjacent meas pairs, excluding those sharing drive electrodes
protocol_obj = protocol.create(N_EL, dist_exc=1, step_meas=1, parser_meas="std")
assert protocol_obj.n_meas_tot == len(ARD_IDX), (
    f"Protocol expects {protocol_obj.n_meas_tot} meas, ARD_IDX has {len(ARD_IDX)}")

print(f"Protocol: {protocol_obj.n_meas_tot} measurements  "
      f"(extracted from {N_MEAS} full-matrix values)")
print("Arduino→pyeit index map (first 10):", ARD_IDX[:10])

# ─── JAC solver ────────────────────────────────────────────────────────────────
jac = JAC(mesh_obj, protocol_obj)
jac.setup(p=0.2, lamb=args.lamb, method="kotre")

# ─── Serial port ───────────────────────────────────────────────────────────────
def find_port():
    for p in serial.tools.list_ports.comports():
        if any(k in p.device for k in ("usbmodem", "usbserial", "ACM", "USB")):
            return p.device
    pl = serial.tools.list_ports.comports()
    return pl[0].device if pl else None

port = args.port or find_port()
if not port:
    sys.exit("No serial port found.  Use --port /dev/cu.xxx")

print(f"Connecting to {port} @ {args.baud} baud...")
ser = serial.Serial(port, args.baud, timeout=5)
time.sleep(2)
print("Connected.")

# ─── Serial reader thread ──────────────────────────────────────────────────────
raw_q = queue.Queue(maxsize=40)

def _read_loop():
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or line.startswith("["):
                continue
            vals = [float(x) for x in line.split(",")]
            if len(vals) == N_MEAS:
                try:
                    raw_q.put_nowait(vals)
                except queue.Full:
                    raw_q.get_nowait()
                    raw_q.put_nowait(vals)
        except Exception:
            pass

threading.Thread(target=_read_loop, daemon=True).start()

# ─── Baseline ──────────────────────────────────────────────────────────────────
print(f"Collecting baseline ({args.ref} frames of {N_MEAS} ch)...")
buf = []
while len(buf) < args.ref:
    try:
        buf.append(raw_q.get(timeout=15))
        print(f"  {len(buf)}/{args.ref}", end="\r")
    except queue.Empty:
        sys.exit("Timeout: no data received.")

v0_full = np.mean(buf, axis=0)          # shape (64,)  all measurements
v0_eit  = v0_full[ARD_IDX]              # shape (40,)  pyeit-compatible subset
print(f"\nBaseline ready  mean={v0_full.mean():.4f} V")

# ─── Figure ────────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
fig.patch.set_facecolor("#2d2d2d")
try:
    fig.canvas.manager.set_window_title("EIT Live Monitor  (8-electrode full matrix)")
except Exception:
    pass

for ax in (ax1, ax2):
    ax.set_facecolor("#1e1e1e")
    ax.tick_params(colors="white", labelsize=9)
    for sp in ax.spines.values():
        sp.set_edgecolor("#555")

# ── Left: JAC reconstruction ──
n_elem = tri.shape[0]
tri_plot = ax1.tripcolor(pts[:, 0], pts[:, 1], tri, np.zeros(n_elem),
                          cmap="RdBu_r", shading="flat",
                          vmin=-args.clim, vmax=args.clim)
el_xy = pts[el_pos]
ax1.plot(el_xy[:, 0], el_xy[:, 1], "go", ms=7, zorder=5)
for i, xy in enumerate(el_xy):
    ax1.text(xy[0] * 1.18, xy[1] * 1.18, str(i),
             color="lime", fontsize=8, ha="center", va="center")
ax1.set_aspect("equal")
ax1.axis("off")
ax1.set_title("EIT reconstruction  (Δσ)", color="white", fontsize=11)
cb1 = fig.colorbar(tri_plot, ax=ax1, fraction=0.04, pad=0.02)
cb1.ax.yaxis.set_tick_params(color="white")
plt.setp(cb1.ax.yaxis.get_ticklabels(), color="white")

# ── Right: 8×8 raw measurement heatmap ──
mat0 = v0_full.reshape(N_EL, N_EL)
heat = ax2.imshow(mat0, cmap="viridis", aspect="auto",
                  origin="upper",
                  extent=[-0.5, N_EL - 0.5, N_EL - 0.5, -0.5])
ax2.set_xticks(range(N_EL))
ax2.set_yticks(range(N_EL))
ax2.set_xticklabels([f"({m},{(m+1)%N_EL})" for m in range(N_EL)],
                     color="white", fontsize=7, rotation=45)
ax2.set_yticklabels([f"({d},{(d+1)%N_EL})" for d in range(N_EL)],
                     color="white", fontsize=7)
ax2.set_xlabel("Meas pair  (VP, VN)", color="white", fontsize=9)
ax2.set_ylabel("Drive pair (SRC, SINK)", color="white", fontsize=9)
ax2.set_title("Raw measurement matrix  [V]", color="white", fontsize=11)
cb2 = fig.colorbar(heat, ax=ax2, fraction=0.04, pad=0.02)
cb2.ax.yaxis.set_tick_params(color="white")
plt.setp(cb2.ax.yaxis.get_ticklabels(), color="white")

# Mark the 3 invalid (2-wire) cells per row with an X pattern
for d in range(N_EL):
    for m in [d, (d + 1) % N_EL, (d - 1 + N_EL) % N_EL]:
        ax2.add_patch(plt.Rectangle((m - 0.5, d - 0.5), 1, 1,
                                     fill=False, edgecolor="red",
                                     lw=0.8, linestyle="--"))

status_txt = fig.text(0.5, 0.01, "Running…", ha="center",
                       color="#aaa", fontsize=9)

# ─── Animation ─────────────────────────────────────────────────────────────────
_fc = [0]

def update(_):
    try:
        vals = raw_q.get_nowait()
    except queue.Empty:
        return [tri_plot, heat, status_txt]

    v1_full = np.array(vals, dtype=float)   # (64,)
    v1_eit  = v1_full[ARD_IDX]              # (40,) valid 4-wire measurements
    _fc[0] += 1

    # ── JAC reconstruction (40 valid measurements) ──
    try:
        ds = np.real(jac.solve(v1_eit, v0_eit, normalize=True))
    except Exception as e:
        print(f"[JAC error] {e}")
        return
    tri_plot.set_array(ds)

    # ── Heatmap update (all 64 measurements) ──
    mat = v1_full.reshape(N_EL, N_EL)
    heat.set_data(mat)
    heat.set_clim(mat.min(), mat.max())

    status_txt.set_text(
        f"Frame {_fc[0]}  |  mean={v1_full.mean():.4f} V  |  "
        f"Δσ range=[{ds.min():.3f}, {ds.max():.3f}]")

    return [tri_plot, heat, status_txt]

anim = FuncAnimation(fig, update, interval=150, blit=False)
plt.tight_layout(rect=[0, 0.03, 1, 1])

try:
    plt.show()
finally:
    ser.close()
    print("Serial closed.")
