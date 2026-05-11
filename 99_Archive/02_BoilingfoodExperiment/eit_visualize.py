"""
8-electrode EIT Reconstruction & Visualization
Using pyEIT library (https://github.com/eitcom/pyEIT)

CSV format: timestamp + 40 measurements (8 drive pairs x 5 meas pairs)
Arduino: adjacent drive/measurement (dist_exc=1, step_meas=1)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyeit.visual.plot import create_plot
import pyeit.mesh as mesh
import pyeit.eit.protocol as protocol
from pyeit.eit.jac import JAC
from pyeit.eit.bp import BP
import os

# ─── Settings ─────────────────────────────────────────────────────────────────
CSV_FILE    = "eit_20260427_160535.csv"
N_EL        = 8
H0          = 0.08    # mesh density (smaller = finer)
LAMB        = 0.01    # JAC regularization parameter
REF_MODE    = "first" # "first" | "median" | "mean"
SHOW_FRAMES = [0, 50, 100, 150, 200, 250, 300, 350]

# ─── 1. Load CSV ──────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path   = os.path.join(script_dir, CSV_FILE)

df         = pd.read_csv(csv_path)
timestamps = df["timestamp"].values
meas_cols  = [c for c in df.columns if c != "timestamp"]
V          = df[meas_cols].values.astype(float)  # (n_frames, 40)
n_frames   = V.shape[0]
t_rel      = timestamps - timestamps[0]

print(f"Frames: {n_frames},  Measurements per frame: {V.shape[1]}")
print(f"Voltage range: {V.min():.4f} ~ {V.max():.4f} V")

# ─── 2. Build pyEIT mesh (circular domain) ────────────────────────────────────
mesh_obj = mesh.create(N_EL, h0=H0)
el_pos   = mesh_obj.el_pos

print(f"Mesh: {mesh_obj.node.shape[0]} nodes, {mesh_obj.element.shape[0]} elements")

# ─── 3. Protocol matching Arduino (dist_exc=1, step_meas=1) ──────────────────
protocol_obj = protocol.create(N_EL, dist_exc=1, step_meas=1, parser_meas="std")
print(f"Protocol: {protocol_obj.n_meas} total measurements")

# ─── 4. Reference frame ───────────────────────────────────────────────────────
if REF_MODE == "first":
    v0 = V[0]
elif REF_MODE == "median":
    v0 = np.median(V, axis=0)
else:
    v0 = np.mean(V, axis=0)

# ─── 5. Initialize solvers ────────────────────────────────────────────────────
jac_solver = JAC(mesh_obj, protocol_obj)
jac_solver.setup(p=0.2, lamb=LAMB, method="kotre")

bp_solver = BP(mesh_obj, protocol_obj)
bp_solver.setup(weight="none")

# ─── 6. Reconstruct all frames with JAC ───────────────────────────────────────
print("Reconstructing all frames...")
n_elem  = mesh_obj.element.shape[0]
all_ds  = np.zeros((n_frames, n_elem))
for i in range(n_frames):
    all_ds[i] = np.real(jac_solver.solve(V[i], v0, normalize=True))
print(f"Done. ds range: {all_ds.min():.4f} ~ {all_ds.max():.4f}")
ds_lim = float(np.percentile(np.abs(all_ds), 98))

# ─── 7. Figure 1: Multi-frame JAC vs BP ───────────────────────────────────────
valid_frames = [f for f in SHOW_FRAMES if f < n_frames]
n_show = len(valid_frames)

fig1, axes = plt.subplots(2, n_show, figsize=(n_show * 2.8, 6))
fig1.suptitle("EIT Reconstruction  (JAC / BP Comparison)", fontsize=13, fontweight="bold")

for col, fi in enumerate(valid_frames):
    v1 = V[fi]
    ds_jac = np.real(jac_solver.solve(v1, v0, normalize=True))
    ds_bp  = np.real(bp_solver.solve(v1, v0, normalize=True))

    create_plot(axes[0, col], ds_jac, mesh_obj,
                electrodes=el_pos,
                vmin=-ds_lim, vmax=ds_lim,
                ax_kwargs={"title": f"t={t_rel[fi]:.0f}s (f{fi})"},
                flat_plane="z")

    bp_lim = float(np.abs(ds_bp).max()) or 1.0
    create_plot(axes[1, col], ds_bp, mesh_obj,
                electrodes=el_pos,
                vmin=-bp_lim, vmax=bp_lim,
                ax_kwargs={"title": ""},
                flat_plane="z")

axes[0, 0].set_ylabel("JAC", fontsize=11)
axes[1, 0].set_ylabel("BP",  fontsize=11)
plt.tight_layout()
fig1.savefig(os.path.join(script_dir, "eit_frames.png"), dpi=150, bbox_inches="tight")
print("Saved: eit_frames.png")
plt.close(fig1)

# ─── 8. Figure 2: Single frame detail ────────────────────────────────────────
fi_mid     = n_frames // 2
v1_mid     = V[fi_mid]
ds_jac_mid = np.real(jac_solver.solve(v1_mid, v0, normalize=True))
ds_bp_mid  = np.real(bp_solver.solve(v1_mid,  v0, normalize=True))

fig2, axes2 = plt.subplots(1, 3, figsize=(14, 5))
fig2.suptitle(f"EIT Detail  (frame {fi_mid} / t={t_rel[fi_mid]:.1f}s)",
              fontsize=12, fontweight="bold")

lim_jac = float(np.abs(ds_jac_mid).max()) or 1.0
create_plot(axes2[0], ds_jac_mid, mesh_obj, electrodes=el_pos,
            vmin=-lim_jac, vmax=lim_jac,
            ax_kwargs={"title": "JAC Reconstruction"}, flat_plane="z")

lim_bp = float(np.abs(ds_bp_mid).max()) or 1.0
create_plot(axes2[1], ds_bp_mid, mesh_obj, electrodes=el_pos,
            vmin=-lim_bp, vmax=lim_bp,
            ax_kwargs={"title": "BP Reconstruction"}, flat_plane="z")

# Raw voltage bar chart
x = np.arange(40)
axes2[2].bar(x, v0,     width=0.8, alpha=0.55, color="#5C9BD6", label="v0 (reference)")
axes2[2].bar(x, v1_mid, width=0.4, alpha=0.85, color="#E05C5C", label=f"v1 (frame {fi_mid})")
for k in range(1, N_EL):
    axes2[2].axvline(k * 5 - 0.5, color="gray", lw=0.6, ls="--", alpha=0.5)
for k in range(N_EL):
    ymax = axes2[2].get_ylim()[1]
    axes2[2].text(k * 5 + 2, ymax * 0.93, f"Tx{k}", ha="center", fontsize=7, color="dimgray")
axes2[2].set_xlabel("Channel index")
axes2[2].set_ylabel("RMS Voltage [V]")
axes2[2].set_title("Raw Measurement: v0 vs v1")
axes2[2].legend(fontsize=8)
axes2[2].grid(True, alpha=0.3, axis="y")
axes2[2].set_xlim(-0.5, 39.5)

plt.tight_layout()
fig2.savefig(os.path.join(script_dir, "eit_detail.png"), dpi=150, bbox_inches="tight")
print("Saved: eit_detail.png")
plt.close(fig2)

# ─── 9. Figure 3: Time series ─────────────────────────────────────────────────
ds_mean = all_ds.mean(axis=1)
ds_std  = all_ds.std(axis=1)

fig3, axes3 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig3.suptitle("EIT Time Series Analysis", fontsize=13, fontweight="bold")

im1 = axes3[0].imshow(V.T, aspect="auto", cmap="viridis",
                      extent=[t_rel[0], t_rel[-1], 0, 40], origin="lower")
for k in range(1, N_EL):
    axes3[0].axhline(k * 5, color="white", lw=0.5, alpha=0.5)
axes3[0].set_ylabel("Channel #")
axes3[0].set_title("(a) Raw RMS Voltage Heatmap")
fig3.colorbar(im1, ax=axes3[0], label="RMS [V]", fraction=0.015)

im2 = axes3[1].imshow(all_ds.T, aspect="auto", cmap="RdBu_r",
                      vmin=-ds_lim, vmax=ds_lim,
                      extent=[t_rel[0], t_rel[-1], 0, n_elem], origin="lower")
axes3[1].set_ylabel("Element #")
axes3[1].set_title("(b) Reconstructed \u0394\u03c3 (JAC)")
fig3.colorbar(im2, ax=axes3[1], label="\u0394\u03c3", fraction=0.015)

axes3[2].plot(t_rel, ds_mean, lw=1.3, color="#2196F3", label="Spatial mean")
axes3[2].fill_between(t_rel, ds_mean - ds_std, ds_mean + ds_std, alpha=0.2, color="#2196F3")
axes3[2].plot(t_rel, ds_std, lw=1.3, color="#F44336", ls="--", label="Spatial std (change intensity)")
axes3[2].axhline(0, color="k", lw=0.7, ls=":")
axes3[2].set_xlabel("Elapsed time [s]")
axes3[2].set_ylabel("\u0394\u03c3")
axes3[2].set_title("(c) Spatial Statistics over Time")
axes3[2].legend(fontsize=9)
axes3[2].grid(True, alpha=0.3)

plt.tight_layout()
fig3.savefig(os.path.join(script_dir, "eit_timeseries.png"), dpi=150, bbox_inches="tight")
print("Saved: eit_timeseries.png")
plt.close(fig3)

print("\nDone! 3 images saved.")