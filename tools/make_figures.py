"""Plots the Simulink results exported by src/export_results.m.

    python tools/make_figures.py

Reads  results/simulation_response.csv, results/closed_loop_poles.csv
Writes media/figures/*.png and results/metrics.csv
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
OUT = ROOT / "media" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

BASE = "#eb6834"   # baseline PID-LQR (Q = I, R = I)
OPT = "#2a78d6"    # GA-tuned PID-LQR
REF = "#52514e"
INK = "#0b0b0b"
GRID = "#e4e3de"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10.5,
    "axes.edgecolor": "#9b9a94", "axes.labelcolor": INK, "axes.titleweight": "bold",
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "xtick.color": REF, "ytick.color": REF, "legend.frameon": False,
    "lines.linewidth": 2.0,
})

d = np.genfromtxt(RES / "simulation_response.csv", delimiter=",", names=True)
t = d["t"]
LBL_B, LBL_O = "Baseline PID-LQR  (Q = I, R = I)", "GA-tuned PID-LQR"


def shade_bump(ax):
    ax.axvspan(3, 5, color="#f6e3d9", lw=0, zorder=0)


def save(fig, name):
    fig.savefig(OUT / name, dpi=160, bbox_inches="tight", facecolor="white")
    plt.close(fig)


# 1. body response ------------------------------------------------------------
def body_response():
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True,
                            gridspec_kw={"height_ratios": [0.9, 2, 2]})
    axs[0].plot(t, d["ref"], color=REF, lw=1.6)
    axs[0].set_ylabel("r(t)")
    axs[0].set_title("Speed-bump input: step active between 3 s and 5 s", loc="left")
    for ax, key, ylab, title in [
        (axs[1], "y1", "$y_c$  [m]", "Body displacement $y_c$"),
        (axs[2], "y2", "$\\dot y_c$  [m/s]", "Body vertical velocity $\\dot y_c$"),
    ]:
        ax.plot(t, d[f"{key}_base"], color=BASE, label=LBL_B)
        ax.plot(t, d[f"{key}_opt"], color=OPT, label=LBL_O)
        ax.axhline(0, color=REF, lw=0.8)
        ax.set_ylabel(ylab)
        ax.set_title(title, loc="left")
    for ax in axs:
        shade_bump(ax)
    axs[1].legend(loc="upper right")
    axs[2].set_xlabel("Time [s]")
    axs[2].set_xlim(0, 30)
    fig.suptitle("Closed-loop body response (Simulink, 30 s)", x=0.01, ha="left", fontsize=14, weight="bold")
    fig.tight_layout()
    save(fig, "body_response.png")


# 1b. zoom on the bump --------------------------------------------------------
def body_zoom():
    fig, ax = plt.subplots(figsize=(10, 4.2))
    m = (t >= 2) & (t <= 14)
    ax.plot(t[m], d["y1_base"][m] * 1000, color=BASE, label=LBL_B)
    ax.plot(t[m], d["y1_opt"][m] * 1000, color=OPT, label=LBL_O)
    shade_bump(ax)
    i_b, i_o = np.argmax(d["y1_base"]), np.argmax(d["y1_opt"])
    ax.annotate(f"peak {d['y1_base'][i_b]*1000:.1f} mm", (t[i_b], d["y1_base"][i_b] * 1000),
                xytext=(0, 6), textcoords="offset points", color=INK, fontsize=10, ha="center", va="bottom")
    ax.annotate(f"peak {d['y1_opt'][i_o]*1000:.1f} mm", (t[i_o], d["y1_opt"][i_o] * 1000),
                xytext=(10, 10), textcoords="offset points", color=INK, fontsize=10)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("$y_c$  [mm]")
    ax.set_title("Body displacement around the bump (shaded = bump active)", loc="left", pad=16)
    ax.legend(loc="upper right")
    fig.tight_layout()
    save(fig, "body_displacement_zoom.png")


# 2. all four states (small multiples) ---------------------------------------
def states():
    names = [("x1", "$x_1 = y_k$  wheel displacement [m]"), ("x2", "$x_2 = \\dot y_k$  wheel velocity [m/s]"),
             ("x3", "$x_3 = y_c$  body displacement [m]"), ("x4", "$x_4 = \\dot y_c$  body velocity [m/s]")]
    fig, axs = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for ax, (k, title) in zip(axs.flat, names):
        ax.plot(t, d[f"{k}_base"], color=BASE, label="Baseline")
        ax.plot(t, d[f"{k}_opt"], color=OPT, label="GA-tuned")
        ax.axhline(0, color=REF, lw=0.8)
        shade_bump(ax)
        ax.set_title(title, loc="left", fontsize=10.5)
        ax.set_xlim(0, 30)
    for ax in axs[1]:
        ax.set_xlabel("Time [s]")
    axs[0, 0].legend(loc="upper right")
    fig.suptitle("State trajectories: baseline vs GA-tuned", x=0.01, ha="left", fontsize=14, weight="bold")
    fig.tight_layout()
    save(fig, "state_trajectories.png")


# 3. control input ------------------------------------------------------------
def control_input():
    # u1 and u2 differ by < 0.1 % away from the step edges, so one panel is enough
    fig, ax = plt.subplots(figsize=(10, 4.2))
    b, o = d["u1_base"], d["u1_opt"]
    ax.plot(t, b, color=BASE, label=LBL_B)
    ax.plot(t, o, color=OPT, label=LBL_O)
    lim = np.percentile(np.abs(np.concatenate([b, o])), 97) * 1.6
    ax.set_ylim(-0.25 * lim, lim)
    ax.axhline(0, color=REF, lw=0.8)
    shade_bump(ax)
    ax.legend(loc="upper right")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("$u$")
    ax.set_xlim(0, 30)
    ax.set_title("Controller effort $u$ (both input channels are practically identical)", loc="left")
    fig.text(0.01, -0.03, "Y-axis clipped: the derivative term produces impulsive spikes exactly at the step "
             "edges (t = 3 s and 5 s).", fontsize=9, color=REF)
    fig.tight_layout()
    save(fig, "control_input.png")


# 4. closed-loop poles --------------------------------------------------------
def poles():
    p = np.genfromtxt(RES / "closed_loop_poles.csv", delimiter=",", names=True)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.4), gridspec_kw={"width_ratios": [1, 1.4]})
    for ax, xlim in [(axs[0], (-36, 1)), (axs[1], (-4.8, 0.3))]:
        ax.scatter(p["re_base"], p["im_base"], s=70, marker="o", fc="white", ec=BASE, lw=2, label="Baseline", zorder=3)
        ax.scatter(p["re_opt"], p["im_opt"], s=55, marker="x", color=OPT, lw=2.2, label="GA-tuned", zorder=4)
        ax.axvline(0, color=REF, lw=0.9)
        ax.axhline(0, color=REF, lw=0.9)
        ax.set_xlim(*xlim)
        ax.set_xlabel("Real axis")
    axs[0].set_ylabel("Imaginary axis")
    axs[0].set_title("All six poles", loc="left")
    axs[1].set_title("Zoom on the dominant poles", loc="left")
    axs[1].legend(loc="upper left")
    fig.suptitle("Closed-loop poles of the augmented system  $A_a - B_aK$  (all in the left half-plane → stable)",
                 x=0.01, ha="left", fontsize=12.5, weight="bold")
    fig.tight_layout()
    save(fig, "closed_loop_poles.png")


# 5. performance metrics --------------------------------------------------------
def metrics():
    def m(sfx):
        yc, vc = d[f"y1_{sfx}"], d[f"y2_{sfx}"]
        return {
            "Peak body displacement |y_c| [mm]": np.max(np.abs(yc)) * 1e3,
            "Peak body velocity |dy_c/dt| [mm/s]": np.max(np.abs(vc)) * 1e3,
            "ISE of y_c [mm^2 s]": np.trapezoid(yc ** 2, t) * 1e6,
            "RMS body velocity [mm/s]": np.sqrt(np.trapezoid(vc ** 2, t) / t[-1]) * 1e3,
            "Residual y_c at t = 30 s [mm]": abs(yc[-1]) * 1e3,
        }
    mb, mo = m("base"), m("opt")
    rows = [(k, mb[k], mo[k], 100 * (1 - mo[k] / mb[k])) for k in mb]
    with open(RES / "metrics.csv", "w") as f:
        f.write("metric,baseline,ga_tuned,reduction_percent\n")
        for r in rows:
            f.write(f"\"{r[0]}\",{r[1]:.4f},{r[2]:.4f},{r[3]:.1f}\n")

    fig, ax = plt.subplots(figsize=(10, 4.2))
    y = np.arange(len(rows))[::-1]
    h = 0.36
    ax.barh(y + h / 2 + 0.01, [100] * len(rows), height=h, color=BASE, label="Baseline (= 100 %)")
    ax.barh(y - h / 2 - 0.01, [100 * r[2] / r[1] for r in rows], height=h, color=OPT, label="GA-tuned")
    for yi, r in zip(y, rows):
        ax.text(100 * r[2] / r[1] + 1.5, yi - h / 2, f"{100 * r[2] / r[1]:.1f} %  (−{r[3]:.0f} %)",
                va="center", fontsize=9.5, color=INK)
    ax.set_yticks(y, [r[0] for r in rows])
    ax.set_xlim(0, 112)
    ax.set_xlabel("Relative to baseline [%]  (lower is better)")
    ax.grid(axis="y", visible=False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.45, -0.2), ncol=2)
    ax.set_title("Performance of the GA-tuned controller relative to the baseline", loc="left")
    fig.tight_layout()
    save(fig, "performance_metrics.png")
    for r in rows:
        print(f"{r[0]:40s} base {r[1]:10.4f}  GA {r[2]:10.4f}  (-{r[3]:.1f} %)")


if __name__ == "__main__":
    body_response()
    body_zoom()
    states()
    control_input()
    poles()
    metrics()
    print("figures written to", OUT)
