"""Animates the Simulink results as two side-by-side quarter cars.

    python tools/make_animation.py

Reads  results/simulation_response.csv
Writes media/animations/quarter_car_bump.gif
       (body/wheel motion exaggerated x15 so millimetre motion is visible)
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "media" / "animations"
OUT.mkdir(parents=True, exist_ok=True)

BASE, OPT, INK, INK2, BUMP = "#eb6834", "#2a78d6", "#0b0b0b", "#52514e", "#f6e3d9"
SCALE = 15.0          # visual exaggeration of displacements
T_END, FPS, SPEED = 16.0, 15, 2.0   # animate 0-16 s at 2x real time

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10})

d = np.genfromtxt(ROOT / "results" / "simulation_response.csv", delimiter=",", names=True)
frames_t = np.arange(0, T_END + 1e-9, SPEED / FPS)
tt = d["t"]


def at(key, tk):
    return np.interp(tk, tt, d[key])


fig = plt.figure(figsize=(9.6, 6.4), dpi=90)
gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 1], hspace=0.35, wspace=0.12,
                      left=0.07, right=0.98, top=0.9, bottom=0.09)
fig.suptitle("Quarter car driving over a speed bump  (motion exaggerated ×15)", fontsize=13, weight="bold")

cars = {}
for col, (sfx, name, color) in enumerate([("base", "Baseline PID-LQR  (Q = I, R = I)", BASE),
                                          ("opt", "GA-tuned PID-LQR", OPT)]):
    ax = fig.add_subplot(gs[0, col])
    ax.set_xlim(-2.2, 2.2)
    ax.set_ylim(-0.3, 4.9)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(name, color=INK, fontsize=11.5, weight="bold")
    ax.plot([-2.2, 2.2], [0, 0], color=INK2, lw=2)
    ax.axhline(3.1, color=INK2, lw=0.8, ls=":")   # body rest position
    road = Rectangle((-0.9, 0), 1.8, 0.0, fc=BUMP, ec=INK2, lw=1)
    ax.add_patch(road)
    wheel = Rectangle((-0.6, 0.9), 1.2, 0.55, fc="#d9f0e6", ec=INK, lw=1.5)
    body = Rectangle((-1.5, 2.4), 3.0, 0.7, fc=color, ec=INK, lw=1.5, alpha=0.9)
    ax.add_patch(wheel)
    ax.add_patch(body)
    tire, = ax.plot([], [], color=INK, lw=1.6)
    susp, = ax.plot([], [], color=INK, lw=1.6)
    act, = ax.plot([], [], color=INK2, lw=3, solid_capstyle="butt")
    txt = ax.text(0, 4.6, "", ha="center", fontsize=10, color=INK)
    cars[sfx] = dict(road=road, wheel=wheel, body=body, tire=tire, susp=susp, act=act, txt=txt)

axp = fig.add_subplot(gs[1, :])
axp.axvspan(3, 5, color=BUMP, lw=0)
axp.plot(tt, d["y1_base"] * 1000, color=BASE, lw=2, label="Baseline")
axp.plot(tt, d["y1_opt"] * 1000, color=OPT, lw=2, label="GA-tuned")
axp.axhline(0, color=INK2, lw=0.8)
axp.set_xlim(0, T_END)
axp.set_ylim(-5, 90)
axp.set_xlabel("Time [s]")
axp.set_ylabel("Body displacement $y_c$ [mm]")
axp.grid(color="#e4e3de")
for s in ("top", "right"):
    axp.spines[s].set_visible(False)
axp.legend(loc="upper right", frameon=False)
axp.text(4, 84, "bump", ha="center", color=INK2, fontsize=9)
cursor = axp.axvline(0, color=INK, lw=1.2)
dot_b, = axp.plot([], [], "o", color=BASE, ms=7, mec="white", mew=1.5)
dot_o, = axp.plot([], [], "o", color=OPT, ms=7, mec="white", mew=1.5)
clock = fig.text(0.5, 0.47, "", ha="center", fontsize=11, color=INK)


def zigzag(x, y0, y1, n=5, w=0.14):
    ys = np.linspace(y0, y1, 2 * n + 3)
    xs = np.r_[x, [x + (w if i % 2 else -w) for i in range(1, 2 * n + 2)], x]
    return xs, ys


def update(i):
    tk = frames_t[i]
    bump = 0.35 * at("ref", tk)
    for sfx, c in cars.items():
        yw = SCALE * at(f"x1_{sfx}", tk)     # drawing unit = 1 m, exaggerated x15
        yb = SCALE * at(f"x3_{sfx}", tk)
        c["road"].set_height(bump)
        c["wheel"].set_y(0.9 + yw)
        c["body"].set_y(2.4 + yb)
        c["tire"].set_data(*zigzag(0.0, bump, 0.9 + yw, n=3))
        c["susp"].set_data(*zigzag(-0.35, 1.45 + yw, 2.4 + yb))
        c["act"].set_data([0.35, 0.35], [1.45 + yw, 2.4 + yb])
        c["txt"].set_text(f"$y_c$ = {at(f'x3_{sfx}', tk) * 1000:5.1f} mm")
    cursor.set_xdata([tk, tk])
    dot_b.set_data([tk], [at("y1_base", tk) * 1000])
    dot_o.set_data([tk], [at("y1_opt", tk) * 1000])
    clock.set_text(f"t = {tk:4.1f} s" + ("   ·   bump active" if 3 <= tk < 5 else ""))
    return []


anim = FuncAnimation(fig, update, frames=len(frames_t), blit=False)
out = OUT / "quarter_car_bump.gif"
anim.save(out, writer=PillowWriter(fps=FPS))
print("animation written to", out, f"({out.stat().st_size / 1e6:.1f} MB)")
