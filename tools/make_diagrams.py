"""Draws the explanatory diagrams used in the README.

    python tools/make_diagrams.py

Outputs (media/figures/):
    quarter_car_model.png     2-DOF quarter-car schematic with actuator
    system_architecture.png   end-to-end workflow (MATLAB -> GA -> LQR -> PID -> Simulink)
    ga_flowchart.png          Genetic Algorithm loop used to tune Q and R
"""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

OUT = Path(__file__).resolve().parents[1] / "media" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

INK = "#0b0b0b"
INK2 = "#52514e"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
VIOLET = "#4a3aa7"
PANEL = "#f3f2ee"

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})


def box(ax, xy, w, h, text, fc="white", ec=INK, lw=1.6, fs=11, ls="-", weight="normal", tc=INK):
    x, y = xy
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fc, ec=ec, lw=lw, ls=ls))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            color=tc, weight=weight, linespacing=1.35)


def arrow(ax, p0, p1, color=INK, ls="-", lw=1.6, rad=0.0, text=None, toff=(0, 0.12), fs=9.5):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=14, color=color,
                                 lw=lw, ls=ls, connectionstyle=f"arc3,rad={rad}"))
    if text:
        mx, my = (p0[0] + p1[0]) / 2 + toff[0], (p0[1] + p1[1]) / 2 + toff[1]
        ax.text(mx, my, text, ha="center", va="bottom", fontsize=fs, color=INK2)


# ----------------------------------------------------------------------------
def spring(ax, x, y0, y1, n=6, w=0.18, color=INK):
    import numpy as np
    L = y1 - y0
    ys = np.linspace(y0 + 0.12 * L, y1 - 0.12 * L, 2 * n + 1)
    xs = [x] + [x + (w if i % 2 else -w) for i in range(1, 2 * n)] + [x]
    ax.plot([x, x], [y0, ys[0]], color=color, lw=1.8)
    ax.plot(xs, ys, color=color, lw=1.8)
    ax.plot([x, x], [ys[-1], y1], color=color, lw=1.8)


def damper(ax, x, y0, y1, color=INK):
    m = (y0 + y1) / 2
    ax.plot([x, x], [y0, m + 0.12], color=color, lw=1.8)
    ax.plot([x - 0.2, x - 0.2, x + 0.2, x + 0.2], [m + 0.35, m - 0.15, m - 0.15, m + 0.35], color=color, lw=1.8)
    ax.plot([x - 0.14, x + 0.14], [m + 0.12, m + 0.12], color=color, lw=3)
    ax.plot([x, x], [y1, m + 0.12], color=color, lw=1.8)
    ax.plot([x, x], [y0, m - 0.15], color=color, lw=1.8)


def quarter_car():
    fig, ax = plt.subplots(figsize=(7.2, 7.4))
    ax.set_xlim(-1, 9)
    ax.set_ylim(-0.6, 9.4)
    ax.axis("off")

    # body (sprung mass)
    ax.add_patch(Rectangle((1, 6.2), 5, 1.8, fc="#dbe8f8", ec=INK, lw=1.8))
    ax.text(3.5, 7.1, "Car body  $m_c$ = 408 kg\n(sprung mass, 1/4 car)", ha="center", va="center", fontsize=11.5)
    # wheel (unsprung mass)
    ax.add_patch(Rectangle((1.8, 2.6), 3.4, 1.4, fc="#d9f0e6", ec=INK, lw=1.8))
    ax.text(3.5, 3.3, "Wheel  $m_{us}$ = 48.3 kg\n(unsprung mass)", ha="center", va="center", fontsize=11.5)

    # suspension
    spring(ax, 2.3, 4.0, 6.2)
    ax.text(1.75, 5.1, "$k_r$\n30 kN/m", ha="right", va="center", fontsize=10.5)
    damper(ax, 3.5, 4.0, 6.2)
    ax.text(3.8, 5.55, "$b_r$", ha="left", va="center", fontsize=10.5)
    # actuator
    ax.plot([4.7, 4.7], [4.0, 6.2], color=ORANGE, lw=2.2)
    ax.add_patch(plt.Circle((4.7, 5.1), 0.22, fc=ORANGE, ec=INK, lw=1.2, zorder=3))
    ax.text(5.05, 5.1, "$F_a$  active\nactuator", ha="left", va="center", fontsize=10.5, color=INK)

    # tire
    spring(ax, 3.5, 0.6, 2.6)
    ax.text(3.95, 1.6, "$k_k$ = 340 kN/m\n(tire stiffness)", ha="left", va="center", fontsize=10.5)
    # road
    ax.plot([0.2, 7.0], [0.6, 0.6], color=INK2, lw=2)
    for xx in [0.4 + 0.4 * i for i in range(17)]:
        ax.plot([xx, xx - 0.25], [0.6, 0.35], color=INK2, lw=1)
    ax.annotate("", xy=(6.4, 1.4), xytext=(6.4, 0.6), arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.6))
    ax.text(6.55, 1.0, "$u(t)$ road\ninput", ha="left", va="center", fontsize=10.5)

    # displacement arrows
    for y, lab in [(7.1, "$y_c = x_3$"), (3.3, "$y_k = x_1$")]:
        x0 = 6.2 if y > 5 else 5.4
        ax.annotate("", xy=(x0 + 0.1, y + 0.8), xytext=(x0 + 0.1, y),
                    arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1.8))
        ax.plot([x0 - 0.1, x0 + 0.3], [y, y], color=BLUE, lw=1.8)
        ax.text(x0 + 0.4, y + 0.4, lab, color=BLUE, fontsize=12, va="center")

    ax.text(4, 9.0, "Quarter-car active suspension (2-DOF)", ha="center", fontsize=14, weight="bold")
    ax.text(4, 8.5, "BMW 530i front-axle parameters, $b_r$ = 1450 Ns/m", ha="center", fontsize=10.5, color=INK2)
    fig.savefig(OUT / "quarter_car_model.png", dpi=170, bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ----------------------------------------------------------------------------
def architecture():
    fig, ax = plt.subplots(figsize=(13, 6.4))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 6.5)
    ax.axis("off")

    ax.text(6.5, 6.25, "Method overview: GA-tuned PID-LQR for quarter-car suspension",
            ha="center", fontsize=15, weight="bold")

    # row 1 – design (offline, MATLAB)
    ax.add_patch(FancyBboxPatch((0.15, 3.0), 12.7, 2.95, boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc=PANEL, ec="none"))
    ax.text(0.35, 5.7, "OFFLINE DESIGN  (MATLAB)", fontsize=10, color=INK2, weight="bold")
    box(ax, (0.4, 3.4), 2.1, 1.6, "Quarter-car\nstate space\n$\\dot{x}=Ax+Bu$\n$y=Cx$")
    box(ax, (3.0, 3.4), 2.3, 1.6, "Integral\naugmentation\n$A_a,\\ B_a,\\ \\Gamma$")
    box(ax, (5.8, 3.4), 2.2, 1.6, "Genetic\nAlgorithm\nsearch $Q,\\ R$", ec=VIOLET, lw=2.2)
    box(ax, (8.5, 3.4), 1.9, 1.6, "LQR\n(Riccati)\n$K = R^{-1}B_a^TP$", ec=BLUE, lw=2.2)
    box(ax, (10.8, 3.4), 1.9, 1.6, "PID gains\n$\\hat K = K\\,\\Gamma^{-1}$\n$K_P, K_I, K_D$", ec=AQUA, lw=2.2)
    arrow(ax, (2.5, 4.2), (3.0, 4.2))
    arrow(ax, (5.3, 4.2), (5.8, 4.2))
    arrow(ax, (8.0, 4.2), (8.5, 4.2), text="$Q, R$")
    arrow(ax, (10.4, 4.2), (10.8, 4.2), text="$K$")
    # GA feedback loop: fitness of each candidate goes back to the GA
    ax.add_patch(FancyArrowPatch((9.45, 5.0), (6.9, 5.0), arrowstyle="-|>", mutation_scale=14, color=VIOLET,
                                 lw=1.6, ls="--", connectionstyle="arc3,rad=0.5"))
    ax.text(8.2, 5.62, "fitness $J$ = ISE + overshoot + settling time + 100·SSE",
            ha="center", fontsize=9.5, color=VIOLET,
            bbox=dict(fc=PANEL, ec="none", pad=1))
    # row 2 – simulation (Simulink)
    ax.add_patch(FancyBboxPatch((0.15, 0.1), 12.7, 2.25, boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc=PANEL, ec="none"))
    ax.text(0.35, 2.1, "CLOSED-LOOP SIMULATION  (Simulink, 30 s)", fontsize=10, color=INK2, weight="bold")
    box(ax, (0.4, 0.65), 2.3, 1.2, "Speed bump\n$r(t)=1,\\ 3\\leq t<5$ s")
    box(ax, (3.5, 0.65), 2.5, 1.2, "PID controller\n$u=K_I\\!\\int\\! e+K_Pe+K_D\\dot e$", ec=AQUA, lw=2.2)
    box(ax, (6.6, 0.65), 2.0, 1.2, "Quarter-car\nplant")
    box(ax, (9.3, 0.65), 3.4, 1.2, "Compare: baseline ($Q=I, R=I$)\nvs GA-tuned  →  $x,\\ y,\\ u$")
    arrow(ax, (2.7, 1.25), (3.5, 1.25), text="$r$")
    arrow(ax, (6.0, 1.25), (6.6, 1.25), text="$u$")
    arrow(ax, (8.6, 1.25), (9.3, 1.25), text="$y$")
    # output feedback  y -> controller (e = r - y)
    ax.plot([8.95, 8.95, 4.75], [1.25, 0.35, 0.35], color=INK2, lw=1.6)
    arrow(ax, (4.75, 0.35), (4.75, 0.65), color=INK2)
    ax.text(6.9, 0.4, "feedback $y$  →  $e=r-y$", fontsize=9.5, color=INK2, ha="center", va="bottom")
    # tuned gains go down into the controller
    ax.plot([11.75, 11.75, 5.4], [3.4, 2.62, 2.62], color=AQUA, lw=1.8)
    arrow(ax, (5.4, 2.62), (5.4, 1.85), color=AQUA, lw=1.8)
    ax.text(8.2, 2.68, "tuned $K_P,\\ K_I,\\ K_D$ loaded into Simulink", color=AQUA, fontsize=10, ha="center", va="bottom")

    fig.savefig(OUT / "system_architecture.png", dpi=160, bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ----------------------------------------------------------------------------
def ga_flowchart():
    fig, ax = plt.subplots(figsize=(9.0, 9.2))
    ax.set_xlim(0, 9.0)
    ax.set_ylim(0, 9.4)
    ax.axis("off")
    cx, w = 3.2, 4.6
    steps = [
        (8.3, "Initialise population (20 individuals)\n$\\Psi=[q_1\\dots q_6,\\ r_1,\\ r_2]$ in [lb, ub]", "white"),
        (7.0, "For each individual: $K=\\mathrm{lqr}(A_a,B_a,Q,R)$\nsimulate $\\dot x=(A_a-B_aK)x,\\ x_0=\\mathbf{1}$", "white"),
        (5.7, "Evaluate fitness\n$J=\\int\\|e\\|^2dt+\\sum OS_i+\\sum t_{s,i}+100\\sum SSE_i$", "#ecebf8"),
        (4.4, "Selection  →  Crossover\n$\\Psi_c=\\alpha\\Psi_{p1}+(1-\\alpha)\\Psi_{p2}$", "white"),
        (3.1, "Mutation  $\\Psi_m=\\Psi+\\psi$\n(keeps diversity, avoids premature convergence)", "white"),
    ]
    for y, t, fc in steps:
        box(ax, (cx - w / 2 + 0.4, y - 0.5), w + 0.8, 1.0, t, fc=fc, fs=10.5)
    for (y0, *_), (y1, *_) in zip(steps, steps[1:]):
        arrow(ax, (cx + 0.4, y0 - 0.5), (cx + 0.4, y1 + 0.5))
    # decision
    ax.add_patch(plt.Polygon([[cx + 0.4, 2.1], [cx + 2.2, 1.45], [cx + 0.4, 0.8], [cx - 1.4, 1.45]],
                             fc="white", ec=INK, lw=1.6))
    ax.text(cx + 0.4, 1.45, "1000 generations\nor stalled?", ha="center", va="center", fontsize=10)
    arrow(ax, (cx + 0.4, 2.6), (cx + 0.4, 2.1))
    # loop back
    ax.plot([cx - 1.4, 0.35, 0.35], [1.45, 1.45, 7.0], color=VIOLET, lw=1.6)
    arrow(ax, (0.35, 7.0), (cx - w / 2 + 0.4, 7.0), color=VIOLET)
    ax.text(0.2, 4.3, "no → next generation", color=VIOLET, fontsize=10, ha="center", va="center", rotation=90,
            bbox=dict(fc="white", ec="none", pad=2))
    arrow(ax, (cx + 2.2, 1.45), (cx + 3.0, 1.45), text="yes")
    box(ax, (cx + 3.0, 0.85), 1.9, 1.2, "$Q_{opt},\\ R_{opt}$\n→ $K_{opt}$\n→ $K_P,K_I,K_D$", ec=AQUA, lw=2.2, fs=10)
    ax.text(4.5, 9.2, "Genetic Algorithm for LQR weight tuning", ha="center", fontsize=14, weight="bold")
    fig.savefig(OUT / "ga_flowchart.png", dpi=160, bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    quarter_car()
    architecture()
    ga_flowchart()
    print("diagrams written to", OUT)
