"""
Headland Turn Planner for Agricultural Vehicles
================================================
INPUT : start pose (x, y, heading°), end pose (x, y, heading°), R_min (m)
OUTPUT: shortest Dubins path (LSL / RSR / LSR / RSL) with clothoid
        transitions so curvature is always continuous — no sudden steering jumps.

Algorithm
---------
1. Solve all 4 Dubins word types, pick shortest feasible path.
2. At every curvature-discontinuous join (arc↔straight, arc↔arc),
   insert a clothoid (Euler spiral) transition segment.
3. Plot path, curvature κ(s), and steering angle δ(s).

Dependencies: numpy, scipy (Fresnel integrals), matplotlib
"""

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec


# ═══════════════════════════════════════════════════════════════
# Utilities
# ═══════════════════════════════════════════════════════════════

def wrap2pi(a):   return a % (2 * np.pi)             # [0, 2π)
def wrappi(a):    return (a + np.pi) % (2*np.pi) - np.pi  # (−π, π]

def rot2d(pts, theta):
    c, s = np.cos(theta), np.sin(theta)
    return (np.array([[c,-s],[s,c]]) @ pts.T).T

def circle_center(p, theta, R, turn):
    """turn: +1=left/CCW, -1=right/CW"""
    perp = theta + turn * np.pi / 2
    return p + R * np.array([np.cos(perp), np.sin(perp)])

def arc_sweep(a_start, a_end, turn):
    """Positive sweep angle from a_start to a_end going in `turn` direction."""
    if turn > 0:
        sw = wrap2pi(a_end - a_start)
    else:
        sw = wrap2pi(a_start - a_end)
    return sw   # always in [0, 2π)

def arc_sample(center, R, a_start, sweep_rad, turn, n=200):
    angles = np.linspace(a_start, a_start + sweep_rad * turn, n)
    return center + R * np.column_stack([np.cos(angles), np.sin(angles)])

def heading_at(p, center, turn):
    """Vehicle heading when at point p on a circle."""
    angle = np.arctan2(p[1]-center[1], p[0]-center[0])
    return wrappi(angle + turn * np.pi / 2)


# ═══════════════════════════════════════════════════════════════
# Dubins Path Solver (4 word types, handles d→0 edge cases)
# ═══════════════════════════════════════════════════════════════

def _base(word, p0, p1, t0, t1, R, c0, c1, sw0, sw1, straight, T0, T1, turn0, turn1,
          ang0_start, ang0_end, ang1_start, ang1_end):
    L = sw0*R + straight + sw1*R
    return dict(word=word, L=L, R_min=R, p0=p0, p1=p1, t0=t0, t1=t1,
                c0=c0, c1=c1, sw0=sw0, sw1=sw1, d=straight,
                T0=T0, T1=T1, turn0=turn0, turn1=turn1,
                ang0_start=ang0_start, ang0_end=ang0_end,
                ang1_start=ang1_start, ang1_end=ang1_end,
                L0=sw0*R, Ls=straight, L1=sw1*R)

def solve_same_turn(p0, t0, p1, t1, R, turn, word):
    """LSL or RSR — both arcs in same direction."""
    c0 = circle_center(p0, t0, R, turn)
    c1 = circle_center(p1, t1, R, turn)
    v  = c1 - c0;  d = np.linalg.norm(v)
    a0_p0 = np.arctan2(p0[1]-c0[1], p0[0]-c0[0])
    a1_p1 = np.arctan2(p1[1]-c1[1], p1[0]-c1[0])

    if d < 1e-6:
        # Same circle — single direct arc
        sw0 = arc_sweep(a0_p0, a1_p1, turn)
        return _base(word, p0, p1, t0, t1, R, c0, c1,
                     sw0, 0.0, 0.0, p0, p1, turn, turn,
                     a0_p0, a1_p1, a1_p1, a1_p1)

    psi = np.arctan2(v[1], v[0])
    if turn > 0:
        a0_dep = wrappi(psi - np.pi/2)   # LSL tangent depart angle on c0
    else:
        a0_dep = wrappi(psi + np.pi/2)   # RSR

    sw0 = arc_sweep(a0_p0, a0_dep, turn)
    T0  = c0 + R * np.array([np.cos(a0_dep), np.sin(a0_dep)])
    T1  = c1 + R * np.array([np.cos(a0_dep), np.sin(a0_dep)])
    straight = np.linalg.norm(T1 - T0)
    a1_arr = a0_dep
    sw1 = arc_sweep(a1_arr, a1_p1, turn)

    return _base(word, p0, p1, t0, t1, R, c0, c1,
                 sw0, sw1, straight, T0, T1, turn, turn,
                 a0_p0, a0_dep, a1_arr, a1_p1)

def solve_cross_turn(p0, t0, p1, t1, R, turn0, turn1, word):
    """LSR or RSL — arcs in opposite directions."""
    c0 = circle_center(p0, t0, R, turn0)
    c1 = circle_center(p1, t1, R, turn1)
    v  = c1 - c0;  d = np.linalg.norm(v)
    if d < 2*R - 1e-9:
        return None   # circles overlap, no cross-tangent

    psi   = np.arctan2(v[1], v[0])
    alpha = np.arccos(2*R / d)

    if turn0 > 0:  # LSR
        a0_dep = wrappi(psi + alpha - np.pi/2)
        a1_arr = wrappi(psi + alpha + np.pi/2)
    else:          # RSL
        a0_dep = wrappi(psi - alpha + np.pi/2)
        a1_arr = wrappi(psi - alpha - np.pi/2)

    a0_p0 = np.arctan2(p0[1]-c0[1], p0[0]-c0[0])
    a1_p1 = np.arctan2(p1[1]-c1[1], p1[0]-c1[0])
    sw0   = arc_sweep(a0_p0, a0_dep, turn0)
    sw1   = arc_sweep(a1_arr, a1_p1, turn1)
    T0    = c0 + R * np.array([np.cos(a0_dep), np.sin(a0_dep)])
    T1    = c1 + R * np.array([np.cos(a1_arr), np.sin(a1_arr)])
    straight = np.linalg.norm(T1 - T0)

    return _base(word, p0, p1, t0, t1, R, c0, c1,
                 sw0, sw1, straight, T0, T1, turn0, turn1,
                 a0_p0, a0_dep, a1_arr, a1_p1)

def solve_dubins(p0, t0_deg, p1, t1_deg, R):
    t0 = np.deg2rad(t0_deg)
    t1 = np.deg2rad(t1_deg)
    candidates = [
        solve_same_turn(p0, t0, p1, t1, R, +1, 'LSL'),
        solve_same_turn(p0, t0, p1, t1, R, -1, 'RSR'),
        solve_cross_turn(p0, t0, p1, t1, R, +1, -1, 'LSR'),
        solve_cross_turn(p0, t0, p1, t1, R, -1, +1, 'RSL'),
    ]
    valid = [c for c in candidates if c is not None]
    if not valid:
        raise ValueError("No feasible Dubins path found for these poses and R_min.")
    return min(valid, key=lambda c: c['L'])


# ═══════════════════════════════════════════════════════════════
# Clothoid spiral: curvature ramps linearly with arc length
# ═══════════════════════════════════════════════════════════════

def clothoid_segment(k_start, k_end, L, n=200):
    """
    Euler spiral from curvature k_start → k_end over arc length L.
    Starting at origin, heading 0.
    Returns x, y arrays (world-frame after caller applies rotation+translation).
    """
    if L < 1e-9:
        return np.zeros(n), np.zeros(n), np.zeros(n)
    s     = np.linspace(0, L, n)
    # heading at s: θ(s) = k_start·s + (k_end-k_start)·s²/(2L)
    theta = k_start * s + (k_end - k_start) * s**2 / (2 * L)
    # integrate dx=cos θ ds, dy=sin θ ds  via trapezoidal
    x = np.zeros(n); y = np.zeros(n)
    for i in range(1, n):
        ds      = s[i] - s[i-1]
        x[i]    = x[i-1] + np.cos(theta[i-1]) * ds
        y[i]    = y[i-1] + np.sin(theta[i-1]) * ds
    return x, y, theta   # theta[-1] = heading at end of spiral


def place_segment(x_loc, y_loc, heading_world, origin_world):
    """Rotate local segment to heading_world and translate to origin_world."""
    pts = rot2d(np.column_stack([x_loc, y_loc]), heading_world)
    return pts + origin_world


# ═══════════════════════════════════════════════════════════════
# Path Sampling — raw Dubins (for comparison)
# ═══════════════════════════════════════════════════════════════

def sample_dubins(sol, n=300):
    R   = sol['R_min']
    c0, c1 = sol['c0'], sol['c1']
    seg0 = arc_sample(c0, R, sol['ang0_start'], sol['sw0'], sol['turn0'], n)
    if sol['d'] > 1e-4:
        t    = np.linspace(0, 1, n)
        seg1 = sol['T0'][None,:] + t[:,None] * (sol['T1']-sol['T0'])
    else:
        seg1 = np.empty((0,2))
    seg2 = arc_sample(c1, R, sol['ang1_start'], sol['sw1'], sol['turn1'], n)

    xs = np.concatenate([seg0[:,0], seg1[:,0], seg2[:,0]])
    ys = np.concatenate([seg0[:,1], seg1[:,1], seg2[:,1]])
    kappa = np.concatenate([
        np.full(len(seg0), sol['turn0']/R),
        np.full(len(seg1), 0.0),
        np.full(len(seg2), sol['turn1']/R),
    ])
    ds    = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    arc_s = np.concatenate([[0], np.cumsum(ds)])
    return dict(xs=xs, ys=ys, arc_s=arc_s, kappa=kappa)


# ═══════════════════════════════════════════════════════════════
# Smooth path: replace curvature joins with clothoid transitions
# ═══════════════════════════════════════════════════════════════

def smooth_dubins(sol, L_trans=None, n=250):
    """
    Build a G2-continuous path by inserting clothoid spirals at every
    curvature discontinuity in the Dubins path.

    Join types:
      Arc0-Straight:   k0 → 0     (clothoid)
      Straight-Arc1:   0  → k1    (clothoid)
      Arc0-Arc1 (d=0): k0 → k1    (single clothoid)
    """
    R   = sol['R_min']
    k0  = sol['turn0'] / R
    k1  = sol['turn1'] / R
    L0, Ls, L1 = sol['L0'], sol['Ls'], sol['L1']

    # Transition length: auto or user-supplied, clamped
    if L_trans is None:
        L_trans = min(R * np.pi / 4, L0 * 0.35, L1 * 0.35)
        if Ls > 0.01:
            L_trans = min(L_trans, Ls * 0.35)
    L_trans = max(L_trans, 0.05)
    L_trans = min(L_trans, L0 * 0.45, L1 * 0.45)
    if Ls > 0.01:
        L_trans = min(L_trans, Ls * 0.45)

    all_xy = []; all_k = []

    has_straight = Ls > 1e-3

    if has_straight:
        # ── 5-segment layout ──────────────────────────────────
        # [A] pure arc0  →  [B] clothoid k0→0  →  [C] straight  →  [D] clothoid 0→k1  →  [E] pure arc1

        # A: pure arc0
        sw_A = max((L0 - L_trans) / R, 1e-6)
        pts_A = arc_sample(sol['c0'], R, sol['ang0_start'], sw_A, sol['turn0'], n)
        k_A   = np.full(len(pts_A), k0)

        head_A = heading_at(pts_A[-1], sol['c0'], sol['turn0'])

        # B: clothoid k0 → 0
        xb, yb, th_b = clothoid_segment(k0, 0.0, L_trans, n)
        pts_B = place_segment(xb, yb, head_A, pts_A[-1])
        k_B   = np.linspace(k0, 0, len(pts_B))
        head_B = head_A + th_b[-1]   # heading at end of B

        # C: straight (trimmed to remove transition zones)
        # Walk along the straight tangent direction from end of B
        tang  = np.array([np.cos(head_B), np.sin(head_B)])
        Ls_pure = max(Ls - 2 * L_trans, 0)
        if Ls_pure > 1e-3:
            t     = np.linspace(0, Ls_pure, n)
            pts_C = pts_B[-1][None,:] + t[:,None] * tang[None,:]
            k_C   = np.zeros(len(pts_C))
            head_C = head_B
        else:
            pts_C = pts_B[-1:]; k_C = np.array([0.0]); head_C = head_B

        # D: clothoid 0 → k1
        xd, yd, th_d = clothoid_segment(0.0, k1, L_trans, n)
        pts_D = place_segment(xd, yd, head_C, pts_C[-1])
        k_D   = np.linspace(0, k1, len(pts_D))
        head_D = head_C + th_d[-1]

        # E: pure arc1 — advance start angle by L_trans already consumed
        sw_adv = L_trans / R
        ang1_s = sol['ang1_start'] + sw_adv * sol['turn1']
        sw_E   = max(sol['sw1'] - sw_adv, 1e-6)
        pts_E  = arc_sample(sol['c1'], R, ang1_s, sw_E, sol['turn1'], n)
        k_E    = np.full(len(pts_E), k1)

        segs = [pts_A, pts_B, pts_C, pts_D, pts_E]
        kaps = [k_A,   k_B,   k_C,   k_D,   k_E]

    else:
        # ── 3-segment layout (no straight) ────────────────────
        # [A] pure arc0  →  [B] clothoid k0→k1  →  [C] pure arc1

        L_cross = 2 * L_trans   # use 2x for arc-to-arc transition

        # A: pure arc0
        sw_A = max((L0 - L_trans) / R, 1e-6)
        pts_A = arc_sample(sol['c0'], R, sol['ang0_start'], sw_A, sol['turn0'], n)
        k_A   = np.full(len(pts_A), k0)
        head_A = heading_at(pts_A[-1], sol['c0'], sol['turn0'])

        # B: clothoid k0 → k1
        xb, yb, th_b = clothoid_segment(k0, k1, L_cross, n)
        pts_B = place_segment(xb, yb, head_A, pts_A[-1])
        k_B   = np.linspace(k0, k1, len(pts_B))
        head_B = head_A + th_b[-1]

        # C: pure arc1 — advance by L_trans
        sw_adv = L_trans / R
        ang1_s = sol['ang1_start'] + sw_adv * sol['turn1']
        sw_C   = max(sol['sw1'] - sw_adv, 1e-6)
        pts_C  = arc_sample(sol['c1'], R, ang1_s, sw_C, sol['turn1'], n)
        k_C    = np.full(len(pts_C), k1)

        segs = [pts_A, pts_B, pts_C]
        kaps = [k_A,   k_B,   k_C]

    xs    = np.concatenate([s[:,0] for s in segs])
    ys    = np.concatenate([s[:,1] for s in segs])
    kappa = np.concatenate(kaps)

    ds    = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    arc_s = np.concatenate([[0], np.cumsum(ds)])

    return dict(xs=xs, ys=ys, arc_s=arc_s, kappa=kappa, L_trans=L_trans)


# ═══════════════════════════════════════════════════════════════
# Plot
# ═══════════════════════════════════════════════════════════════

DARK='#0d1117'; PANEL='#161b22'; GRID='#21262d'; SPINE='#30363d'
DIM='#8b949e'; BLUE='#58a6ff'; GREEN='#3fb950'
ORANGE='#ffa657'; RED='#f85149'; PURPLE='#bc8cff'

def _sax(ax):
    ax.set_facecolor(PANEL)
    ax.tick_params(colors=DIM, labelsize=8)
    for sp in ax.spines.values(): sp.set_edgecolor(SPINE)
    ax.grid(True, color=GRID, lw=0.6)
    ax.xaxis.label.set_color(DIM); ax.yaxis.label.set_color(DIM)

def _arrow(ax, pos, deg, length, color, label=None):
    h = np.deg2rad(deg)
    d = np.array([np.cos(h), np.sin(h)]) * length
    ax.annotate("", xy=pos+d, xytext=pos,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=2, mutation_scale=14))
    if label:
        off = np.array([np.cos(h+0.5), np.sin(h+0.5)]) * length * 0.65
        ax.text(*(pos+d+off), label, color=color, fontsize=9, ha='center', fontweight='bold')

def plot_result(sol, raw, smooth, t0_deg, t1_deg):
    R    = sol['R_min']
    kmax = 1.0 / R
    p0, p1 = sol['p0'], sol['p1']

    fig = plt.figure(figsize=(16, 9), facecolor=DARK)
    fig.suptitle(
        f"Headland Turn  ·  {sol['word']} path  ·  R_min = {R} m  ·  "
        f"Total length = {sol['L']:.2f} m",
        color='white', fontsize=14, fontweight='bold', y=0.97)

    gs = GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.36,
                  left=0.06, right=0.97, top=0.91, bottom=0.09)
    ax_path  = fig.add_subplot(gs[:, :2])
    ax_kappa = fig.add_subplot(gs[0, 2])
    ax_steer = fig.add_subplot(gs[1, 2])
    for ax in [ax_path, ax_kappa, ax_steer]: _sax(ax)

    # ── Path ─────────────────────────────────────────────────
    scale = R * 0.55
    # approach/departure dashes
    for P, deg in [(p0, t0_deg), (p1, t1_deg)]:
        h = np.deg2rad(deg)
        P_b = P - np.array([np.cos(h), np.sin(h)]) * R * 1.5
        ax_path.plot([P_b[0], P[0]], [P_b[1], P[1]],
                     color=DIM, lw=1.5, ls='--', alpha=0.4)
    h1 = np.deg2rad(t1_deg)
    P_f = p1 + np.array([np.cos(h1), np.sin(h1)]) * R * 1.5
    ax_path.plot([p1[0], P_f[0]], [p1[1], P_f[1]],
                 color=DIM, lw=1.5, ls='--', alpha=0.4)

    # raw Dubins (ghost)
    ax_path.plot(raw['xs'], raw['ys'], color=BLUE, lw=1.0, ls=':', alpha=0.35,
                 label='Dubins skeleton (κ discontinuous)')

    # smooth path
    ax_path.plot(smooth['xs'], smooth['ys'], color=GREEN, lw=3.0, zorder=4,
                 label='Clothoid-smoothed path')

    # turn circles
    for c in [sol['c0'], sol['c1']]:
        circ = patches.Circle(c, R, color=BLUE, fill=False, ls=':', lw=0.8, alpha=0.18)
        ax_path.add_patch(circ)

    # tangent points T0, T1
    if sol['d'] > 0.1:
        ax_path.scatter(*sol['T0'], s=60, color=ORANGE, zorder=5, marker='D', label='Tangent pts')
        ax_path.scatter(*sol['T1'], s=60, color=ORANGE, zorder=5, marker='D')

    # poses
    ax_path.scatter(*p0, s=160, color=ORANGE,  zorder=7)
    ax_path.scatter(*p1, s=220, color=PURPLE,  zorder=7, marker='*')
    ax_path.text(*p0+np.array([-0.2,-0.5]), 'Start', color=ORANGE, fontsize=9, fontweight='bold')
    ax_path.text(*p1+np.array([ 0.1, 0.3]), 'End',   color=PURPLE, fontsize=9, fontweight='bold')
    _arrow(ax_path, p0, t0_deg, scale, ORANGE, f'{t0_deg}°')
    _arrow(ax_path, p1, t1_deg, scale, PURPLE, f'{t1_deg}°')

    ax_path.set_aspect('equal')
    ax_path.set_title('Path  (dashed = crop rows / approach)', color='white', fontsize=12, pad=8)
    ax_path.set_xlabel('x (m)'); ax_path.set_ylabel('y (m)')
    ax_path.legend(fontsize=9, facecolor='#21262d', labelcolor='white',
                   edgecolor=SPINE, loc='best')

    # ── Curvature ─────────────────────────────────────────────
    ax_kappa.plot(raw['arc_s'],    raw['kappa'],    color=BLUE,  lw=1.2, ls=':', alpha=0.6,
                  label='Dubins (step jumps)')
    ax_kappa.plot(smooth['arc_s'], smooth['kappa'], color=GREEN, lw=2.2,
                  label='Clothoid-smoothed')
    for y, ls in [(kmax,'--'),(-kmax,':')]:
        ax_kappa.axhline(y, color=RED, ls=ls, lw=1.3)
    ax_kappa.axhline(0, color=DIM, lw=0.7)
    ax_kappa.text(smooth['arc_s'][-1]*0.02,  kmax*1.05, f'+κ_max = {kmax:.3f}',
                  color=RED, fontsize=7.5)
    ax_kappa.text(smooth['arc_s'][-1]*0.02, -kmax*1.18, f'−κ_max',
                  color=RED, fontsize=7.5)
    ax_kappa.set_title('Curvature κ(s)', color='white', fontsize=10, pad=6)
    ax_kappa.set_xlabel('Arc length s (m)'); ax_kappa.set_ylabel('κ  (1/m)')
    ax_kappa.set_ylim(-kmax*1.7, kmax*1.7)
    ax_kappa.legend(fontsize=8, facecolor='#21262d', labelcolor='white', edgecolor=SPINE)

    # ── Steering angle ─────────────────────────────────────────
    L_wb  = 2.8   # tractor wheelbase (m)
    d_raw = np.rad2deg(np.arctan(L_wb * raw['kappa']))
    d_sm  = np.rad2deg(np.arctan(L_wb * smooth['kappa']))
    d_max = np.rad2deg(np.arctan(L_wb * kmax))

    ax_steer.plot(raw['arc_s'],    d_raw, color=BLUE,   lw=1.2, ls=':', alpha=0.6, label='Dubins')
    ax_steer.plot(smooth['arc_s'], d_sm,  color=ORANGE, lw=2.2, label='Smoothed')
    for y, ls in [(d_max,'--'),(-d_max,':')]:
        ax_steer.axhline(y, color=RED, ls=ls, lw=1.3)
    ax_steer.text(smooth['arc_s'][-1]*0.02,  d_max*1.05, f'+δ_max = {d_max:.1f}°',
                  color=RED, fontsize=7.5)
    ax_steer.axhline(0, color=DIM, lw=0.7)
    ax_steer.set_title(f'Steering angle δ(s)  (wheelbase {L_wb} m)',
                       color='white', fontsize=10, pad=6)
    ax_steer.set_xlabel('Arc length s (m)'); ax_steer.set_ylabel('δ (°)')
    ax_steer.set_ylim(-d_max*1.7, d_max*1.7)
    ax_steer.legend(fontsize=8, facecolor='#21262d', labelcolor='white', edgecolor=SPINE)

    # ── Info bar ──────────────────────────────────────────────
    info = (f"Word: {sol['word']}  |  Arc₀={sol['L0']:.2f} m ({np.rad2deg(sol['sw0']):.1f}°)  "
            f"Straight={sol['Ls']:.2f} m  Arc₁={sol['L1']:.2f} m ({np.rad2deg(sol['sw1']):.1f}°)  |  "
            f"Total={sol['L']:.2f} m  |  "
            f"Clothoid trans={smooth['L_trans']:.2f} m  |  "
            f"|κ|_max achieved={np.max(np.abs(smooth['kappa'])):.4f} (limit {kmax:.4f})")
    fig.text(0.5, 0.005, info, ha='center', va='bottom', color=DIM, fontsize=8.5,
             bbox=dict(facecolor='#21262d', edgecolor=SPINE, boxstyle='round,pad=0.35'))
    plt.show()


# ═══════════════════════════════════════════════════════════════
# MAIN — edit the inputs below
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":

    # ── INPUTS ────────────────────────────────────────────────────
    p0       = np.array([0.0,  0.0])    # start position (m)
    t0_deg   = 90.0                     # start heading  (degrees, CCW from east/+x)

    p1       = np.array([10.0,  -10.0])    # end position   (m)
    t1_deg   = -90.0                    # end heading

    R_min    = 3.0                      # minimum turning radius (m)

    L_trans  = None   # clothoid transition length (m), None = auto-sized
    # ─────────────────────────────────────────────────────────────

    print("=" * 65)
    print("  Headland Turn Planner  —  Agricultural Vehicle")
    print("=" * 65)
    print(f"  Start : {p0}  heading {t0_deg}°")
    print(f"  End   : {p1}  heading {t1_deg}°")
    print(f"  R_min : {R_min} m  (κ_max = {1/R_min:.4f} 1/m)")

    sol    = solve_dubins(p0, t0_deg, p1, t1_deg, R_min)
    raw    = sample_dubins(sol)
    smooth = smooth_dubins(sol, L_trans=L_trans)

    print(f"\n  Best path : {sol['word']}")
    print(f"    Arc0    : {sol['L0']:.3f} m  ({np.rad2deg(sol['sw0']):.1f}°)")
    print(f"    Straight: {sol['Ls']:.3f} m")
    print(f"    Arc1    : {sol['L1']:.3f} m  ({np.rad2deg(sol['sw1']):.1f}°)")
    print(f"    Total   : {sol['L']:.3f} m")
    print(f"\n  |κ|_max in smooth path : {np.max(np.abs(smooth['kappa'])):.4f}  (limit {1/R_min:.4f})")
    print(f"  Start error : {np.linalg.norm(np.array([smooth['xs'][0], smooth['ys'][0]]) - p0):.4f} m")
    print(f"  End   error : {np.linalg.norm(np.array([smooth['xs'][-1], smooth['ys'][-1]]) - p1):.4f} m")
    print("=" * 65)

    plot_result(sol, raw, smooth, t0_deg, t1_deg)
