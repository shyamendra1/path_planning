"""
Clothoid Turn Planner for Agricultural Vehicles
================================================
Generates feasible headland turns using the Clothoid (Euler Spiral) method.

Path structure:  Straight → Clothoid in → Circular arc → Clothoid out → Straight
                 (a proper C-S-C path with continuous curvature everywhere)

Curvature profile:
  0 ──ramp up──▶ κ_max ──hold──▶ κ_max ──ramp down──▶ 0

This is exactly what a vehicle does when it steers at a constant rate.
No sudden curvature jumps are possible.

Dependencies: numpy, scipy (Fresnel integrals), matplotlib
"""

import numpy as np
from scipy.special import fresnel   # only scipy use — Fresnel integrals
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec


# ═══════════════════════════════════════════════════════════════
#  1.  CLOTHOID (EULER SPIRAL) PRIMITIVES
# ═══════════════════════════════════════════════════════════════

def clothoid_points(kappa_max, L_spiral, n=200, direction=1):
    """
    Compute points on a clothoid that ramps curvature from 0 → kappa_max
    over arc-length L_spiral.

    The clothoid parameter A satisfies:  kappa(s) = s / A²
    At s = L_spiral:  kappa_max = L_spiral / A²  →  A² = L_spiral / kappa_max

    Uses Fresnel integrals for exact x,y.

    direction: +1 = left turn (CCW),  -1 = right turn (CW)
    Returns arrays x, y (local frame, starting at origin heading right)
    """
    A_sq  = L_spiral / kappa_max          # clothoid parameter squared
    A     = np.sqrt(A_sq)

    s = np.linspace(0, L_spiral, n)

    # Fresnel: x(s) = ∫cos(π t²/2) dt,  y(s) = ∫sin(π t²/2) dt
    # but our angle is θ(s) = s²/(2A²), so we use raw integration
    # via the substitution matching scipy's fresnel convention:
    #   scipy.fresnel(u): S(u)=∫₀ᵘ sin(πt²/2)dt, C(u)=∫₀ᵘ cos(πt²/2)dt
    #   Our θ(s) = s²/(2A²)
    #   u = s / (A√π)
    u = s / (A * np.sqrt(np.pi))
    S, C = fresnel(u)   # note: scipy returns (S, C) — S=sin integral, C=cos integral

    x =  A * np.sqrt(np.pi) * C
    y =  A * np.sqrt(np.pi) * S * direction

    # heading at end of spiral
    theta_end = (L_spiral**2) / (2 * A_sq)   # radians

    return x, y, theta_end * direction


def rotate_translate(x, y, theta, ox, oy):
    """Rotate local (x,y) by theta then translate to (ox,oy)."""
    xr = np.cos(theta) * x - np.sin(theta) * y + ox
    yr = np.sin(theta) * x + np.cos(theta) * y + oy
    return xr, yr


# ═══════════════════════════════════════════════════════════════
#  2.  FULL CSC PATH:  Clothoid-in → Arc → Clothoid-out
# ═══════════════════════════════════════════════════════════════

def build_csc_turn(P_start, heading_in_deg, heading_out_deg,
                   R_min, L_spiral=None, direction=None, n=300):
    """
    Build a Clothoid → Circular Arc → Clothoid path between two headings.

    Parameters
    ----------
    P_start        : (x,y) start position
    heading_in_deg : entry heading (degrees, CCW from +x)
    heading_out_deg: exit  heading (degrees, CCW from +x)
    R_min          : minimum turning radius (m)
    L_spiral       : arc-length of each spiral transition (auto if None)
    direction      : +1 left, -1 right (auto-detect if None)
    n              : points per segment

    Returns
    -------
    dict with full path arrays and curvature profile
    """
    kappa_max = 1.0 / R_min

    # ── heading change ──────────────────────────────────────────
    h_in  = np.deg2rad(heading_in_deg)
    h_out = np.deg2rad(heading_out_deg)

    delta = h_out - h_in                    # signed heading change
    # normalise to (-π, π]
    delta = (delta + np.pi) % (2 * np.pi) - np.pi

    if direction is None:
        direction = +1 if delta >= 0 else -1

    delta = abs(delta)                      # total turn angle (positive)

    # ── spiral parameters ────────────────────────────────────────
    # Each spiral contributes  θ_spiral = L²/(2A²) = L·kappa_max/2  heading change
    # Arc contributes  θ_arc = delta - 2·θ_spiral
    # We need θ_arc ≥ 0, so  L_spiral ≤ 2·delta / kappa_max

    L_spiral_max = 2.0 * delta / kappa_max
    if L_spiral is None:
        # Use up to 40% of the turn for each spiral (smooth transition)
        L_spiral = min(0.4 * L_spiral_max, R_min * np.pi / 2)
    L_spiral = min(L_spiral, L_spiral_max)   # clamp

    theta_spiral = (L_spiral * kappa_max) / 2.0   # heading change per spiral
    theta_arc    = delta - 2 * theta_spiral        # remaining arc angle
    L_arc        = R_min * theta_arc               # arc length

    # ── build segments in LOCAL frame (start at origin, heading=0) ──

    # Segment 1: spiral in  (0 → kappa_max)
    sx1, sy1, _ = clothoid_points(kappa_max, L_spiral, n, direction)

    # End pose of spiral-in
    pos1  = np.array([sx1[-1], sy1[-1]])
    head1 = theta_spiral * direction          # heading after spiral-in

    # Segment 2: constant-radius arc
    R = R_min
    # centre of curvature is perpendicular to current heading
    perp = head1 + direction * np.pi / 2
    cx = pos1[0] + R * np.cos(perp)
    cy = pos1[1] + R * np.sin(perp)

    ang_start = head1 - direction * np.pi / 2
    ang_end   = ang_start + direction * theta_arc
    arc_angles = np.linspace(ang_start, ang_end, n)
    ax2 = cx + R * np.cos(arc_angles)
    ay2 = cy + R * np.sin(arc_angles)

    pos2  = np.array([ax2[-1], ay2[-1]])
    head2 = head1 + direction * theta_arc

    # Segment 3: spiral out  (kappa_max → 0)
    # Mirror of spiral-in, then rotate/translate
    sx3_loc, sy3_loc, _ = clothoid_points(kappa_max, L_spiral, n, direction)
    # Mirror: reverse parameter (ramp down)
    sx3_loc = sx3_loc[-1] - sx3_loc[::-1]
    sy3_loc = sy3_loc[-1] - sy3_loc[::-1]

    # Rotate by head2, translate to pos2
    # But the spiral-out starts at head2 and ends at head2+theta_spiral = h_out (local)
    # The mirror trick: rotate by (head2 - theta_spiral*direction) after flip
    rot = head2
    sx3 = np.cos(rot)*sx3_loc - np.sin(rot)*sy3_loc + pos2[0]
    sy3 = np.sin(rot)*sx3_loc + np.cos(rot)*sy3_loc + pos2[1]

    # ── assemble in LOCAL frame ─────────────────────────────────
    lx = np.concatenate([sx1, ax2, sx3])
    ly = np.concatenate([sy1, ay2, sy3])

    # ── transform to WORLD frame ────────────────────────────────
    wx, wy = rotate_translate(lx, ly, h_in,
                               P_start[0], P_start[1])

    # ── curvature profile ────────────────────────────────────────
    N1, N2, N3 = n, n, n
    s1 = np.linspace(0, L_spiral, N1)
    s2 = np.linspace(0, L_arc,    N2)
    s3 = np.linspace(0, L_spiral, N3)

    k1 = direction * s1 * kappa_max / L_spiral         # ramp up
    k2 = direction * np.full(N2, kappa_max)            # hold
    k3 = direction * (1 - s3/L_spiral) * kappa_max     # ramp down

    kappa_all = np.concatenate([k1, k2, k3])

    S1 = L_spiral
    S2 = L_spiral + L_arc
    S3 = 2 * L_spiral + L_arc
    arc_s = np.concatenate([s1, S1 + s2, S2 + s3])

    # ── end point & heading ─────────────────────────────────────
    P_end        = np.array([wx[-1], wy[-1]])
    heading_actual_deg = np.rad2deg(h_in + direction * delta)

    return {
        'wx': wx, 'wy': wy,
        'arc_s': arc_s, 'kappa': kappa_all,
        'P_end': P_end,
        'heading_out_actual_deg': heading_actual_deg,
        'L_spiral': L_spiral, 'L_arc': L_arc,
        'theta_spiral_deg': np.rad2deg(theta_spiral),
        'theta_arc_deg': np.rad2deg(theta_arc),
        'total_length': S3,
        'kappa_max': kappa_max,
        'R_min': R_min,
        'direction': direction,
    }


# ═══════════════════════════════════════════════════════════════
#  3.  STRAIGHT APPROACH / DEPARTURE LEGS
# ═══════════════════════════════════════════════════════════════

def straight_leg(P, heading_deg, length, n=100):
    h = np.deg2rad(heading_deg)
    t = np.linspace(0, length, n)
    x = P[0] + t * np.cos(h)
    y = P[1] + t * np.sin(h)
    return x, y


# ═══════════════════════════════════════════════════════════════
#  4.  PLOTTING
# ═══════════════════════════════════════════════════════════════

DARK_BG   = '#0d1117'
PANEL_BG  = '#161b22'
GRID_COL  = '#21262d'
SPINE_COL = '#30363d'
TEXT_COL  = '#c9d1d9'
DIM_COL   = '#8b949e'

BLUE   = '#58a6ff'
GREEN  = '#3fb950'
ORANGE = '#ffa657'
RED    = '#f85149'
PURPLE = '#bc8cff'


def style_ax(ax):
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors=DIM_COL, labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor(SPINE_COL)
    ax.grid(True, color=GRID_COL, lw=0.6)
    ax.xaxis.label.set_color(DIM_COL)
    ax.yaxis.label.set_color(DIM_COL)


def arrow(ax, P, hdeg, length, color, label=None):
    h = np.deg2rad(hdeg)
    d = np.array([np.cos(h), np.sin(h)]) * length
    ax.annotate("", xy=P + d, xytext=P,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8,
                                mutation_scale=12))
    if label:
        off = np.array([np.cos(h + 0.5), np.sin(h + 0.5)]) * length * 0.6
        ax.text(*(P + d + off), label, color=color, fontsize=8, ha='center')


def plot_turn(result, P_start, heading_in_deg, heading_out_deg,
              straight_len=5.0):

    # Build full path with straight legs
    sx_in,  sy_in  = straight_leg(
        P_start - np.deg2rad(heading_in_deg) * 0 ,
        heading_in_deg, straight_len)
    # Straight in ends at P_start
    ix = np.linspace(P_start[0] - straight_len*np.cos(np.deg2rad(heading_in_deg)),
                     P_start[0], 80)
    iy = np.linspace(P_start[1] - straight_len*np.sin(np.deg2rad(heading_in_deg)),
                     P_start[1], 80)

    P_end = result['P_end']
    ox = np.linspace(P_end[0],
                     P_end[0] + straight_len*np.cos(np.deg2rad(heading_out_deg)), 80)
    oy = np.linspace(P_end[1],
                     P_end[1] + straight_len*np.sin(np.deg2rad(heading_out_deg)), 80)

    # ── figure ──────────────────────────────────────────────────
    fig = plt.figure(figsize=(15, 8), facecolor=DARK_BG)
    fig.suptitle("Clothoid Turn Planner  ·  Agricultural Vehicle",
                 color='white', fontsize=15, fontweight='bold', y=0.97)

    gs = GridSpec(2, 3, figure=fig,
                  hspace=0.45, wspace=0.38,
                  left=0.06, right=0.97, top=0.90, bottom=0.09)

    ax_path  = fig.add_subplot(gs[:, :2])
    ax_kappa = fig.add_subplot(gs[0, 2])
    ax_steer = fig.add_subplot(gs[1, 2])

    for ax in [ax_path, ax_kappa, ax_steer]:
        style_ax(ax)

    # ── Path ────────────────────────────────────────────────────
    # Approach / departure
    ax_path.plot(ix, iy, color=DIM_COL, lw=2, ls='--', alpha=0.6,
                 label='Straight rows')
    ax_path.plot(ox, oy, color=DIM_COL, lw=2, ls='--', alpha=0.6)

    # Segment shading: spiral-in, arc, spiral-out
    wx, wy = result['wx'], result['wy']
    N = len(wx)
    n_each = N // 3

    ax_path.plot(wx[:n_each],         wy[:n_each],
                 color=ORANGE, lw=2.8, label='Clothoid in  (κ: 0→κ_max)')
    ax_path.plot(wx[n_each:2*n_each], wy[n_each:2*n_each],
                 color=BLUE,   lw=2.8, label='Circular arc (κ = κ_max)')
    ax_path.plot(wx[2*n_each:],       wy[2*n_each:],
                 color=GREEN,  lw=2.8, label='Clothoid out (κ: κ_max→0)')

    # Endpoints & arrows
    ax_path.scatter(*P_start, s=120, color=RED,    zorder=6)
    ax_path.scatter(*P_end,   s=120, color=PURPLE, zorder=6, marker='*')
    ax_path.text(*P_start + np.array([-0.3, -0.5]), 'Entry',
                 color=RED, fontsize=9)
    ax_path.text(*P_end   + np.array([0.1,  0.3]),  'Exit',
                 color=PURPLE, fontsize=9)

    scale = result['R_min'] * 0.35
    arrow(ax_path, P_start, heading_in_deg,  scale, ORANGE,
          f'{heading_in_deg}°')
    arrow(ax_path, P_end,   heading_out_deg, scale, GREEN,
          f'{heading_out_deg:.0f}°')

    # Min-radius circle (reference)
    cx = P_start[0]
    cy = P_start[1] + result['direction'] * result['R_min']
    circ = plt.Circle((cx, cy), result['R_min'],
                       color=BLUE, fill=False, ls=':', lw=1, alpha=0.3)
    ax_path.add_patch(circ)
    ax_path.text(cx, cy, f"R_min\n{result['R_min']} m",
                 color=BLUE, fontsize=7.5, ha='center', va='center', alpha=0.5)

    ax_path.set_aspect('equal')
    ax_path.set_title("Feasible Headland Turn Path", color='white',
                       fontsize=12, pad=8)
    ax_path.set_xlabel("x (m)"); ax_path.set_ylabel("y (m)")
    ax_path.legend(fontsize=8.5, facecolor='#21262d',
                   labelcolor='white', edgecolor=SPINE_COL, loc='best')

    # ── Curvature profile ───────────────────────────────────────
    s = result['arc_s']
    k = result['kappa']
    km = result['kappa_max']

    ax_kappa.plot(s, k, color=BLUE, lw=2.2)
    ax_kappa.axhline( km, color=RED, ls='--', lw=1.2,
                      label=f'+κ_max = {km:.3f} (R={result["R_min"]} m)')
    ax_kappa.axhline(-km, color=RED, ls=':', lw=1.2,
                      label=f'−κ_max')
    ax_kappa.axhline(0, color=DIM_COL, lw=0.7)

    # shade segments
    n1 = len(s) // 3
    ax_kappa.axvspan(s[0],  s[n1],    alpha=0.12, color=ORANGE)
    ax_kappa.axvspan(s[n1], s[2*n1],  alpha=0.12, color=BLUE)
    ax_kappa.axvspan(s[2*n1], s[-1],  alpha=0.12, color=GREEN)

    ax_kappa.set_title("Curvature κ(s)  — no sudden jumps", color='white',
                        fontsize=10, pad=6)
    ax_kappa.set_xlabel("Arc length s (m)")
    ax_kappa.set_ylabel("κ  (1/m)")
    ax_kappa.legend(fontsize=7.5, facecolor='#21262d',
                    labelcolor='white', edgecolor=SPINE_COL)
    ax_kappa.set_ylim(-km * 1.5, km * 1.5)

    # ── Steering angle (δ = atan(L·κ),  L = wheelbase) ─────────
    L_wb = 2.5   # typical tractor wheelbase (m)
    delta_steer = np.rad2deg(np.arctan(L_wb * k))
    delta_max   = np.rad2deg(np.arctan(L_wb * km))

    ax_steer.plot(s, delta_steer, color=ORANGE, lw=2.2)
    ax_steer.axhline( delta_max, color=RED, ls='--', lw=1.2,
                      label=f'+δ_max = {delta_max:.1f}°')
    ax_steer.axhline(-delta_max, color=RED, ls=':',  lw=1.2)
    ax_steer.axhline(0, color=DIM_COL, lw=0.7)
    ax_steer.set_title(f"Steering angle δ(s)  (wheelbase {L_wb} m)",
                        color='white', fontsize=10, pad=6)
    ax_steer.set_xlabel("Arc length s (m)")
    ax_steer.set_ylabel("δ  (degrees)")
    ax_steer.legend(fontsize=7.5, facecolor='#21262d',
                    labelcolor='white', edgecolor=SPINE_COL)
    ax_steer.set_ylim(-delta_max * 1.5, delta_max * 1.5)

    # ── Summary ─────────────────────────────────────────────────
    info = (f"Turn: {heading_in_deg}° → {heading_out_deg}°  |  "
            f"R_min = {result['R_min']} m  |  "
            f"Spiral L = {result['L_spiral']:.2f} m  (Δθ = {result['theta_spiral_deg']:.1f}°)  |  "
            f"Arc L = {result['L_arc']:.2f} m  (Δθ = {result['theta_arc_deg']:.1f}°)  |  "
            f"Total turn L = {result['total_length']:.2f} m")
    fig.text(0.5, 0.006, info, ha='center', va='bottom',
             color=DIM_COL, fontsize=8.5,
             bbox=dict(facecolor='#21262d', edgecolor=SPINE_COL,
                       boxstyle='round,pad=0.35'))

    plt.show()


# ═══════════════════════════════════════════════════════════════
#  5.  MAIN  — edit parameters here
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":

    # ── User-defined inputs ─────────────────────────────────────
    P_start         = np.array([0.0, 0.0])   # entry point (m)
    heading_in_deg  = 90.0    # vehicle heading at entry  (degrees, CCW from +x)
    heading_out_deg = -90.0   # vehicle heading at exit   (-90° = pointing down = 180° turn)

    R_min    = 3.0            # minimum turning radius (m)  — hard mechanical limit
    L_spiral = None           # spiral transition length (m); None = auto

    # direction: +1 = turn left (CCW), -1 = turn right (CW), None = auto
    direction = None
    # ────────────────────────────────────────────────────────────

    print("=" * 60)
    print("  Clothoid Headland Turn Planner  —  Agricultural Vehicle")
    print("=" * 60)

    result = build_csc_turn(
        P_start, heading_in_deg, heading_out_deg,
        R_min, L_spiral=L_spiral, direction=direction
    )

    print(f"  Entry heading     : {heading_in_deg}°")
    print(f"  Exit  heading     : {heading_out_deg}°")
    print(f"  R_min             : {R_min} m  →  κ_max = {result['kappa_max']:.4f} 1/m")
    print(f"  Spiral arc length : {result['L_spiral']:.3f} m  "
          f"(Δθ = {result['theta_spiral_deg']:.2f}° each)")
    print(f"  Circular arc len  : {result['L_arc']:.3f} m  "
          f"(Δθ = {result['theta_arc_deg']:.2f}°)")
    print(f"  Total turn length : {result['total_length']:.3f} m")
    print(f"  Exit point        : {result['P_end']}")
    print(f"  |κ|_max in path   : {np.max(np.abs(result['kappa'])):.4f}  "
          f"(limit: {result['kappa_max']:.4f})")
    print("=" * 60)

    plot_turn(result, P_start, heading_in_deg, heading_out_deg,
              straight_len=4.0)
