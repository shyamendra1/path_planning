"""
Bezier Curve Generator with Curvature Constraint
--------------------------------------------------
Generates a cubic Bezier curve between two points given:
  - Start/end positions
  - Start/end heading directions (in degrees)
  - Maximum curvature constraint (1/min_turning_radius)

Usage: python bezier_curve.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec


# ─────────────────────────────────────────────
#  Core Bezier math
# ─────────────────────────────────────────────

def cubic_bezier(P0, P1, P2, P3, t):
    """Evaluate cubic Bezier at parameter t ∈ [0,1]."""
    t = np.asarray(t)[:, None]          # (N,1) for broadcasting
    return (  (1 - t)**3 * P0
            + 3*(1 - t)**2 * t  * P1
            + 3*(1 - t)   * t**2 * P2
            +                t**3 * P3  )


def cubic_bezier_derivative(P0, P1, P2, P3, t):
    """First derivative d/dt of cubic Bezier."""
    t = np.asarray(t)[:, None]
    return (  3*(1 - t)**2        * (P1 - P0)
            + 6*(1 - t)   * t     * (P2 - P1)
            + 3            * t**2 * (P3 - P2)  )


def cubic_bezier_second_derivative(P0, P1, P2, P3, t):
    """Second derivative d²/dt² of cubic Bezier."""
    t = np.asarray(t)[:, None]
    return (  6*(1 - t) * (P2 - 2*P1 + P0)
            + 6 * t      * (P3 - 2*P2 + P1)  )


def curvature(P0, P1, P2, P3, t):
    """
    Signed curvature κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
    """
    t = np.asarray(t)
    d1 = cubic_bezier_derivative(P0, P1, P2, P3, t)       # (N,2)
    d2 = cubic_bezier_second_derivative(P0, P1, P2, P3, t) # (N,2)
    cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    speed_sq = np.sum(d1**2, axis=1)
    denom = speed_sq**1.5
    denom = np.where(denom < 1e-12, 1e-12, denom)
    return cross / denom


# ─────────────────────────────────────────────
#  Control-point construction
# ─────────────────────────────────────────────

def heading_to_unit(deg):
    """Convert heading angle (deg, measured from +x CCW) to unit vector."""
    rad = np.deg2rad(deg)
    return np.array([np.cos(rad), np.sin(rad)])


def build_control_points(P0, heading0_deg, P3, heading3_deg, tangent_scale=1/3):
    """
    Place interior control points along the tangent directions.
    tangent_scale: fraction of chord length used as handle length.
    """
    chord = np.linalg.norm(P3 - P0)
    handle = tangent_scale * chord
    t0 = heading_to_unit(heading0_deg)
    t3 = heading_to_unit(heading3_deg)
    P1 = P0 + handle * t0
    P2 = P3 - handle * t3
    return P1, P2


def max_abs_curvature(P0, P1, P2, P3, n=500):
    t = np.linspace(0, 1, n)
    kappa = curvature(P0, P1, P2, P3, t)
    return np.max(np.abs(kappa))


def fit_with_curvature_constraint(P0, heading0_deg, P3, heading3_deg,
                                   kappa_max, tol=1e-4, max_iter=60):
    """
    Binary-search on tangent_scale so that |κ|_max ≤ kappa_max.
    Returns control points and whether the constraint is satisfied.
    """
    lo, hi = 0.01, 2.0          # search range for tangent_scale
    satisfied = False

    # Check if unconstrained solution already satisfies the limit
    P1, P2 = build_control_points(P0, heading0_deg, P3, heading3_deg, hi)
    if max_abs_curvature(P0, P1, P2, P3) <= kappa_max:
        satisfied = True
        return P1, P2, satisfied

    for _ in range(max_iter):
        mid = (lo + hi) / 2
        P1, P2 = build_control_points(P0, heading0_deg, P3, heading3_deg, mid)
        km = max_abs_curvature(P0, P1, P2, P3)
        if km <= kappa_max:
            lo = mid           # can increase scale (smoother)
        else:
            hi = mid           # must decrease scale

        if (hi - lo) < tol:
            break

    P1, P2 = build_control_points(P0, heading0_deg, P3, heading3_deg, lo)
    km_final = max_abs_curvature(P0, P1, P2, P3)
    satisfied = km_final <= kappa_max + 1e-6
    return P1, P2, satisfied


# ─────────────────────────────────────────────
#  Plotting
# ─────────────────────────────────────────────

def draw_arrow(ax, origin, direction_deg, length=0.6, color='k', lw=1.8, label=None):
    d = heading_to_unit(direction_deg) * length
    ax.annotate("", xy=origin + d, xytext=origin,
                arrowprops=dict(arrowstyle="->", color=color, lw=lw))
    if label:
        offset = heading_to_unit(direction_deg + 25) * (length * 0.6)
        ax.text(*(origin + d + offset), label, color=color,
                fontsize=9, ha='center', va='center')


def plot_bezier(P0, P1_free, P2_free, P3, heading0, heading3,
                P1_con, P2_con, kappa_max, n=400):

    t = np.linspace(0, 1, n)

    # Curves
    curve_free = cubic_bezier(P0, P1_free, P2_free, P3, t)
    curve_con  = cubic_bezier(P0, P1_con,  P2_con,  P3, t)

    kappa_free = curvature(P0, P1_free, P2_free, P3, t)
    kappa_con  = curvature(P0, P1_con,  P2_con,  P3, t)

    # ── Figure layout ──────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 8), facecolor='#0d1117')
    fig.suptitle("Bézier Path Planner  ·  Vehicle Curvature Constraint",
                 color='white', fontsize=15, fontweight='bold', y=0.97)
    gs = GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.35,
                  left=0.07, right=0.97, top=0.91, bottom=0.08)

    ax_path  = fig.add_subplot(gs[:, 0])   # left – both curves
    ax_kfree = fig.add_subplot(gs[0, 1])   # top-right – free curvature
    ax_kcon  = fig.add_subplot(gs[1, 1])   # bot-right – constrained curvature

    bg = '#161b22'
    for ax in [ax_path, ax_kfree, ax_kcon]:
        ax.set_facecolor(bg)
        ax.tick_params(colors='#8b949e', labelsize=9)
        for spine in ax.spines.values():
            spine.set_edgecolor('#30363d')

    # ── Path plot ──────────────────────────────────────────────────
    ax_path.plot(*curve_free.T, color='#58a6ff', lw=2.5,
                 label='Unconstrained path', zorder=3)
    ax_path.plot(*curve_con.T,  color='#3fb950', lw=2.5,
                 ls='--', label='Constrained path', zorder=3)

    # Control polygons
    for pts, col in [([P0, P1_free, P2_free, P3], '#58a6ff'),
                     ([P0, P1_con,  P2_con,  P3], '#3fb950')]:
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        ax_path.plot(xs, ys, 'o--', color=col, alpha=0.35,
                     markersize=5, lw=1, zorder=2)

    # Endpoints
    ax_path.scatter(*P0, s=100, color='#f78166', zorder=5)
    ax_path.scatter(*P3, s=100, color='#f78166', marker='*', zorder=5)
    ax_path.text(*P0 + np.array([-0.15, 0.15]), 'Start',
                 color='#f78166', fontsize=9)
    ax_path.text(*P3 + np.array([0.05, 0.15]),  'End',
                 color='#f78166', fontsize=9)

    # Heading arrows
    draw_arrow(ax_path, P0, heading0, length=0.5, color='#ffa657',
               label=f'{heading0}°')
    draw_arrow(ax_path, P3, heading3, length=0.5, color='#ffa657',
               label=f'{heading3}°')

    ax_path.set_title("Planned Path", color='white', fontsize=12, pad=8)
    ax_path.set_xlabel("x (m)", color='#8b949e', fontsize=9)
    ax_path.set_ylabel("y (m)", color='#8b949e', fontsize=9)
    ax_path.set_aspect('equal')
    ax_path.legend(fontsize=9, facecolor='#21262d',
                   labelcolor='white', edgecolor='#30363d')
    ax_path.grid(True, color='#21262d', lw=0.6)

    # ── Curvature plots ────────────────────────────────────────────
    for ax, kappa, col, title, sub in [
        (ax_kfree, kappa_free, '#58a6ff', 'Curvature – Unconstrained', 'free'),
        (ax_kcon,  kappa_con,  '#3fb950', 'Curvature – Constrained',   'con'),
    ]:
        ax.plot(t, kappa, color=col, lw=2)
        ax.axhline( kappa_max, color='#f85149', lw=1.4, ls='--',
                    label=f'+κ_max = {kappa_max:.3f}')
        ax.axhline(-kappa_max, color='#f85149', lw=1.4, ls=':',
                    label=f'−κ_max = {kappa_max:.3f}')
        ax.axhline(0, color='#8b949e', lw=0.7)
        ax.fill_between(t, kappa, 0,
                        where=(np.abs(kappa) > kappa_max),
                        color='#f85149', alpha=0.25, label='Violation zone')
        ax.set_title(title, color='white', fontsize=10, pad=6)
        ax.set_xlabel("t (parameter)", color='#8b949e', fontsize=8)
        ax.set_ylabel("κ  (1/m)", color='#8b949e', fontsize=8)
        ax.legend(fontsize=7.5, facecolor='#21262d',
                  labelcolor='white', edgecolor='#30363d')
        ax.grid(True, color='#21262d', lw=0.6)

    km_free = np.max(np.abs(kappa_free))
    km_con  = np.max(np.abs(kappa_con))
    status  = "✓  within limit" if km_con <= kappa_max + 1e-6 else "✗  still violates"
    info = (f"κ_max allowed : {kappa_max:.4f}  (R_min ≈ {1/kappa_max:.2f} m)\n"
            f"Unconstrained : {km_free:.4f}   |   Constrained : {km_con:.4f}  {status}")
    fig.text(0.5, 0.005, info, ha='center', va='bottom',
             color='#8b949e', fontsize=9,
             bbox=dict(facecolor='#21262d', edgecolor='#30363d',
                       boxstyle='round,pad=0.4'))

    plt.show()


# ─────────────────────────────────────────────
#  Main – edit parameters here
# ─────────────────────────────────────────────

if __name__ == "__main__":
    # ── User-defined inputs ──────────────────────────────────────
    P0 = np.array([0.0, 0.0])       # start point  (x, y)  in metres
    P3 = np.array([2.0, 0.0])       # end   point

    heading0_deg = 90.0             # start heading  (degrees from +x, CCW)
    heading3_deg = 270.0             # end   heading

    min_turning_radius = 5.0        # metres  →  kappa_max = 1/R
    kappa_max = 1.0 / min_turning_radius
    # ────────────────────────────────────────────────────────────

    print("=" * 55)
    print("  Bézier Path Planner  –  Curvature-Constrained")
    print("=" * 55)
    print(f"  Start : {P0}   heading {heading0_deg}°")
    print(f"  End   : {P3}   heading {heading3_deg}°")
    print(f"  R_min : {min_turning_radius} m   →   κ_max = {kappa_max:.4f} 1/m")

    # --- Unconstrained (default tangent_scale = 1/3) ---
    P1_free, P2_free = build_control_points(P0, heading0_deg, P3, heading3_deg)
    km_free = max_abs_curvature(P0, P1_free, P2_free, P3)
    print(f"\n  Unconstrained |κ|_max = {km_free:.4f}")

    # --- Constrained ---
    P1_con, P2_con, ok = fit_with_curvature_constraint(
        P0, heading0_deg, P3, heading3_deg, kappa_max
    )
    km_con = max_abs_curvature(P0, P1_con, P2_con, P3)
    print(f"  Constrained   |κ|_max = {km_con:.4f}  →  {'OK ✓' if ok else 'Not satisfiable with this geometry'}")
    print("=" * 55)

    plot_bezier(P0, P1_free, P2_free, P3, heading0_deg, heading3_deg,
                P1_con, P2_con, kappa_max)
