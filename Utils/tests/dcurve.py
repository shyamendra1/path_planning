import numpy as np

import matplotlib.pyplot as plt

def wrap2pi(a):   return a % (2 * np.pi)             # [0, 2π)
def wrappi(a):    return (a + np.pi) % (2*np.pi) - np.pi  # (−π, π]



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
# Plot
# ═══════════════════════════════════════════════════════════════

DARK='#0d1117'; PANEL='#161b22'; GRID='#21262d'; SPINE='#30363d'
DIM='#8b949e'; BLUE='#58a6ff'; GREEN='#3fb950'
ORANGE='#ffa657'; RED='#f85149'; PURPLE='#bc8cff'

def draw_arc(c, R, ang_start, ang_end, turn, n=100):
    """Generate arc points."""
    if turn > 0:  # CCW
        if ang_end < ang_start:
            ang_end += 2*np.pi
        ang = np.linspace(ang_start, ang_end, n)
    else:        # CW
        if ang_end > ang_start:
            ang_end -= 2*np.pi
        ang = np.linspace(ang_start, ang_end, n)

    x = c[0] + R*np.cos(ang)
    y = c[1] + R*np.sin(ang)
    return x, y


def plot_result(sol):
    R = sol['R_min']
    p0, p1 = sol['p0'], sol['p1']

    plt.figure(figsize=(10, 8))
    
    # ---- First arc ----
    x0, y0 = draw_arc(
        sol['c0'], R,
        sol['ang0_start'], sol['ang0_end'],
        sol['turn0']
    )
    plt.plot(x0, y0, color=BLUE, linewidth=2, label='Arc 1')

    # ---- Straight segment ----
    if sol['d'] > 1e-6:
        xs = [sol['T0'][0], sol['T1'][0]]
        ys = [sol['T0'][1], sol['T1'][1]]
        plt.plot(xs, ys, color=GREEN, linewidth=2, label='Straight')

    # ---- Second arc ----
    x1, y1 = draw_arc(
        sol['c1'], R,
        sol['ang1_start'], sol['ang1_end'],
        sol['turn1']
    )
    plt.plot(x1, y1, color=PURPLE, linewidth=2, label='Arc 2')

    # ---- Tangent points ----
    if sol['d'] > 0.1:
        plt.scatter(*sol['T0'], color=ORANGE, s=60, marker='D', label='Tangent pts')
        plt.scatter(*sol['T1'], color=ORANGE, s=60, marker='D')

    # ---- Start & End ----
    plt.scatter(*p0, s=100, color=GREEN, marker='o', label='Start')
    plt.scatter(*p1, s=100, color=RED, marker='X', label='End')

    plt.text(p0[0], p0[1], ' Start', color=GREEN)
    plt.text(p1[0], p1[1], ' End', color=RED)

    plt.title(f"Dubins Path ({sol['word']})  |  Length = {sol['L']:.2f}")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

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

    R_min    = 6.0                      # minimum turning radius (m)

    L_trans  = None   # clothoid transition length (m), None = auto-sized
    # ─────────────────────────────────────────────────────────────

    print("=" * 65)
    print("  Headland Turn Planner  —  Agricultural Vehicle")
    print("=" * 65)
    print(f"  Start : {p0}  heading {t0_deg}°")
    print(f"  End   : {p1}  heading {t1_deg}°")
    print(f"  R_min : {R_min} m  (κ_max = {1/R_min:.4f} 1/m)")

    sol    = solve_dubins(p0, t0_deg, p1, t1_deg, R_min)

    print("=" * 65)

    plot_result(sol)
