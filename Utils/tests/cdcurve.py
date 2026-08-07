import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path


# ============================================================
# Utility functions
# ============================================================

def mod2pi(theta):
    return theta - 2.0 * np.pi * np.floor(theta / (2.0 * np.pi))


def polar(x, y):
    return np.hypot(x, y), np.arctan2(y, x)


def rot_mat(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]])


# ============================================================
# Dubins path formulas
# Each returns normalized segment lengths t, p, q
# Actual lengths are multiplied by turning radius R
# ============================================================

def dubins_LSL(alpha, beta, d):
    tmp0 = d + np.sin(alpha) - np.sin(beta)
    p_sq = 2 + d**2 - 2*np.cos(alpha - beta) + 2*d*(np.sin(alpha) - np.sin(beta))

    if p_sq < 0:
        return None

    p = np.sqrt(p_sq)
    tmp1 = np.arctan2(np.cos(beta) - np.cos(alpha), tmp0)

    t = mod2pi(-alpha + tmp1)
    q = mod2pi(beta - tmp1)

    return t, p, q


def dubins_RSR(alpha, beta, d):
    tmp0 = d - np.sin(alpha) + np.sin(beta)
    p_sq = 2 + d**2 - 2*np.cos(alpha - beta) + 2*d*(-np.sin(alpha) + np.sin(beta))

    if p_sq < 0:
        return None

    p = np.sqrt(p_sq)
    tmp1 = np.arctan2(np.cos(alpha) - np.cos(beta), tmp0)

    t = mod2pi(alpha - tmp1)
    q = mod2pi(-beta + tmp1)

    return t, p, q


def dubins_LSR(alpha, beta, d):
    p_sq = -2 + d**2 + 2*np.cos(alpha - beta) + 2*d*(np.sin(alpha) + np.sin(beta))

    if p_sq < 0:
        return None

    p = np.sqrt(p_sq)
    tmp0 = np.arctan2(-np.cos(alpha) - np.cos(beta),
                      d + np.sin(alpha) + np.sin(beta)) - np.arctan2(-2.0, p)

    t = mod2pi(-alpha + tmp0)
    q = mod2pi(-mod2pi(beta) + tmp0)

    return t, p, q


def dubins_RSL(alpha, beta, d):
    p_sq = -2 + d**2 + 2*np.cos(alpha - beta) - 2*d*(np.sin(alpha) + np.sin(beta))

    if p_sq < 0:
        return None

    p = np.sqrt(p_sq)
    tmp0 = np.arctan2(np.cos(alpha) + np.cos(beta),
                      d - np.sin(alpha) - np.sin(beta)) - np.arctan2(2.0, p)

    t = mod2pi(alpha - tmp0)
    q = mod2pi(beta - tmp0)

    return t, p, q


def dubins_RLR(alpha, beta, d):
    tmp0 = (6.0 - d**2 + 2*np.cos(alpha - beta) +
            2*d*(np.sin(alpha) - np.sin(beta))) / 8.0

    if abs(tmp0) > 1:
        return None

    p = mod2pi(2*np.pi - np.arccos(tmp0))
    t = mod2pi(alpha - np.arctan2(np.cos(alpha) - np.cos(beta),
                                  d - np.sin(alpha) + np.sin(beta)) + p / 2.0)
    q = mod2pi(alpha - beta - t + p)

    return t, p, q


def dubins_LRL(alpha, beta, d):
    tmp0 = (6.0 - d**2 + 2*np.cos(alpha - beta) +
            2*d*(-np.sin(alpha) + np.sin(beta))) / 8.0

    if abs(tmp0) > 1:
        return None

    p = mod2pi(2*np.pi - np.arccos(tmp0))
    t = mod2pi(-alpha - np.arctan2(np.cos(alpha) - np.cos(beta),
                                   d + np.sin(alpha) - np.sin(beta)) + p / 2.0)
    q = mod2pi(beta - alpha - t + p)

    return t, p, q


DUBINS_WORDS = {
    "LSL": dubins_LSL,
    "RSR": dubins_RSR,
    "LSR": dubins_LSR,
    "RSL": dubins_RSL,
    "RLR": dubins_RLR,
    "LRL": dubins_LRL,
}


# ============================================================
# Dubins path solver
# ============================================================

def compute_dubins_candidates(start, goal, radius):
    """
    start = (x, y, yaw_rad)
    goal  = (x, y, yaw_rad)
    radius = minimum turning radius
    """

    sx, sy, syaw = start
    gx, gy, gyaw = goal

    dx = gx - sx
    dy = gy - sy

    # Transform goal into start coordinate frame
    local = rot_mat(-syaw) @ np.array([dx, dy])
    local_x, local_y = local

    D = np.hypot(local_x, local_y)
    d = D / radius

    theta = mod2pi(np.arctan2(local_y, local_x))
    alpha = mod2pi(-theta)
    beta = mod2pi(gyaw - syaw - theta)

    candidates = []

    for word, fn in DUBINS_WORDS.items():
        result = fn(alpha, beta, d)

        if result is None:
            continue

        t, p, q = result
        normalized_length = t + p + q
        actual_length = normalized_length * radius

        candidates.append({
            "word": word,
            "params": (t, p, q),
            "length": actual_length,
            "start": start,
            "radius": radius,
        })

    return candidates


# ============================================================
# Path sampling
# ============================================================

def sample_segment(x, y, yaw, segment_type, segment_length, radius, step_size):
    """
    segment_type: 'L', 'R', or 'S'
    segment_length is actual length in meters.
    """

    points = []

    travelled = 0.0

    while travelled < segment_length:
        ds = min(step_size, segment_length - travelled)

        if segment_type == "S":
            x += ds * np.cos(yaw)
            y += ds * np.sin(yaw)

        else:
            direction = 1.0 if segment_type == "L" else -1.0
            dtheta = direction * ds / radius

            # Exact integration for circular arc
            x += radius / direction * (np.sin(yaw + dtheta) - np.sin(yaw))
            y += -radius / direction * (np.cos(yaw + dtheta) - np.cos(yaw))
            yaw += dtheta
            yaw = mod2pi(yaw)

        travelled += ds
        points.append((x, y, yaw))

    return x, y, yaw, points


def sample_dubins_path(candidate, step_size=0.1):
    word = candidate["word"]
    t, p, q = candidate["params"]
    radius = candidate["radius"]

    x, y, yaw = candidate["start"]

    lengths = [t * radius, p * radius, q * radius]

    all_points = [(x, y, yaw)]

    for seg_type, seg_len in zip(word, lengths):
        x, y, yaw, pts = sample_segment(
            x, y, yaw,
            segment_type=seg_type,
            segment_length=seg_len,
            radius=radius,
            step_size=step_size
        )
        all_points.extend(pts)

    return np.array(all_points)


# ============================================================
# Headland / field constraint checking
# ============================================================

def points_inside_polygon(points_xy, polygon_xy):
    """
    Returns True if all path points are inside polygon.
    """

    poly_path = Path(polygon_xy)
    return np.all(poly_path.contains_points(points_xy))


def path_stays_in_headland(path_points, headland_polygon=None, headland_y_min=None):
    """
    You can use either:
    1. headland_polygon: polygon describing the allowed headland area
    2. headland_y_min: simple horizontal boundary.
       Example: field is y < 0, headland is y >= 0.
    """

    xy = path_points[:, :2]

    if headland_polygon is not None:
        return points_inside_polygon(xy, headland_polygon)

    if headland_y_min is not None:
        return np.all(xy[:, 1] >= headland_y_min)

    return True


# ============================================================
# Main constrained Dubins planner
# ============================================================

def plan_dubins_with_headland_constraint(
    start,
    goal,
    radius,
    step_size=0.1,
    headland_polygon=None,
    headland_y_min=None,
):
    candidates = compute_dubins_candidates(start, goal, radius)

    feasible = []
    rejected = []

    for candidate in candidates:
        path = sample_dubins_path(candidate, step_size=step_size)

        allowed = path_stays_in_headland(
            path,
            headland_polygon=headland_polygon,
            headland_y_min=headland_y_min
        )

        candidate["path"] = path
        candidate["allowed"] = allowed

        if allowed:
            feasible.append(candidate)
        else:
            rejected.append(candidate)

    if len(feasible) == 0:
        return None, feasible, rejected

    best = min(feasible, key=lambda c: c["length"])

    return best, feasible, rejected


# ============================================================
# Plotting
# ============================================================

def plot_result(
    start,
    goal,
    best,
    feasible,
    rejected,
    headland_polygon=None,
    headland_y_min=None,
    show_rejected=True
):
    fig, ax = plt.subplots(figsize=(10, 7))

    # Plot simple headland boundary
    if headland_y_min is not None:
        ax.axhline(headland_y_min, color="black", linestyle="--", linewidth=2)
        ax.text(
            start[0],
            headland_y_min,
            " Field boundary / headland edge",
            verticalalignment="bottom"
        )

        # Shade field side
        xmin = min(start[0], goal[0]) - 20
        xmax = max(start[0], goal[0]) + 20
        ymin = headland_y_min - 20
        ymax = headland_y_min

        ax.fill_between(
            [xmin, xmax],
            ymin,
            ymax,
            color="red",
            alpha=0.15,
            label="Forbidden field area"
        )

    # Plot polygon headland if provided
    if headland_polygon is not None:
        poly = np.array(headland_polygon)
        ax.fill(
            poly[:, 0],
            poly[:, 1],
            color="lightgreen",
            alpha=0.25,
            label="Allowed headland area"
        )
        ax.plot(
            np.r_[poly[:, 0], poly[0, 0]],
            np.r_[poly[:, 1], poly[0, 1]],
            color="green",
            linewidth=2
        )

    # Plot rejected paths
    if show_rejected:
        for c in rejected:
            path = c["path"]
            ax.plot(
                path[:, 0],
                path[:, 1],
                color="gray",
                linestyle=":",
                linewidth=1,
                alpha=0.7
            )
            mid = len(path) // 2
            ax.text(path[mid, 0], path[mid, 1], c["word"], color="gray")

    # Plot feasible paths lightly
    for c in feasible:
        path = c["path"]
        ax.plot(
            path[:, 0],
            path[:, 1],
            color="blue",
            linestyle="--",
            linewidth=1,
            alpha=0.35
        )

    # Plot best path
    if best is not None:
        path = best["path"]
        ax.plot(
            path[:, 0],
            path[:, 1],
            color="blue",
            linewidth=3,
            label=f"Best Dubins path: {best['word']}, length={best['length']:.2f} m"
        )

    # Start and goal
    ax.scatter(start[0], start[1], color="green", s=100, label="Start")
    ax.scatter(goal[0], goal[1], color="red", s=100, label="Goal")

    # Direction arrows
    arrow_len = 3.0
    ax.arrow(
        start[0],
        start[1],
        arrow_len * np.cos(start[2]),
        arrow_len * np.sin(start[2]),
        head_width=0.5,
        color="green",
        length_includes_head=True
    )

    ax.arrow(
        goal[0],
        goal[1],
        arrow_len * np.cos(goal[2]),
        arrow_len * np.sin(goal[2]),
        head_width=0.5,
        color="red",
        length_includes_head=True
    )

    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")
    ax.set_title("Dubins turn constrained to headland area")

    plt.show()


# ============================================================
# Example usage
# ============================================================

if __name__ == "__main__":

    # --------------------------------------------------------
    # User inputs
    # --------------------------------------------------------
    start_x = 0.0
    start_y = 5.0
    start_heading_deg = 0.0

    goal_x = 20.0
    goal_y = 5.0
    goal_heading_deg = 180.0

    min_turning_radius = 4.0

    start = (
        start_x,
        start_y,
        np.deg2rad(start_heading_deg)
    )

    goal = (
        goal_x,
        goal_y,
        np.deg2rad(goal_heading_deg)
    )

    # --------------------------------------------------------
    # Headland constraint option 1:
    # Simple boundary.
    #
    # Example:
    # field is y < 0
    # headland is y >= 0
    # So path must never go below y = 0
    # --------------------------------------------------------
    headland_y_min = 0.0

    # --------------------------------------------------------
    # Headland constraint option 2:
    # Polygon describing allowed headland region.
    #
    # Use this if your headland is not a simple straight strip.
    # Uncomment this and set headland_y_min = None.
    # --------------------------------------------------------
    headland_polygon = None

    # Example polygon:
    # headland_polygon = [
    #     (-10, 0),
    #     (35, 0),
    #     (35, 15),
    #     (-10, 15)
    # ]
    # headland_y_min = None

    # --------------------------------------------------------
    # Plan
    # --------------------------------------------------------
    best, feasible, rejected = plan_dubins_with_headland_constraint(
        start=start,
        goal=goal,
        radius=min_turning_radius,
        step_size=0.1,
        headland_polygon=headland_polygon,
        headland_y_min=headland_y_min
    )

    # --------------------------------------------------------
    # Result
    # --------------------------------------------------------
    if best is None:
        print("No feasible Dubins path found that stays inside the headland.")
        print("Try increasing headland width or reducing minimum turning radius.")
    else:
        print("Best path type:", best["word"])
        print("Path length:", best["length"])
        print("Segment parameters t, p, q:", best["params"])
        print("Number of feasible paths:", len(feasible))
        print("Number of rejected paths:", len(rejected))

    # --------------------------------------------------------
    # Plot
    # --------------------------------------------------------
    plot_result(
        start=start,
        goal=goal,
        best=best,
        feasible=feasible,
        rejected=rejected,
        headland_polygon=headland_polygon,
        headland_y_min=headland_y_min,
        show_rejected=True
    )
