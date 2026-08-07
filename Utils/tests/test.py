import numpy as np

R = 6371000  # Earth radius (m)

def to_rad(p):
    return np.radians(p[0]), np.radians(p[1])

def to_deg(lat, lon):
    return np.degrees(lat), np.degrees(lon)

def bearing(p1, p2):
    lat1, lon1 = to_rad(p1)
    lat2, lon2 = to_rad(p2)

    dlon = lon2 - lon1

    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(dlon)

    return np.arctan2(x, y)

def angular_distance(p1, p2):
    lat1, lon1 = to_rad(p1)
    lat2, lon2 = to_rad(p2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = np.sin(dlat/2)**2 + np.cos(lat1)*np.cos(lat2)*np.sin(dlon/2)**2
    return 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))

def destination(p, brng, dist):
    lat1, lon1 = to_rad(p)
    δ = dist / R

    lat2 = np.arcsin(np.sin(lat1)*np.cos(δ) +
                     np.cos(lat1)*np.sin(δ)*np.cos(brng))

    lon2 = lon1 + np.arctan2(
        np.sin(brng)*np.sin(δ)*np.cos(lat1),
        np.cos(δ) - np.sin(lat1)*np.sin(lat2)
    )

    return to_deg(lat2, lon2)

def intersection_ray_segment(P, bearing_deg, A, B):
    θ13 = np.radians(bearing_deg)
    θ12 = bearing(A, B)
    θ21 = bearing(B, A)

    δ13 = angular_distance(A, P)

    θ13_from_A = bearing(A, P)

    # angle between paths
    α1 = (θ13_from_A - θ12 + np.pi) % (2*np.pi) - np.pi
    α2 = (θ21 - θ13 + np.pi) % (2*np.pi) - np.pi

    if np.sin(α1) == 0 and np.sin(α2) == 0:
        return None  # collinear
    if np.sin(α1)*np.sin(α2) < 0:
        return None  # ambiguous

    α3 = np.arccos(-np.cos(α1)*np.cos(α2) +
                   np.sin(α1)*np.sin(α2)*np.cos(δ13))

    δ13_intersect = np.arctan2(
        np.sin(δ13)*np.sin(α1)*np.sin(α2),
        np.cos(α2) + np.cos(α1)*np.cos(α3)
    )

    # intersection point from A
    lat_i, lon_i = destination(A, θ12, δ13_intersect * R)

    # --- VALIDATION ---

    # 1. Check if on segment AB
    d_AB = angular_distance(A, B)
    d_Ai = angular_distance(A, (lat_i, lon_i))
    d_iB = angular_distance((lat_i, lon_i), B)

    if abs((d_Ai + d_iB) - d_AB) > 1e-6:
        return None

    # 2. Check if forward along ray
    θ_Pi = bearing(P, (lat_i, lon_i))
    if abs((θ_Pi - θ13 + np.pi) % (2*np.pi) - np.pi) > np.radians(5):
        return None

    return lat_i, lon_i
    
P = (28.12222, 77.3456)
bearing_deg = 23

A = (28.13550, 77.33420)
B = (28.12880, 77.35790)

print(intersection_ray_segment(P, bearing_deg, A, B))
