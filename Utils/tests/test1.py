import numpy as np
import matplotlib.pyplot as plt
R = 6371000  # Earth radius (m)

def to_rad(p):
    return np.radians(p[0]), np.radians(p[1])

def to_deg(lat, lon):
    return np.degrees(lat), np.degrees(lon)

# Compute initial bearing from p1 → p2
def bearing(p1, p2):
    lat1, lon1 = to_rad(p1)
    lat2, lon2 = to_rad(p2)

    dlon = lon2 - lon1

    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(dlon)

    return (np.degrees(np.arctan2(x, y)) + 360) % 360


# Angular distance between two points
def angular_distance(p1, p2):
    lat1, lon1 = to_rad(p1)
    lat2, lon2 = to_rad(p2)

    return np.arccos(
        np.sin(lat1)*np.sin(lat2) +
        np.cos(lat1)*np.cos(lat2)*np.cos(lon2 - lon1)
    )


# Check if intersection lies on segment
def is_between(p, a, b):
    d_ab = angular_distance(a, b)
    d_ap = angular_distance(a, p)
    d_pb = angular_distance(p, b)

    return abs((d_ap + d_pb) - d_ab) < 1e-6


# Compute intersection of two great circles
def great_circle_intersection(p1, brng1, p2, brng2):
    lat1, lon1 = to_rad(p1)
    lat2, lon2 = to_rad(p2)

    brng1 = np.radians(brng1)
    brng2 = np.radians(brng2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    dist12 = 2 * np.arcsin(np.sqrt(
        np.sin(dlat/2)**2 +
        np.cos(lat1)*np.cos(lat2)*np.sin(dlon/2)**2
    ))

    if dist12 == 0:
        return None

    # bearings between points
    brngA = np.arccos(
        (np.sin(lat2) - np.sin(lat1)*np.cos(dist12)) /
        (np.sin(dist12)*np.cos(lat1))
    )

    if np.sin(lon2 - lon1) > 0:
        brng12 = brngA
        brng21 = 2*np.pi - brngA
    else:
        brng12 = 2*np.pi - brngA
        brng21 = brngA

    alpha1 = (brng1 - brng12 + np.pi) % (2*np.pi) - np.pi
    alpha2 = (brng21 - brng2 + np.pi) % (2*np.pi) - np.pi

    if np.sin(alpha1) == 0 and np.sin(alpha2) == 0:
        return None

    if np.sin(alpha1) * np.sin(alpha2) < 0:
        return None

    alpha3 = np.arccos(
        -np.cos(alpha1)*np.cos(alpha2) +
        np.sin(alpha1)*np.sin(alpha2)*np.cos(dist12)
    )

    dist13 = np.arctan2(
        np.sin(dist12)*np.sin(alpha1)*np.sin(alpha2),
        np.cos(alpha2) + np.cos(alpha1)*np.cos(alpha3)
    )

    lat3 = np.arcsin(
        np.sin(lat1)*np.cos(dist13) +
        np.cos(lat1)*np.sin(dist13)*np.cos(brng1)
    )

    dlon13 = np.arctan2(
        np.sin(brng1)*np.sin(dist13)*np.cos(lat1),
        np.cos(dist13) - np.sin(lat1)*np.sin(lat3)
    )

    lon3 = lon1 + dlon13
    lon3 = (lon3 + 3*np.pi) % (2*np.pi) - np.pi

    return to_deg(lat3, lon3)


# Main: point in polygon (geodesic ray casting)
def point_in_polygon_geo(point, polygon):
    ray_bearing = 90  # East
    count = 0

    for i in range(len(polygon)):
        a = polygon[i]
        b = polygon[(i+1) % len(polygon)]

        edge_bearing = bearing(a, b)

        intersect = great_circle_intersection(point, ray_bearing, a, edge_bearing)
        print(intersect)
        if intersect is None:
            continue

        # Check if intersection lies on edge
        if not is_between(intersect, a, b):
            continue

        # Check if intersection is in forward ray direction
        if angular_distance(point, intersect) < 1e-6:
            continue

        count += 1

    return count % 2 == 1
    
point=[
             28.431914497944177,
            77.33570527529365
        ]
polygon=[
            [
                28.43175588303852,
                77.33568996191026
            ],
            [
                28.432053078053446,
                77.33571946620943
            ],
        
            [
                28.432048360678788,
                77.33619153499605
            ],
            [
                28.431777111281544,
                77.33621567487717
            ]
       
    ]

inside = point_in_polygon_geo(point,polygon)
print(inside)


lats = [p[0] for p in polygon] + [polygon[0][0]]
lons = [p[1] for p in polygon] + [polygon[0][1]]

plt.figure()

# Boundary
plt.plot(lons, lats, marker='o', label="Boundary")

# Point
color = 'green' if inside else 'red'
plt.scatter(point[1], point[0], color=color, s=100, label="Test Point")

# Labels
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title(f"Point is {'INSIDE' if inside else 'OUTSIDE'} polygon")

plt.legend()
plt.grid()

plt.show()

























