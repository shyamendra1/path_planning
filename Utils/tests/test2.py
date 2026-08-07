import numpy as np
import matplotlib.pyplot as plt

# -------------------------------
# Core Math
# -------------------------------

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

    return (np.degrees(np.arctan2(x, y)) + 360) % 360

def angular_distance(p1, p2):
    lat1, lon1 = to_rad(p1)
    lat2, lon2 = to_rad(p2)

    return np.arccos(
        np.sin(lat1)*np.sin(lat2) +
        np.cos(lat1)*np.cos(lat2)*np.cos(lon2 - lon1)
    )

def is_between(p, a, b):
    return abs(
        angular_distance(a, p) +
        angular_distance(p, b) -
        angular_distance(a, b)
    ) < 1e-6


# -------------------------------
# Great Circle Intersection
# -------------------------------

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


# -------------------------------
# Main Function with intersections
# -------------------------------

def point_in_polygon_geo_with_intersections(point, polygon):
    ray_bearing = 120
    intersections = []
    points=[]
    for i in range(len(polygon)):
        a = polygon[i]
        b = polygon[(i+1) % len(polygon)]

        edge_bearing = bearing(a, b)
        intersect = great_circle_intersection(point, ray_bearing, a, edge_bearing)
        print(intersect)
        if intersect is not None:
            points.append(intersect)
        if intersect is None:
            continue

        if not is_between(intersect, a, b):
            continue

        if angular_distance(point, intersect) < 1e-6:
            continue
        
        intersections.append(intersect)

    inside = len(intersections) % 2 == 1
    return inside, points


# -------------------------------
# Example Data
# -------------------------------

test_point=[
           28.431890492069094,
            77.3361605154835
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


# -------------------------------
# Run
# -------------------------------

inside, intersections = point_in_polygon_geo_with_intersections(test_point, polygon)
print(inside)

# -------------------------------
# Plot
# -------------------------------

plt.figure()

# Polygon
lats = [p[0] for p in polygon] + [polygon[0][0]]
lons = [p[1] for p in polygon] + [polygon[0][1]]
plt.plot(lons, lats, marker='o', label="Boundary")

# Test Point
color = 'green' if inside else 'red'
plt.scatter(test_point[1], test_point[0], color=color, s=100, label="Test Point")

# Plot intersections
'''if intersections:
    inter_lats = [p[0] for p in intersections]
    inter_lons = [p[1] for p in intersections]
    plt.scatter(inter_lons, inter_lats, color='blue', s=80, label="Intersections")'''


# Labels
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title(f"Point is {'INSIDE' if inside else 'OUTSIDE'} polygon")

plt.legend()
plt.grid()
plt.show()
