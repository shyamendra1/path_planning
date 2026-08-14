import matplotlib.pyplot as plt
import numpy as np
import json
from shapely.geometry import Polygon, LineString, box
from shapely.ops import split
from path_planning.Utils.helper.geodesy import Geodesy


farm_name = "C7"

with open("path_planning/data/farms.json", "r") as f:
    farms = json.load(f)

farm = next((farm["boundary"] for farm in farms
        if farm["name"] == farm_name), None
)

origin = farm[0]
xy_points = []

for point in farm:
    distance = Geodesy.distancebet(origin,point)

    bearing,_ = Geodesy.angle(origin,point)

    bearing_rad = np.radians(bearing)

    x = distance * np.sin(bearing_rad)
    y = distance * np.cos(bearing_rad)

    xy_points.append((x,y))

field = Polygon(xy_points)

print("Polygon area:", round(field.area/10000, 2), "ha")

#BOUSTROPHEDON DECOMPOSITION

from shapely.geometry import (
    Polygon,
    LineString,
    MultiPolygon,
    GeometryCollection,
    box,
)
import numpy as np


def boustrophedon_decomposition(polygon, dx=None, min_cell_area=1.0):

    # Fix invalid polygons
    if not polygon.is_valid:
        polygon = polygon.buffer(0)

    minx, miny, maxx, maxy = polygon.bounds

    width = maxx - minx

    # Automatic resolution
    if dx is None:
        dx = max(width / 500.0, 0.25)

    # Avoid sampling exactly on vertices
    x_values = np.arange(
        minx + dx / 2.0,
        maxx,
        dx
    )

    connectivity = []

    for x in x_values:

        sweep = LineString([
            (x, miny - 1000),
            (x, maxy + 1000)
        ])

        inter = polygon.intersection(sweep)

        if inter.is_empty:
            count = 0

        elif inter.geom_type == "LineString":
            count = 1

        elif inter.geom_type == "MultiLineString":
            count = len(inter.geoms)

        elif inter.geom_type == "GeometryCollection":

            count = sum(
                1
                for g in inter.geoms
                if g.geom_type == "LineString"
            )

        else:
            count = 0

        connectivity.append(count)

    if len(connectivity) == 0:
        return [polygon]

    critical_indices = [0]

    for i in range(1, len(connectivity)):

        if connectivity[i] != connectivity[i - 1]:
            critical_indices.append(i)

    critical_indices.append(len(x_values) - 1)

    critical_indices = sorted(set(critical_indices))

    cells = []

    for i in range(len(critical_indices) - 1):

        left_idx = critical_indices[i]
        right_idx = critical_indices[i + 1]

        x_left = x_values[left_idx]
        x_right = x_values[right_idx]

        if (x_right - x_left) < dx:
            continue

        strip = box(
            x_left,
            miny - 1000,
            x_right,
            maxy + 1000
        )

        piece = polygon.intersection(strip)

        if piece.is_empty:
            continue

        if isinstance(piece, Polygon):

            if piece.area > min_cell_area:
                cells.append(piece.buffer(0))

        elif isinstance(piece, MultiPolygon):

            for p in piece.geoms:

                if p.area > min_cell_area:
                    cells.append(p.buffer(0))

        elif isinstance(piece, GeometryCollection):

            for g in piece.geoms:

                if (
                    isinstance(g, Polygon)
                    and g.area > min_cell_area
                ):
                    cells.append(g.buffer(0))

    # Remove tiny duplicates
    cleaned = []

    for c in cells:

        if c.area < min_cell_area:
            continue

        duplicate = False

        for existing in cleaned:

            if c.equals(existing):
                duplicate = True
                break

        if not duplicate:
            cleaned.append(c)

    return cleaned



#FINAL DECOMPOSITION

cells = boustrophedon_decomposition(
    field,
    dx=None, 
    min_cell_area=30   # m²
)

print("Final Cell Count:", len(cells))


#DECOMPOSITION RESULT

fig, ax = plt.subplots(figsize=(7, 6))

colors = [
    "#ff9999",
    "#99ff99",
    "#9999ff",
    "#ffd699",
    "#99ffff",
    "#ffccff",
    "#c2f0c2",
    "#f0c2c2"
]

for idx, cell in enumerate(cells):

    x, y = cell.exterior.xy

    ax.fill(
        x,
        y,
        color=colors[idx % len(colors)],
        alpha=0.6
    )

    ax.plot(
        x,
        y,
        color="black",
        linewidth=1.5
    )

    centroid = cell.centroid

    ax.text(
        centroid.x,
        centroid.y,
        f"C{idx}",
        fontsize=11,
        fontweight="bold",
        ha="center"
    )

#Original Boundary

bx, by = field.exterior.xy

ax.plot(
    bx,
    by,
    color="black",
    linewidth=2.5,
    label="Field Boundary"
)

ax.scatter(
    bx,
    by,
    color="red"
)

ax.set_title(
    f"Boustrophedon Decomposition, Farm {farm_name}\n"
)

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

ax.set_aspect("equal")
ax.grid(True)

ax.legend()

plt.tight_layout()
plt.show()