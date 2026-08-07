import numpy as np
from math import radians, degrees, sin, cos, asin, atan2, sqrt

class GeodesicDubins:

    EARTH_RADIUS = 6378100

    # ===================== BASIC GEODESY =====================

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        return GeodesicDubins.EARTH_RADIUS * c

    @staticmethod
    def bearing(p1, p2):
        lat1, lon1 = map(radians, p1)
        lat2, lon2 = map(radians, p2)

        dlon = lon2 - lon1
        X = cos(lat2)*sin(dlon)
        Y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon)

        brg = degrees(atan2(X, Y))
        return (brg + 360) % 360

    @staticmethod
    def forward(p, dist, bearing):
        lat1 = radians(p[0])
        lon1 = radians(p[1])
        brg = radians(bearing)

        ang = dist / GeodesicDubins.EARTH_RADIUS

        lat2 = asin(
            sin(lat1)*cos(ang) +
            cos(lat1)*sin(ang)*cos(brg)
        )

        lon2 = lon1 + atan2(
            sin(brg)*sin(ang)*cos(lat1),
            cos(ang) - sin(lat1)*sin(lat2)
        )

        return [degrees(lat2), degrees(lon2)]

    # ===================== GEOMETRY =====================

    @staticmethod
    def normalize(a):
        return (a + 360) % 360

    @staticmethod
    def angle_diff(a, b):
        return ((b - a + 180) % 360) - 180

    # ===================== CIRCLE =====================

    @staticmethod
    def circle_center(p, heading, R, turn):
        # turn: +1 left, -1 right
        bearing = GeodesicDubins.normalize(heading + 90*turn)
        return GeodesicDubins.forward(p, R, bearing)

    # ===================== ARC (HIGH ACCURACY) =====================

    @staticmethod
    def simulate_arc(start, heading, center, R, turn, target_bearing, step=0.5):
        pts = [start]
        current = start
        h = heading

        for _ in range(5000):
            h += (turn * step / R) * (180/np.pi)
            current = GeodesicDubins.forward(current, step, h)
            pts.append(current)

            # stopping condition based on direction to tangent
            b = GeodesicDubins.bearing(center, current)
            if abs(GeodesicDubins.angle_diff(b, target_bearing)) < 1:
                break

        return pts, h, current

    # ===================== STRAIGHT =====================

    @staticmethod
    def simulate_straight(start, heading, end, step=1.0):
        pts = [start]
        current = start

        total_d = GeodesicDubins.haversine(start[0], start[1], end[0], end[1])
        steps = int(total_d / step)

        for _ in range(steps):
            current = GeodesicDubins.forward(current, step, heading)
            pts.append(current)

        return pts, current

    # ===================== HIGH ACCURACY TANGENT (BINARY SEARCH) =====================

    @staticmethod
    def find_tangent(c0, c1, R):
        best_bearing = None
        best_err = 1e9

        # coarse search
        for b in np.linspace(0, 360, 72):
            p = GeodesicDubins.forward(c0, R, b)
            d = GeodesicDubins.haversine(p[0], p[1], c1[0], c1[1])
            err = abs(d - R)

            if err < best_err:
                best_err = err
                best_bearing = b

        # refine search (binary)
        low = best_bearing - 5
        high = best_bearing + 5

        for _ in range(20):
            mid1 = (2*low + high)/3
            mid2 = (low + 2*high)/3

            def error(b):
                p = GeodesicDubins.forward(c0, R, b)
                d = GeodesicDubins.haversine(p[0], p[1], c1[0], c1[1])
                return abs(d - R)

            if error(mid1) < error(mid2):
                high = mid2
            else:
                low = mid1

        best_bearing = (low + high)/2

        T0 = GeodesicDubins.forward(c0, R, best_bearing)
        T1 = GeodesicDubins.forward(c1, R, best_bearing)

        return best_bearing, T0, T1

    # ===================== MAIN SOLVER =====================

    @staticmethod
    def solve(p0, h0, p1, h1, R):

        configs = [
            (+1,+1,'LSL'),
            (-1,-1,'RSR'),
            (+1,-1,'LSR'),
            (-1,+1,'RSL')
        ]

        best = None

        for t0, t1, name in configs:

            c0 = GeodesicDubins.circle_center(p0, h0, R, t0)
            c1 = GeodesicDubins.circle_center(p1, h1, R, t1)

            try:
                tb, T0, T1 = GeodesicDubins.find_tangent(c0, c1, R)
            except:
                continue

            # ---- ARC 1 ----
            arc1, h_mid, mid = GeodesicDubins.simulate_arc(
                p0, h0, c0, R, t0, tb
            )

            # ---- STRAIGHT ----
            h_line = GeodesicDubins.bearing(T0, T1)
            straight, mid2 = GeodesicDubins.simulate_straight(mid, h_line, T1)

            # ---- ARC 2 ----
            arc2, _, end = GeodesicDubins.simulate_arc(
                mid2, h_line, c1, R, t1, h1
            )

            path = arc1 + straight + arc2

            length = len(path)

            if best is None or length < best[0]:
                best = (length, name, path)

        if best is None:
            raise ValueError("No valid path found")
        print(best[2])
        #return {"mode": best[1],"path": best[2],"points": len(best[2])}
        return best[2]
