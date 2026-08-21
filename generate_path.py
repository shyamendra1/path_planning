import json
from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import math
from Utils.helper.geodesy import Geodesy

class Polygon:

    def __init__(self, vertices):
        self.vertices = vertices

    def edges(self):
        """
        Generator returning polygon edges.
        yields (start, end)
        """

        n = len(self.vertices)

        for i in range(n):
            yield (
                self.vertices[i],
                self.vertices[(i + 1) % n]
            )

    def intersect_track(self, point, heading):
        """
        Intersect an infinite track with the polygon.

        Parameters
        ----------
        point : [lat, lon]
            Point lying on the track.

        heading : degrees
            Track direction.

        Returns
        -------
        list
            Two intersection points.
        """

        intersections = []

        for edge_start, edge_end in self.edges():

            edge_heading = Geodesy.angle(
                edge_start,
                edge_end
            )[0]

            found = None


            for track_heading in (
                    heading,
                    (heading + 180) % 360):

                for edge_dir in (
                        edge_heading,
                        (edge_heading + 180) % 360):

                    try:

                        pt = Geodesy.intersection(
                            point[0],
                            point[1],
                            track_heading,

                            edge_start[0],
                            edge_start[1],
                            edge_dir
                        )

                        
                        d1 = Geodesy.distancebet(
                            edge_start,
                            pt
                        )

                        d2 = Geodesy.distancebet(
                            pt,
                            edge_end
                        )

                        edge_len = Geodesy.distancebet(
                            edge_start,
                            edge_end
                        )

                        #
                        # Allow 10 cm tolerance
                        #

                        if abs((d1 + d2) - edge_len) < 0.10:

                            found = pt
                            break

                    except ValueError:
                        pass

                if found is not None:
                    break

            if found is not None:

                duplicate = False

                for p in intersections:

                    if Geodesy.distancebet(
                            p,
                            found
                    ) < 0.05:

                        duplicate = True
                        break

                if not duplicate:
                    intersections.append(found)

        return intersections

class CoveragePlanner:

    def __init__(self, polygon, implement_width, heading):

        self.polygon = Polygon(polygon)
        self.heading = heading
        self.width = implement_width

    def _longest_edge(self):

        max_len = -1
        longest_start = None
        longest_end = None

        verts = self.polygon.vertices

        for i in range(len(verts)):

            p1 = verts[i]
            p2 = verts[(i + 1) % len(verts)]

            edge_len = Geodesy.distancebet(
                p1,
                p2
            )

            if edge_len > max_len:

                max_len = edge_len
                longest_start = p1
                longest_end = p2

        midpoint = Geodesy.midPoint(
            longest_start,
            longest_end
        )[1]

        edge_heading = Geodesy.angle(
            longest_start,
            longest_end
        )[0]

        offset_heading = (edge_heading + 90) % 360

        print("Longest Edge Start:", longest_start)
        print("Longest Edge End:", longest_end)
        print("Longest Edge Length:", max_len)
        print("Edge Heading:", edge_heading)
        print("Offset Heading:", offset_heading)
        print("Midpoint:", midpoint)

        return (
            longest_start,
            longest_end,
            midpoint,
            edge_heading,
            offset_heading
        )

    def _normalize_intersections(self, pts):

        if len(pts) < 2:
            return None

        if len(pts) == 2:
            return pts

        #
        # If a swath passes through a polygon
        # vertex, 3 intersections are possible.
        #
        # Choose the longest valid segment.
        #
        max_dist = -1
        best_pair = None

        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):

                d = Geodesy.distancebet(
                    pts[i],
                    pts[j]
                )

                if d > max_dist:

                    max_dist = d
                    best_pair = [
                        pts[i],
                        pts[j]
                    ]

        return best_pair

    def _normalize_track_direction(
        self,
        pts,
        edge_heading
    ):

        if pts is None:
            return None

        bearing = Geodesy.angle(
            pts[0],
            pts[1]
        )[0]

        diff = abs(
            Geodesy.norm_180(
                bearing - edge_heading
            )
        )

        if diff > 90:
            return [
                pts[1],
                pts[0]
            ]

        return pts

    def generate_tracks(self):

        (
            longest_start,
            longest_end,
            midpoint,
            edge_heading,
            offset_heading
        ) = self._longest_edge()

        #
        # Measure polygon extents
        # along perpendicular axis.
        #
        projections = []

        for v in self.polygon.vertices:

            dist = Geodesy.distancebet(
                midpoint,
                v
            )

            bearing = Geodesy.angle(
                midpoint,
                v
            )[0]

            delta = math.radians(
                bearing - offset_heading
            )

            projection = dist * math.cos(delta)

            projections.append(projection)

        min_offset = min(projections)
        max_offset = max(projections)

        #
        # Store:
        # (offset_value, track)
        #
        track_data = []

        #
        # Center swath
        #
        center_pts = self.polygon.intersect_track(
            midpoint,
            edge_heading
        )

        center_pts = self._normalize_intersections(
            center_pts
        )

        center_pts = self._normalize_track_direction(
            center_pts,
            edge_heading
        )

        if center_pts is not None:
            track_data.append(
                (0.0, center_pts)
            )

        #
        # Positive direction
        #
        step = self.width

        while step <= max_offset + self.width:

            shifted_point = Geodesy.points(
                midpoint,
                step,
                offset_heading
            )

            pts = self.polygon.intersect_track(
                shifted_point,
                edge_heading
            )

            pts = self._normalize_intersections(
                pts
            )

            pts = self._normalize_track_direction(
                pts,
                edge_heading
            )

            if pts is not None:
                track_data.append(
                    (step, pts)
                )

            step += self.width

        #
        # Negative direction
        #
        step = self.width

        while step <= abs(min_offset) + self.width:

            shifted_point = Geodesy.points(
                midpoint,
                step,
                (offset_heading + 180) % 360
            )

            pts = self.polygon.intersect_track(
                shifted_point,
                edge_heading
            )

            pts = self._normalize_intersections(
                pts
            )

            pts = self._normalize_track_direction(
                pts,
                edge_heading
            )

            if pts is not None:
                track_data.append(
                    (-step, pts)
                )

            step += self.width

        #
        # Sort tracks spatially.
        #
        track_data.sort(
            key=lambda x: x[0]
        )

        tracks = [
            item[1]
            for item in track_data
        ]

        return tracks

     
class Headland:

    MITER_LIMIT = 3.0
    MIN_ANGLE = 30.0
    MAX_INTERSECTION_FACTOR = 3.0

    @staticmethod
    def signed_turn(prev, curr, nxt):

        x1 = curr[1] - prev[1]
        y1 = curr[0] - prev[0]

        x2 = nxt[1] - curr[1]
        y2 = nxt[0] - curr[0]

        return x1 * y2 - y1 * x2

    @staticmethod
    def polygon_clockwise(poly):

        area = 0

        for i in range(len(poly)):

            x1, y1 = poly[i][1], poly[i][0]
            x2, y2 = poly[(i + 1) % len(poly)][1], poly[(i + 1) % len(poly)][0]

            area += x1 * y2 - x2 * y1

        return area < 0

    @staticmethod
    def clean_polygon(poly, min_edge):

        if len(poly) < 3:
            return poly

        cleaned = []

        n = len(poly)

        for i in range(n):

            curr = poly[i]
            nxt = poly[(i + 1) % n]

            if Geodesy.distancebet(curr, nxt) > min_edge:
                cleaned.append(curr)

        return cleaned

    @staticmethod
    def interior_angle(prev, curr, nxt):

        b1 = Geodesy.angle(curr, prev)[0]
        b2 = Geodesy.angle(curr, nxt)[0]

        return abs((b2 - b1 + 180) % 360 - 180)

    @staticmethod
    def excessive_miter(angle_deg,
                        offset_dist,
                        limit=MITER_LIMIT):

        angle_deg = max(angle_deg, 0.5)

        miter = offset_dist / math.sin(
            math.radians(angle_deg / 2.0)
        )

        return miter > limit * offset_dist

    @staticmethod
    def safe_intersection(
            prev_start,
            prev_end,
            prev_heading,

            next_start,
            next_end,
            next_heading,

            offset_dist):

        heading_diff = abs(
            (prev_heading - next_heading + 180) % 360 - 180
        )

        #
        # nearly parallel
        #
        if heading_diff < 3.0:
            return None

        try:

            pt = Geodesy.intersection(

                prev_start[0],
                prev_start[1],
                prev_heading,

                next_start[0],
                next_start[1],
                next_heading
            )

        except ValueError:

            return None

        d1 = Geodesy.distancebet(prev_end, pt)
        d2 = Geodesy.distancebet(next_start, pt)

        #
        # spike protection
        #
        if d1 > Headland.MAX_INTERSECTION_FACTOR * offset_dist:
            return None

        if d2 > Headland.MAX_INTERSECTION_FACTOR * offset_dist:
            return None

        return pt

    @staticmethod
    def offset_edge(
            edge_start,
            edge_end,
            track_heading,
            width,
            turning_radius):

        edge_heading = Geodesy.angle(
            edge_start,
            edge_end
        )[0]

        normal = (edge_heading + 90) % 360

        theta = math.radians(
            track_heading - edge_heading
        )

        offset = (
            (abs(math.sin(theta)) + 1.0)
            * turning_radius
            + width / 2.0
        )

        s = Geodesy.points(
            edge_start,
            offset,
            normal
        )

        e = Geodesy.points(
            edge_end,
            offset,
            normal
        )

        offseth = (
            width / 2.0
        ) * abs(math.sin(theta))

        sh = Geodesy.points(
            edge_start,
            offseth,
            normal
        )

        eh = Geodesy.points(
            edge_end,
            offseth,
            normal
        )

        return (

            s,
            e,
            edge_heading,
            offset,

            sh,
            eh,
            edge_heading,
            offseth
        )

    @staticmethod
    def build_headland(
            polygon,
            track_heading,
            width,
            turning_radius):

        #
        # aggressive cleaning
        #

        min_edge = max(
            width * 0.75,
            turning_radius * 0.50
        )

        polygon = Headland.clean_polygon(
            polygon,
            min_edge
        )

        if len(polygon) < 3:
            return [], []

        n = len(polygon)

        clockwise = Headland.polygon_clockwise(
            polygon
        )

        offset_edges = []
        headland_pass = []

        for i in range(n):

            a = polygon[i]
            b = polygon[(i + 1) % n]

            h = Headland.offset_edge(
                a,
                b,
                track_heading,
                width,
                turning_radius
            )

            offset_edges.append(h[:4])
            headland_pass.append(h[4:])

        headland = []
        hpass = []

        for i in range(n):

            prev = polygon[(i - 1) % n]
            curr = polygon[i]
            nxt = polygon[(i + 1) % n]

            turn = Headland.signed_turn(
                prev,
                curr,
                nxt
            )

            if clockwise:
                convex = turn < 0
            else:
                convex = turn > 0

            prev_start, prev_end, prev_heading, prev_offset = \
                offset_edges[(i - 1) % n]

            next_start, next_end, next_heading, next_offset = \
                offset_edges[i]

            pprev_start, pprev_end, pprev_heading, pprev_offset = \
                headland_pass[(i - 1) % n]

            pnext_start, pnext_end, pnext_heading, pnext_offset = \
                headland_pass[i]

            #
            # local geometry scale
            #

            prev_len = Geodesy.distancebet(
                prev,
                curr
            )

            next_len = Geodesy.distancebet(
                curr,
                nxt
            )

            local_scale = max(
                0.001,
                min(prev_len, next_len)
            )

            offset_dist = min(
                prev_offset,
                next_offset
            )

            #
            # bottleneck protection
            #
            offset_dist = min(
                offset_dist,
                0.70 * local_scale
            )

            angle = Headland.interior_angle(
                prev,
                curr,
                nxt
            )

            miter_bad = Headland.excessive_miter(
                angle,
                offset_dist
            )

            #
            # convex
            #

            if convex:

                #
                # acute corner
                #
                if angle < Headland.MIN_ANGLE or miter_bad:

                    headland.append(prev_end)
                    headland.append(next_start)

                else:

                    pt = Headland.safe_intersection(

                        prev_start,
                        prev_end,
                        prev_heading,

                        next_start,
                        next_end,
                        next_heading,

                        offset_dist
                    )

                    if pt is None:

                        headland.append(prev_end)
                        headland.append(next_start)

                    else:

                        headland.append(list(pt))

                #
                # pass boundary
                #

                try:

                    ppt = Geodesy.intersection(

                        pprev_start[0],
                        pprev_start[1],
                        pprev_heading,

                        pnext_start[0],
                        pnext_start[1],
                        pnext_heading
                    )

                    hpass.append(list(ppt))

                except ValueError:

                    hpass.append(pprev_end)

            #
            # concave
            #

            else:

                headland.append(prev_end)
                headland.append(next_start)

                hpass.append(pprev_end)
                hpass.append(pnext_start)

        #
        # final cleanup
        #

        try:
            headland = Headland.clean_polygon(
                headland,
                width * 0.25
            )
        except:
            pass

        return headland, hpass


class Path_plan:
    def __init__(self, gcp, save_path, Application_width,turning_radius,tractor_wheelbase, field_name):
        """
        :param gcp- gcp pooints of path or boundary
        """
        self.gcp = gcp
        self.savepath=save_path
        self.Application_width=Application_width
        self.turning_radius=turning_radius
        self.tractor_wheelbase=tractor_wheelbase
        # self.turns=GenerateTurn()   
        # self.geofence = Geofence()
        self.field_name = field_name

        
    def path(self):
        tp, headland, bearing = self.path_planning(
            self.gcp,
            self.Application_width,
            self.turning_radius,
            self.tractor_wheelbase,
            2
        )
        self.save_path(tp, headland)

        return tp, headland

    def save_path_txt(self, data):
        f = open(self.save_path, 'w')
        for item in data:
            f.write(str(item[0])+","+str(item[1])+"\r\n")
        f.close()
        # print("path data stored to file in data/path_points.txt")
    
    def save_path(self, path_points,headland):
        data = {
            "farm_boundary": self.gcp,
            "Application_width": self.Application_width,
            "Turning_radius":self.turning_radius,
            "Tractor_wheelbase": self.tractor_wheelbase,
            "path_points":path_points,
            "headland":headland
        }

       
        with open(self.savepath, "w") as file:
            json.dump(data, file, indent=4)

        #print("Data saved to ",self.savepath) 
    #------------------------------------------------------------------------------------------LOC

    def path_planning(
        self,
        gcp,
        Application_width,
        turning_radius,
        tractor_wheelbase,
        bearing_override=None):
        polygon=[]
        for edge in gcp:
            polygon.append(edge[0])

        headings = list(range(0, 180, 20))

        for i in range(len(polygon)):
            try:

                edge_bearing = Geodesy.angle(
                    polygon[i],
                    polygon[(i + 1) % len(polygon)]
                )[0]

                headings.append(edge_bearing)

            except Exception:
                pass

        best_tracks = []
        best_headland_polygon = []
        best_heading = None
        cost_max = 0

        for heading in headings:

            try:

                headland_polygon, headland_pass = (
                    Headland.build_headland(
                        polygon,
                        heading,
                        Application_width,
                        turning_radius
                    )
                )

                cost = Geodesy.area_of(headland_polygon)

                if cost > cost_max:
                    cost_max = cost
                    
                    best_headland_polygon = headland_polygon
                    best_heading = heading
            except Exception as e:

                print(
                    f"Heading {heading} failed: {e}"
                )

                continue               

        planner = CoveragePlanner(
                    best_headland_polygon,
                    Application_width,
                    best_heading
                )

        tracks = planner.generate_tracks()

        h_gcpp = []

        if len(best_headland_polygon) > 1:

            for i in range(len(best_headland_polygon)):

                p1 = best_headland_polygon[i]
                p2 = best_headland_polygon[
                    (i + 1) % len(best_headland_polygon)
                ]

                h_gcpp.append([p1, p2])

        print(
            f"Selected heading: {best_heading}"
        )
        area_field = Geodesy.area_of(headland_polygon)/10000
        print(f'No. of tracks: {len(tracks)}')
        print(f'Area = {area_field:.3f} ha')

        return (
            tracks,
            h_gcpp,
            best_heading
        )
