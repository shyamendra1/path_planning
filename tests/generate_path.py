import json
from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import math
import numpy as np
from requests import request
from Utils.helper.geodesy import Geodesy
from Utils.helper.turn_generation import GenerateTurn
from Utils.helper.generate_headland import GenerateHeadland
from Utils.helper.geofence import Geofence
import csv

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

    def generate_bcd_cells(self):
        reference = self.polygon.vertices[0]

        reference_end = Geodesy.points(
            reference,
            100,
            self.heading
        )

        distances = []

        for v in self.polygon.vertices:

            d = Geodesy.cross_track_distance(
                v,
                reference,
                reference_end
            )

            distances.append(d)

        min_offset = min(distances)
        max_offset = max(distances)

        current = min_offset + self.width / 2

        sweep_lines = []

        while current <= max_offset:

            shifted_point = Geodesy.points(
                reference,
                current,
                self.heading + 90
            )

            pts = self.polygon.intersect_track(
                shifted_point,
                self.heading
            )

            if len(pts) >= 2:

                #
                # sort along track direction
                #
                pts = sorted(
                    pts,
                    key=lambda p: Geodesy.cross_track_distance(
                        p,
                        shifted_point,
                        Geodesy.points(
                            shifted_point,
                            100,
                            self.heading + 90
                        )
                    )
                )

                intervals = []

                for i in range(0, len(pts), 2):

                    if i + 1 < len(pts):

                        intervals.append(
                            [pts[i], pts[i+1]]
                        )

                sweep_lines.append(
                    {
                        "offset": current,
                        "intervals": intervals
                    }
                )

            current += self.width

        #
        # Build BCD cells
        #

        cells = []

        active_cells = []

        for sweep in sweep_lines:

            intervals = sweep["intervals"]

            #
            # first sweep
            #
            if not active_cells:

                for interval in intervals:

                    active_cells.append(
                        {
                            "left": interval[0],
                            "right": interval[1],
                            "points": [
                                interval[0],
                                interval[1]
                            ]
                        }
                    )

                continue

            next_active = []

            for interval in intervals:

                matched = False

                for cell in active_cells:

                    d1 = Geodesy.distancebet(
                        interval[0],
                        cell["left"]
                    )

                    d2 = Geodesy.distancebet(
                        interval[1],
                        cell["right"]
                    )

                    if d1 < self.width * 3 and \
                    d2 < self.width * 3:

                        cell["points"].append(
                            interval[0]
                        )

                        cell["points"].append(
                            interval[1]
                        )

                        cell["left"] = interval[0]
                        cell["right"] = interval[1]

                        next_active.append(cell)

                        matched = True
                        break

                if not matched:

                    next_active.append(
                        {
                            "left": interval[0],
                            "right": interval[1],
                            "points": [
                                interval[0],
                                interval[1]
                            ]
                        }
                    )

            #
            # cells that disappeared are completed
            #
            for cell in active_cells:

                if cell not in next_active:

                    cells.append(cell)

            active_cells = next_active

        cells.extend(active_cells)

        return cells

    def build_cell_polygon(self, cell):

        pts = cell["points"]

        left_chain = pts[0::2]
        right_chain = pts[1::2]

        poly = (
            left_chain +
            list(reversed(right_chain))
        )

        return poly

    def generate_tracks_bcd(self):

        cells = self.generate_bcd_cells()

        all_tracks = []

        for cell in cells:

            poly = self.build_cell_polygon(cell)

            if len(poly) < 3:
                continue

            planner = CoveragePlanner(
                poly,
                self.width,
                self.heading
            )

            all_tracks.extend(
                planner.generate_tracks_convex()
            )

        return all_tracks

    def generate_tracks(self):

        reference = self.polygon.vertices[0]

        reference_end = Geodesy.points(
            reference,
            100,
            self.heading
        )

        distances = []

        for v in self.polygon.vertices:

            d = Geodesy.cross_track_distance(
                v,
                reference,
                reference_end
            )

            distances.append(d)

        min_offset = min(distances)
        max_offset = max(distances)

        current = min_offset + self.width / 2

        concave = False

        while current <= max_offset:

            shifted_point = Geodesy.points(
                reference,
                current,
                self.heading + 90
            )

            pts = self.polygon.intersect_track(
                shifted_point,
                self.heading
            )

            if len(pts) > 2:

                concave = True
                break

            current += self.width

        if concave:

            print("BCD decomposition activated")

            return self.generate_tracks_bcd()

        return self.generate_tracks_convex()

    def generate_tracks_convex(self):

        reference = self.polygon.vertices[0]

        reference_end = Geodesy.points(
            reference,
            100,
            self.heading
        )

        distances = []

        for v in self.polygon.vertices:

            d = Geodesy.cross_track_distance(
                v,
                reference,
                reference_end
            )

            distances.append(d)

        min_offset = min(distances)
        max_offset = max(distances)

        

        #
        # First pass
        #

        current = min_offset + self.width / 2

        tracks = []

        while current <= max_offset:

            shifted_point = Geodesy.points(
                reference,
                current,
                self.heading + 90
            )

            pts = self.polygon.intersect_track(
                shifted_point,
                self.heading
            )
            

            if len(pts) == 2:

                tracks.append(pts)
            else:
                print("len ", len(pts))
                print("it is a concave shape. it requires decomposition.")
                tracks.append(pts)
                
            current += self.width

        return tracks
     
class Headland:

    @staticmethod
    def signed_turn(prev, curr, nxt):
        """
        >0 : left turn
        <0 : right turn
        """
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
    def offset_edge(edge_start, edge_end, track_heading, width, turning_radius):

        edge_heading = Geodesy.angle(edge_start, edge_end)[0]

        normal = (edge_heading + 90) % 360

        theta = (math.radians(track_heading - edge_heading))

        
        offset = (abs(sin(theta))+1) * turning_radius + width/2
        #print(offset, edge_heading, theta)
        s = Geodesy.points(edge_start, offset, normal)
        e = Geodesy.points(edge_end, offset, normal)
        
        
        
        offseth =  (width/2)*abs(sin(theta))
        #print(offseth)
        #print(offset, edge_heading, theta)
        sh = Geodesy.points(edge_start, offseth, normal)
        eh = Geodesy.points(edge_end, offseth, normal)       
        

        return s, e, edge_heading, sh, eh , edge_heading

    @staticmethod
    def build_headland(polygon, track_heading, width, turning_radius):

        n = len(polygon)

        clockwise = Headland.polygon_clockwise(polygon)

        offset_edges = []
        
        headland_pass =[]

        for i in range(n):

            a = polygon[i]
            b = polygon[(i + 1) % n]
            h=Headland.offset_edge(
                    a,
                    b,
                    track_heading,
                    width,
                    turning_radius
                )

            offset_edges.append(h[:3])
            
            headland_pass.append(h[3:])
            
            

        headland = []
        hpass =[]
        for i in range(n):

            prev = polygon[(i - 1) % n]
            curr = polygon[i]
            nxt  = polygon[(i + 1) % n]

            turn = Headland.signed_turn(prev, curr, nxt)

            if clockwise:
                convex = turn < 0
            else:
                convex = turn > 0

            prev_start, prev_end, prev_heading = offset_edges[(i - 1) % n]
            next_start, next_end, next_heading = offset_edges[i]


            pprev_start, pprev_end, pprev_heading = headland_pass[(i - 1) % n]
            pnext_start, pnext_end, pnext_heading = headland_pass[i]
            
            #
            # convex -> intersect
            #
            if convex:

                try:

                    pt = Geodesy.intersection(
                        prev_start[0],
                        prev_start[1],
                        prev_heading,

                        next_start[0],
                        next_start[1],
                        next_heading
                    )
                    
                    

                    headland.append(list(pt))
                except ValueError:

                    headland.append(prev_end)


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
            # concave -> no intersection
            #
            else:

                headland.append(prev_end)
                headland.append(next_start)
                
                hpass.append(pprev_end)
                hpass.append(pnext_start)
        headland, hpass
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
