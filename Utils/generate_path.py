import json
from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import math
import numpy as np
from requests import request
from Utils.helper.geodesy import Geodesy
from Utils.helper.turn_generation import GenerateTurn
from Utils.helper.generate_headland import GenerateHeadland
from Utils.helper.geofence import Geofence
from Utils.helper.headland_generator import ModernHeadlandGenerator
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

    def _longest_edge_reference(self):

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
        ) = self._longest_edge_reference()

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


        for i, trk in enumerate(tracks):

            midpoint_track = Geodesy.midPoint(
                trk[0],
                trk[1]
            )[1]

        return tracks

      
class Headland:
    """
    Production-grade headland generation.
    Uses ModernHeadlandGenerator (Shapely-based offsetting).
    This class maintains backward compatibility while providing robust edge case handling.
    """

    @staticmethod
    def build_headland(polygon, track_heading, width, turning_radius):
        """
        Generate headland using industry-standard algorithms.
        
        This replaces the previous edge-intersection approach with:
        1. Shapely buffer (primary method) - handles all edge cases
        2. Geodetic fallback - works without external libraries
        
        Parameters:
        -----------
        polygon : list of [lat, lon]
            Field boundary vertices
        track_heading : float (degrees)
            Primary track direction
        width : float (meters)
            Implement width
        turning_radius : float (meters)
            Tractor turning radius
        
        Returns:
        --------
        tuple: (headland_points, headland_pass_points)
            Both lists contain [lat, lon] vertices
        """
        
        try:
            headland, hpass = ModernHeadlandGenerator.build_headland_robust(
                polygon,
                track_heading,
                width,
                turning_radius
            )
            return headland, hpass
            
        except Exception as e:
            print(f"❌ Headland generation failed: {e}")
            import traceback
            traceback.print_exc()
            return [], []


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

                if not headland_polygon:
                    continue

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

        if not best_headland_polygon:
            print("❌ No valid headland generated for any heading")
            return [], [], None

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
        
        try:
            area_field = Geodesy.area_of(best_headland_polygon)/10000
            print(f'No. of tracks: {len(tracks)}')
            print(f'Area = {area_field:.3f} ha')
        except:
            print(f'No. of tracks: {len(tracks)}')

        return (
            tracks,
            h_gcpp,
            best_heading
        )
