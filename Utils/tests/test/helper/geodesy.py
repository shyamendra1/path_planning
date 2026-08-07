from math import atan2, radians, cos, sin, asin, sqrt, degrees, pi, tan
import math
import json
import geopandas as gpd
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import orient

class Geodesy:
    EARTH_RADIUS = 6378100  # meters

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        # Convert degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * asin(sqrt(a))
        return Geodesy.EARTH_RADIUS * c

    @staticmethod
    def angle(gcp_1, gcp_2):
        lat1, lon1 = map(radians, gcp_1)
        lat2, lon2 = map(radians, gcp_2)
        delta_lon = lon2 - lon1

        X = cos(lat2) * sin(delta_lon)
        Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)

        beta = degrees(atan2(X, Y))
        final_bearing = (beta + 360) % 360 
        return final_bearing, beta

    @staticmethod
    def distancebet(gcp_1, gcp_2):
        return Geodesy.haversine(gcp_1[0], gcp_1[1], gcp_2[0], gcp_2[1])
    
    @staticmethod
    def cross_track_distance(p, a, b):
        

        d13 = Geodesy.haversine(a[0],a[1],p[0],p[1])/Geodesy.EARTH_RADIUS
        #print("dist",d13)
        theta13 = radians(Geodesy.angle(a, p)[0])
        theta12 = radians(Geodesy.angle(a, b)[0])
        #print("theta", theta13, theta12)
        return asin(sin(d13) * sin(theta13 - theta12)) * Geodesy.EARTH_RADIUS

    @staticmethod
    def points(gcp_1, dist, angle):
        lat1 = radians(gcp_1[0])
        lon1 = radians(gcp_1[1])
        bearing = radians(angle)
        angular_distance = dist / Geodesy.EARTH_RADIUS

        new_lat = asin(sin(lat1) * cos(angular_distance) +
                       cos(lat1) * sin(angular_distance) * cos(bearing))

        new_lon = lon1 + atan2(sin(bearing) * sin(angular_distance) * cos(lat1),
                               cos(angular_distance) - sin(lat1) * sin(new_lat))

        return [degrees(new_lat), degrees(new_lon)]

    @staticmethod
    def midPoint(gcp_1, gcp_2):
        lat1, lon1 = map(radians, gcp_1)
        lat2, lon2 = map(radians, gcp_2)

        dlon = lon2 - lon1
        Bx = cos(lat2) * cos(dlon)
        By = cos(lat2) * sin(dlon)

        lat_m = atan2(sin(lat1) + sin(lat2),
                      sqrt((cos(lat1) + Bx) ** 2 + By ** 2))
        lon_m = lon1 + atan2(By, cos(lat1) + Bx)

        midpoint = [degrees(lat_m), degrees(lon_m)]
        return [gcp_1, midpoint, gcp_2]

    @staticmethod
    def norm_180(angle):
        if angle > 180:
            angle = angle - 360
        if angle < -180:
            angle = angle+360
        
        return angle

    @staticmethod
    def diff_abs(angle1, angle2):
        diff = abs(angle1-angle2)
        actual = diff%360
        return actual
       
    @staticmethod
    def area_of(polygon):
        #polygon = [[28.43121088667339,77.33822388427893],[28.431220707811,77.33926098387042],[28.43170007818224,77.33949082494256],[28.43169024833912,77.33822381089942]]
        R = Geodesy.EARTH_RADIUS
        
        polygon.append(polygon[0])

        n_vertices = len(polygon) - 1
        #print(f"total sides {n_vertices}")
        S = 0  

        for v in range(n_vertices):
            t1 = radians(polygon[v][0])
            t2 = radians(polygon[v+1][0])
            Δλ = radians(polygon[v+1][1] - polygon[v][1])

            tan_1_2 = tan(t1 / 2)
            tan_2_2 = tan(t2 / 2)
            numerator = tan(Δλ / 2) * (tan_1_2 + tan_2_2)
            denominator = 1 + tan_1_2 * tan_2_2
            E = 2 * atan2(numerator, denominator)
            S += E

        A = abs(S * R * R)
        
        return A


    @staticmethod
    def track_area(gcp, Application_width):
        total_dist=0
        
        for i in range(0, len(gcp)):
            
            try: 
                d=Geodesy.distancebet(gcp[i],gcp[i+1])
                total_dist+=d
                
            except:
                pass
        
        return total_dist* Application_width



    @staticmethod
    def simplify_from_file(coords, tolerance=0.000002):

        polygon = Polygon(coords)
        simplified_polygon = polygon.simplify(tolerance=tolerance, preserve_topology=True)
        #simplified_polygon = polygon.simplify(tolerance=0.000002, preserve_topology=True)
        simplified_polygon = orient(simplified_polygon, sign=-1.0)

        simplified_coords = list(simplified_polygon.exterior.coords)[:-1]
         
        return simplified_coords

    @staticmethod
    def verify_convex(gcpp):
        for i in range(0,len(gcpp)-1):
            angle_1 = Geodesy.norm_180(Geodesy.angle(gcpp[i-1],gcpp[i])[0])
            angle_2= Geodesy.norm_180(Geodesy.angle(gcpp[i],gcpp[i+1])[0])
            diff=Geodesy.norm_180(angle_1-angle_2)
            if diff>0:
                #print("concave edge found")
                return False
        return True


    '''@staticmethod
    def perpendicular_distance(point, start, end):
        if start == end:
            return math.dist(point, start)
        x0, y0 = point
        x1, y1 = start
        x2, y2 = end
        return abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.hypot(x2 - x1, y2 - y1)
    
    @staticmethod
    def douglas_peucker(points, epsilon=1.1):
        
        if len(points) < 3:
            return points
        
        start, end = points[0], points[-1]
        max_dist = 0
        index = 0
        for i in range(1, len(points) - 1):
            dist = Geodesy.perpendicular_distance(points[i], start, end)
            if dist > max_dist:
                max_dist = dist
                index = i
            print(dist,max_dist)
        if max_dist > epsilon:
            left = Geodesy.douglas_peucker(points[:index+1], epsilon)
            right = Geodesy.douglas_peucker(points[index:], epsilon)
            return left[:-1] + right
        else:
            return [start, end]   '''             
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                

