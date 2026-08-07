from .geodesy import Geodesy
from math import atan2, radians, cos, sin, asin, sqrt, degrees, acos, pi,tan
import numpy as np

class GenerateHeadland:
    def __init__(self, long_bearing, application_width, turning_radius):
        #Geodesy = Geodesy()
        self.application_width = application_width
        self.turning_radius = turning_radius
        self.long_bearing = long_bearing
        
    def gen_headland(self, gcpp):
        boundary_points = self.cut_headland(gcpp)
        pts = self.intersect(boundary_points)
        
        edge_new = self.verify(pts)

        return [[edge_new[i - 1], edge_new[i]] for i in range(len(edge_new))]

    def verify(self,gcp):
        gcpp=gcp
        
        count=[]
        for i in range(len(gcpp)-1,0,-1):
    
            angle_1 = Geodesy.norm_180(Geodesy.angle(gcpp[i],gcpp[i-1])[0])
            angle_2= Geodesy.norm_180(Geodesy.angle(gcpp[i-1],gcpp[i-2])[0])
            diff=Geodesy.norm_180(angle_1-angle_2)
            if diff<0:
                count.append(gcpp[i-2])
        for i in count:
            gcpp.remove(i)
        
        return gcpp


    def determine_pt(self, gcp1, bearing1, gcp2, bearing2):
        lat1, lon1 = map(radians, gcp1)
        lat2, lon2 = map(radians, gcp2)
        bearing1 = radians(bearing1)
        bearing2 = radians(bearing2)

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1

        a = sin(delta_lat / 2)**2 + cos(lat1) * cos(lat2) * sin(delta_lon / 2)**2
        angular_dist = 2 * asin(sqrt(a))

        theta_a = acos((sin(lat2) - sin(lat1) * cos(angular_dist)) / (sin(angular_dist) * cos(lat1)))
        theta_b = acos((sin(lat1) - sin(lat2) * cos(angular_dist)) / (sin(angular_dist) * cos(lat2)))

        if sin(delta_lon) > 0:
            theta_12 = theta_a
            theta_21 = 2 * pi - theta_b
        else:
            theta_12 = 2 * pi - theta_a
            theta_21 = theta_b

        alpha1 = (bearing1 - theta_12 + pi) % (2 * pi) - pi
        alpha2 = (theta_21 - bearing2 + pi) % (2 * pi) - pi

        alpha3 = acos(-cos(alpha1) * cos(alpha2) + sin(alpha1) * sin(alpha2) * cos(angular_dist))

        ang13 = atan2(
            sin(angular_dist) * sin(alpha1) * sin(alpha2),
            cos(alpha2) + cos(alpha1) * cos(alpha3)
        )

        lat3 = asin(sin(lat1) * cos(ang13) + cos(lat1) * sin(ang13) * cos(bearing1))
        delta_lon3 = atan2(
            sin(bearing1) * sin(ang13) * cos(lat1),
            cos(ang13) - sin(lat1) * sin(lat3)
        )
        lon3 = lon1 + delta_lon3

        return abs(degrees(lat3)), degrees(lon3) % 180

    def intersect(self, gcp):
        out = []
        for i in range(1, len(gcp)):
            pt1_start, pt1_end = gcp[i]
            pt0_end, pt0_start = gcp[i - 1][1], gcp[i - 1][0]

            angle_a = Geodesy.angle(pt1_start, pt1_end)[0]
            angle_z = Geodesy.angle(pt0_end, pt0_start)[0]

            pt = self.determine_pt(pt1_start, angle_a, pt0_end, angle_z)
            out.append(pt)
        return out

    def cut_headland(self, gcpp):
        boundary_points = []

        n = len(gcpp)

        for i, (p0, p1) in enumerate(gcpp):

          
            prev_pair = gcpp[i - 1]
            next_pair = gcpp[(i + 1) % n]

            angle_between = Geodesy.angle(p0, p1)[0]
            delta_angle = abs(self.long_bearing - angle_between)
            delta_angle = min(delta_angle, 360 - delta_angle)
            
            #print(self.application_width/tan(radians(delta_angle)))
            # Calculate distance using turning radius and application width
            offset_dist = self.application_width / 2 + self.turning_radius
            
            #abhi k liye we will offset it by 5 m constantly
            #offset_dist=5

            prev_angle = Geodesy.angle(prev_pair[1], prev_pair[0])[0]
            next_angle = Geodesy.angle(next_pair[0], next_pair[1])[0]

            del_prev = Geodesy.norm_180(angle_between - prev_angle)
            del_next = Geodesy.norm_180(angle_between - next_angle)
            #print(f"prev diff = {del_prev}, and next diff ={del_next}")

            try:
                gcp1 = Geodesy.points(p0, offset_dist / (-1*sin(radians(del_prev))), prev_angle)
            except ZeroDivisionError:
                gcp1 = p0

            try:
                gcp2 = Geodesy.points(p1, offset_dist / (-1*sin(radians(del_next))), next_angle)
            except ZeroDivisionError:
                gcp2 = p1

            boundary_points.append([gcp1, gcp2])

        return boundary_points

