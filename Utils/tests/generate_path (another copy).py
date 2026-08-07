import json
from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import numpy as np
from Utils.helper.geodesy import Geodesy
from Utils.helper.turn_generation import GenerateTurn
from Utils.helper.generate_headland import GenerateHeadland
from Utils.helper.geofence import Geofence
class Path_plan:
    def __init__(self, gcp, save_path, Application_width,turning_radius,tractor_wheelbase):
        """
        :param gcp- gcp pooints of path or boundary
        """
        self.gcp = gcp
        self.savepath=save_path
        self.Application_width=Application_width
        self.turning_radius=turning_radius
        self.tractor_wheelbase=tractor_wheelbase
        self.turns=GenerateTurn()   
        self.geofence = Geofence()

        
    def path(self):
        tp, headland= self.path_planning(self.gcp,self.Application_width,self.turning_radius,self.tractor_wheelbase)
        self.save_path(tp,headland)
        
        return tp,headland

    def save_path_txt(self, data):
        f = open(self.save_path, 'w')
        for item in data:
            f.write(str(item[0])+","+str(item[1])+"\r\n")
        f.close()
        print("path data stored to file in data/path_points.txt")
    
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

    def rotate(self,track):
        
        xyz=[]
        if len(track)>1:
            for i in range(0,len(track)):
                xyz.append(track[(len(track)-1)-i])
         
        return xyz
   
    

    #------------------------------------------------------------------------------------------LOC


    def track(self,gcp_1,gcp_2,scale_div): 
        plot_pt = list()
        plot_pt.append(gcp_1)

        delta_L = radians(gcp_2[1] - gcp_1[1])
        dist = Geodesy.haversine(gcp_1[0],gcp_1[1],gcp_2[0],gcp_2[1])
        
        lat1=radians(gcp_1[0])
        lon1=radians(gcp_1[1])
        lat2=radians(gcp_2[0])
        lon2=radians(gcp_2[1])
        
        X = cos(lat2) * sin(delta_L)
        Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_L)
        beta = degrees(atan2(X,Y))

        if beta >= 0:
            final_bearing = beta
        else:
            final_bearing = 360 + beta

        earth_radius = 6378100
        data_div = scale_div
        number_pts = int((dist) / data_div)

        for i in range(0,number_pts):
            angular_dist = data_div/earth_radius
            new_lat = degrees(asin(sin(lat1) * cos(angular_dist) + cos(lat1)*sin(angular_dist)*cos(radians(beta))))
            new_long = gcp_1[1] + degrees(atan2(sin(radians(beta))*sin(angular_dist)*cos(lat1),cos(angular_dist)-sin(lat1)*sin(radians(new_lat))))
            plot_pt.append([new_lat,new_long]) 
            data_div = data_div + scale_div
        plot_pt.append(gcp_2)
        return plot_pt,dist,final_bearing

        
    def arange_tracks(self, lista, m):
        chunks = [lista[i:i+m] for i in range(0, len(lista), m)]
       
        if len(chunks) > 1 and len(chunks[-1]) < m:
            chunks[-2].extend(chunks[-1])
            chunks.pop() 
        
        return chunks

    def remove_duplicates(self,data):
        seen = set()
        result = []

        for item in data:
            p1 = tuple(item[0])
            p2 = tuple(item[1])

            key = (p1, p2)

            if key not in seen:
                seen.add(key)
                result.append([list(p1), list(p2)])  

        return result

    def angle_diff(self,a,b):
        return abs((a-b+180)%360-180)
        
    def path_planning(self,gcp,Application_width,turning_radius,tractor_wheelbase):
    
        #define variables 
        final_track=[]
        filt_side=[]
        sides=[]
        sides_k=[]
        trak=[]
        green=[]
        inf_data={}
        blue=[]
        trakk=[]
        defa=[]
        turnss=3
        boundary=[]
        for i in gcp:
            boundary.append(i[0])
        
        polygon=[]
        for lo in range(0,len(gcp)):
            gcp_1 = gcp[lo][0]
            gcp_2 = gcp[lo][1]
            polygon.append(gcp_1)
            dist_data = Geodesy.distancebet(gcp_1,gcp_2)
            inf_data[((gcp_1[0],gcp_1[1]),(gcp_2[0],gcp_2[1]))]=(dist_data)
      
        
        #area= Geodesy.area_of(polygon)
        #print(f"area of field = {area}")   
        coord = (max(inf_data, key=inf_data.get))
        
        lis=list(coord[0])
        coor=[]
        coor.append(lis)
        lis=list(coord[1])
        coor.append(lis)

        long_pts,long_dist,long_bearing = self.track(coord[0],coord[1],1)
        inf_data.clear()

        m=gcp.index(coor)
        
        gcpp=[]
        gcpp=(gcp[m:len(gcp)]).copy()
        gcpp.extend(gcp[:m+1])

        a = GenerateHeadland(long_bearing, Application_width, turning_radius)    
        #generate headland
        
        h_gcpp= a.gen_headland(gcpp)
        headland=[]
        for i in h_gcpp:
            headland.append(i[0])

        for lo in range(0,len(h_gcpp)):
            gcp_1 = h_gcpp[lo][0]
            gcp_2 = h_gcpp[lo][1]
            dist_data = Geodesy.distancebet(gcp_1,gcp_2)
            inf_data[((gcp_1[0],gcp_1[1]),(gcp_2[0],gcp_2[1]))]=(dist_data)
        
        coord = (max(inf_data, key=inf_data.get))    
        long_pts,long_dist,long_bearing = self.track(coord[0],coord[1],1)
        
        dis=Application_width/2
        #dis=0
        point=[]
        for n in inf_data:
            try:
                if n == coord:
                    continue

                theta, th=Geodesy.angle(n[0],n[1])
                delta=abs(long_bearing-theta)
                delta=min(delta,360-delta)
                
                diss=dis/sin(radians(delta))
                angular_dist=diss/6378100
            
                A_lat=degrees(asin(sin(radians(n[0][0])) * cos(angular_dist) + cos(radians(n[0][0]))*sin(angular_dist)*cos(radians(theta))))    
                A_long= n[0][1] + degrees(atan2(sin(radians(theta))*sin(angular_dist)*cos(radians(n[0][0])),cos(angular_dist)-sin(radians(n[0][0]))*sin(radians(A_lat))))
                point.append([A_lat,A_long])
                
                
                p=abs(Application_width/sin(radians(delta)))
                pok,nok,gok = self.track([point[0][0],point[0][1]],n[1],p)
                po,no,go = self.track(n[0],n[1],0.05)

                d=Geodesy.distancebet(pok[len(pok)-1],pok[len(pok)-2])
                
                #print("this dist : ", d,p)
                if d<p:
                    pok.pop()
                    dis=Application_width-d*abs(sin(radians(delta)))

                green.append(pok)
                blue.append(po)
                point.clear()
            except:
                pass
        
        #return green, h_gcpp
        for k in blue:
            for h in k:
                sides.append(h)
                   
        for k in green:
            for h in k:               
                sides_k.append(h)    

        
        z_in=[]
        defa =[]         
        for i in range(len(sides_k)):
            best_dt = None
            min_diff = float('inf')
            for k in range(len(sides)):
                z_out = sides[k]
                sh, sh_dis = Geodesy.angle(z_out, sides_k[i])
                diff = abs(self.angle_diff(sh, long_bearing))
                if diff <= 0.3 and diff < min_diff:
                    min_diff = diff
                    best_dt = [z_out, sides_k[i]]
            if best_dt is not None and Geodesy.distancebet(best_dt[0],best_dt[1])>2*tractor_wheelbase:
                filt_side.append(best_dt)

        defa =[]
        
        defa = self.remove_duplicates(filt_side)
                
        
        trakk=[]
               
        #return defa, h_gcpp
        
        trakk=defa
        trakk=trakk[:]      

        number_of_skips= int((2*self.turning_radius +self.tractor_wheelbase )/(self.Application_width)) +1
        
        
        skip_factor = number_of_skips*2-1
          
        result = self.arange_tracks(trakk, skip_factor)
        
        list0=[]
        list1=[]
        count=0

        for i in result:
            if len(i)%2==0:
                k=int(len(i)/2)
            else:
                k=int(len(i)/2)+1

            for j in range(0,k): 
                try:
                    trak.append(i[j])
                    try:
                        if count%2==0:    
                            list0.append(trakk.index(i[j]))
                            list0.append(trakk.index(i[j+k]))
                        else:
                            list1.append(trakk.index(i[j]))
                            list1.append(trakk.index(i[j+k]))
                    except:
                        pass
                    trak.append(i[j+k])
                except:
                    pass
            count+=1
        
        
        b=GenerateTurn()   
        
        for i in range(0,len(trak)):
            
            try:
                if i in list0:
                    if i%2==0:
                        
                        final_track.append(trak[i])
                        final_track.append(self.rotate(b.flatturn(trak[i+1][len(trak[i+1])-1],trak[i][len(trak[i])-1],turning_radius)))
                    
                    if i%2==1:
                        
                        final_track.append(self.rotate(trak[i]))
                        
                        final_track.append(self.rotate(b.flatturn(trak[i+1][0],trak[i][0],turning_radius)))
                        

                if i in list1:
                    if i%2==0:
                        final_track.append((trak[i]))
                        final_track.append((b.flatturn(trak[i][len(trak[i])-1],trak[i+1][len(trak[i+1])-1],turning_radius)))
                    if i%2==1:
                        final_track.append(self.rotate(trak[i]))
                        final_track.append((b.flatturn(trak[i][0],trak[i+1][0],turning_radius)))

            except:
                pass

        

        #print(len(final_track))
        flat_track=[]
        final_track = [ele for ele in final_track if ele != []]
        count=0
        for i in range(0, len(final_track)):
            for j in range(0, len(final_track[i])):
                flat_track.append(final_track[i][j])
        
                               
        return flat_track,h_gcpp
