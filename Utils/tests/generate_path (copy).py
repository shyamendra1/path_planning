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

        print("Data saved to ",self.savepath)

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
        number_pts = int(round(dist) / data_div)

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
        #turnss=1 : uturn, turnss=2 : omega turn, turnss=3 : flat turn
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

        #coord_dist = Geodesy.distancebet(coord[0],coord[1])
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
        check_headland_geofence=self.geofence.check_geofence(h_gcpp, boundary)
        print(check_headland_geofence)
        #geofence headland
        '''for i in h_gcpp:
            if self.geofence.is_inside( Geodesy.midPoint(i[0],i[1])[1], boundary)==False or self.geofence.is_inside( i[0], boundary)==False or self.geofence.is_inside( i[1], boundary)==False:     
                print("this point is removed")
                h_gcpp.remove(i)'''
        
        
        #h_gcpp=gcpp
           
  
        for lo in range(0,len(h_gcpp)):
            gcp_1 = h_gcpp[lo][0]
            gcp_2 = h_gcpp[lo][1]
            dist_data = Geodesy.distancebet(gcp_1,gcp_2)
            inf_data[((gcp_1[0],gcp_1[1]),(gcp_2[0],gcp_2[1]))]=(dist_data)
         
        coord = (max(inf_data, key=inf_data.get))    
        long_pts,long_dist,long_bearing = self.track(coord[0],coord[1],1)
        
        dis=0
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
                po,no,go = self.track(n[0],n[1],0.1)

                d=Geodesy.distancebet(pok[len(pok)-1],pok[len(pok)-2])
                
                
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
        
        for i in range(0,len(sides_k)):
            z_in=[]
            for k in range(0,len(sides)):
                z_out = sides[k]
                sh,sh_dis = Geodesy.angle(z_out,sides_k[i])
                if int(sh) == int(long_bearing):
                    
                    dt = [z_out,sides_k[i]]
                       
                    z_in.append(dt)
                        
                    if len(z_in) > 1:
                        filt_side.append(z_in[0])
                    else:
                        filt_side.append(dt)
        defa =[]
        
        defa = self.remove_duplicates(filt_side)
        

        
        '''for i in defa:
            if self.geofence.is_inside( Geodesy.midPoint(i[0],i[1])[1], boundary)==False or self.geofence.is_inside( i[0], boundary)==False or self.geofence.is_inside( i[1], boundary)==False:     
                print("this point is removed")
                defa.remove(i)'''
                
                
                 
        
        trakk=[]
        
        #verify distance between tracks
        
        for i in range(0,len(defa)-1):
            mid = Geodesy.midPoint(defa[i+1][0],defa[i+1][1])[1]
            #mid= defa[i+1][1]
            ctd=Geodesy.cross_track_distance(mid, defa[i][0],defa[i][1])
            print("ctd ",ctd)
        
        return defa, h_gcpp
        #verify all tracks are within headland : geofence tracks
        #check_track_geofenced=self.geofence.check_geofence(defa, boundary)
        
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
        
        '''for i in range(0,len(trak)):
            
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
                pass'''

        for i in range(0,len(trak)):
            
            try:
                if i in list0:
                    if i%2==0:
                        
                        final_track.append(trak[i])
                        turn_angle = Geodesy.angle(trak[i+1][len(trak[i+1])-1],trak[i][len(trak[i])-1])[0]
                        path_angle= long_bearing
                        
                        diff = abs((turn_angle-path_angle))
                        print(diff)
                        if diff<90:
                            print("true")
                            dist = self.Application_width/tan(diff)
                            print("dist", dist)
                            pt=Geodesy.points(trak[i][len(trak[i])-1], dist, long_bearing)
                            #final_track.append(pt)
                            print("here", diff, pt)
                            turn=self.rotate(b.flatturn(trak[i+1][len(trak[i+1])-1],pt,turning_radius))
                        else:
                            dist = abs(self.Application_width/tan(diff))
                            pt=Geodesy.points(trak[i+1][len(trak[i+1])-1], dist, long_bearing)
                            turn=self.rotate(b.flatturn(pt,trak[i][len(trak[i])-1],turning_radius))
                        final_track.append(turn)
                    
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

            except Exception as e:
                print(e)
                #pass

        print(len(final_track))
        flat_track=[]
        final_track = [ele for ele in final_track if ele != []]
        count=0
        for i in range(0, len(final_track)):
            for j in range(0, len(final_track[i])):
                flat_track.append(final_track[i][j])
        
                               
        return flat_track,h_gcpp
            
