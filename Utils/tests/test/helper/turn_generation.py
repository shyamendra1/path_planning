from .geodesy import Geodesy

from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import numpy as np

class GenerateTurn:

    def __init__(self):
        self.geo = Geodesy() 


    def flatturn(self,gcp_1,gcp_2, turning_radius):
        turn=[]

        bear,coef = self.geo.angle(gcp_1,gcp_2)
        
        
        widthoftrack=self.geo.distancebet(gcp_1,gcp_2)
        dist = turning_radius

        alpha=90
        
        
        new_center, new_center2=self.center_circle(gcp_1,gcp_2, bear+180,dist)
        
        a=np.linspace(0,alpha,1)
        
        for i in range(0,len(a)):           
            newgcp2=self.arcfirst(new_center, bear+(90-a[i]), dist)
            
       
        for i in reversed(range(0,len(a))):       
            newgcp=self.arcfirst(new_center2,bear+90+a[i], dist)      
            
        
        dist_pts = self.geo.distancebet(newgcp2[len(newgcp2)-1],newgcp[len(newgcp)-1])

        if self.geo.diff_abs(self.geo.angle(gcp_1,gcp_2)[0],self.geo.angle(newgcp2[len(newgcp2)-1],newgcp[len(newgcp)-1])[0])<3:
            
            turn.append(newgcp2[len(newgcp2)-1])     
            turn.append(newgcp[len(newgcp)-1])
        else:
            
            turn.append(newgcp[len(newgcp)-1])
            turn.append(newgcp2[len(newgcp2)-1])            

        #turn.append(newgcp2[len(newgcp2)-1])     
        #turn.append(newgcp[len(newgcp)-1])
        
        #print(len(turn))

        return turn

  
    def turn(self,gcp_1,gcp_2, turning_radius):
        turn=[]
        bear,coef = self.geo.angle(gcp_1,gcp_2)

        widthoftrack=self.geo.distancebet(gcp_1,gcp_2)
        dist = turning_radius
        w=widthoftrack
        q=(2*dist+w)/(4*dist)
        try:
            al=acos(q)
            alpha=al*(180/pi)
            
            new_center, new_center2=self.center_circle(gcp_1,gcp_2, bear,dist)
            centerofcircle=self.arcfirst(new_center,bear+alpha,dist*2)

            a=np.linspace(0,alpha,2)
            a=a[1:]
            
            for i in range(0,len(a)): 
    
                newgcp=self.arcfirst(new_center,bear+a[i], dist)      
                turn.append(newgcp[len(newgcp)-1])
                
            angle1=bear-alpha
            angle2=alpha+bear
            angle3=180+(alpha+bear)            
               
            h=np.linspace(angle1,angle3,5)
            h=[h[1],h[3]]

            for i in reversed(range(0,len(h))):
                circle=self.arcfirst(centerofcircle, h[i],dist)
                turn.append(circle[len(newgcp)-1]) 

            for i in reversed(range(0,len(a))):                           
                newgcp2=self.arcfirst(new_center2, bear+(180-a[i]), dist)
                turn.append(newgcp2[len(newgcp2)-1]) 
            
        except:
            pass


        return turn

    def center_circle(self,gcp_1, gcp_2, beta, radius):
        plot_pt=[]
        plot_pt2=[]
        center=[]
        center1=[]
        dist=radius
        
        if beta>=0:
            final_bearing = (beta+180)
            beta1=beta
        else:
            mid_angle=360+beta
            final_bearing = mid_angle+180
            beta1=360+beta
            
        earth_radius = 6378100

        angular_dist = dist/earth_radius
        
        new_lat = degrees(asin(sin(radians(gcp_1[0])) * cos(angular_dist) + cos(radians(gcp_1[0]))*sin(angular_dist)*cos(radians(final_bearing))))
        new_long = gcp_1[1] + degrees(atan2(sin(radians(final_bearing))*sin(angular_dist)*cos(radians(gcp_1[0])),cos(angular_dist)-sin(radians(gcp_1[0]))*sin(radians(new_lat))))
       
        plot_pt.append([new_lat, new_long])

     
        
        new_lat2 = degrees(asin(sin(radians(gcp_2[0])) * cos(angular_dist) + cos(radians(gcp_2[0]))*sin(angular_dist)*cos(radians(beta1))))
        new_long2 = gcp_2[1] + degrees(atan2(sin(radians(beta1))*sin(angular_dist)*cos(radians(gcp_2[0])),cos(angular_dist)-sin(radians(gcp_2[0]))*sin(radians(new_lat2))))
        
        plot_pt2.append([new_lat2, new_long2])
        
        
        return plot_pt, plot_pt2
        
    def arcfirst(self,new_center, final_bearing, radius):
        plot_pt=[]
        dist=radius
        
        earth_radius=6378100
        angular_dist=dist/earth_radius
        
        A_lat=degrees(asin(sin(radians(new_center[0][0])) * cos(angular_dist) + cos(radians(new_center[0][0]))*sin(angular_dist)*cos(radians(final_bearing))))    
        A_long= new_center[0][1] + degrees(atan2(sin(radians(final_bearing))*sin(angular_dist)*cos(radians(new_center[0][0])),cos(angular_dist)-sin(radians(new_center[0][0]))*sin(radians(A_lat))))
        plot_pt.append([A_lat,A_long])
        
        return plot_pt   
 
