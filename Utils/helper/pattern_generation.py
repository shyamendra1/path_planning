from .geodesy import Geodesy

from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import numpy as np

class GeneratePattern:

    def __init__(self):
        self.geo = Geodesy() 
        
    def four_skip(self, trakk):
        trak=[]
        print(len(trakk))
        for i in range(0,len(trakk),9):
            try:
                trak.append(trakk[i])
                
            except:
                continue

            try:
                trak.append(trakk[i+5])
              
            except:
                try:
                    trak.append(trakk[i+4])
                    trak.append(trakk[i+1])
                    trak.append(trakk[i+3])
                    trak.append(trakk[i+2]) 
         
                except:
                    try:
                        trak.append(trakk[i+3]) 
                        
                    except:
                        try:
                            trak.append(trakk[i+2])
                            
                            trak.append(trakk[i+1])
                            
                        except:
                            try:
                                trak.append(trakk[i+1])
                               
                            except:
                                continue
                            continue
                        continue

                    try:
                        trak.append(trakk[i+1])     
                                    
                    except:                        
                        continue
                    try:
                        trak.append(trakk[i+2])   
                       
                    except:     
                        continue
                    continue
                continue
                
            try:
                trak.append(trakk[i+1])
               
            except:
                continue
                
            try:
                trak.append(trakk[i+6])
              
            except:
                try:
                    trak.append(trakk[i+4])   
                    trak.append(trakk[i+2])
                    trak.append(trakk[i+3])
                 
                    continue 
                except:
                    continue
            
            try:
                trak.append(trakk[i+2])
               
            except:
                continue
            
            try:
                trak.append(trakk[i+7])
                
            except:
                trak.append(trakk[i+4])
                trak.append(trakk[i+3])
              
                continue
            
            try:
                trak.append(trakk[i+3])
         
            except:
                continue
            
            try:
                trak.append(trakk[i+8])

            except:
                trak.append(trakk[i+4])
                continue
            try:
                trak.append(trakk[i+4])

            except: 
                continue
        return trak
