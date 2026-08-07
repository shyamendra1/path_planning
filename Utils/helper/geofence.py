class Geofence:

    def __init__(self):
        pass
        
        
    def is_left(self, p0, p1, p2):
        """Returns >0 if p2 is left of the line p0-p1, 0 if on, <0 if right."""
        return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1])

    def is_inside(self, point):
        wn = 0
        
        n = len(self.poly)
        for i in range(n):
            v1 = self.poly[i]
            v2 = self.poly[(i + 1) % n]
            
            if v1[1] <= point[1]:
                if v2[1] > point[1]:  # Upward crossing
                    if self.is_left(v1, v2, point) > 0:
                        wn += 1
            else:
                if v2[1] <= point[1]: # Downward crossing
                    if self.is_left(v1, v2, point) < 0:
                        wn -= 1
        if wn!=0:
            return True
        return False

    def check_geofence(self, points, boundary):
        self.poly=boundary
        count=[]
        for i in points:
            
            if(self.is_inside(i[0]) and self.is_inside(i[1])):
                pass
            else:
                return False
        
        return True
        

