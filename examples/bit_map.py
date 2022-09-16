
import picar_4wd as fc
import numpy as np
import sys
np.set_printoptions(threshold = sys.maxsize)
speed = 30
car = (50,0)
def get_points_from_distances(distance):
    points = []
    angle_step = 10
    cur_angle = -90
    i=0
    while(cur_angle<=90):
        if(distance[i]> 0):
            x = distance[i]*np.sin(cur_angle*np.pi/180.0)+car[0]
            y = distance[i]*np.cos(cur_angle*np.pi/180.0)
            points.append((int(x),int(y)))
        i+=1
        cur_angle+=angle_step
    return points
def get_map_from_distances(distance):
    points = get_points_from_distances(distance)
    map = np.zeros((100,100))
    for i in points:
        if(i[0] < 100 and i[1]<100):
            map[i[0]][i[1]] = 1
    i=0
    while(i<len(points) - 1):
        point1 = points[i]
        point2 = points[i+1]
        xdif = np.abs(point1[0]-point2[0])
        ydif = np.abs(point1[1]-point2[1])
        if(xdif*xdif+ydif*ydif >25):
            i+=1
            continue
        scale = float(ydif/xdif)
        #if(scale>1):
            #every 1 x, scale ys  s and then 


        i+=1
    return map



def main():
    
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue
        break

    right = -1
    while True:
        scan_map = []
        for i in range(right*90,-right*91,-right*10):
            dist = fc.get_distance_at(i)
            if(dist == -2):
                scan_map.append(100)
            else:
                scan_map.append(dist)        
        right *=-1 
        m = get_map_from_distances(scan_map)
        m = np.array(m, dtype = int)
        for a in range(100):
            print(*m[a])
        break

        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
