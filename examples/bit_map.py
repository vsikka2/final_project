
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
            x = int(distance[i]*np.sin(cur_angle*np.pi/180.0)+car[0])
            y = int(distance[i]*np.cos(cur_angle*np.pi/180.0))
            if(x>-1 and x<100 and y>-1 and y<100):        
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
        xdif_abs = 1 if point2[0]-point1[0]>0 else -1
        ydif_abs = 1 if point2[1]-point1[1]>0 else -1
        if(xdif*xdif+ydif*ydif > 3000):
            i+=1
            continue
        print(point1)
        print(point2)
        scale = float(ydif/xdif)
        cur_point = [point1[0],point1[1]]
        # -2 to -5 
        while(cur_point[0] != point2[0] and cur_point[1] != point2[1]):
            xdif = np.abs(cur_point[0]-point2[0])
            ydif = np.abs(cur_point[1]-point2[1])
            if(xdif == 0):
                for yval in range(cur_point[1],point2[1]+ydif_abs,ydif_abs):
                    map[cur_point[0]][yval] = 1
                cur_point = point2
                break
            if(ydif == 0):
                for xval in range(cur_point[0],point2[0]+xdif_abs,xdif_abs):
                    map[xval][cur_point[1]] = 1
                cur_point = point2
                break
            

    
            scale = float(ydif/xdif)
            
            if(scale>1):
                #every 1 x, scale ys  s and then 
                cur_point[0]+=xdif_abs
                map[cur_point[0]][cur_point[1]] = 1
                scale_int = int(scale)
                for scale_val in range(scale_int):
                    cur_point[1]+=ydif_abs
                    map[cur_point[0]][cur_point[1]] = 1
            elif(scale<1):
                cur_point[1]+=ydif_abs
                map[cur_point[0]][cur_point[1]] = 1
                scale_int = int(1/scale)
                for scale_val in range(scale_int):
                    cur_point[0]+=xdif_abs
                    map[cur_point[0]][cur_point[1]] = 1
            else:
                map[cur_point[0]][cur_point[1]] = 1
                map[cur_point[0]+xdif_abs][cur_point[1]] = 1
                map[cur_point[0]][cur_point[1]+ydif_abs] = 1
                cur_point=[cur_point[0]+xdif_abs,cur_point[1]+ydif_abs]



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
        for aa in m:
            for bb in aa:
                print(bb,end='',sep='')
            print()
        break

        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
