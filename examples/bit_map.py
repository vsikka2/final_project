
import picar_4wd as fc
import numpy as np
speed = 30
car = (50,0)
def get_points_from_distances(distance):
    points = []
    angle_step = 180/len(distance)
    cur_angle = -90
    i=0
    while(cur_angle<=90):
        if(distance[i]> 0):
            x = distance[i]*np.sin(cur_angle*np.pi/180.0)+car[0]
            y = distance[i]*np.cos(cur_angle*np.pi/180.0)
            points.append((x,y))
        i+=1
        cur_angle+=angle_step
    return points
def get_map_from_distances(distance):
    points = get_points_from_distances(distance)
    map = np.zeros((100,100))
    for i in points:
        map[i[0]][i[1]] = 1
    i=0
    while(i<len(points) - 1):
        point1 = points[i]
        point2 = points[i+1]
        xdif = np.abs(point1[0]-point2[0])
        ydif = np.abs(point1[1]-point2[1])

        i+=1
    return map



def main():
    
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue
        break
        
    while True:
        scan_map = []
        for i in range(15,90,5):           
            scan_list=[]
            while(not scan_list):
                scan_list = fc.scan_step(i)    
            if(0 in scan_list):
                scan_map.append(0)
            elif(1 not in scan_list):
                scan_map.append(100)
            else:
                idx = scan_list.index(1)
                scan_map.append(15+idx*5)
            #scan_map.append(scan_list)
        print(scan_map)
        #map = get_map_from_distances(scan_list)
        #print(map)
    

        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()