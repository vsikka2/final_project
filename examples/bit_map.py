
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
            points.append[(x,y)]
        i+=1
        cur_angle+=angle_step
    return points
def get_map_from_distances(distance):
    points = get_points_from_distances(distance)
    map = np.zeros((100,100))
    for i in points:
        map[i[0]][i[1]] = 1
    return map



def main():
    while True:
        scan_list = fc.make_distance_list()
        if not scan_list:
            continue
        print(scan_list)
        map = get_map_from_distances(scan_list)
        print(map)
        break
    fc.forward()
    sleep(2)
    fc.stop()
    while True:
        scan_list = fc.make_distance_list()
        if not scan_list:
            continue
        print(scan_list)
        break
    

        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
