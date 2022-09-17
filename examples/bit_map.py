
import picar_4wd as fc
import numpy as np
import sys
import heapq
from time import sleep
np.set_printoptions(threshold = sys.maxsize)
speed = 30
ANGLE_STEP = 10
CAR_START = [50,0]
MIN_DISTANCE_LINE = 3500
DESTINATION = [20,20]
MIN_MOVE_DISTANCE = 10
CAMERA_RESCAN_DIST = 50
def get_points_from_distances(distance):
    points = []
    cur_angle = -90
    i=0
    while(cur_angle<=90):
        if(distance[i]> 0):
            x = int(distance[i]*np.sin(cur_angle*np.pi/180.0)+CAR_START[0])
            y = int(distance[i]*np.cos(cur_angle*np.pi/180.0))
            if(x>-1 and x<100 and y>-1 and y<100):        
                points.append((int(x),int(y)))
        i+=1
        cur_angle+=ANGLE_STEP
    return points
def get_map_from_distances(distance):
    points = get_points_from_distances(distance)
    map = np.zeros((100,100))
    map[50][0] = -1
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
        if(xdif*xdif+ydif*ydif > MIN_DISTANCE_LINE):
            i+=1
            continue
        scale = float(ydif/xdif)
        cur_point = [point1[0],point1[1]]
        while(cur_point[0] != point2[0] or cur_point[1] != point2[1]):
            xdif = np.abs(cur_point[0]-point2[0])
            ydif = np.abs(cur_point[1]-point2[1])
            if(xdif == 0):
                for yval in range(cur_point[1],point2[1]+ydif_abs,ydif_abs):
                    map[cur_point[0]][yval] = 1
                cur_point = point2
                continue
            if(ydif == 0):
                for xval in range(cur_point[0],point2[0]+xdif_abs,xdif_abs):
                    map[xval][cur_point[1]] = 1
                cur_point = point2
                continue
            

    
            scale = float(ydif/xdif)
            
            if(scale>1):
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

def setCameraPos():
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue
        break

def getMap(right):
    scan_map = []
    for i in range(right*90,-right*91,-right*ANGLE_STEP):
        dist = fc.get_distance_at(i)
        if(dist == -2):
            scan_map.append(200)
        else:
            scan_map.append(dist)        
    m = get_map_from_distances(scan_map)
    m = np.array(m, dtype = int)
    return m
def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def local_destination():
    #has DESTINATION which is with respect to CAR_START 
    # need to get point in 100,100 which is equivalent
    local = [DESTINATION[0]+CAR_START[0],DESTINATION[1]+CAR_START[1]]
    if(local[0]<100 and local[1]<100 and local[0]>=0 and local[1]>=0):
        return local
    if(local[1]-CAR_START[1] == 0):
        if(local[0]-CAR_START[0]>0):
            return [99,CAR_START[1]]
        else:
            return [0,CAR_START[1]]
    if(local[0]-CAR_START[0]==0):
        return [50,99]
    
    slope = (local[1]-CAR_START[1])/(local[0]-CAR_START[0])
    y_for_x_100 = int(slope*49)
    x_for_y_100 = int(99.0/slope+50)
    if(x_for_y_100 <100 and x_for_y_100 >=0):
        return [x_for_y_100,99]
    elif(y_for_x_100>0):
        return [99,y_for_x_100]
    else:
        return [0,-y_for_x_100]
    return local

def astar(array):
    start = (CAR_START[0],CAR_START[1])

    goal = local_destination()
    goal = (goal[0],goal[1])
    neighbors = [(0,MIN_MOVE_DISTANCE),(0,-MIN_MOVE_DISTANCE),(MIN_MOVE_DISTANCE,0),(-MIN_MOVE_DISTANCE,0)]

    close_set = set()

    came_from = {}

    gscore = {start:0}

    fscore = {start:heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    while oheap:

        current = heapq.heappop(oheap)[1]
        if current == goal:

            data = []

            while current in came_from:

                data.append(current)

                current = came_from[current]
            data.reverse()
            path=[]
            for i in range(0,CAMERA_RESCAN_DIST,MIN_MOVE_DISTANCE):
                if(i <len(data)):
                    path.append(data[i])
            return path
        close_set.add(current)
        for i, j in neighbors:

            neighbor = current[0] + i, current[1] + j            
            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:

                if 0 <= neighbor[1] < array.shape[1]:                

                    if array[neighbor[0]][neighbor[1]] == 1:

                        continue

                else:
                    
                    # array bound y walls

                    continue

            else:

                # array bound x walls

                continue
 

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):

                continue
 

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:

                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal) 

                heapq.heappush(oheap, (fscore[neighbor], neighbor)) 

def traverse_path(path):
    cur_location = CAR_START
    facing = 1
    #facing 0 is left 1 is front 2 is right 3 is back
    for i in path:
        cur_movement = 0
        if(i[0]-cur_location[0]>0):
            if(facing == 1):
                fc.turn_right(speed)
            facing = 2
            cur_movement = i[0]-cur_location[0]
            DESTINATION[0] -=cur_movement
        elif(i[0]-cur_location[0]<0):
            if(facing == 1):
                fc.turn_left(speed)
            facing = 0
            cur_movement = cur_location[0] - i[0]  
            DESTINATION[0] +=cur_movement
        else:
            if(facing == 2):
                fc.turn_left(speed)
            if(facing == 0):
                fc.turn_right(speed)
            facing = 1
            cur_movement = i[1]-cur_location[1]
            DESTINATION[1] -=cur_movement
        
        fc.forward(cur_movement)
        sleep(1)
        fc.stop()
        cur_location = i
    
def main():
    setCameraPos();    
    right = -1
    while True:
        bit_map = getMap(right)
        right*=-1
        
        path = astar(bit_map)
        print(path)
        traverse_path(path)
        for x in bit_map:
            for y in x:
                print(y,end='',sep='')
            print()
        break
        if(DESTINATION==[0,0]):
            break

        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
