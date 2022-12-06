
import picar_4wd as fc
import numpy as np
import sys
import heapq
from time import sleep


def main():
    while True:
        dist = fc.get_distance_at(90)
        if(dist!=-2):
            print(dist)
        sleep(5)

        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
