
import picar_4wd as fc

speed = 30

def main():
    while True:
        scan_list = fc.make_distance_list()
        if not scan_list:
            continue
        print(scan_list)
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
