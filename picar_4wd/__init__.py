#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from picar_4wd.pin import Pin
from picar_4wd.ultrasonic import Ultrasonic 

us = Ultrasonic(Pin('D8'), Pin('D9'))

def get_distance_at(angle):
    time.sleep(0.04)
    distance = us.get_distance()
    return distance
if __name__ == '__main__':
    a=1