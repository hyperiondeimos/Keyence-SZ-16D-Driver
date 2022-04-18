#!/usr/bin/env python3

# Script for simulate random points in a list
import random
import time

class KeyenceSZ16Dping_Test:

    def __init__(self):
        pass

    def tick(self):
        points = []
        min_distance = 3200
        max_distance = 4000
        points = random.sample(range(min_distance, max_distance), 751)

        # convert to float
        points_f = []
        for i in range(len(points)):
            # points_f.append(float(points[i]))
            points_f.append(float(points[i] / 1000.0))
        # return a list of points
        # return points # if you want to return a list of points (int)
        return points_f # if you want to return a list of points (float)

if __name__ == '__main__':
    app = KeyenceSZ16Dping_Test()
    while True:
        data = app.tick()
        print(data)
        # print(type(data))
        # print(type(data[0]))
        print(len(data))
        time.sleep(1)
