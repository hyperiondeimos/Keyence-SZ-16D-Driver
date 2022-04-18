#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from sz16d import KeyenceSZ16Dping
from laser_geometry import LaserProjection
from fake_laser_data import KeyenceSZ16Dping_Test
import time


class Laser:
    def __init__(self) -> None:
        self.laser_freq = 66.0e-06 # 66ms
        self.num_readings = 751 # 751 readings points
        self.intensities = [self.num_readings]
        # real data
        # self.lidar = KeyenceSZ16Dping()
        # fake data
        self.lidar = KeyenceSZ16Dping_Test()

        self.ros_init()
    
    def ros_init(self):
        try:
            rospy.init_node('keyence_scan', anonymous=True)
            rospy.loginfo("Node Python Started...")
            laser_sub = "/keyence_scan"
            # self.sub = rospy.Subscriber("/scan", LaserScan, self.cb_laser_listener)
            self.pub = rospy.Publisher(laser_sub, LaserScan, queue_size=10)
            self.rate = rospy.Rate(1.0) # 1 Hz
            # point cloud convertion
            self.laserProj = LaserProjection()
            pc_pub = "/keyence_PointCloud"
            self.pc_pub = rospy.Publisher(pc_pub, pc2, queue_size=10)
            self.laser_sub = rospy.Subscriber(laser_sub, LaserScan, callback=self.laser_cb)

            while not rospy.is_shutdown():
                self.cb_laser_listener()
                self.rate.sleep()
            
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Node terminated...")

    def cb_laser_listener(self) -> None:
        scanner = LaserScan()
        current_time = rospy.Time.now()
        scanner.header.stamp = current_time
        scanner.header.frame_id = "laser_frame"
        scanner.angle_min = -0.785398 # -45 degrees
        scanner.angle_max = 3.92699 # 225 degrees
        scanner.angle_increment = 3.14 / self.num_readings
        # scanner.angle_increment = 0.005483909574468 # 270 degrees -> 4.71239 rads / 752 points 
        scanner.time_increment = float(1 / self.laser_freq / self.num_readings) 
        scanner.range_min = 0.004 # 40mm
        scanner.range_max = 10.0 # 10000mm
        laser_values_list = list(self.lidar.tick())
        # scanner.ranges = list(x / 1000.0 for x in laser_values_list) # take the values from physical lidar
        scanner.ranges = laser_values_list
        scanner.scan_time = (1 / self.laser_freq)

        # resize the lists
        # scanner.ranges = scanner.ranges[:self.num_readings]
        # scanner.intensities = scanner.intensities[:self.num_readings]

        # TEST ONLY
        # for i in range(self.num_readings):
        #     self.intensities[i] = 300.0
        #     scanner.intensities.append(300.0)

        self.pub.publish(scanner)

    def laser_cb(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        self.pc_pub.publish(cloud_out)


def main():
    app = Laser()
    # rospy.spin()

if __name__ == "__main__":
    main()
