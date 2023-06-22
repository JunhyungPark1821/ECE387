#!/usr/bin/env python3

# C3C Junhyung Park and C3C Ryan Buckton
# 03072023: Began working ICE8

import rospy, math
# TODO: import correct message
from sensor_msgs.msg import LaserScan


# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg

class LIDAR:    
    """Class to read lidar data from the Turtlebot3 LIDAR"""
    def __init__(self):
        self.rangeCount = 0 # Variable to count the ranges 30 degrees off the nose of the robot
        self.rangeSum = 0 # Variable to sum the ranges 30 degrees off the nose of the robot
        
        # TODO: create a subscriber to the scan topic published by the lidar launch file
        rospy.Subscriber('scan', LaserScan, self.callback_lidar)
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def callback_lidar(self, scan):
        if not self.ctrl_c:
            degrees = []
            ranges = []

            self.rangeCount = 0 # Reset
            self.rangeSum = 0 # Reset
            
            # determine how many scans were taken during rotation
            # count = int(scan.scan_time / scan.time_increment) #problem line, divide by zero issue. Use len(ranges) to find number of scans.
            # Or... Do (anglemax - anglemin)/angleincrement?
            count = len(scan.ranges)
            
            for i in range(count):
                # using min angle and incr data determine curr angle, 
                # convert to degrees, convert to 360 scale
                degrees.append(int(DEG_CONV(RAD2DEG(scan.angle_min + scan.angle_increment*i))))
                rng = scan.ranges[i]
                
                # ensure range values are valid; set to 0 if not
                if rng < scan.range_min or rng > scan.range_max:
                    ranges.append(0.0)
                else:
                    ranges.append(rng)
            
            # python way to iterate two lists at once!
            for deg, rng in zip(degrees, ranges):
                # TODO: sum and count the ranges 30 degrees off the nose of the robot
                if deg >= 345 or deg <= 15:
                    self.rangeCount += 1
                    self.rangeSum += rng

                
            # TODO: ensure you don't divide by 0 and print average off the nose
            if self.rangeCount == 0:
                raise Exception("The number of ranges 30 degrees off the nose of the robot is 0")
            else:
                print(self.rangeSum/self.rangeCount)
                
    def shutdownhook(self):
        print("Shutting down lidar subscriber")
        self.ctrl_c = True
        
if __name__ == '__main__':
    rospy.init_node('lidar_sub')
    LIDAR()
    rospy.spin()