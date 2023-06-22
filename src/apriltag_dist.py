#!/usr/bin/env python3

# C3C Junhyung Park and C3C Ryan Buckton
# 04102023 Finished Checkpoint 2
# 04162023 Finished Lab 4

import rospy, cv2, dlib

#TODO Import the appropriate AprilTag message
from apriltag_ros.msg import AprilTagDetectionArray

class ApriltagDetector(object):
    SCALE = 50

    def __init__(self):
        self.ctrl_c = False
        #TODO Subscribe to the tag_detections topic
        self.tag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)

        rospy.on_shutdown(self.shutdownhook)

    # Linear relationship, so we scaled it by 50 based on experiments
    def calc_dist(self,raw):
        return raw*self.SCALE
    
    #TODO: Print the identified AprilTag ID and distance
    def tag_callback(self,data):
        if not self.ctrl_c:
            for tag in data.detections:
                print(tag.id[0])
                act_dist = self.calc_dist(tag.pose.pose.pose.position.z)
                print(act_dist)

    #Shut down
    def shutdownhook(self):
        print("Shutting down")
        self.ctrl_c = True
    #    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('apriltag_dist')
    apriltag_dist_detector = ApriltagDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass