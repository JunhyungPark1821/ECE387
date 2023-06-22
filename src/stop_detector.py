#!/usr/bin/env python3

# C3C Junhyung Park and C3C Ryan Buckton
# 04102023 Finished Checkpoint 2

import rospy, cv2, dlib
from cv_bridge import CvBridge, CvBridgeError

# TODO: import usb_cam message type
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class StopDetector(object):
    FOCAL = 430
    STOP_WIDTH = 13.0
    DISTANCE = 0.0

    def __init__(self, detectorLoc):
        self.ctrl_c = False
        
        #TODO: create subscriber to usb_cam image topic
        # subscribe to the topic created by the usb_cam node
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)

        # Create a new publisher that will publish the distance using Float32 std_msgs messages over the /stop_dist topic.
        self.pub = rospy.Publisher('stop_dist', Float32, queue_size = 1)
        
        self.bridge_object = CvBridge()
        self.detector = dlib.simple_object_detector(detectorLoc)

        rospy.on_shutdown(self.shutdownhook)

    # Apply the distance equation
    def calc_dist(self, box):
        return (self.STOP_WIDTH*self.FOCAL)/(box.right()-box.left())

    def camera_callback(self,data):
        if not self.ctrl_c:
            #TODO: write code to get ROS image, convert to OpenCV image,
            # apply detector, add boxes to image, and display image
            
            # convert ROS image to OpenCV image
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # Convert it to gradient
            self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

            gX = cv2.Sobel(self.cv_image,cv2.CV_64F,1,0)
            gY = cv2.Sobel(self.cv_image,cv2.CV_64F,0,1)

            gX = cv2.convertScaleAbs(gX)
            gY = cv2.convertScaleAbs(gY)
            self.cv_image = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)
            boxes = self.detector(self.cv_image)

            # loop over the bounding boxes and draw them
            for b in boxes:
                (x, y, w, h) = (b.left(), b.top(), b.right(), b.bottom())
                cv2.rectangle(self.cv_image, (x, y), (w, h), (255, 0, 0), 2)
                self.DISTANCE = self.calc_dist(b)
                self.pub.publish(self.DISTANCE)

            cv2.waitKey(1)
            # show the image (waitKey(1) allows for automatic refressh creating video)
            cv2.imshow('image', self.cv_image)

    def shutdownhook(self):
        print("Shutting down")
        self.ctrl_c = True
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
    rospy.init_node('stop_detector')
    detector = rospy.get_param("/stop_detector/detector")
    stop_detector = StopDetector(detector)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass