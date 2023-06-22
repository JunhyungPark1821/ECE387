#!/usr/bin/env python3

# C3C Junhyung Park and Ryan Buckton
# 04042023 Changed the image to grayscale and gradient

import rospy, cv2, argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SavingImage(object):
    def __init__(self, img_dest):
        self.img_dest = img_dest
        self.ctrl_c = False
        self.count = 0
        
        # subscribe to the topic created by the usb_cam node
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)
        
        # CV bridge converts between ROS Image messages and OpenCV images
        self.bridge_object = CvBridge()
        
        # callback to save images when user presses button
        rospy.Timer(rospy.Duration(.1), self.callback_save)
        
        rospy.on_shutdown(self.shutdownhook)

    def camera_callback(self,img):
        if not self.ctrl_c:
            try:
                # convert ROS image to OpenCV image
                self.cv_image = self.bridge_object.imgmsg_to_cv2(img, desired_encoding="bgr8")

                # Convert it to gradient
                self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

                gX = cv2.Sobel(self.cv_image,cv2.CV_64F,1,0)
                gY = cv2.Sobel(self.cv_image,cv2.CV_64F,0,1)

                gX = cv2.convertScaleAbs(gX)
                gY = cv2.convertScaleAbs(gY)
                self.cv_image = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)
            except CvBridgeError as e:
                print(e)
            
            # show the image (waitKey(1) allows for automatic refressh creating video)
            cv2.imshow('image', self.cv_image)
            cv2.waitKey(1)
        
    def callback_save(self, event):
        # when user is ready to take picture press button
        _ = input("Press enter to save the next image.")
        dest = self.img_dest + "img" + str(self.count) + ".jpg"
        self.count += 1
        print(dest)
        try:
            # write to file
            cv2.imwrite(dest, self.cv_image)
        except:
            print("Not valid image name. Try restarting with valid path.")
            
    def shutdownhook(self):
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous=True)
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", required=True, help="path to output img")
    args = vars(ap.parse_args())
    saving_image_object = SavingImage(args["output"])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
