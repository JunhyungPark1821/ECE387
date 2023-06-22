#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter',String,queue_size=10)
    rospy.init_node('talker',anonymous=False)
    rate = rospy.Rate(10) #10hz
    cnt = 0
    while not rospy.is_shutdown():
        #Create a var to store a basic string to publish
        chat_str = "Hello World, this is my first ROS node."
        #Concatenate to the string a message to show you how many times it was published
        chat_str = chat_str + "This message published %s times" % cnt
        cnt = cnt+1
        #Publish the string to the 'chatter' topic
        pub.publish(chat_str)
        #Pause for .1 second based on the rate set above
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass