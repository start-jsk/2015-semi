#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def pub_input_key():
    rospy.init_node('send_input_key', anonymous=True)
    pub = rospy.Publisher('input_key', String, queue_size=10)
    while not rospy.is_shutdown():
        str = "%s"%raw_input()
        rospy.loginfo(str)
        pub.publish(str)

if __name__ == '__main__':
    try:
        pub_input_key()
    except rospy.ROSInterruptException: pass

