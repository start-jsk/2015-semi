#!/usr/bin/env python

import roslib
roslib.load_manifest('baxter_vision_example')
import rospy
import math
import tf

if __name__ =='__main__':
    rospy.init_node('baxter_tf_listener')
    trans = []
    rot = []

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('right_gripper','base',rospy.Time(0))
               
        except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
            pass
        
        if (trans and rot):
            print 'trans: %f %f %f ' % (trans[0], trans[1], trans[2])
            print 'rot  : %f %f %f %f ' % (rot[0], rot[1], rot[2],rot[3])

        rate.sleep()
