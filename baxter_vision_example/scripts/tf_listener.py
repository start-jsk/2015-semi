#!/usr/bin/env python

import roslib
roslib.load_manifest('baxter_vision_example')
import rospy
import tf

class tf_listener:
    def __init__ (self):
         rospy.init_node('baxter_tf_listener')
        
    def listener(self,from_tf, to_tf):
        trans = []
        rot = []

        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(from_tf,to_tf,rospy.Time(0))
               
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                pass
        
            if (trans and rot):
               print 'trans: %f %f %f ' % (trans[0], trans[1], trans[2])
               print 'rot  : %f %f %f %f ' % (rot[0], rot[1], rot[2],rot[3])
            rate.sleep()

if __name__ =='__main__':
    tfl = tf_listener()
    tfl.listener('base','right_gripper')


