#!/usr/bin/env python

import roslib
roslib.load_manifest('baxter_vision_example')
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class tf_listener:
    def __init__ (self):
        rospy.init_node('baxter_tf_listener')
        self.pub = rospy.Publisher('base_to_right_gripper_tf',PoseStamped,queue_size=1)
        self.rate = rospy.Rate(10.0)
        self.trans = []
        self.rot = []
 
    def print_tf(self):
        if (self.trans and self.rot):
           print 'trans: %f %f %f ' % (self.trans[0], self.trans[1], self.trans[2])
           print 'rot  : %f %f %f %f ' % (self.rot[0], self.rot[1], self.rot[2],self.rot[3])
 
    def listener(self,from_tf, to_tf):
        self.listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                (self.trans,self.rot) = self.listener.lookupTransform(from_tf,to_tf,rospy.Time(0))
               
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                pass

            self.print_tf()
            self.rate.sleep()

#def publish(self,trans,rot):
              


if __name__ =='__main__':
    tfl = tf_listener()
    tfl.listener('base','right_gripper')

