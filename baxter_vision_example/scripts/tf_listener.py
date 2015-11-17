#!/usr/bin/env python

import roslib
roslib.load_manifest('baxter_vision_example')
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class tf_listener:
    def __init__ (self):
        rospy.init_node('baxter_tf_listener')
        self.publisher = rospy.Publisher('base_to_right_gripper_tf',PoseStamped,queue_size=1)
        self.rate = rospy.Rate(10.0)

    def print_tf(self):
        if (self.trans and self.rot):
           print 'trans: %f %f %f ' % (self.trans[0], self.trans[1], self.trans[2])
           print 'rot  : %f %f %f %f ' % (self.rot[0], self.rot[1], self.rot[2],self.rot[3])
 
    def listener(self,to_tf, from_tf):
        tflistener = tf.TransformListener()
        trans = []
        rot = []
        while not rospy.is_shutdown():
            try:
                (trans,rot) = tflistener.lookupTransform(to_tf,from_tf,rospy.Time(0))
                self.publishPoseStamped(trans,rot)
               
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                pass

            #self.print_tf()
            self.rate.sleep()

    def publishPoseStamped(self,trans,rot):
        pub = PoseStamped()
        pub.header.frame_id = "/base_to_right_gripper_tf"
        pub.header.stamp = rospy.Time.now()
        pub.pose.position.x = trans[0]
        pub.pose.position.y = trans[1]
        pub.pose.position.z = trans[2]
        pub.pose.orientation.x = rot[0]
        pub.pose.orientation.y = rot[1]
        pub.pose.orientation.z = rot[2]
        pub.pose.orientation.w = rot[3]
        self.publisher.publish(pub)

if __name__ =='__main__':
    tfl = tf_listener()
    tfl.listener('right_gripper_base','base')

