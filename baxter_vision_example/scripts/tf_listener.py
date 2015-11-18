#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('baxter_vision_example')
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class tf_listener:
    def __init__ (self):
        rospy.init_node('baxter_tf_listener')
        self.rate = rospy.Rate(10.0)

    def print_tf(self):
        if (self.trans and self.rot):
           print 'trans: %f %f %f ' % (self.trans[0], self.trans[1], self.trans[2])
           print 'rot  : %f %f %f %f ' % (self.rot[0], self.rot[1], self.rot[2],self.rot[3])
 
    def listener(self,to_tf, from_tf):
        tflistener = tf.TransformListener()
        trans = []
        rot = []
        topic_name = u"/"+ from_tf + u"_to_" + to_tf + u"_tf"
        self.publisher = rospy.Publisher(topic_name,PoseStamped,queue_size=1)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = tflistener.lookupTransform(to_tf,from_tf,rospy.Time(0))
                self.publishPoseStamped(from_tf,trans,rot)
               
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                pass

            #self.print_tf()
            self.rate.sleep()

    def publishPoseStamped(self,from_tf,trans,rot):
        pub = PoseStamped()
        pub.header.frame_id = from_tf
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
    if (len(sys.argv) != 3):
        print "Usage: python tf_listener.py to_tf from_tf"
    else:
        tfl.listener(sys.argv[1],sys.argv[2])

