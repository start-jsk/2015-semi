##how to use it:
#1.minimal.launch
#2.roslaunch turtlebot_bringup 3dsensor.launch
#3.this
##changed algorithm

#!/usr/bin/env python
import roslib

import sys, time
import rospy
import cv2
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from kobuki_msgs.msg import MotorPower
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import Sound
import numpy as np
from scipy import signal
import transformations

key=0

size_x=640
size_y=480
odom_key=True


class odom_nav:
    def __init__(self,x_goal,y_goal):
        self.x_goal=x_goal
        self.y_goal=y_goal
        #####################
        self.odom_init_v=0.1
        self.odom_init_w=0.3
        #####################
        print self.x_goal,self.y_goal


    def calc_vel(self,pos_x,pos_y,quater):
        global odom_key
        if odom_key==True:
            self.first_x=pos_x
            self.first_y=pos_y
            self.x_goal+=self.first_x
            self.y_goal+=self.first_y
            odom_key=False
        
        rel_x=self.x_goal-pos_x
        rel_y=self.y_goal-pos_y
        dest_angle=math.atan2(rel_y,rel_x)
        quaterArr=np.array([quater.x,quater.y,quater.z,quater.w])
        robot_euler=transformations.euler_from_quaternion(quaterArr,'sxyz')
        robot_angle=robot_euler[2]
        angle=dest_angle-robot_angle
        if math.sqrt(rel_x**2+rel_y**2) < 0.1:
            self.v=0.0;
        else :
            self.v=self.odom_init_v


        if angle > 0.3 :
            self.w=self.odom_init_w
        elif angle < -0.3:
            self.w=-self.odom_init_w
        else :
            self.w=0.0

        return (self.v,self.w)


class nao_finder:
    def __init__(self):
        self.neuron = np.zeros(size_x)
        self.neuron_w = 1 - signal.gaussian(size_x,std=150)
        self.neuron_w = self.neuron_w.reshape(size_x,)
        self.alpha = 0.00005
        self.v0=0.0


    def calc_neuron(self,array):

        self.neuron = self.alpha*sum(array**2)
        self.neuron = self.neuron.reshape(size_x,)
        self.vel_right_inc=np.dot(self.neuron[:size_x/2],self.neuron_w[:size_x/2])
        self.vel_left_inc=np.dot(self.neuron[size_x/2:],self.neuron_w[size_x/2:])
        self.vel_right_wheel = self.v0 + self.vel_right_inc
        self.vel_left_wheel = self.v0 +  self.vel_left_inc
        self.v = (self.vel_right_wheel+self.vel_left_wheel) / 2.0
        self.w = (self.vel_right_wheel-self.vel_left_wheel) / 2.0
        return (self.v,self.w)


        
class depth_finder:
    def __init__(self):
        self.neuron = np.zeros(size_x)
        self.neuron_w = signal.gaussian(size_x,std=500)
        self.neuron_w = self.neuron_w.reshape(size_x,)
        self.alpha = 0.000005
        self.v0=0.5

        self.array_biase=1.0

    def calc_neuron(self,array):
        inv_array=1.0/array
        self.neuron = self.alpha*sum(inv_array)
        self.neuron = self.neuron.reshape(size_x,)
        self.vel_right_dec=np.dot(self.neuron[:size_x/2],self.neuron_w[:size_x/2])
        self.vel_left_dec=np.dot(self.neuron[size_x/2:],self.neuron_w[size_x/2:])
        self.vel_right_wheel = self.v0 - self.vel_right_dec
        self.vel_left_wheel = self.v0 - self.vel_left_dec
        self.v = (self.vel_right_wheel+self.vel_left_wheel) / 2.0
        self.w = (self.vel_right_wheel-self.vel_left_wheel) / 2.0
        return (self.v,self.w)

        

class img_processor:
  
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1000)
        self.sound_pub = rospy.Publisher("mobile_base/commands/sound",Sound,queue_size= 1 )
        cv2.namedWindow("Mask Image", 1)
        self.bridge = CvBridge()
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.callback_odom)

        self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.callback_color)
        self.depth_sub = rospy.Subscriber("camera/depth_registered/image",Image,self.callback_depth)

        self.pub_motor_move=rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=100)
        self.pub_motor_power=rospy.Publisher('mobile_base/commands/motor_power',MotorPower,queue_size=10)
        self.pub_motor_power.publish(1)

        ######################Configure parameters here############################
        self.array_biases = 1.0
        self.nao_v_config=0.0
        self.nao_w_config=0.0
        self.depth_v_config=1.0
        self.depth_w_config=5.0
        self.odom_v_config=1.0
        self.odom_w_config=3.5
        self.odom_v_key=1


        self.danger_v = 0.0
        self.danger_w = 0.5

        self.max_vel = 0.15
        self.max_wel = 0.3
        #        self.lower_blue = np.array([27,150,100])##yellow block
        #        self.upper_blue = np.array([28,255,255])
        #        self.lower_blue = np.array([107,170,100]) ##this is for nao
        #        self.upper_blue = np.array([108,220,250])
        self.lower_blue = np.array([10,170,100])##mikan
        self.upper_blue = np.array([20,220,255])

        self.nao_find_param = 99999999999999
        self.goal_dest_param = 0.5        


        self.goal_dest_found = False
        self.nao_found = False

        ##########################################################
    def callback_odom(self,data):
        self.pos_x=data.pose.pose.position.x
        self.pos_y=data.pose.pose.position.y
        self.quater=data.pose.pose.orientation


    def callback_color(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv_image,self.lower_blue,self.upper_blue)
        self.mask_filtered = cv2.medianBlur(self.mask,3)

        cv2.waitKey(3)

    def callback_depth(self,data):
        depth_image = self.bridge.imgmsg_to_cv2(data, '16UC1')
        self.depth_array = np.array(depth_image) + self.array_biases
        self.depth_array = np.nan_to_num(self.depth_array)

        
        blue_depth = cv2.bitwise_not(depth_image,depth_image,mask= self.mask_filtered)


        self.blue_array = np.array(blue_depth)
        self.blue_array = np.nan_to_num(self.blue_array)




        self.blue_nao_find =  -self.blue_array+ .5
        
        cv2.imshow("Mask Image",self.blue_nao_find)

        self.move_robot()
        cv2.waitKey(3)

    def recog_environment(self,odom_v,odom_w,nao_near_check,nao_w,depth_v,depth_w):
        goal_dest = math.sqrt((x_goal - self.pos_x) ** 2 + (y_goal - self.pos_y) ** 2)
        if self.danger_v > depth_v:
            self.odom_v_key = 0
            self.odom_w_key = 0
        else :
            self.odom_v_key = 1
            self.odom_w_key = 1

        

        if self.goal_dest_param > goal_dest :##goal destination reached
            
            self.odom_v_key=0
            self.odom_w_key=0
            self.nao_v_config=0.0
            self.nao_w_config=0.0
            self.depth_v_config=0.0
            self.depth_w_config=0.0



            if self.goal_dest_found == False :
                rospy.sleep(0.1)
                self.sound_pub.publish(5)
                self.goal_dest_found = True

            print "goal destination reached!"
            return


        if self.nao_find_param < nao_near_check :####nao reached
            self.nao_v_config=0.0
            self.nao_w_config=0.0
            self.depth_v_config=0.0
            self.depth_w_config=0.0
            self.odom_v_key=0.0
            self.odom_w_key=0.0
            if self.nao_found == False :
                rospy.sleep(0.1)
                self.sound_pub.publish(6)
                self.nao_found = True
                
            print "nao reached!"
            return


        else :
            
            return

        

    def move_robot(self):
        odom_v,odom_w=odometry.calc_vel(self.pos_x,self.pos_y,self.quater)
        nao_v,nao_w=nao.calc_neuron(self.blue_array)
        depth_v,depth_w=depth.calc_neuron(self.depth_array)
        nao_near_check, dummy =nao_near.calc_neuron(self.blue_nao_find)
        

        self.recog_environment(odom_v,odom_w,nao_near_check,nao_w,depth_v,depth_w)        
        self.odom_v_config = self.odom_v_key*depth_v*8.0
        self.odom_w_config = self.odom_v_key*1 / (100.0 * depth_w)

        print nao_near_check,dummy


        self.v = self.nao_v_config * nao_v +self.depth_v_config * depth_v + self.odom_v_config * odom_v
        self.w = self.nao_w_config * nao_w +self.depth_w_config * depth_w + self.odom_w_config * odom_w

        if self.w > self.max_wel:
            self.w = self.max_wel
        if self.w < -self.max_wel:
            self.w = -self.max_wel

        if self.v > self.max_vel:
            self.v = self.max_vel
        if self.v < -self.max_vel:
            self.v = -self.max_vel
        robot_vel=Vector3(self.v,0,0)
        robot_w=Vector3(0,0,self.w)
        twist=Twist(robot_vel,robot_w)

        ###################################################
        self.pub_motor_move.publish(twist)
        ####################################################

        rospy.sleep(0.1)



def main(args):
    rospy.init_node('complex_navigator', anonymous=True)
    ic = img_processor()
    global nao
    global depth
    global odometry
    global nao_near
    global x_goal
    global y_goal
    x_goal=float(args[1])
    y_goal=float(args[2])
    nao = nao_finder()
    nao_near = depth_finder()
    depth = depth_finder()
    odometry = odom_nav(x_goal,y_goal)
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
