This is turtlebot program avoids objects and reaches goal destination.

How to use :
1.
roscore

2.
roslaunch turtlebot_bringup minimal.launch

3.
roslaunch turtlebot_bringup 3dsensor.launch

4.
go to /src/
python nao_depth_odom.py

The other programs are trivial. you can erase all python files exept transformations.pyc (Copyright (c) 2006, Christoph Gohlke)
.


The problem remaining : finding nao is not done yet.
			This is good for avoiding objects, but not good for reach goal destination. Much better algorithm is required.
			Maybe reinforcement learning for weight vector produce better performance
