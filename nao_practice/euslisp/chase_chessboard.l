#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://naoeus/nao-interface.l")

(nao-init)
(setq *tfl* (instance ros::transform-listener :init))

(setq *camera-coords* (send *nao* :camera 
