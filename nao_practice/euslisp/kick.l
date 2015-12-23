#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://naoeus/nao-interface.l")

(ros::roseus "naotalk")
(ros::advertise "speech" std_msgs::string 1)
(ros::rate 10)

(nao-init)

(setq kick0 #f(110.0 70.0 -90.0 -40.0 0.0 110.0 -10.0 90.0 40.0 0.0 0.0 -15.0 0.0 0.0 0.0 15.0 0.0 -15.0 0.0 0.0 0.0 15.0 0.0 0.0))
(setq kick1 #f(110.0 70.0 -90.0 -40.0 0.0 110.0 -10.0 90.0 40.0 0.0 0.0 0.0 0.0 0.0 0.0 15.0 0.0 -15.0 -35.0 70.0 -35.0 15.0 0.0 0.0))
;;(setq kick2 #f(110.0 70.0 -90.0 -40.0 0.0 110.0 -10.0 90.0 40.0 0.0 0.0 0.0 -20.0 0.0 0.0 15.0 0.0 -15.0 25.0 70.0 -35.0 15.0 0.0 0.0))
(setq kick2 #f(0.0 50.0 0.0 -20.0 -20.0 110.0 -10.0 90.0 40.0 0.0 0.0 0.0 -20.0 0.0 0.0 15.0 0.0 -15.0 25.0 70.0 -35.0 15.0 0.0 0.0))
;;(setq kick3 #f(110.0 40.0 0.0 -20.0 0.0 110.0 -10.0 90.0 40.0 0.0 0.0 0.0 10.0 0.0 0.0 15.0 0.0 0.0 -40.0 0.0 20.0 0.0 0.0 0.0))
(setq kick3 #f(110.0 20.0 -90.0 -10.0 0.0 110.0 -10.0 90.0 40.0 0.0 0.0 0.0 0.0 0.0 0.0 15.0 0.0 0.0 -40.0 0.0 20.0 0.0 0.0 0.0))

(defun kick ()
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "キックします" (send (ros::time-now) :sec-nsec)))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "speech" msg)
  (send *ri* :angle-vector kick0 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector kick1)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector kick2)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector kick3 500)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector kick1)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector kick0 1000)
  (send *ri* :wait-interpolation)
  )

(defun lookaround()
  (send *nao* :head :angle-vector #f(0 25))
  (send *ri* :angle-vector (send *nao* :angle-vector))
  (send *ri* :wait-interpolation))

(defun nao-default-pose ()
  (send *nao* :reset-pose)
  (send *nao* :lleg :angle-vector #f(0 0 0 0 0 0))
  (send *nao* :rleg :angle-vector #f(0 0 0 0 0 0))
  (send *ri* :angle-vector (send *nao* :angle-vector))
  )

