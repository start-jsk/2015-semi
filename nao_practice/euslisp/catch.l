;;roslaunch jsk_pcl_ros hsi_color_filter.launch DEFAULT_NAMESPACE:=/camera/depth_registered INPUT:=points h_min:=-105 h_max:=-80 s_min:=50 ← 色抽出のパラメータ

;; robotの初期化
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)

;; jsk_pcl_rosのmsgを使うため、loadする
;;(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
;;(defvar *topic-name* "/camera/depth_registered/cluster_decomposer/boxes")
(defvar *topic-name* "/camera/depth_registered/boxes")
(defvar *bounding-box-list* nil)
(defvar cx nil)

;; ros::initする
(ros::roseus "boundingboxarray_subscriber")
(send (send *dxl-armed-turtlebot* :arm :end-coords) :draw-on :flush t)

;;b（キューブオブジェクトを想定）までの距離を返す
(defun range (b)
  (sqrt (+ (* (elt (send b :pos) 0) (elt (send b :pos) 0)) (* (elt (send b :pos) 1) (elt (send b :pos) 1)) (* (elt (send b :pos) 2) (elt (send b :pos) 2)))))
;; コールバック関数
(defun bounding-box-array-cb (msg)
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  ;;BoundingBoxには検出された青色の領域についての情報が入っている
  ;; BoundingBoxがあれば表示する
  (when *bounding-box-list*
    (send *irtviewer* :draw-objects :flush nil)
	(setq cx nil)
    (mapcar #'(lambda (b)
                ;; BoundingBoxは、dimsntion(直方体の幅・奥行き・高さ)をもつ
                (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                       (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
                       ;; (1) BoundingBoxのカメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、取得する
                       (cam-coords (send (send *dxl-armed-turtlebot* :camera_rgb_optical_frame_lk) :copy-worldcoords))
					   (min-range 10000))
				       (send cam-coords :rotate (deg2rad -35) :x) ;;キネクトの角度変更分補正
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにmake-cubeの箱を設置する
                  (send bx :newcoords (send cam-coords :transform cam->obj-coords))
                  (send bx :worldcoords)
				  (send bx :locate #f(0 -250 30)) ;;キネクトの位置変化分補正
                  (send bx :worldcoords)
                  (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画
				  (if (and (< (elt dims 0) 60) (< (elt dims 1) 60) (< (elt dims 2) 60)) ;;複数の青色オブジェクトが検出された場合、最も距離が近いオブジェクトを選択（正しく機能しているかはしらない）
				  	(if (< (range bx) min-range)
					  (progn
						(setq min-range (range bx))
						(setq cx bx)))) ;;掴みたいオブジェクトをcxにセット
                  bx))
            *bounding-box-list*)
	(if (not (equal cx nil)) ;;cxがセットされている場合
	  (progn (print (send cx :pos))
	  (setq ax (make-cube 30 30 30))
	  (send ax :locate (send cx :pos))
	  (send ax :locate #f(0 0 180)) ;;アームの高さ変化分を補正
	  (send ax :worldcoords)
	  (send ax :draw-on :flush nil :color #f(0 1 0))
      (send *irtviewer* :viewer :viewsurface :flush)
	  (send *dxl-armed-turtlebot* :reset-pose2) ;;アームを高く掲げるポーズ （逆運動学を解きやすいように）
	  (if (send *dxl-armed-turtlebot* :inverse-kinematics (make-coords :pos (send ax :pos)) :rotation-axis :z) ;;axの位置にアームを動かす逆運動学が解けたらprogn以下を行う
	    (progn 
		  (send *ri* :go-velocity 0 0 0) ;;車体の移動を停止（動いてないから無意味）
		  (send *dxl-armed-turtlebot* :reset-pose2) ;;アームを高く掲げるポーズ
		  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 5000) ;;５秒間かけて上記のポーズをロボットにとらせる。
	      (send *ri* :wait-interpolation) ;;動作が終わるまで待つ
		  (send *dxl-armed-turtlebot* :inverse-kinematics (make-coords :pos (send ax :pos)) :rotation-axis :z) ;;逆運動学をもう一度解く（解けることは分かっている）
		  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 5000) ;;５秒間かけて上記のポーズをロボットにとらせる。
	      (send *ri* :wait-interpolation)
		  (send *ri* :start-grasp) ;;つかむ動作
	      (send *ri* :wait-interpolation)
          (send *dxl-armed-turtlebot* :angle-vector #f(0.0 150.0 -90.0 0.0 90.0 0.0 -10.0)) ;;掴んだままアームを高く掲げるポーズ
		  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3000)
	      (send *ri* :wait-interpolation)
		  (send *dxl-armed-turtlebot* :angle-vector #f(86.4258 90.5273 5.27344 -0.585937 -89.6484 89.9414 -10)) ;;回収箱の上に移動
		  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3000)
	      (send *ri* :wait-interpolation)
		  (send *ri* :stop-grasp) ;;離す動作
	      (send *ri* :wait-interpolation)
		  )
		))
	)
  )
)

(print "init1")
(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

(print "init2")
(do-until-key
 (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
 (ros::spin-once)
 (ros::sleep)
 )
