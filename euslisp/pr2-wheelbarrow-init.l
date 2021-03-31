#!/usr/bin/env roseus
(require "package://pr2eus/pr2-interface.l")
(setq *ri* (instance pr2-interface :init :namespace "PR2"))
(setq *robot* (pr2))

;; move to grasp pos
(format t "move to grasp pos~%")
(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #f(0 200 -200) :world)
(send *robot* :rarm :move-end-pos #f(0 -200 -200) :world)
(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(200 0 0) :world)
(send *robot* :rarm :move-end-pos #f(200 0 0) :world)
(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

;; rarm
;; (setq p (send (send *robot* :rarm :end-coords) :worldpos))
(setq p #f(600 -400 530))
;; (setq c (make-cascoords :pos p :rpy #f(0 0 0)))
(setq c (make-cascoords :pos p :rpy (float-vector (/ pi 2) 0 (/ pi 2))))
(send *robot*
      :inverse-kinematics c
      :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
      :move-target (send *robot* :rarm :end-coords)
      :translation-axis t
      :rotation-axis t)

;; larm
;; (setq p (send (send *robot* :larm :end-coords) :worldpos))
(setq p #f(600 400 530))
(setq c (make-cascoords :pos p :rpy (float-vector (/ -pi 2) 0 (/ pi 2))))
(send *robot*
      :inverse-kinematics c
      :link-list (send *robot* :link-list (send *robot* :larm :end-coords :parent))
      :move-target (send *robot* :larm :end-coords)
      :translation-axis t
      :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *ri* :move-gripper :rarm 1 :effort 10 :wait t)
(send *ri* :move-gripper :larm 1 :effort 10 :wait t)

;; grasp
(format t "start grasp~%")
(send *robot* :rarm :move-end-pos #f(0 120 0) :world)
(send *robot* :larm :move-end-pos #f(0 -120 0) :world)
(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *ri* :move-gripper :rarm 0.6 :effort 10 :wait t)
(send *ri* :move-gripper :larm 0.6 :effort 10 :wait t)

;; pull up wheelbarrow
(format t "pull up wheelbarrow~%")
(send *robot* :torso_lift_joint :joint-angle 200)
(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
(send *ri* :wait-interpolation)
