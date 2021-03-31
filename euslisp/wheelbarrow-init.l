#!/usr/bin/env roseus
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "package://jsk_path_planner/euslisp/walking-parameters.l")
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

(format t "start footguided modification~%")
(start-footguided-modification)

;; move to grasp pos
(format t "move to grasp pos~%")
(progn
  (push-init t)
  (format t "push init done~%")
  (send *ri* :stop-grasp)
  (format t "stop grasp done~%")
  ;; (send *ri* :stop-impedance :arms)
  (send *robot* :angle-vector
        ;; #f(-0.001866 0.021374 -47.7351 71.9455 -35.1258 -0.021768 -0.001866 0.021375 -47.7364 71.9443 -35.1233 -0.021769 0.0 0.0 0.0 0.0 48.0 -7.47416 -49.0177 -60.8971 82.381 -80.8795 -85.2616 13.3253 41.6723 7.47369 -49.0177 60.8985 -82.3808 -80.8766 85.2619 -13.3292 41.6719)
        ;; )
        ;; #f(-0.001866 0.021374 -47.7351 71.9455 -35.1258 -0.021768 -0.001866 0.021375 -47.7364 71.9443 -35.1233 -0.021769 0.0 0.0 0.0 0.0 -10 -7.47416 -49.0177 -60.8971 82.381 -80.8795 -85.2616 13.3253 41.6723 7.47369 -49.0177 60.8985 -82.3808 -80.8766 85.2619 -13.3292 41.6719)
        #f(-0.001866 0.021374 -47.7351 71.9455 -35.1258 -0.021768 -0.001866 0.021375 -47.7364 71.9443 -35.1233 -0.021769 0.0 0.0 0.0 0.0 -10.0 -8.29273 -29.4782 -79.509 89.2353 -102.428 -74.1901 7.16538 17.2656 8.29273 -29.4794 79.5126 -89.2347 -102.419 74.1904 -7.17828 17.2635)
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (format t "sent angle-vector pre~%")
  (send *ri* :wait-interpolation)
  (send *robot* :angle-vector
        ;; #f(-0.001136 0.025157 -59.2599 94.6662 -46.3217 -0.02562 -0.001136 0.025161 -59.2617 94.6652 -46.3189 -0.025625 0.0 0.0 0.0 0.0 48.0 0.594566 -41.9019 -49.5366 80.0926 -69.1473 -87.9244 -14.6162 39.3819 -0.594549 -41.9019 49.5364 -80.0926 -69.1475 87.9244 14.6166 39.3819)
        ;; #f(-0.001136 0.025157 -59.2599 94.6662 -46.3217 -0.02562 -0.001136 0.025161 -59.2617 94.6652 -46.3189 -0.025625 0.0 0.0 0.0 0.0 -10.0 0.594566 -41.9019 -49.5366 80.0926 -69.1473 -87.9244 -14.6162 39.3819 -0.594549 -41.9019 49.5364 -80.0926 -69.1475 87.9244 14.6166 39.3819)
        #f(-0.001136 0.025157 -59.2599 94.6662 -46.3217 -0.02562 -0.001136 0.025161 -59.2617 94.6652 -46.3189 -0.025625 0.0 0.0 0.0 0.0 -10.0 -8.29203 -26.244 -58.4998 85.2069 -91.2643 -81.9938 -22.1612 23.8066 8.29203 -26.2445 58.503 -85.2065 -91.2604 81.9945 22.154 23.8063)
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (format t "sent angle-vector grasp~%")
  )

(send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :frame "world")
(send *ri* :start-reference-force-updater :arms)

;; grasp
(format t "start grasp~%")
;; for heavy object
;; (send *ri* :set-reference-force-updater-param :arms :frame "world" :motion-dir #f(0 0 -1))
;; (send *ri* :start-reference-force-updater :arms)

(send *ri* :move-gripper :arms 0.6 :effort 10 :wait t)
(send *ri* :wait-interpolation)
(send *ri* :emergency-release-motion)

(send *ri* :start-impedance :arms)
;; (send *ri* :go-velocity 1 0 0)

;; pull up wheelbarrow
(format t "pull up wheelbarrow~%")
(send *robot* :angle-vector
      ;; #f(-0.001866 0.021373 -47.7351 71.9454 -35.1257 -0.021766 -0.001866 0.021377 -47.7364 71.9442 -35.1232 -0.021771 0.0 0.0 0.0 0.0 48.0 0.594566 -41.9019 -49.5366 80.0926 -69.1473 -87.9244 -14.6162 39.3819 -0.594549 -41.9019 49.5364 -80.0926 -69.1475 87.9244 14.6166 39.3819)
      ;; #f(-0.001866 0.021373 -47.7351 71.9454 -35.1257 -0.021766 -0.001866 0.021377 -47.7364 71.9442 -35.1232 -0.021771 0.0 0.0 0.0 0.0 -20.0 0.594566 -41.9019 -49.5366 80.0926 -69.1473 -87.9244 -14.6162 39.3819 -0.594549 -41.9019 49.5364 -80.0926 -69.1475 87.9244 14.6166 39.3819)
      #f(-0.001866 0.021373 -47.7351 71.9454 -35.1257 -0.021766 -0.001866 0.021377 -47.7364 71.9442 -35.1232 -0.021771 0.0 0.0 0.0 0.0 -20.0 -8.29301 -27.9616 -63.6131 84.9092 -99.7427 -82.4947 -25.359 26.125 8.29301 -27.9616 63.6128 -84.9092 -99.7429 82.4947 25.3594 26.125)
      )
(send *ri* :angle-vector (send *robot* :angle-vector) 10000)
(send *ri* :wait-interpolation)

;; (send *ri* :emergency-release-motion)

;; if object is heavy
;; (send *robot* :arms :move-end-pos #f(0 0 50) :world)
;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
;; (send *ri* :wait-interpolation)
