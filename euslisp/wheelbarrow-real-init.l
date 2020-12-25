#!/usr/bin/env roseus
;; (require "/home/ohmori/prog/euslib/demo/kojio/push-recovery/push-recovery-foot-guided.l")
(load "/home/ohmori/prog/euslib/demo/kojio/push-recovery/push-recovery-foot-guided.l")
(unless (boundp '*jaxon_red*)
  (robots-init "jaxon_red")
  (setq *robot* *jaxon_red*)
  )

(format t "start footguided modification~%")
(start-footguided-modification t)

;; move to grasp pos
(format t "move to grasp pos~%")
(progn
  (push-init t)
  (format t "push init done~%")
  (send *ri* :stop-grasp)
  (format t "stop grasp done~%")
  ;; (send *ri* :stop-impedance :arms)
  (send *robot* :angle-vector
        #f(-0.001866 0.021374 -47.7351 71.9455 -35.1258 -0.021768 -0.001866 0.021375 -47.7364 71.9443 -35.1233 -0.021769 0.0 0.0 0.0 0.0 -10.0 -8.29028 -48.805 -57.1603 82.6109 -87.2152 -84.3984 3.98598 42.2862 8.29028 -48.8053 57.1617 -82.6108 -87.2127 84.3986 -3.98984 42.286)
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (format t "sent angle-vector pre~%")
  (send *ri* :wait-interpolation)
  (send *robot* :angle-vector
        #f(-0.001866 0.021374 -47.7351 71.9455 -35.1258 -0.021768 -0.001866 0.021375 -47.7364 71.9443 -35.1233 -0.021769 0.0 0.0 0.0 0.0 -10.0 21.9425 -29.2283 -41.1765 79.4707 -27.3479 -92.6119 27.3777 18.286 -21.9433 -29.2274 41.1771 -79.4705 -27.344 92.6123 -27.3818 18.2853)
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (format t "sent angle-vector grasp~%")
  )

(format t "press enter if the robot holds grip")
(read-line)

(send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :frame "world")
(send *ri* :start-reference-force-updater :arms)

;; grasp
(format t "start grasp~%")
;; for heavy object
;; (send *ri* :set-reference-force-updater-param :arms :frame "world" :motion-dir #f(0 0 -1))
;; (send *ri* :start-reference-force-updater :arms)

;; (send *ri* :move-gripper :arms 0.6 :effort 10 :wait t)
(send *ri* :start-grasp)
(send *ri* :wait-interpolation)
(send *ri* :emergency-release-motion)

(send *ri* :start-impedance :arms)
;; (send *ri* :go-velocity 1 0 0)

;; pull up wheelbarrow
(format t "pull up wheelbarrow~%")
(send *robot* :angle-vector
      #f(-0.002399 0.018603 -30.4191 40.6039 -21.1038 -0.018943 -0.002396 0.018646 -30.4198 40.6021 -21.1012 -0.01899 0.0 0.0 0.0 0.0 -10.0 21.9425 -29.2283 -41.1765 79.4707 -27.3479 -92.6119 27.3777 18.286 -21.9433 -29.2274 41.1771 -79.4705 -27.344 92.6123 -27.3818 18.2853)
      )
(send *ri* :angle-vector (send *robot* :angle-vector) 10000)
;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
(send *ri* :wait-interpolation)
;; (send *ri* :emergency-release-motion)

;; if object is heavy
;; (send *robot* :arms :move-end-pos #f(0 0 50) :world)
;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
;; (send *ri* :wait-interpolation)
