#!/usr/bin/env roseus
;; (require "/home/ohmori/prog/euslib/demo/kojio/push-recovery/push-recovery-foot-guided.l")
(load "/home/ohmori/prog/euslib/demo/kojio/push-recovery/push-recovery-foot-guided.l")
(unless (boundp '*jaxon_red*)
  (robots-init "jaxon_red")
  (setq *robot* *jaxon_red*)
  )

(format t "start footguided modification~%")
;; (start-footguided-modification t)

;; move to grasp pos
(defun move-to-grasp-pos ()
  (format t "move to grasp pos~%")
  (start-footguided-modification t)
  (progn
    (push-init t)
    ;; (push-init nil)
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
    (send *ri* :remove-force-sensor-offset-rmfo-arms)
    (format t "remove force sensor offset grasp~%")
    )
  )

;; (format t "press enter if the robot holds grip")
;; (read-line)

(defun grasp-wb ()
  ;; (send *ri* :set-reference-force-updater-param :arms
  ;;       :motion-dir #f(0 0 -1)
  ;;       :p-gain 0.05
  ;;       :frame "world")

  ;; (send *ri* :start-reference-force-updater :arms)
  ;; grasp
  (format t "start grasp~%")
  ;; for heavy object
  ;; (send *ri* :set-reference-force-updater-param :arms :frame "world" :motion-dir #f(0 0 -1))
  ;; (send *ri* :start-reference-force-updater :arms)

  ;; (send *ri* :move-gripper :arms 0.6 :effort 10 :wait t)
  ;; (send *ri* :move-gripper :arms 1.0 :effort 20 :wait t)
  ;; (send *ri* :start-grasp)
  ;; (send *ri* :wait-interpolation)
  ;; (send *ri* :emergency-release-motion)

  ;; (send *ri* :set-impedance-controller-param :arms :force-gain #f(1 1 1))
  (send *ri* :start-impedance :arms)
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  ;; (send *ri* :go-velocity 1 0 0)
  )

(defun pull-up-wb ()
  ;; pull up wheelbarrow
  (format t "pull up wheelbarrow~%")
  (send *robot* :angle-vector
        ;; #f(-0.002399 0.018603 -30.4191 40.6039 -21.1038 -0.018943 -0.002396 0.018646 -30.4198 40.6021 -21.1012 -0.01899 0.0 0.0 0.0 0.0 -10.0 21.9425 -29.2283 -41.1765 79.4707 -27.3479 -92.6119 27.3777 18.286 -21.9433 -29.2274 41.1771 -79.4705 -27.344 92.6123 -27.3818 18.2853)
        ;; #f(-0.002399 0.018603 -30.4191 40.6039 -21.1038 -0.018943 -0.002396 0.018646 -30.4198 40.6021 -21.1012 -0.01899 0.0 0.0 0.0 0.0 -10.0 -2.58943 -50.578 -46.4714 87.2731 -67.9968 -87.6678 7.51289 37.1966 2.59104 -50.5793 46.4714 -87.2741 -67.993 87.6677 -7.51784 37.1968)
        ;; #f(-0.002399 0.018603 -30.4191 40.6039 -21.1038 -0.018943 -0.001853 0.02146 -46.2504 72.5244 -37.1929 -0.021856 0.0 0.0 0.0 0.0 -10.0 -2.58943 -50.578 -46.4714 87.2731 -67.9968 -87.6678 7.51289 37.1966 2.59104 -50.5793 46.4714 -87.2741 -67.993 87.6677 -7.51784 37.1968)
        ;; #f(-0.001866 0.021374 -47.7351 71.9455 -35.1258 -0.021768 -0.001866 0.021375 -47.7364 71.9443 -35.1233 -0.021769 0.0 0.0 0.0 0.0 -10.0 -1.42281 -46.7896 -40.2661 86.611 -54.6056 -85.8723 13.7475 33.3315 1.42429 -46.7907 40.2657 -86.612 -54.6016 85.8721 -13.7522 33.3315)
        #f(-0.001866 0.021374 -47.7351 71.9455 -35.1258 -0.021768 -0.001866 0.021375 -47.7364 71.9443 -35.1233 -0.021769 0.0 0.0 0.0 0.0 -10.0 -3.89805 -34.448 -52.4147 89.491 -78.9459 -79.653 1.94009 21.2703 3.89798 -34.449 52.4189 -89.4917 -78.9405 79.6534 -1.94951 21.2686)
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;; (send *ri* :emergency-release-motion)

  (send *robot* :angle-vector
        ;; #f(-0.002399 0.018603 -30.4191 40.6039 -21.1038 -0.018943 -0.002396 0.018646 -30.4198 40.6021 -21.1012 -0.01899 0.0 0.0 0.0 0.0 -10.0 -8.29157 -36.1125 -59.6406 93.8515 -94.616 -77.3775 -4.12873 21.3909 8.29157 -36.114 59.6467 -93.8516 -94.6094 77.378 4.11613 21.389)
        #f(-0.002399 0.018603 -30.4191 40.6039 -21.1038 -0.018943 -0.001853 0.02146 -46.2504 72.5244 -37.1929 -0.021856 0.0 0.0 0.0 0.0 -10.0 -8.29157 -36.1125 -59.6406 93.8515 -94.616 -77.3775 -4.12873 21.3909 8.29157 -36.114 59.6467 -93.8516 -94.6094 77.378 4.11613 21.389)
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :set-reference-force-updater-param :arms
        :motion-dir #f(0 0 -1)
        :p-gain 0.05
        :frame "world")

  ;; (send *ri* :start-reference-force-updater :arms)
  )

;; if object is heavy
;; (send *robot* :arms :move-end-pos #f(0 0 50) :world)
;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
;; (send *ri* :wait-interpolation)


;; (progn
;;   (send *robot* :reset-manip-pose)
;;   ;; (send *robot* :head-neck-p :joint-angle 15)
;;   (send *robot* :head-neck-p :joint-angle 10)
;;   (send *robot* :legs :move-end-pos #f(0 0 30))
;;   (send *robot* :rarm-elbow-p :joint-angle -80)
;;   (send *robot* :larm-elbow-p :joint-angle -80)
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
;;   )
