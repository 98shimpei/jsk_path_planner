#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")

(defun calc-plane-height (polygon)
  ;; calc z mean of points
  (setq points (send (send polygon :polygon) :points))
  (setq z-sum 0)
  (dotimes (i (length points))
    (setq point (elt points i))
    (setq z-sum (+ z-sum (send point :z)))
    )
  (/ z-sum (length points))
  )

(defun compare-plane-height (a b)
  (> (calc-plane-height a) (calc-plane-height b)))

(defun polygon-msg-to-face (polygon)
  (setq points (send (send polygon :polygon) :points))
  (setq vertices (list))
  (dotimes (i (length points))
    (setq new-point (float-vector
                     (send (elt points i) :x)
                     (send (elt points i) :y)
                     0))
    (setq vertices (nconc vertices (list new-point)))
    )
  (instance face :init :vertices vertices)
  )

(defun callback(msg)

  (setq header (send msg :header))
  (setq polygons (send msg :polygons))

  ;; sort polygons by height mean of points (descending)
  (setq sorted-polygons
        (sort (copy-list polygons) 'compare-plane-height)
        )

  (dotimes (i (length sorted-polygons))
    (setq base-face (polygon-msg-to-face (elt sorted-polygons i)))
    ;; make base face smaller
    ;; (setq base-face (change-face-size base-face -3))
    (ros::ros-info "i: ~A  area: ~A" i (send base-face :area))
    (when (> i 0)
      (dotimes (j i)
        (setq upper-face (polygon-msg-to-face (elt sorted-polygons j)))
        (ros::ros-info "  j: ~A  area: ~A" j (send upper-face :area))
        ;; make hole larger
        ;; (setq upper-face (change-face-size upper-face 3))
        ;; (setq common-face (car (face* base-face upper-face)))
        (setq common-face (face* base-face upper-face))
        (when (eq (find :area (send common-face :methods)) nil)
          (setq common-face (car common-face))
          )
        ;; remove common face as hole
        (when (not (eq common-face nil))
          ;; set common face as hole
          (ros::ros-info "  j: ~A  common area: ~A" j (send common-face :area))
          (send base-face :enter-hole common-face)
          (ros::ros-info "  j: ~A  removed area: ~A" j (send base-face :area))
          )
        )
      )

    ;; make triangle meshes
    )

  ;; publish sorted-polygons for debug
  (setq polygon-array-msg
        (instance jsk_recognition_msgs::PolygonArray
                  :init :header header :polygons sorted-polygons))

  (ros::publish "~sorted_polygons" polygon-array-msg)
  (ros::ros-info "published")
  ) ;; defun

(ros::roseus "polygon_to_mesh")
(format t "init~%")
(ros::subscribe "~input" jsk_recognition_msgs::PolygonArray #'callback 1)

;; for debug
(ros::advertise "~sorted_polygons" jsk_recognition_msgs::PolygonArray 1)

(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")
