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

(defun callback(msg)

  (setq header (send msg :header))
  (setq polygons (send msg :polygons))

  ;; sort polygons by height mean of points (descending)
  (setq sorted-polygons
        (sort (copy-list polygons) 'compare-plane-height)
        )

  (dotimes (i (length sorted-polygons))
    ;; find upper polygons

    ;; connect upper polygons

    ;; make plane with holes

    ;; make triangle meshes
    )

  ;; publish sorted-polygons for debug
  (setq polygon-array-msg
        (instance jsk_recognition_msgs::PolygonArray
                  :init :header header :polygons sorted-polygons))

  (ros::publish "~sorted_polygons" polygon-array-msg)
  (format t "published~%")
  ) ;; defun

(ros::roseus "polygon_to_mesh")
(format t "init~%")
(ros::subscribe "~input" jsk_recognition_msgs::PolygonArray #'callback 1)

;; for debug
(ros::advertise "~sorted_polygons" jsk_recognition_msgs::PolygonArray 1)

(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")
