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

(defun change-face-size (f margin)
  (setq centroid (second (send f :centroid)))
  (setq vertices (send f :vertices))
  (setq new-vertices (list))
  (dotimes (i (- (length vertices) 1))
    (setq v (v- (elt vertices i) centroid))
    (setq new-point (v+ (elt vertices i)
                        (scale (/ margin (norm v)) v)))
    (setq new-vertices (nconc new-vertices (list new-point)))
    )
  (instance face :init :vertices new-vertices)
  )

(defun make-triangles-msg (header triangles)
  ;; prepare polygon stamped msg list
  (setq polygons-msg (list))

  ;; add polygon stamped msg to list
  (dotimes (i (length triangles))
    ;; get triangle's vertices
    (setq vertices (send (elt triangles i) :vertices))

    ;; prepare point msg list
    (setq points-msg (list))

    ;; add point msg to list
    (dotimes (j 3)
      (setq points-msg
            (nconc points-msg (list (instance geometry_msgs::Point32 :init
                                              :x (float (elt (elt vertices j) 0))
                                              :y (float (elt (elt vertices j) 1))
                                              :z 0))))
      )

    ;; make polygon stamped msg and add it to the list
    (setq polygon-stamped-msg
          (instance geometry_msgs/PolygonStamped :init
                    :header header
                    :polygon (instance geometry_msgs::Polygon :init
                                       :points points-msg)))
    (setq polygons-msg (nconc polygons-msg (list polygon-stamped-msg)))
    )

  ;; return polygon array msg
  (instance jsk_recognition_msgs::PolygonArray :init
            :header header
            :polygons polygons-msg)
  )

(defun callback(msg)

  (setq header (send msg :header))
  (setq polygons (send msg :polygons))

  ;; sort polygons by height mean of points (descending)
  (setq sorted-polygons
        (sort (copy-list polygons) 'compare-plane-height)
        )

  ;; prepare ouput triangles list
  (setq triangles (list))

  (dotimes (i (length sorted-polygons))
    (setq base-face (polygon-msg-to-face (elt sorted-polygons i)))
    ;; make base face smaller
    (setq base-face (change-face-size base-face -0.03))
    (ros::ros-info "i: ~A  area: ~A" i (send base-face :area))

    ;; skip small polygon
    (when (< (send base-face :area) 0.01)
      (ros::ros-info "i: ~A  area is too small" i)
      (return)
      )

    (when (> i 0)
      (dotimes (j i)
        (setq upper-face (polygon-msg-to-face (elt sorted-polygons j)))
        (ros::ros-info "  j: ~A  area: ~A" j (send upper-face :area))
        ;; make hole larger
        (setq upper-face (change-face-size upper-face 0.03))
        ;; (setq common-face (car (face* base-face upper-face)))
        (setq common-face nil)
        (setq common-face (face* base-face upper-face))

        ;; cast common-face if it is cons type
        (when (eq (class common-face) (class (list 1)))
          (ros::ros-info "  j: ~A  common area type is cons" j)
          (if (eq (class (car common-face)) (class base-face))
              (progn
                (setq common-face (car common-face))
                (ros::ros-info "  j: ~A  cast common area type" j))
            (progn
              (ros::ros-error "  j: ~A  common area type error" j)
              (return)))
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

    ;; make triangle meshes and add them to list
    (ros::ros-info "make triangles from base-face")
    (ros::ros-info "base-face type ~A" (class base-face))
    (ros::ros-info "base face vertices: ~A" (length (send base-face :vertices)))
    (ros::ros-info "base face holes: ~A" (length (send base-face :holes)))
    (print (send base-face :holes))
    (setq triangles (nconc triangles (geometry::face-to-triangle base-face)))
    (ros::ros-info "make triangles from base-face  length: ~A"
      (length (geometry::face-to-triangle base-face)))
    )

  ;; make polygon array msg of triangle meshes
  (setq triangles-msg (make-triangles-msg header triangles))

  ;; publish sorted-polygons for debug
  (setq polygon-array-msg
        (instance jsk_recognition_msgs::PolygonArray
                  :init :header header :polygons sorted-polygons))

  (ros::publish "~sorted_polygons" polygon-array-msg)
  (ros::publish "~meshed_polygons" triangles-msg)
  (ros::ros-info "published")
  ) ;; defun

(ros::roseus "polygon_to_mesh")
(format t "init~%")
(ros::subscribe "~input" jsk_recognition_msgs::PolygonArray #'callback 1)

;; for debug
(ros::advertise "~sorted_polygons" jsk_recognition_msgs::PolygonArray 1)

(ros::advertise "~meshed_polygons" jsk_recognition_msgs::PolygonArray 1)

(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")
