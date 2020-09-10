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

(defun calc-theta (v)
  (setq x (elt v 0))
  (setq y (elt v 1))
  (setq theta (atan y x))
  (if (< x 0) (setq theta (+ theta pi)))
  theta
  )

(defun compare-theta (a b)
  (> (calc-theta a) (calc-theta b)))

(defun sort-face-vertices-clockwise (f inv)
  (setq centroid (second (send f :centroid)))
  (setq vertices (cdr (send f :vertices)))

  ;; transform vertices centroid to origin
  (setq transformed-vertices
        (mapcar #'(lambda (v) (v- v centroid)) vertices))

  ;; sort vertices clockwise
  (setq sorted-transformed-vertices
        (sort (copy-list transformed-vertices) 'compare-theta)
        )

  ;; transform vertices to original pos
  (setq sorted-vertices
        (mapcar #'(lambda (v) (v+ v centroid)) sorted-transformed-vertices))

  ;; reverse list if inv value is negative
  (if (< inv 0)
      (setq sorted-vertices (reverse sorted-vertices)))

  ;; return face object
  (instance face :init :vertices sorted-vertices)
  )

(defun remove-near-vertices (f margin)
  (setq vertices (send f :vertices))
  (setq cleaned-vertices (list (elt vertices 0)))
  (dotimes (i (- (length vertices) 2))
    ;; get candidate point
    (setq cp (elt vertices (+ i 1)))
    ;; get last added point from cleaned-vertices list
    (setq lp (car (reverse cleaned-vertices)))

    ;; add cp to cleaned-vertices if it is far enough from lp
    (when (> (norm (v- cp lp)) margin)
      (setq cleaned-vertices (nconc cleaned-vertices (list cp)))
      )
    )

  ;; return face object
  (instance face :init :vertices cleaned-vertices)
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
          (instance geometry_msgs::PolygonStamped :init
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
    ;; sort base face vertices clockwise
    (setq base-face (sort-face-vertices-clockwise base-face 1))
    ;; remove near vertices
    (setq base-face (remove-near-vertices base-face 0.03))
    (ros::ros-info "i: ~A  area: ~A" i (send base-face :area))

    ;; skip small polygon
    (if (< (send base-face :area) 0.01)
        (ros::ros-info "i: ~A  area is too small" i)

      ;; else
      (progn
        (when (> i 0)
          (dotimes (j i)
            (setq upper-face (polygon-msg-to-face (elt sorted-polygons j)))
            ;; make hole larger
            (setq upper-face (change-face-size upper-face 0.03))
            ;; sort upper face vertices clockwise
            (setq upper-face (sort-face-vertices-clockwise upper-face 1))
            ;; remove near vertices
            (setq upper-face (remove-near-vertices upper-face 0.03))
            (ros::ros-info "  j: ~A  area: ~A" j (send upper-face :area))

            ;; get common-faces
            (setq common-faces (face* base-face upper-face))

            ;; remove common face as hole
            (dotimes (k (length common-faces))
              (setq common-face (elt common-faces k))
              (when (eq (class common-face) (class base-face))
                ;; sort common face vertices anticlockwise
                (ros::ros-info "  j: ~A  common area k: ~A : ~A" j k (send common-face :area))
                (setq common-face (sort-face-vertices-clockwise common-face -1))

                ;; set common face as hole
                (setq h (instance hole :init :vertices (cdr (send common-face :vertices))))
                (send base-face :enter-hole h)
                (ros::ros-info "  j: ~A  removed area k: ~A : ~A" j k (send base-face :area))
                )
              )
            )
          )
        )
      )

    ;; make triangle meshes and add them to list
    (ros::ros-info "base face vertices: ~A" (length (send base-face :vertices)))
    (ros::ros-info "base face holes: ~A" (length (send base-face :holes)))
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
