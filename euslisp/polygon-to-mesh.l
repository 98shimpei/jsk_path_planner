#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "safe_footstep_planner")

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

(defun change-face-scale (f s)
  (setq vertices (send f :vertices))
  (setq new-vertices (list))
  (dotimes (i (- (length vertices) 1))
    (setq new-point (scale s (elt vertices i)))
    (setq new-vertices (nconc new-vertices (list new-point)))
    )
  (instance face :init :vertices new-vertices)
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

(defun make-sf-triangles-msg (triangles)
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

    ;; make polygon msg and add it to the list
    (setq polygon-msg (instance geometry_msgs::Polygon :init :points points-msg))
    (setq polygons-msg (nconc polygons-msg (list polygon-msg)))
    )

  ;; return polygon array msg
  (instance safe_footstep_planner::PolygonArray :init
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

  ;; for debug
  (setq base-faces (list))

  ;; set face scale
  (setq face-scale 1000.0)

  (dotimes (i (length sorted-polygons))
    (setq base-face (polygon-msg-to-face (elt sorted-polygons i)))
    ;; scale face size x1000
    (setq base-face (change-face-scale base-face face-scale))
    ;; make base face smaller
    (setq base-face (change-face-size base-face (* -0.1 face-scale)))
    ;; sort base face vertices clockwise
    (setq base-face (sort-face-vertices-clockwise base-face 1))
    ;; remove near vertices
    (setq base-face (remove-near-vertices base-face (* 0.05 face-scale)))
    (ros::ros-debug "i: ~A  area: ~A" i (send base-face :area))

    ;; skip small polygon
    (if (< (send base-face :area) (* 0.01 face-scale face-scale))
        (ros::ros-warn "i: ~A  area is too small" i)

      ;; else
      (progn
        (when (> i 0)
          (dotimes (j i)
            (setq upper-face (polygon-msg-to-face (elt sorted-polygons j)))
            ;; scale face size x1000
            (setq upper-face (change-face-scale upper-face face-scale))
            ;; make hole larger
            (setq upper-face (change-face-size upper-face (* 0.12 face-scale)))
            ;; sort upper face vertices clockwise
            (setq upper-face (sort-face-vertices-clockwise upper-face 1))
            ;; remove near vertices
            (setq upper-face (remove-near-vertices upper-face (* 0.1 face-scale)))
            (ros::ros-debug "  j: ~A  area: ~A" j (send upper-face :area))

            ;; get common-faces
            (setq common-faces (face* base-face upper-face))

            ;; skip if common area is nil
            (if (eq common-faces nil)
                (ros::ros-info "i: ~A  j: ~A  common area is nil" i j)

              ;; else
              (progn
                ;; cast common-faces type to list
                (when (not (eq (class common-faces) (class (list 1))))
                  (setq common-faces (list common-faces))
                  (ros::ros-warn "i: ~A  j: ~A  cast common-faces to list" i j)
                  )

                ;; remove common face as hole
                (dotimes (k (length common-faces))
                  (setq common-face (elt common-faces k))
                  (when (eq (class common-face) (class base-face))
                    ;; length of vertices <= 3 is not face (line or point)
                    (when (> (length (send common-face :vertices)) 3)
                      ;; sort common face vertices anticlockwise
                      (ros::ros-debug "  j: ~A  common area k: ~A : ~A" j k (send common-face :area))
                      (ros::ros-debug "  vertices: ~A" (length (send common-face :vertices)))
                      (setq common-face (sort-face-vertices-clockwise common-face -1))

                      (setq common-face (change-face-size common-face (* -0.001 face-scale)))

                      ;; set common face as hole
                      (send base-face :enter-hole
                            (instance hole :init :vertices (cdr (send common-face :vertices))))
                      (ros::ros-debug "  j: ~A  removed area k: ~A : ~A" j k (send base-face :area))
                      (when (>= 0 (send base-face :area))
                        (ros::ros-error "i: ~A  j: ~A  base-face area is negative" i j)
                        )
                      )
                    )
                  (when (eq (class common-face) (class (make-line #f(0 0 0) #f(1 0 0))))
                    (ros::ros-warn "i: ~A  j: ~A  common area k: ~A is line" i j k)
                    )
                  )
                )
              )
            )
          )
        )
      )

    ;; for debug
    (setq base-faces (nconc base-faces (list base-face)))

    ;; make triangle meshes and add them to list
    (ros::ros-debug "base face vertices: ~A" (length (send base-face :vertices)))
    (ros::ros-debug "base face holes: ~A" (length (send base-face :holes)))
    (setq tm (send (ros::time-now) :to-sec))
    (setq triangles (nconc triangles (geometry::face-to-triangle-aux base-face)))
    (setq tm-diff (- (send (ros::time-now) :to-sec) tm))
    (if (> tm-diff 0.2)
        (ros::ros-warn "make triangles i: ~A  ~A [s]" i tm-diff)
      (ros::ros-debug "make triangles i: ~A  ~A [s]" i tm-diff)
      )
    )

  ;; debug viewer
  ;; (when (> (length base-faces) 0)
  ;;   (setq base-fs (instance faceset :init :faces base-faces))
  ;;   (send base-fs :set-color #f(1 0 0))
  ;;   (setq triangle-fs (instance faceset :init :faces triangles))
  ;;   (send triangle-fs :set-color #f(0 1 0))
  ;;   (objects (list base-fs triangle-fs))
  ;;   ;; (objects (list base-fs))
  ;;   )

  ;; change face scale to original
  (dotimes (i (length triangles))
    (setf (elt triangles i) (change-face-scale (elt triangles i) (/ 1 face-scale))))

  ;; make polygon array msg of triangle meshes (jsk_recognition_msgs/PolyogonArray)
  (setq triangles-msg (make-triangles-msg header triangles))

  ;; make polygon array msg of triangle meshes (safe_footstep_planner/PolygonArray)
  (setq triangles-sf-msg (make-sf-triangles-msg triangles))

  ;; publish sorted-polygons for debug
  (setq polygon-array-msg
        (instance jsk_recognition_msgs::PolygonArray
                  :init :header header :polygons sorted-polygons))

  (ros::publish "~sorted_polygons" polygon-array-msg)
  (ros::publish "~meshed_polygons" triangles-msg)
  (ros::publish "/meshed_polygons" triangles-sf-msg)
  (ros::ros-info "published")

  ) ;; defun

(ros::roseus "polygon_to_mesh")
(format t "init~%")
(ros::subscribe "~input" jsk_recognition_msgs::PolygonArray #'callback 1)

;; for debug
(ros::advertise "~sorted_polygons" jsk_recognition_msgs::PolygonArray 1)

(ros::advertise "~meshed_polygons" jsk_recognition_msgs::PolygonArray 1)
(ros::advertise "/meshed_polygons" safe_footstep_planner::PolygonArray 1)

(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")
