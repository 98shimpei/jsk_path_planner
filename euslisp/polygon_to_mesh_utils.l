#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "safe_footstep_planner")

(defun make-local-plane (x-len y-len x y z header)
  (setq points-msg (list))
  (setq points-msg
        (nconc points-msg (list (instance geometry_msgs::Point32 :init
                                          :x (+ x (float (/ x-len 2)))
                                          :y (+ y (float (/ y-len 2)))
                                          :z z))))
  (setq points-msg
        (nconc points-msg (list (instance geometry_msgs::Point32 :init
                                          :x (+ x (float (/ x-len 2)))
                                          :y (+ y (float (* -1 (/ y-len 2))))
                                          :z z))))
  (setq points-msg
        (nconc points-msg (list (instance geometry_msgs::Point32 :init
                                          :x (+ x (float (* -1 (/ x-len 2))))
                                          :y (+ y (float (* -1 (/ y-len 2))))
                                          :z z))))
  (setq points-msg
        (nconc points-msg (list (instance geometry_msgs::Point32 :init
                                          :x (+ x (float (* -1 (/ x-len 2))))
                                          :y (+ y (float (/ y-len 2)))
                                          :z z))))

  (instance geometry_msgs::PolygonStamped :init
            :header header
            :polygon (instance geometry_msgs::Polygon :init
                               :points points-msg))
  )

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

;; old function
;; (defun change-face-size (f margin)
;;   (setq centroid (second (send f :centroid)))
;;   (setq vertices (send f :vertices))
;;   (setq new-vertices (list))
;;   (dotimes (i (- (length vertices) 1))
;;     (setq v (v- (elt vertices i) centroid))
;;     (setq new-point (v+ (elt vertices i)
;;                         (scale (/ margin (norm v)) v)))
;;     (setq new-vertices (nconc new-vertices (list new-point)))
;;     )
;;   (instance face :init :vertices new-vertices)
;;   )

;; new function (this func removes near vertices)
(defun change-face-size (f inner-margin near-margin)
  (setq vertices (reverse (cdr (reverse (send f :vertices)))))
  (setq new-vertices (list))
  (setq p-len (length vertices))
  (dotimes (j (length vertices))
    (setq v0 (float-vector (elt (elt vertices (mod (+ j 1) p-len)) 0) (elt (elt vertices (mod (+ j 1) p-len)) 1) 0))
    (setq va (float-vector (elt (elt vertices (mod (+ j 2) p-len)) 0) (elt (elt vertices (mod (+ j 2) p-len)) 1) 0))
    (setq vb (float-vector (elt (elt vertices j) 0) (elt (elt vertices j) 1) 0))
    (setq ea (normalize-vector (v- va v0)))
    (setq eb (normalize-vector (v- vb v0)))
    (setq theta (acos (v. ea eb)))
    (if (> (elt (v* ea eb) 2) 0) (setq theta (- (* 2 pi) theta)))
    (setq np (v+
              v0
              (scale
               (/ inner-margin (* (norm (v+ ea eb)) (sin (/ theta 2))))
               (v+ ea eb))))

    ;; skip adding a new vertice if the previous vertice is too close
    (when (= j 0)
      (setq new-vertices (nconc new-vertices (list (float-vector (elt np 0) (elt np 1) 0))))
      (setq pre-np np)
      )
    (when (> j 0)
      (when (> (norm (v- np pre-np)) near-margin)
        (setq new-vertices (nconc new-vertices (list (float-vector (elt np 0) (elt np 1) 0))))
        (setq pre-np np)
        )
      )
    )
  (instance face :init :vertices new-vertices)
  )

(defun change-face-size-use-body (f inner-margin inv)
  (setq *epsilon* 1e-6)
  (setq *contact-threshold* 8e-6)
  (setq vertices (reverse (cdr (reverse (send f :vertices)))))
  (setq edges (send f :edges))

  (setq org-box (make-prism vertices 10))
  (setq edge-box-height 30)
  (setq cp (make-plane :normal #f(0 0 1) :point #f(0 0 5)))
  (setq rot #3F((0 -1 0) (1 0 0) (0 0 1)))
  (dotimes (edge-id (length edges))
    (setq p1 (send (elt edges edge-id) :point 0))
    (setq p2 (send (elt edges edge-id) :point 1))
    (setq ea (normalize-vector (v- p2 p1)))
    (setq a1 (v- (v+ (scale inner-margin (transform rot ea)) p1) (scale 1000 ea)))
    (setq a2 (v+ (v+ (scale inner-margin (transform rot ea)) p2) (scale 1000 ea)))
    (setq a3 (v+ (v+ (scale inner-margin (transform (transpose rot) ea)) p2) (scale 1000 ea)))
    (setq a4 (v- (v+ (scale inner-margin (transform (transpose rot) ea)) p1) (scale 1000 ea)))
    (setq edge-box (make-prism (list a1 a2 a3 a4) edge-box-height))
    (send edge-box :translate (float-vector 0 0 (/ (- 10 edge-box-height) 2)))
    (setq edge-box-height (+ edge-box-height 10))
    (if (> inv 0)
        (setq org-box (body+ org-box edge-box))
      (setq org-box (body- org-box edge-box))
      )
    )

  (car (cut-body org-box cp))
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

(defun make-polygons-msg (header polygons)
  ;; prepare polygon stamped msg list
  (setq polygons-msg (list))

  ;; add polygon stamped msg to list
  (dotimes (i (length polygons))
    ;; get polygon's vertices
    (setq vertices (send (elt polygons i) :vertices))

    ;; prepare point msg list
    (setq points-msg (list))

    ;; add point msg to list
    (dotimes (j (- (length vertices) 1))
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
    (setq polygon-msg
          (instance geometry_msgs::Polygon :init
                    :points points-msg))
    (setq polygons-msg (nconc polygons-msg (list polygon-msg)))
    )

  ;; return polygon array msg
  (instance safe_footstep_planner::PolygonArray :init
            :polygons polygons-msg)
  )

(defun combine-holes (hole-candidates)
  (when (> (length hole-candidates) 1)
    (setq x 0)
    (while (< x (length hole-candidates))
      (setq y (+ x 1))
      (while (< y (length hole-candidates))
        (setq combined-holes (face+ (elt hole-candidates x) (elt hole-candidates y)))
        (if (eq (length combined-holes) 1)
            (progn
              (setq hole-candidates (remove (elt hole-candidates y) hole-candidates))
              (setf (elt hole-candidates x) (car combined-holes))
              )
          (setq y (+ y 1))
          )
        )
      (setq x (+ x 1))
      )
    )
  hole-candidates
  )

(defun combine-same-height-polygons (polygons height-th)
  (setq projected-polygons (list))
  (dotimes (polygon-id (length polygons))
    (setq projected-polygons
          (nconc projected-polygons
                 (list (sort-face-vertices-clockwise
                        (polygon-msg-to-face (elt polygons polygon-id)) 1)))))
  (format t "projected polygons: ~A~%" (length projected-polygons))
  (when (> (length polygons) 1)
    (setq x 0)
    (while (< x (length polygons))
      (setq y (+ x 1))
      (while (< y (length polygons))
        (setq height-diff (- (calc-plane-height (elt polygons x))
                             (calc-plane-height (elt polygons y))))
        ;; (print height-diff)
        (setq combined-polygons (face+ (elt projected-polygons x) (elt projected-polygons y)))
        (if (and (eq (length combined-polygons) 1) (< height-diff height-th))
            (progn
              ;; (print (class (car combined-polygons)))
              (format t "polygon x area: ~A~%" (send (elt projected-polygons x) :area))
              (format t "polygon y area: ~A~%" (send (elt projected-polygons y) :area))
              (format t "combined  area: ~A~%" (send (car combined-polygons) :area))
              (setq polygons (remove (elt polygons y) polygons))
              (setq projected-polygons (remove (elt projected-polygons y) projected-polygons))
              (setf (elt projected-polygons x) (car combined-polygons))
              )
          (setq y (+ y 1))
          )
        )
      (setq x (+ x 1))
      )
    )
  ;; polygons
  ;; (print "combine done")
  ;; (print (length polygons))
  ;; (print (length projected-polygons))
  projected-polygons
  )

(defun remove-same-height-small-polygons (polygons height-th)
  (setq cleaned-polygons (list))
  (dotimes (polygon-id (length polygons))
    (setq cleaned-polygons
          (nconc cleaned-polygons
                 (list (polygon-msg-to-face (elt polygons polygon-id))))))
  (when (> (length polygons) 1)
    (setq x 0)
    (while (< x (length polygons))
      (setq y (+ x 1))
      (while (< y (length polygons))
        (setq height-diff (- (calc-plane-height (elt polygons x))
                             (calc-plane-height (elt polygons y))))
        ;; (print height-diff)
        (setq combined-polygons (face+ (elt cleaned-polygons x) (elt cleaned-polygons y)))
        (if (and (eq (length combined-polygons) 1) (< height-diff height-th))
            (progn
              (if (> (send (elt cleaned-polygons x) :area)
                     (send (elt cleaned-polygons y) :area))
                  (progn
                    (setq polygons (remove (elt polygons y) polygons))
                    (setq cleaned-polygons (remove (elt cleaned-polygons y) cleaned-polygons))
                    )
                (progn
                  (setq tmp-polygon (elt polygons y))
                  (setq polygons (remove (elt polygons y) polygons))
                  (setf (elt polygons x) tmp-polygon)
                  (setq tmp-polygon (elt cleaned-polygons y))
                  (setq cleaned-polygons (remove (elt cleaned-polygons y) cleaned-polygons))
                  (setf (elt cleaned-polygons x) tmp-polygon)
                  )
                )
              )
          (setq y (+ y 1))
          )
        )
      (setq x (+ x 1))
      )
    )
  ;; polygons
  ;; (print "combine done")
  ;; (print (length polygons))
  ;; (print (length cleaned-polygons))
  cleaned-polygons
  )


(defun make-triangles-from-msg(header polygons)
  ;; set face scale
  (setq face-scale 1000.0)

  ;; sort polygons by height mean of points (descending)
  (setq sorted-polygons
        (sort (copy-list polygons) 'compare-plane-height)
        )

  ;; (format t "polygons before combine: ~A~%" (length sorted-polygons))
  ;; (setq projected-polygons
  ;;       (combine-same-height-polygons sorted-polygons (* 0.01)))
  ;; (format t "polygons after combine:  ~A~%" (length projected-polygons))

  ;; (format t "polygons before removal: ~A~%" (length sorted-polygons))
  (setq cleaned-polygons
        (remove-same-height-small-polygons sorted-polygons 0.01))
        ;; (combine-same-height-polygons sorted-polygons (* 0.01)))
  ;; (format t "polygons after removal:  ~A~%" (length cleaned-polygons))
  ;; (print "removed")

  ;; prepare ouput triangles list
  (setq triangles (list))

  ;; for debug
  (setq base-faces (list))

  ;; (dotimes (i (length sorted-polygons))
  ;; (dotimes (i (length projected-polygons))
  (dotimes (i (length cleaned-polygons))
    ;; (format t "i: ~A~%" i)
    ;; for debug
    (setq base-box-up-faces (list))
    (setq exist-base-box t)

    ;; (setq base-face (polygon-msg-to-face (elt sorted-polygons i)))
    ;; (print (send base-face :vertices))
    ;; (setq base-face (elt projected-polygons i))
    (setq base-face (elt cleaned-polygons i))
    ;; scale face size x1000
    (setq base-face (change-face-scale base-face face-scale))
    ;; make base face smaller
    ;; (setq base-face (change-face-size base-face (* -0.03 face-scale)))
    ;; (setq base-face (change-face-size base-face (* -0.1 face-scale)))
    ;; (setq base-face (change-face-size base-face (* -0.1 face-scale) (* 0.08 face-scale)))
    ;; remove near vertices
    ;; (setq base-face (remove-near-vertices base-face (* 0.05 face-scale))) ;; debug
    (setq base-face (remove-near-vertices base-face (* 0.15 face-scale))) ;; real success
    ;; (setq base-face (change-face-size base-face (* 0.05 face-scale) -1)) ;; debug
    ;; (setq base-face (change-face-size base-face (* 0.09 face-scale) -1)) ;; real success
    (setq base-face (change-face-size base-face (* 0.11 face-scale) -1)) ;; real success only for stepping stones
    ;; (print "change face size")
    ;; (setq base-face (change-face-size-use-body base-face (* 0.05 face-scale) -1))
    (if (eq base-face nil)
        (setq exist-base-box nil))
    ;; (setq base-face (change-face-size base-face (* 0.1 face-scale) (* 0.05 face-scale)))
    ;; (setq base-face (change-face-size base-face (* 0.05 face-scale) (* 0.001 face-scale)))
    ;; sort base face vertices clockwise
    ;; (setq base-face (sort-face-vertices-clockwise base-face 1))
    ;; remove near vertices
    ;; (setq base-face (remove-near-vertices base-face (* 0.03 face-scale)))
    ;; (setq base-face (remove-near-vertices base-face (* 0.08 face-scale)))
    ;; (ros::ros-info "i: ~A  area: ~A" i (send base-face :area))

    ;; skip small polygon
    (if (< (send base-face :area) (* 0.01 face-scale face-scale))
        ;; (ros::ros-info "i: ~A  area is too small" i)
        (format t "area is too small: ~A  required: ~A~%"
                (send base-face :area) (* 0.01 face-scale face-scale))

      ;; else
      (progn
        (when (eq i 0)
          ;; (print "i=0 !!!")
          ;; (print base-face)
          ;; (print base-box-up-faces)
          (setq base-box-up-faces (append base-box-up-faces (list base-face)))
          ;; (print base-box-up-faces)
          )
        (when (and (> i 0) exist-base-box)
          (setq hole-candidates (list))
          (dotimes (j i)
            ;; (setq upper-face (polygon-msg-to-face (elt sorted-polygons j)))
            ;; (setq upper-face (elt projected-polygons j))
            (setq upper-face (elt cleaned-polygons j))
            ;; scale face size x1000
            (setq upper-face (change-face-scale upper-face face-scale))
            ;; make hole larger
            ;; (setq upper-face (change-face-size upper-face (* 0.03 face-scale)))
            ;; (setq upper-face (change-face-size upper-face (* 0.2 face-scale) (* 0.03 face-scale)))
            ;; (setq upper-face (change-face-size upper-face (* -0.1 face-scale) (* 0.03 face-scale)))
            ;; remove near vertices
            (setq upper-face (remove-near-vertices upper-face (* 0.03 face-scale)))
            ;; (setq upper-face (change-face-size upper-face (* -0.05 face-scale) (* 0.03 face-scale))) ;;debug
            (setq upper-face (change-face-size upper-face (* -0.19 face-scale) (* 0.05 face-scale))) ;; real success
            ;; sort upper face vertices clockwise
            (setq upper-face (sort-face-vertices-clockwise upper-face 1))
            ;; remove near vertices
            ;; (setq upper-face (remove-near-vertices upper-face (* 0.03 face-scale)))
            ;; (ros::ros-info "  j: ~A  area: ~A" j (send upper-face :area))

            ;; get common-faces
            (setq common-faces (face* base-face upper-face))
            ;; cast common-faces type to list
            (if (not (eq (class common-faces) (class (list 1))))
                (setq common-faces (list common-faces)))
            ;; (format t "common-faces: ~A~%" (length common-faces))
            ;; remove common face as hole
            (dotimes (k (length common-faces))
              (setq common-face (elt common-faces k))
              (when (eq (class common-face) (class base-face))
                ;; length of vertices <= 3 is not face (line or point)
                ;; (format t "common-face vertices: ~A~%" (length (send common-face :vertices)))
                (when (> (length (send common-face :vertices)) 3)
                  ;; sort common face vertices anticlockwise
                  ;; (ros::ros-info "  j: ~A  common area k: ~A : ~A" j k (send common-face :area))
                  ;; (ros::ros-info "  vertices: ~A" (length (send common-face :vertices)))
                  ;; (setq common-face (sort-face-vertices-clockwise common-face -1))
                  ;; (setq common-face (remove-near-vertices common-face (* 0.03 face-scale)))
                  ;; (setq common-face (sort-face-vertices-clockwise common-faxce 1))

                  ;; set common face as hole
                  ;; (setq h (instance hole :init :vertices (cdr (reverse (send common-face :vertices)))))
                  ;; (setq hole-candidates (nconc hole-candidates (list h)))
                  (setq hole-candidates (nconc hole-candidates (list common-face)))
                  ;; (send base-face :enter-hole h)
                  ;; (ros::ros-info "  j: ~A  removed area k: ~A : ~A" j k (send base-face :area))
                  )
                )
              )
            )

          ;; combine holes in hole-candidates
          (setq hole-candidates (combine-holes hole-candidates))

          ;; set hole using box
          ;; (format t "hole candidates ~A~%" (length hole-candidates))
          (setq base-box (make-prism (reverse (cdr (reverse (send base-face :vertices)))) 10))
          ;; (print "base-face")
          ;; (print (send base-face :vertices))

          ;; for avoid face cantacts (outer circuit not found)
          (setq *epsilon* 1e-6)
          (setq *contact-threshold* 8e-6)
          ;; (dotimes (hole-id (length hole-candidates))
          (setq hole-id 0)
          (setq tmp-error-handler *error-handler*)
          (while (< hole-id (length hole-candidates))
            (lisp::install-error-handler
              #'(lambda(n txt form a)
                  (format t "my error: ~A at ~A in ~A~%" txt a  form)
                  (throw :my-error nil)
                  (reset)
                  )
              )
            (catch :my-error
              (setq hole-candidate (elt hole-candidates hole-id))
              (setq hole-candidate (remove-near-vertices hole-candidate (* 0.01 face-scale)))
              ;; (setq hole-candidate (sort-face-vertices-clockwise hole-candidate 1))
              (setq hole-candidate
                    (change-face-size hole-candidate (* -0.01 face-scale) (* 0.001 face-scale)))
              ;; (print "hole-face")
              ;; (print (send hole-candidate :vertices))
              (setq hole-is-not-nil t)
              (setq ver-id 0)
              (setq new-vertices (reverse (cdr (reverse (send hole-candidate :vertices)))))
              (while (< ver-id (length new-vertices))
                ;; (dotimes (ver-id (length (send hole-candidate :vertices)))
                (setq ver (elt new-vertices ver-id))
                (if (or (eq (elt ver 0) (sqrt -1))
                        (eq (elt ver 0) (elt #f(inf inf 0.0) 0)))
                    (progn
                      (setf new-vertices (remove (elt new-vertices ver-id) new-vertices))
                      (print "hole-box vertices include nil or inf. remove nan vertices."))
                  (setq ver-id (+ ver-id 1))
                  )
                )
              ;; (format t "new-vertices' length: ~s~%" (length new-vertices))
              (if (not new-vertices) (throw :my-error nil))
              ;; (print new-vertices)
              (setq hole-candidate (instance face :init :vertices new-vertices))
              ;; (print "removed hole-face")
              ;; (print (send hole-candidate :vertices))
              ;; (format t "hole-candidate vertices: ~A~%" (length (send hole-candidate :vertices)))
              (setq hole-box
                    (make-prism
                     (reverse (cdr (reverse (send hole-candidate :vertices)))) 30))
              (if (not (eq (class base-box) (class (make-cube 1 1 1))))
                  (print "base-box is not body class"))
              (if (not (eq (class hole-box) (class base-box)))
                  (print "hole-box is not body class"))
              (if (not (> (send base-box :volume) (/ (send hole-box :volume) 3)))
                  (progn
                    (print "base-box is smaller than hole-box")
                    (print "base-box")
                    (print (send base-box :volume))
                    ;; (print (send base-box :vertices))
                    (print "hole-box")
                    (print (/ (send hole-box :volume) 3))
                    (setq exist-base-box nil)
                    ;; (print (send hole-box :vertices))
                    ))
              ;; (if (not hole-is-not-nil)
              ;;     (print "hole-box vertices include nil or inf"))
              (send hole-box :translate #f(0 0 -10))
              (if (and (eq (class base-box) (class (make-cube 1 1 1)))
                       (eq (class hole-box) (class base-box))
                       (> (send base-box :volume) (/ (send hole-box :volume) 3))
                       hole-is-not-nil)
                  (setq base-box (body- base-box hole-box))
                ;; (setq base-box nil)
                )
              )
            (lisp::install-error-handler tmp-error-handler)
            (setq hole-id (1+ hole-id))
          )

          ;; (print "final base-box")
          ;; (print base-box)
          (when (and base-box exist-base-box)
          ;; (when base-box
            (setq base-box-faces (send base-box :faces))
            (dotimes (face-id (length base-box-faces))
              (when (eq (elt (send (elt base-box-faces face-id) :normal) 2) 1.0)
                ;; (nconc base-box-up-faces (list (elt base-box-faces face-id)))
                (setq base-box-up-faces (append base-box-up-faces (list (elt base-box-faces face-id))))
                )
              )
            )

          )
        )
      )

    ;; for debug
    (setq base-faces (append base-faces base-box-up-faces))

    ;; make triangle meshes and add them to list
    ;; (ros::ros-info "base face vertices: ~A" (length (send base-face :vertices)))
    ;; (ros::ros-info "base face holes: ~A" (length (send base-face :holes)))
    ;; (setq triangles (nconc triangles (geometry::face-to-triangle base-face)))
    (dotimes (face-id (length base-box-up-faces))
      (setq triangles (nconc triangles (geometry::face-to-triangle (elt base-box-up-faces face-id)))))
    ;; (ros::ros-info "make triangles from base-face  length: ~A"
    ;;   (length (geometry::face-to-triangle base-face)))
    )

  ;; (format t "triangles: ~A~%" (length triangles))
  ;; (if (= (length triangles) 0) (throw :my-error nil))

  ;; change face scale to original
  (dotimes (i (length triangles))
    (setf (elt triangles i) (change-face-scale (elt triangles i) (/ 1 face-scale))))
  ;; this is only for debug msg
  (dotimes (i (length base-faces))
    (setf (elt base-faces i) (change-face-scale (elt base-faces i) (/ 1 face-scale))))

  ;; make polygon array msg of triangle meshes
  (setq triangles-msg (make-triangles-msg header triangles))
  (setq triangles-view-msg (make-polygons-msg header triangles))
  (setq base-faces-view-msg (make-polygons-msg header base-faces))

  ;; publish sorted-polygons for debug
  (setq polygon-array-msg
        (instance jsk_recognition_msgs::PolygonArray
                  :init :header header :polygons sorted-polygons))

  (list polygon-array-msg triangles-msg triangles-view-msg base-faces-view-msg)
)
