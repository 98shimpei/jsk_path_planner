#!/usr/bin/env roseus
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "safe_footstep_planner")
;; (require "package://jsk_path_planner/euslisp/polygon_to_mesh_utils")

(comp:compile-file-if-src-newer
  (format nil "~A/euslisp/polygon_to_mesh_utils.l"
          (ros::rospack-find "jsk_path_planner")))
(let ((old-module (find "polygon_to_mesh_utils" *loaded-modules*
                        :key #'lisp::load-module-file-name :test #'equal)))
  (if old-module (sys::unbinload old-module)))
(load "polygon_to_mesh_utils")

(defun callback(msg)

  (setq header (send msg :header))
  (setq polygons (send msg :polygons))

  ;; for debug (ignore subscribed polygons)
  (setq polygons (list))

  (when (ros::get-param "~add_local_plane" nil)
    ;; x-len y-len x y z header
    ;; pattern 1
    (setq local-plane (make-local-plane 1.0 1.0 -2.0 0.0 0.0 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.5 0.5 -2.0 0.0 0.1 header))
    (setq polygons (nconc polygons (list local-plane)))

    ;; pattern 2
    (setq local-plane (make-local-plane 1.0 1.0 0.0 0.0 0.0 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.5 0.5 0.0 0.4 0.1 header))
    (setq polygons (nconc polygons (list local-plane)))

    ;; pattern 3
    (setq local-plane (make-local-plane 1.0 1.0 2.0 0.0 0.0 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.5 0.5 2.0 0.0 0.1 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.2 0.2 2.0 0.0 0.2 header))
    (setq polygons (nconc polygons (list local-plane)))

    ;; pattern 4
    (setq local-plane (make-local-plane 1.0 1.0 -2.0 2.0 0.0 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.8 0.8 -1.8 2.2 0.1 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.8 0.8 -2.2 1.8 0.2 header))
    (setq polygons (nconc polygons (list local-plane)))

    ;; pattern 5
    (setq local-plane (make-local-plane 1.0 1.0 0.0 2.0 0.0 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.5 0.5 0.2 2.2 0.1 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.5 0.5 -0.2 1.8 0.2 header))
    (setq polygons (nconc polygons (list local-plane)))

    ;; pattern 6
    (setq local-plane (make-local-plane 1.0 1.0 2.0 2.0 0.0 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.2 0.2 2.2 2.2 0.1 header))
    (setq polygons (nconc polygons (list local-plane)))
    (setq local-plane (make-local-plane 0.2 0.2 1.8 1.8 0.2 header))
    (setq polygons (nconc polygons (list local-plane)))

    ;; (setq non-convex-plane (make-non-convex-plane header))
    ;; (setq polygons (nconc polygons (list non-convex-plane)))
    )

  ;; use utile functions
  (setq output-msgs (make-triangles-from-msg header polygons))
  (setq polygon-array-msg (elt output-msgs 0))
  (setq triangles-msg (elt output-msgs 1))
  (setq triangles-view-msg (elt output-msgs 2))
  (setq base-faces-view-msg (elt output-msgs 3))
  (setq original-msg (instance jsk_recognition_msgs::PolygonArray
                                :init :header header :polygons polygons))

  (ros::publish "~sorted_polygons" polygon-array-msg)
  (ros::publish "~meshed_polygons" triangles-msg)
  (ros::publish "~meshed_polygons_view" triangles-view-msg)
  (ros::publish "~small_polygons_view" base-faces-view-msg)
  (ros::publish "~original_polygons_view" original-msg)
  (ros::ros-info "published")

  ;; debug viewer
  (setq fs (instance faceset :init :faces base-faces))
  ;; (objects (list fs))

  ) ;; defun

(ros::roseus "polygon_to_mesh")
(format t "init~%")
(ros::subscribe "~input" jsk_recognition_msgs::PolygonArray #'callback 1)

;; for debug
(ros::advertise "~sorted_polygons" jsk_recognition_msgs::PolygonArray 1)

(ros::advertise "~meshed_polygons" safe_footstep_planner::PolygonArray 1)
(ros::advertise "~meshed_polygons_view" jsk_recognition_msgs::PolygonArray 1)
(ros::advertise "~small_polygons_view" jsk_recognition_msgs::PolygonArray 1)
(ros::advertise "~original_polygons_view" jsk_recognition_msgs::PolygonArray 1)

(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")
