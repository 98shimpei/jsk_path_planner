(setq base-face (instance face :init :vertices `(#f(-900.0 -900.0 0.0) #f(-900.0 900.0 0.0) #f(900.0 900.0 0.0) #f(900.0 -900.0 0.0))))
(setq hole-candidate (instance face :init :vertices `(#f(673.423 -757.887 0.0) #f(741.536 -607.821 0.0) #f(832.904 -445.383 0.0) #f(899.693 -328.544 0.0) #f(910.313 -914.276 0.0) #f(728.528 -813.309 0.0))))

(format t "hole candidates ~A~%" (length hole-candidates))
(setq base-box (make-prism (reverse (cdr (reverse (send base-face :vertices)))) 10))

(setq hole-candidate (elt hole-candidates hole-id))
(setq hole-candidate
      (change-face-size hole-candidate (* -0.01 face-scale) (* 0.001 face-scale)))
(print "base-face")
(print (send base-face :vertices))
(print "hole-face")
(print (send hole-candidate :vertices))
(setq hole-box
      (make-prism
       (reverse (cdr (reverse (send hole-candidate :vertices)))) 30))
(send hole-box :translate #f(0 0 -10))
(print "base-box")
(print (send base-box :vertices))
(print "hole-box")
(print (send hole-box :vertices))


(send hole-box :set-color :red)
objects (list base-box hole-box)

(setq base-box (body- base-box hole-box))

(setq *epsilon* 1e-5)
(setq *contact-threshold* 8e-5)

(setq base-box-faces (send base-box :faces))
(dotimes (face-id (length base-box-faces))
  (when (eq (elt (send (elt base-box-faces face-id) :normal) 2) 1.0)
    (print (elt base-box-faces face-id))
    ;; (nconc base-box-up-faces (list (elt base-box-faces face-id)))
    (setq base-box-up-faces (append base-box-up-faces (list (elt base-box-faces face-id))))
    (print "found")
    (print (send (elt base-box-faces face-id) :vertices))
    (print base-box-up-faces)
    )
  )


(setq f1 (instance face
                   :init
                   :vertices (list #f(-200.0 -200.0 0.0)
                                   #f(-200.0 200.0 0.0)
                                   #f(200.0 200.0 0.0)
                                   #f(200.0 -200.0 0.0))))
