;;;
;;; Copyright (c) 2022, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :demos)

(defparameter *apartment-object-spawning-poses*
  '((:jeroen-cup
     "cabinet1_coloksu_level4"
     ((0.20 -0.1 0.08) (0 0 -1 0)))
    (:cup
     "cabinet1_coloksu_level4"
     ((0.15 0.15 0.08) (0 0 -1 0)))))

(defparameter *object-grasps*
  '(;; (:cup . (:left-side :right-side :back :front))
    ))

(defun initialize-apartment ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (kill-and-detach-all)
  (setf (btr:joint-state (btr:get-environment-object)
                         "cabinet1_door_top_left_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "cabinet7_door_bottom_left_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  ;; (coe:clear-belief)

  (btr:clear-costmap-vis-object))

(defun finalize ()
  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

  ;;(when ccl::*is-logging-enabled*
  ;;  (ccl::export-log-to-owl "ease_milestone_2018.owl")
  ;;  (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl"))
  (sb-ext:gc :full t))


(defun apartment-demo ()
  (urdf-proj:with-simulated-robot
    (initialize-apartment)
    (setf btr:*visibility-threshold* 0.7)
    (when cram-projection:*projection-environment*
      (spawn-objects-on-fixed-spots
       :object-types '(:jeroen-cup :cup)
       :spawning-poses-relative *apartment-object-spawning-poses*))
    (park-robot (cl-transforms-stamped:make-pose-stamped
                 cram-tf:*fixed-frame*
                 0.0
                 (cl-transforms:make-3d-vector 1.5 1.5 0.0)
                 (cl-transforms:make-quaternion 0 0 1 0)))

    (let ((?location-in-cupboard
            (a location
               (on (an object
                       (type shelf)
                       (urdf-name cabinet1-coloksu-level4)
                       (part-of apartment)
                       (location (a location
                                    (in (an object
                                            (type cupboard)
                                            (urdf-name cabinet1-door-top-left)
                                            (part-of apartment)))))))))
          (?location-on-island
            (a location
               (on (an object
                       (type surface)
                       (urdf-name island)
                       (part-of apartment)))
               (side back)
               (range 0.4)))
          (?location-in-dishwasher
            (a location
               (above (an object
                          (type drawer)
                          (urdf-name cabinet7-drawer-middle)
                          (part-of apartment)
                          (location (a location
                                       (in (an object
                                               (type dishwasher)
                                               (urdf-name cabinet7)
                                               (part-of apartment)))))))
               (for (desig:an object
                              (type jeroen-cup)
                              ;; need a name because of the attachment
                              (name some-name)))
               (attachments (jeroen-cup-in-dishwasher)))))

      ;; bring cup from cupboard to table
     (exe:perform
      (an action
          (type transporting)
          (object (an object
                      (type jeroen-cup)
                      (location ?location-in-cupboard)))
          (target ?location-on-island)))

     ;; put cup from island into dishwasher
     (exe:perform
      (an action
          (type transporting)
          (object (an object
                      (type jeroen-cup)
                      (location ?location-on-island)))
          (target ?location-in-dishwasher)))

      ;; bring cup from dishwasher to cupboard
      (exe:perform
       (an action
           (type transporting)
           (object (an object
                       (type jeroen-cup)
                       (location ?location-in-dishwasher)))
           (target ?location-in-cupboard)))))

    (finalize))