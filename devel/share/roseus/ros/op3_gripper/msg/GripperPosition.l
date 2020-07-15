;; Auto-generated. Do not edit!


(when (boundp 'op3_gripper::GripperPosition)
  (if (not (find-package "OP3_GRIPPER"))
    (make-package "OP3_GRIPPER"))
  (shadow 'GripperPosition (find-package "OP3_GRIPPER")))
(unless (find-package "OP3_GRIPPER::GRIPPERPOSITION")
  (make-package "OP3_GRIPPER::GRIPPERPOSITION"))

(in-package "ROS")
;;//! \htmlinclude GripperPosition.msg.html


(defclass op3_gripper::GripperPosition
  :super ros::object
  :slots (_Left _Right ))

(defmethod op3_gripper::GripperPosition
  (:init
   (&key
    ((:Left __Left) 0)
    ((:Right __Right) 0)
    )
   (send-super :init)
   (setq _Left (round __Left))
   (setq _Right (round __Right))
   self)
  (:Left
   (&optional __Left)
   (if __Left (setq _Left __Left)) _Left)
  (:Right
   (&optional __Right)
   (if __Right (setq _Right __Right)) _Right)
  (:serialization-length
   ()
   (+
    ;; int64 _Left
    8
    ;; int64 _Right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _Left
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Left (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Left) (= (length (_Left . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Left . bv) 0) 0) s)
              (write-long (ash (elt (_Left . bv) 1) -1) s))
             ((and (class _Left) (= (length (_Left . bv)) 1)) ;; big1
              (write-long (elt (_Left . bv) 0) s)
              (write-long (if (>= _Left 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Left s)(write-long (if (>= _Left 0) 0 #xffffffff) s)))
     ;; int64 _Right
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Right (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Right) (= (length (_Right . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Right . bv) 0) 0) s)
              (write-long (ash (elt (_Right . bv) 1) -1) s))
             ((and (class _Right) (= (length (_Right . bv)) 1)) ;; big1
              (write-long (elt (_Right . bv) 0) s)
              (write-long (if (>= _Right 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Right s)(write-long (if (>= _Right 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _Left
#+(or :alpha :irix6 :x86_64)
      (setf _Left (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Left (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _Right
#+(or :alpha :irix6 :x86_64)
      (setf _Right (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Right (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get op3_gripper::GripperPosition :md5sum-) "5b0e67f9a5c183da3139b32cb4f14d70")
(setf (get op3_gripper::GripperPosition :datatype-) "op3_gripper/GripperPosition")
(setf (get op3_gripper::GripperPosition :definition-)
      "int64 Left
int64 Right

")



(provide :op3_gripper/GripperPosition "5b0e67f9a5c183da3139b32cb4f14d70")


