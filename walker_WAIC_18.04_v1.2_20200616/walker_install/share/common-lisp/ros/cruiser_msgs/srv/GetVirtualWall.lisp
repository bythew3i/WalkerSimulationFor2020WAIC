; Auto-generated. Do not edit!


(cl:in-package cruiser_msgs-srv)


;//! \htmlinclude GetVirtualWall-request.msg.html

(cl:defclass <GetVirtualWall-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetVirtualWall-request (<GetVirtualWall-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVirtualWall-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVirtualWall-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cruiser_msgs-srv:<GetVirtualWall-request> is deprecated: use cruiser_msgs-srv:GetVirtualWall-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVirtualWall-request>) ostream)
  "Serializes a message object of type '<GetVirtualWall-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVirtualWall-request>) istream)
  "Deserializes a message object of type '<GetVirtualWall-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVirtualWall-request>)))
  "Returns string type for a service object of type '<GetVirtualWall-request>"
  "cruiser_msgs/GetVirtualWallRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVirtualWall-request)))
  "Returns string type for a service object of type 'GetVirtualWall-request"
  "cruiser_msgs/GetVirtualWallRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVirtualWall-request>)))
  "Returns md5sum for a message object of type '<GetVirtualWall-request>"
  "bf9b4c73a9da4988a548c8f4357827ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVirtualWall-request)))
  "Returns md5sum for a message object of type 'GetVirtualWall-request"
  "bf9b4c73a9da4988a548c8f4357827ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVirtualWall-request>)))
  "Returns full string definition for message of type '<GetVirtualWall-request>"
  (cl:format cl:nil "# Get the virtual wall as a cruiser_msgs/VirtualWall
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVirtualWall-request)))
  "Returns full string definition for message of type 'GetVirtualWall-request"
  (cl:format cl:nil "# Get the virtual wall as a cruiser_msgs/VirtualWall
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVirtualWall-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVirtualWall-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVirtualWall-request
))
;//! \htmlinclude GetVirtualWall-response.msg.html

(cl:defclass <GetVirtualWall-response> (roslisp-msg-protocol:ros-message)
  ((wall
    :reader wall
    :initarg :wall
    :type cruiser_msgs-msg:VirtualWall
    :initform (cl:make-instance 'cruiser_msgs-msg:VirtualWall)))
)

(cl:defclass GetVirtualWall-response (<GetVirtualWall-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVirtualWall-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVirtualWall-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cruiser_msgs-srv:<GetVirtualWall-response> is deprecated: use cruiser_msgs-srv:GetVirtualWall-response instead.")))

(cl:ensure-generic-function 'wall-val :lambda-list '(m))
(cl:defmethod wall-val ((m <GetVirtualWall-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cruiser_msgs-srv:wall-val is deprecated.  Use cruiser_msgs-srv:wall instead.")
  (wall m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVirtualWall-response>) ostream)
  "Serializes a message object of type '<GetVirtualWall-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wall) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVirtualWall-response>) istream)
  "Deserializes a message object of type '<GetVirtualWall-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wall) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVirtualWall-response>)))
  "Returns string type for a service object of type '<GetVirtualWall-response>"
  "cruiser_msgs/GetVirtualWallResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVirtualWall-response)))
  "Returns string type for a service object of type 'GetVirtualWall-response"
  "cruiser_msgs/GetVirtualWallResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVirtualWall-response>)))
  "Returns md5sum for a message object of type '<GetVirtualWall-response>"
  "bf9b4c73a9da4988a548c8f4357827ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVirtualWall-response)))
  "Returns md5sum for a message object of type 'GetVirtualWall-response"
  "bf9b4c73a9da4988a548c8f4357827ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVirtualWall-response>)))
  "Returns full string definition for message of type '<GetVirtualWall-response>"
  (cl:format cl:nil "cruiser_msgs/VirtualWall wall
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVirtualWall-response)))
  "Returns full string definition for message of type 'GetVirtualWall-response"
  (cl:format cl:nil "cruiser_msgs/VirtualWall wall
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVirtualWall-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wall))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVirtualWall-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVirtualWall-response
    (cl:cons ':wall (wall msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetVirtualWall)))
  'GetVirtualWall-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetVirtualWall)))
  'GetVirtualWall-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVirtualWall)))
  "Returns string type for a service object of type '<GetVirtualWall>"
  "cruiser_msgs/GetVirtualWall")