; Auto-generated. Do not edit!


(cl:in-package llm_agents-srv)


;//! \htmlinclude LLMResponse-request.msg.html

(cl:defclass <LLMResponse-request> (roslisp-msg-protocol:ros-message)
  ((query
    :reader query
    :initarg :query
    :type cl:string
    :initform "")
   (agent_id
    :reader agent_id
    :initarg :agent_id
    :type cl:string
    :initform ""))
)

(cl:defclass LLMResponse-request (<LLMResponse-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LLMResponse-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LLMResponse-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_agents-srv:<LLMResponse-request> is deprecated: use llm_agents-srv:LLMResponse-request instead.")))

(cl:ensure-generic-function 'query-val :lambda-list '(m))
(cl:defmethod query-val ((m <LLMResponse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_agents-srv:query-val is deprecated.  Use llm_agents-srv:query instead.")
  (query m))

(cl:ensure-generic-function 'agent_id-val :lambda-list '(m))
(cl:defmethod agent_id-val ((m <LLMResponse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_agents-srv:agent_id-val is deprecated.  Use llm_agents-srv:agent_id instead.")
  (agent_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LLMResponse-request>) ostream)
  "Serializes a message object of type '<LLMResponse-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'query))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'query))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'agent_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'agent_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LLMResponse-request>) istream)
  "Deserializes a message object of type '<LLMResponse-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'query) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'query) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'agent_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'agent_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LLMResponse-request>)))
  "Returns string type for a service object of type '<LLMResponse-request>"
  "llm_agents/LLMResponseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LLMResponse-request)))
  "Returns string type for a service object of type 'LLMResponse-request"
  "llm_agents/LLMResponseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LLMResponse-request>)))
  "Returns md5sum for a message object of type '<LLMResponse-request>"
  "59dd40d73b99b23145c3779b008631a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LLMResponse-request)))
  "Returns md5sum for a message object of type 'LLMResponse-request"
  "59dd40d73b99b23145c3779b008631a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LLMResponse-request>)))
  "Returns full string definition for message of type '<LLMResponse-request>"
  (cl:format cl:nil "string query~%string agent_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LLMResponse-request)))
  "Returns full string definition for message of type 'LLMResponse-request"
  (cl:format cl:nil "string query~%string agent_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LLMResponse-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'query))
     4 (cl:length (cl:slot-value msg 'agent_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LLMResponse-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LLMResponse-request
    (cl:cons ':query (query msg))
    (cl:cons ':agent_id (agent_id msg))
))
;//! \htmlinclude LLMResponse-response.msg.html

(cl:defclass <LLMResponse-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass LLMResponse-response (<LLMResponse-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LLMResponse-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LLMResponse-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_agents-srv:<LLMResponse-response> is deprecated: use llm_agents-srv:LLMResponse-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <LLMResponse-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_agents-srv:response-val is deprecated.  Use llm_agents-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LLMResponse-response>) ostream)
  "Serializes a message object of type '<LLMResponse-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LLMResponse-response>) istream)
  "Deserializes a message object of type '<LLMResponse-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LLMResponse-response>)))
  "Returns string type for a service object of type '<LLMResponse-response>"
  "llm_agents/LLMResponseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LLMResponse-response)))
  "Returns string type for a service object of type 'LLMResponse-response"
  "llm_agents/LLMResponseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LLMResponse-response>)))
  "Returns md5sum for a message object of type '<LLMResponse-response>"
  "59dd40d73b99b23145c3779b008631a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LLMResponse-response)))
  "Returns md5sum for a message object of type 'LLMResponse-response"
  "59dd40d73b99b23145c3779b008631a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LLMResponse-response>)))
  "Returns full string definition for message of type '<LLMResponse-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LLMResponse-response)))
  "Returns full string definition for message of type 'LLMResponse-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LLMResponse-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LLMResponse-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LLMResponse-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LLMResponse)))
  'LLMResponse-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LLMResponse)))
  'LLMResponse-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LLMResponse)))
  "Returns string type for a service object of type '<LLMResponse>"
  "llm_agents/LLMResponse")