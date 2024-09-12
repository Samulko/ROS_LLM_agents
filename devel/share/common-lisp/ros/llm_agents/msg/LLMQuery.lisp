; Auto-generated. Do not edit!


(cl:in-package llm_agents-msg)


;//! \htmlinclude LLMQuery.msg.html

(cl:defclass <LLMQuery> (roslisp-msg-protocol:ros-message)
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

(cl:defclass LLMQuery (<LLMQuery>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LLMQuery>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LLMQuery)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_agents-msg:<LLMQuery> is deprecated: use llm_agents-msg:LLMQuery instead.")))

(cl:ensure-generic-function 'query-val :lambda-list '(m))
(cl:defmethod query-val ((m <LLMQuery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_agents-msg:query-val is deprecated.  Use llm_agents-msg:query instead.")
  (query m))

(cl:ensure-generic-function 'agent_id-val :lambda-list '(m))
(cl:defmethod agent_id-val ((m <LLMQuery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_agents-msg:agent_id-val is deprecated.  Use llm_agents-msg:agent_id instead.")
  (agent_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LLMQuery>) ostream)
  "Serializes a message object of type '<LLMQuery>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LLMQuery>) istream)
  "Deserializes a message object of type '<LLMQuery>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LLMQuery>)))
  "Returns string type for a message object of type '<LLMQuery>"
  "llm_agents/LLMQuery")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LLMQuery)))
  "Returns string type for a message object of type 'LLMQuery"
  "llm_agents/LLMQuery")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LLMQuery>)))
  "Returns md5sum for a message object of type '<LLMQuery>"
  "1692929dc490ff37a7139bda085c4e52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LLMQuery)))
  "Returns md5sum for a message object of type 'LLMQuery"
  "1692929dc490ff37a7139bda085c4e52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LLMQuery>)))
  "Returns full string definition for message of type '<LLMQuery>"
  (cl:format cl:nil "string query~%string agent_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LLMQuery)))
  "Returns full string definition for message of type 'LLMQuery"
  (cl:format cl:nil "string query~%string agent_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LLMQuery>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'query))
     4 (cl:length (cl:slot-value msg 'agent_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LLMQuery>))
  "Converts a ROS message object to a list"
  (cl:list 'LLMQuery
    (cl:cons ':query (query msg))
    (cl:cons ':agent_id (agent_id msg))
))
