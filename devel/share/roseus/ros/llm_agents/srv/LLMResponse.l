;; Auto-generated. Do not edit!


(when (boundp 'llm_agents::LLMResponse)
  (if (not (find-package "LLM_AGENTS"))
    (make-package "LLM_AGENTS"))
  (shadow 'LLMResponse (find-package "LLM_AGENTS")))
(unless (find-package "LLM_AGENTS::LLMRESPONSE")
  (make-package "LLM_AGENTS::LLMRESPONSE"))
(unless (find-package "LLM_AGENTS::LLMRESPONSEREQUEST")
  (make-package "LLM_AGENTS::LLMRESPONSEREQUEST"))
(unless (find-package "LLM_AGENTS::LLMRESPONSERESPONSE")
  (make-package "LLM_AGENTS::LLMRESPONSERESPONSE"))

(in-package "ROS")





(defclass llm_agents::LLMResponseRequest
  :super ros::object
  :slots (_query _agent_id ))

(defmethod llm_agents::LLMResponseRequest
  (:init
   (&key
    ((:query __query) "")
    ((:agent_id __agent_id) "")
    )
   (send-super :init)
   (setq _query (string __query))
   (setq _agent_id (string __agent_id))
   self)
  (:query
   (&optional __query)
   (if __query (setq _query __query)) _query)
  (:agent_id
   (&optional __agent_id)
   (if __agent_id (setq _agent_id __agent_id)) _agent_id)
  (:serialization-length
   ()
   (+
    ;; string _query
    4 (length _query)
    ;; string _agent_id
    4 (length _agent_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _query
       (write-long (length _query) s) (princ _query s)
     ;; string _agent_id
       (write-long (length _agent_id) s) (princ _agent_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _query
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _query (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _agent_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _agent_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass llm_agents::LLMResponseResponse
  :super ros::object
  :slots (_response ))

(defmethod llm_agents::LLMResponseResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass llm_agents::LLMResponse
  :super ros::object
  :slots ())

(setf (get llm_agents::LLMResponse :md5sum-) "59dd40d73b99b23145c3779b008631a3")
(setf (get llm_agents::LLMResponse :datatype-) "llm_agents/LLMResponse")
(setf (get llm_agents::LLMResponse :request) llm_agents::LLMResponseRequest)
(setf (get llm_agents::LLMResponse :response) llm_agents::LLMResponseResponse)

(defmethod llm_agents::LLMResponseRequest
  (:response () (instance llm_agents::LLMResponseResponse :init)))

(setf (get llm_agents::LLMResponseRequest :md5sum-) "59dd40d73b99b23145c3779b008631a3")
(setf (get llm_agents::LLMResponseRequest :datatype-) "llm_agents/LLMResponseRequest")
(setf (get llm_agents::LLMResponseRequest :definition-)
      "string query
string agent_id
---
string response

")

(setf (get llm_agents::LLMResponseResponse :md5sum-) "59dd40d73b99b23145c3779b008631a3")
(setf (get llm_agents::LLMResponseResponse :datatype-) "llm_agents/LLMResponseResponse")
(setf (get llm_agents::LLMResponseResponse :definition-)
      "string query
string agent_id
---
string response

")



(provide :llm_agents/LLMResponse "59dd40d73b99b23145c3779b008631a3")


