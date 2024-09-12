;; Auto-generated. Do not edit!


(when (boundp 'llm_agents::LLMQuery)
  (if (not (find-package "LLM_AGENTS"))
    (make-package "LLM_AGENTS"))
  (shadow 'LLMQuery (find-package "LLM_AGENTS")))
(unless (find-package "LLM_AGENTS::LLMQUERY")
  (make-package "LLM_AGENTS::LLMQUERY"))

(in-package "ROS")
;;//! \htmlinclude LLMQuery.msg.html


(defclass llm_agents::LLMQuery
  :super ros::object
  :slots (_query _agent_id ))

(defmethod llm_agents::LLMQuery
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

(setf (get llm_agents::LLMQuery :md5sum-) "1692929dc490ff37a7139bda085c4e52")
(setf (get llm_agents::LLMQuery :datatype-) "llm_agents/LLMQuery")
(setf (get llm_agents::LLMQuery :definition-)
      "string query
string agent_id

")



(provide :llm_agents/LLMQuery "1692929dc490ff37a7139bda085c4e52")


