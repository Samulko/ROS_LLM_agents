
(cl:in-package :asdf)

(defsystem "llm_agents-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LLMResponse" :depends-on ("_package_LLMResponse"))
    (:file "_package_LLMResponse" :depends-on ("_package"))
  ))