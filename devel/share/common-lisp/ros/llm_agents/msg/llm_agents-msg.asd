
(cl:in-package :asdf)

(defsystem "llm_agents-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LLMQuery" :depends-on ("_package_LLMQuery"))
    (:file "_package_LLMQuery" :depends-on ("_package"))
  ))