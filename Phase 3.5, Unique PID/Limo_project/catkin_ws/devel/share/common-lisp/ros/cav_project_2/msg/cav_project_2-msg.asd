
(cl:in-package :asdf)

(defsystem "cav_project_2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ControlInfo" :depends-on ("_package_ControlInfo"))
    (:file "_package_ControlInfo" :depends-on ("_package"))
    (:file "QP_solution" :depends-on ("_package_QP_solution"))
    (:file "_package_QP_solution" :depends-on ("_package"))
    (:file "limo_info" :depends-on ("_package_limo_info"))
    (:file "_package_limo_info" :depends-on ("_package"))
    (:file "limo_state" :depends-on ("_package_limo_state"))
    (:file "_package_limo_state" :depends-on ("_package"))
    (:file "limo_state_matrix" :depends-on ("_package_limo_state_matrix"))
    (:file "_package_limo_state_matrix" :depends-on ("_package"))
  ))