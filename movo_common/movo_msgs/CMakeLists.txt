cmake_minimum_required(VERSION 2.8.3)
project(movo_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(DIRECTORY msg FILES 
                                ConfigCmd.msg 
                                Battery.msg 
                                Status.msg 
                                Propulsion.msg 
                                Dynamics.msg 
                                Configuration.msg
                                CtlParams.msg 
                                Faultlog.msg
                                JacoJointCmd.msg
                                JacoStatus.msg
                                GripperCmd.msg
                                GripperStat.msg
                                LinearActuatorCmd.msg
                                JacoCartesianVelocityCmd.msg
                                MotionTestCmd.msg
                                KinovaActuatorFdbk.msg
                                PanTiltActuatorFdbk.msg
                                PVA.msg
                                PanTiltCmd.msg
                                PanTiltFdbk.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

                      
