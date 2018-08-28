#! /bin/bash

rosservice call /gazebo/set_model_configuration --wait \
"model_name: 'movo'
urdf_param_name: 'robot_description'
joint_names:
- 'linear_joint'
- 'pan_joint'
- 'tilt_joint'
- 'right_shoulder_pan_joint'
- 'right_shoulder_lift_joint'
- 'right_elbow_joint'
- 'right_wrist_1_joint'
- 'right_wrist_2_joint'
- 'right_wrist_3_joint'
- 'left_shoulder_pan_joint'
- 'left_shoulder_lift_joint'
- 'left_elbow_joint'
- 'left_wrist_1_joint'
- 'left_wrist_2_joint'
- 'left_wrist_3_joint'
joint_positions:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0"

sleep 5

rosservice call /gazebo/unpause_physics