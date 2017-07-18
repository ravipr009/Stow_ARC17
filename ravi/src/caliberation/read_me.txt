....paste the following commands into the terminals....

roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=ur10-pc

roslaunch ur10_mohit_moveit_config ur10_moveit_planning_execution.launch

roslaunch ur10_mohit_moveit_config moveit_rviz.launch config:=true

rosrun caliberation store_marker_wrt_world_and_joint_angles

rosrun caliberation marker_wrt_wrist2_and_kinect_points_calib

rosrun caliberation calculation_of_R_and_T

rosrun caliberation error_calculation

rosrun caliberation test_caliberation
