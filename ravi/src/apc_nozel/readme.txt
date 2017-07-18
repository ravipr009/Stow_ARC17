connect to ur10 network...

paste the following commands in terminals....

roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=ur5-pc

roslaunch ur5_mohit_moveit_config ur5_moveit_planning_execution.launch

roslaunch ur5_mohit_moveit_config moveit_rviz.launch config:=true

rosrun apc_nozel publish_joint2

rosrun apc_nozel actuator_point_server2

rosrun apc_nozel demo_actuator2
