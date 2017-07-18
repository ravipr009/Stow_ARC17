#ifndef __ARC_2017_STOWING_H_
#define __ARC_2017_STOWING_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/JointLimits.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <math.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <visualization_msgs/Marker.h>

#include <apc_nozel/get_points2.h>
#include <apc_nozel/actuator_frame2.h>

#include <pthread.h>
#include <fstream>

#include <apc_nozel/gripper_suction_controller.h>
#include <apc_nozel/computer_vision.h>
#include <apc_nozel/write_stow_data.h>
#include <apc_nozel/stowToteContents.h>

#include <eigen_conversions/eigen_msg.h>

#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/WrenchStamped.h>
#include<stow_class/computer_vision_stowing.h>
#include<stow_class/pose.h>
#include<stow_class/objects_info.h>


#include <eigen_conversions/eigen_msg.h>

//#include "bin.h"

/*
 * 1. Toilet_Brush, 2. Avery_Binder, 3. Balloons, 4. Band_Aid_Tape, 5. Bath_Sponge, 6. Black_Fashion_Gloves, 7. Burts_Bees_Baby_Wipes
   8. Colgate_Toothbrush_4PK, 9. Composition_Book, 10. Crayons, 11. Duct_Tape, 12. Epsom_Salts, 13. Expo_Eraser, 14. Fiskars_Scissors
   15. Flashlight, 16. Glue_Sticks, 17. Hand_Weight, 18. Hanes_Socks, 19. Hinged_Ruled_Index_Cards, 20. Ice_Cube_Tray, 21. Irish_Spring_Soap
   22. Laugh_Out_Loud_Jokes, 23. Marbles, 24. Measuring_Spoons, 25. Mesh_Cup, 26. Mouse_Traps, 27. Pie_Plates, 28. Plastic_Wine_Glass
   29. Poland_Spring_Water, 30. Reynolds_Wrap, 31. Robots_DVD, 32. Robots_Everywhere, 33. Scotch_Sponges, 34. Speed_Stick, 35. White_Facecloth
   36. Table_Cloth, 37. Tennis_Ball_Container, 38. Ticonderoga_Pencils, 39. Tissue_Box, 40.Windex
*/



class Stowing_ARC_2017 {
public:
    Stowing_ARC_2017(std::string group_name, std::string group_planner_ID, ros::NodeHandle& nh);

    pthread_t broadcast_ensenso_frame;
    void *broadcast_kf(void *threadid);

    std::vector<double> tote_view_joint_angles;

    std::vector<int> ID;
    std::vector<std::string> name;
    std::vector<bool> ignore_normal;
    std::vector<std::string> weight;
    std::vector<std::string> size;
    std::vector<int> push_object;
    std::vector<double> matrix_data;

    int object_ID;

    int total_no_of_objects;

    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroup* group;

    //Bin_ARC_2017* bin;

    ros::Publisher pub_points;
    ros::Publisher pub_lines;


    tf::TransformListener tf_listener;

    geometry_msgs::PointStamped centroid_point_wrt_kinect;
    geometry_msgs::PointStamped centroid_point_wrt_world;

    geometry_msgs::PointStamped tote_wrt_world;
    geometry_msgs::PointStamped tote_reference_point;

    geometry_msgs::PointStamped normal_point_wrt_kinect;
    geometry_msgs::PointStamped normal_point_wrt_world;

    geometry_msgs::PointStamped axis_point_wrt_kinect;
    geometry_msgs::PointStamped axis_point_wrt_world;

    geometry_msgs::PointStamped final_pos_wrt_world;
    geometry_msgs::PointStamped final_pos_wrt_suction_point;

    geometry_msgs::PointStamped point_wrt_rack;
    geometry_msgs::PointStamped point_wrt_world;

    bool read_joson_file();

    bool write_jason_file(int bin_number);

    bool move_to_tote();

    bool verify_joint_goal_position( std::vector<double> &goal);

    bool call_vision_service();
    bool call_vision_service_bajrangi();


    bool visualize_points();

    bool reach_normal();

    bool reach_centroid();

    bool linear_movement(geometry_msgs::Pose pose_goal, float speed_factor);

    bool async_linear_movement(geometry_msgs::Pose pose_goal, float speed_factor);

    void modify_normal();

    bool lift_object();

    bool drop_object();

    bool reach_bin(double location[0]);

    std::vector<double> final_joint_angle(geometry_msgs::Pose pose_goal);

    bool joint_angle_movement(std::vector<double> joint_angles, float speed_factor);

    bool verify_linear_position(geometry_msgs::Pose pose_goal);

    bool move_upwards();

    bool plan_to_reach_centroid_via_normal();

    geometry_msgs::Pose normal_point_with_orientation();

    geometry_msgs::Pose touch_centroid_point();

    bool move_towards_centroid_via_normal();

    std::vector<double> nozzel_angle_calculation();

    void write_data_in_file(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points);
    bool testing();


private:

};



#endif //__ARC_2017_STOWING_H_
