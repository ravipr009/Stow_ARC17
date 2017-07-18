#include <ros/ros.h>
#include <apc_nozel/actuator_frame2.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotState.h>
#include <tf/transform_listener.h>
#include <moveit/robot_trajectory/robot_trajectory.h>


bool set_val(apc_nozel::actuator_frame2::Request &req, apc_nozel::actuator_frame2::Response &res)
{

    tf::TransformListener listener;

    geometry_msgs::PointStamped centroid_point_wrt_world,
            centroid_point_wrt_suction_point,
            normal_point_wrt_world,
            normal_point_wrt_suction_point;

    centroid_point_wrt_world.header.frame_id = "world";

    centroid_point_wrt_world.point = req.centroid_wrt_world;

    listener.waitForTransform( "iitk_link", "world", ros::Time(0), ros::Duration(3));

    listener.transformPoint("iitk_link", centroid_point_wrt_world, centroid_point_wrt_suction_point);



    normal_point_wrt_world.header.frame_id = "world";

    normal_point_wrt_world.point = req.normal_wrt_world;

    listener.waitForTransform( "iitk_link", "world", ros::Time(0), ros::Duration(3));

    listener.transformPoint("iitk_link", normal_point_wrt_world, normal_point_wrt_suction_point);


    res.centroid_wrt_suction_point = centroid_point_wrt_suction_point.point;
    res.normal_wrt_suction_point = normal_point_wrt_suction_point.point;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_points_wrt_suction_point");
    ros::NodeHandle n;
    moveit::planning_interface::MoveGroup group("robo_arm");

    ros::ServiceServer service = n.advertiseService("get_suction_points_topic", set_val);

    ros::spin();

    return 0;
}
