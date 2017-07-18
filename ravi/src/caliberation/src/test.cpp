#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tip_frame_broadcast");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("robo_arm");

    tf::TransformBroadcaster br;

    tf::StampedTransform transform;

    ros::Rate rate(10.0);

    while (ros::ok())
    {


        transform.setOrigin( tf::Vector3(0.053, 0, 0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1));

        //        formation of tip frame

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ee_link", "tip_frame"));

        rate.sleep();
    }
    return 0;
}
