#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <fstream>
#include <iostream>
#include <pthread.h>



char choice = 'l';

void *choices(void *threadid)
{
    while(1)
    {
        std::cin >> choice;
        long tid;
        tid = (long)threadid;
        usleep(1000);
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "store_marker_and_joints");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("robo_arm");

    geometry_msgs::PointStamped tip_point_wrt_world;
    geometry_msgs::PointStamped reference_point_wrt_tip_link;

    tf::TransformListener listener;

    std::vector <double> joint_angles;

    std::ofstream myfile;

    pthread_t threads;

    pthread_create(&threads, NULL, choices, NULL);

    std::cout <<"Press 'm' to record markers coordinates or Press 'j' to record joint angles or Press 'e' to exit\n";

    while(1)
    {

        if(choice == 'e')
            break;

        if(choice == 'm')
        {

            myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates.txt", std::ios::out);

            std::cout <<"press 's' to store marker coordinates or 'b' to exit\n";

            while(1)
            {
                reference_point_wrt_tip_link.header.frame_id = "nozele";

                reference_point_wrt_tip_link.point.x = 0;
                reference_point_wrt_tip_link.point.y = 0;
                reference_point_wrt_tip_link.point.z = 0;

                if(choice == 's')
                {
                    std::cout <<"press 's' to store marker coordinates or 'b' to exit\n";

                    listener.waitForTransform( "world", "nozele", ros::Time(0), ros::Duration(3));

                    listener.transformPoint("world", reference_point_wrt_tip_link, tip_point_wrt_world);

                    myfile << std::setprecision(15) << tip_point_wrt_world.point.x << ", "
                           << tip_point_wrt_world.point.y << ", "
                           << tip_point_wrt_world.point.z << "\n";

                    std::cout << tip_point_wrt_world.point.x << ", "
                              << tip_point_wrt_world.point.y << ", "
                              << tip_point_wrt_world.point.z << "\n";

                    choice = 'm';
                }

                if(choice == 'b')
                {
                    choice = 'l';
                    break;
                }

            }

            myfile.close();
        }

        if(choice == 'j')
        {
            myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/joint_angles.txt", std::ios::out);

            std::cout <<"press 's' to store joint angles or 'b' to exit\n";

            while(1)
            {
                if(choice == 's')
                {
                    std::cout <<"press 's' to store joint angles or 'b' to exit\n";

                    joint_angles.resize(6);

                    joint_angles = group.getCurrentJointValues();

                    std::cout << joint_angles.at(0) <<", "
                              << joint_angles.at(1) <<", "
                              << joint_angles.at(2) <<", "
                              << joint_angles.at(3) <<", "
                              << joint_angles.at(4) <<", "
                              << joint_angles.at(5) <<"\n";

                    myfile << std::setprecision(15) << joint_angles.at(0) <<", "
                           << joint_angles.at(1) <<", "
                           << joint_angles.at(2) <<", "
                           << joint_angles.at(3) <<", "
                           << joint_angles.at(4) <<", "
                           << joint_angles.at(5) <<"\n";

                    joint_angles.clear();

                    choice = 'j';
                }
                if(choice == 'b')
                {
                    choice = 'l';
                    break;
                }

            }

            myfile.close();
        }
    }


    return 0;
}
