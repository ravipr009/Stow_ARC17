#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <pthread.h>


float elbow_angle, shoulder_lift_angle, shoulder_pan_angle, wrist_1_angle ,wrist_2_angle ,wrist_3_angle, angle ;

double joint_value;

bool get_data = true;

void publishJointState(ros::Publisher &publisher, sensor_msgs::JointState &joint_state)
{

    ros::Rate loop_rate(1000);

    // Just publish for few times
    for(int i = 0; i < 5; i++)
    {
        publisher.publish(joint_state);
        loop_rate.sleep();
    }

    return;
}



void *nozel_angle(void *threadid)
{
    while(1)
        std::cin >> joint_value;

}

void chatterCallback(const sensor_msgs::JointState &joint_angles)
{

    for (int i = 0; i < joint_angles.position.size(); i++)
    {
        if(joint_angles.name.at(i) == "elbow_joint")
            elbow_angle = joint_angles.position.at(i);
        if(joint_angles.name.at(i) == "shoulder_lift_joint")
            shoulder_lift_angle = joint_angles.position.at(i);
        if(joint_angles.name.at(i) == "shoulder_pan_joint")
            shoulder_pan_angle = joint_angles.position.at(i);
        if(joint_angles.name.at(i) == "wrist_1_joint")
            wrist_1_angle = joint_angles.position.at(i);
        if(joint_angles.name.at(i) == "wrist_2_joint")
            wrist_2_angle = joint_angles.position.at(i);
        if(joint_angles.name.at(i) == "wrist_3_joint")
            wrist_3_angle = joint_angles.position.at(i);
    }
    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_js");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle n;

    pthread_t threads;

    pthread_create(&threads, NULL, nozel_angle, NULL);

    ros::Publisher sim_joint_publisher = n.advertise<sensor_msgs::JointState>("/suction_suction_joint_topic", 1); // give your topic name here

    ros ::Subscriber joint_angle_sub;

    sensor_msgs::JointState suction_jt_state;

    suction_jt_state.name.resize(7);

    suction_jt_state.name[0] = "elbow_joint";
    suction_jt_state.name[1] = "shoulder_lift_joint";
    suction_jt_state.name[2] = "shoulder_pan_joint";
    suction_jt_state.name[3] = "wrist_1_joint";
    suction_jt_state.name[4] = "wrist_2_joint";
    suction_jt_state.name[5] = "wrist_3_joint";
    suction_jt_state.name[6] = "suction_base_joint"; // provide your revolute joint name


    suction_jt_state.position.resize(7); // total number of joints

    suction_jt_state.position[0] = elbow_angle;
    suction_jt_state.position[1] = shoulder_lift_angle;
    suction_jt_state.position[2] = shoulder_pan_angle;
    suction_jt_state.position[3] = wrist_1_angle;
    suction_jt_state.position[4] = wrist_2_angle;
    suction_jt_state.position[5] = wrist_3_angle;
    suction_jt_state.position[6] = joint_value;

    while(ros::ok())
    {
        joint_angle_sub = n.subscribe("/joint_states",  2, chatterCallback);

        publishJointState(sim_joint_publisher, suction_jt_state);



        suction_jt_state.position[0] = elbow_angle;
        suction_jt_state.position[1] = shoulder_lift_angle;
        suction_jt_state.position[2] = shoulder_pan_angle;
        suction_jt_state.position[3] = wrist_1_angle;
        suction_jt_state.position[4] = wrist_2_angle;
        suction_jt_state.position[5] = wrist_3_angle;
        suction_jt_state.position[6] = joint_value;

    }

    return 0;
}
