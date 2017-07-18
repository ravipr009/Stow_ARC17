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

#include <eigen_conversions/eigen_msg.h>
#include <apc_nozel/apc_simulation.h>

#include <geometry_msgs/WrenchStamped.h>


#include <iostream>


using namespace Eigen;

float force_x, force_y, force_z;
bool next_step = false;
int count_ = 1;



template <class myType>
myType my_sign (myType a)
{
    if(a >= 0)
        return 1;
    else
        return -1;
}

void write_data_in_file(int k, float fx, float fy, float fz)
{

    std::fstream myfile;
    if(k == 1)
    {
        myfile.open ("/home/mohit/Desktop/ros_ws/src/apc_nozel/force.txt", std::ios::out);
        myfile << "clear all\n";
    }
    else
        myfile.open ("/home/mohit/Desktop/ros_ws/src/apc_nozel/force.txt", std::ios::out | std::ios::app);


    myfile << "force(" << k << ", :" <<") = ["<< fx << " " << fy << " " << fz <<"];\n";

    myfile.close();

    return;

}

std::vector<trajectory_msgs::JointTrajectoryPoint> quintic_parabolic_blend(std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points, float max_acc, float T, float t_b1)
{
    int len = vector_points.size();

    float initial_joint_angles[6];
    float final_joint_angles[6];

    for(int j=0; j<6; j++)
    {
        initial_joint_angles[j] = vector_points.at(0).positions.at(j);
        final_joint_angles[j] = vector_points.at(len - 1).positions.at(j);
    }

    float diff[6];
    float max_diff = 0;
    for(int j = 0; j<6; j++)
    {

        if(initial_joint_angles[j]*final_joint_angles[j] > 0)
            diff[j] = final_joint_angles[j] - initial_joint_angles[j] ;
        else
            diff[j] = my_sign<float>(final_joint_angles[j])*(abs(initial_joint_angles[j]) + abs(final_joint_angles[j]));


        if(abs(diff[j]) > max_diff)
            max_diff = abs(diff[j]);
    }

    double a[6][6];

    for (int joint_angle = 0; joint_angle<6; joint_angle++)
    {
        for (int variable = 0; variable<6; variable++)
        {
            if(variable == 0)
                a[joint_angle][variable] = initial_joint_angles[joint_angle];

            if((variable == 1)||(variable == 2))
                a[joint_angle][variable] = 0;

            if(variable == 3)
                a[joint_angle][variable] = (1/(2*pow(T,3)))*(20*(final_joint_angles[joint_angle] - initial_joint_angles[joint_angle]));

            if(variable == 4)
                a[joint_angle][variable] = -(1/(2*pow(T,4)))*(30*(final_joint_angles[joint_angle] - initial_joint_angles[joint_angle]));

            if(variable == 5)
                a[joint_angle][variable] = (1/(2*pow(T,5)))*(12*(final_joint_angles[joint_angle] - initial_joint_angles[joint_angle]));

        }
    }

    float temp_a = a[0][5];
    int temp_j = 0;

    for (int joint_angle = 0; joint_angle<6; joint_angle++)
    {
        if(temp_a < abs(a[joint_angle][5]))
        {
            temp_a = abs(a[joint_angle][5]);
            temp_j = joint_angle;
        }
    }


    float t = (-2*12*a[temp_j][4] + sqrt(pow(2*12*a[temp_j][4],2) - 4*3*20*a[temp_j][5]*6*a[temp_j][3]))/(2*3*20*a[temp_j][5]);
    if(t > T/2)
        t = (-2*12*a[temp_j][4] - sqrt(pow(2*12*a[temp_j][4],2) - 4*3*20*a[temp_j][5]*6*a[temp_j][3]))/(2*3*20*a[temp_j][5]);

    float max_acc_matrix[6];
    float temp_max_acc = 0;

    for(int j = 0; j<6; j++)
    {
        max_acc_matrix[j] = 20*a[j][5]*pow(t,3) + 12*a[j][4]*pow(t,2) + 6*a[j][3]*t + 2*a[j][2];
        if(abs(max_acc_matrix[j]) > temp_max_acc)
            temp_max_acc = abs(max_acc_matrix[j]);
    }

    if(temp_max_acc < max_acc)
        max_acc = temp_max_acc;

    for(int j = 0; j<6; j++)
        max_acc_matrix[j] = my_sign<float>(diff[j])*max_acc*abs(diff[j]/max_diff);


    MatrixXf companion_matrix(3,3);

    companion_matrix << 0, 0 ,-(2*a[temp_j][2]-max_acc_matrix[temp_j])/(20*a[temp_j][5]),
            1, 0, -(6*a[temp_j][3])/(20*a[temp_j][5]),
            0, 1, -(12*a[temp_j][4])/(20*a[temp_j][5]);


    EigenSolver<MatrixXf> myobject;

    std::complex<double> mycomplex(myobject.compute(companion_matrix).eigenvalues().coeff(0));

    t = sqrt(pow(mycomplex.real(),2) + pow(mycomplex.imag(),2));

    float initial_thetha[6];
    float initial_velocity[6];

    for(int i = 0; i<6; i++)
    {
        initial_thetha[i] = a[i][0] + a[i][1]*t + a[i][2]*pow(t,2) + a[i][3]*pow(t,3) + a[i][4]*pow(t,4) + a[i][5]*pow(t,5);
        initial_velocity[i] =  a[i][1] + 2*a[i][2]*pow(t,1) + 3*a[i][3]*pow(t,2) + 4*a[i][4]*pow(t,3) + 5*a[i][5]*pow(t,4);
    }

    float k[6];
    float s_t[6];

    for(int i = 0; i<6; i++)
    {

        k[i] = (initial_velocity[i] + max_acc_matrix[i]*t_b1)/max_acc_matrix[i];
        s_t[i] = ((final_joint_angles[i] - max_acc_matrix[i]*k[i]*k[i]/2) -
                  (initial_thetha[i] + initial_velocity[i]*t_b1 + 0.5*max_acc_matrix[i]*pow(t_b1,2)))/
                (initial_velocity[i] + max_acc_matrix[i]*t_b1);
    }

    float sudo_thetha[6];
    for(int i = 0; i<6; i++)
    {
        sudo_thetha[i] = initial_thetha[i] + initial_velocity[i]*t_b1 + 0.5*max_acc_matrix[i]*pow(t_b1,2) +
                (initial_velocity[i] + max_acc_matrix[i]*t_b1)*s_t[temp_j];
    }

    float t_b2[6];
    float t_b22[6];
    for(int i = 0; i<6; i++)
    {
        t_b2[i] = sqrt(2*(final_joint_angles[i] - sudo_thetha[i])/max_acc_matrix[i]);
        t_b22[i] = (initial_velocity[i] + max_acc_matrix[i]*t_b1)/max_acc_matrix[i];
    }

    float total_time = t + t_b1 + s_t[temp_j] + t_b2[temp_j];

    std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points2;

    int total_no_of_points = 45;

    vector_points2.resize(total_no_of_points+1);

    for(int i = 0; i<total_no_of_points+1; i++)
        vector_points2.at(i).time_from_start = ros::Duration(i*total_time/total_no_of_points);

    for(int i = 0; i<total_no_of_points+1; i++)
    {
        float tt = i*total_time/total_no_of_points;

        vector_points2.at(i).positions.resize(6);

        for(int j=0; j<6; j++)
        {
            if(tt <= t)
                vector_points2.at(i).positions.at(j) = a[j][0]+ a[j][1]*tt + a[j][2]*pow(tt,2) +
                        a[j][3]*pow(tt,3) + a[j][4]*pow(tt,4) + a[j][5]*pow(tt,5);

            else
            {
                if((tt > t) && (tt <= t + t_b1))
                    vector_points2.at(i).positions.at(j) = initial_thetha[j] + initial_velocity[j]*(tt - t) + 0.5*max_acc_matrix[j]*pow((tt - t),2);

                else
                {
                    if((tt > t + t_b1) && (tt <= t + t_b1 + s_t[temp_j]))
                    {
                        vector_points2.at(i).positions.at(j) = initial_thetha[j] + initial_velocity[j]*t_b1 + 0.5*max_acc_matrix[j]*pow(t_b1,2) +
                                (initial_velocity[j] + max_acc_matrix[j]*t_b1)*(tt - t - t_b1);
                    }
                    else
                    {
                       vector_points2.at(i).positions.at(j) = sudo_thetha[j] + (initial_velocity[j] + max_acc_matrix[j]*t_b1)*(tt - t - t_b1 - s_t[temp_j]) -
                                0.5*max_acc_matrix[j]*pow((tt - t - t_b1 - s_t[temp_j]),2);

                    }

                }
            }

        }
    }

    for(int i = 0; i<total_no_of_points+1; i++)
    {
        float tt = i*total_time/total_no_of_points;

        vector_points2.at(i).velocities.resize(6);

        for(int j=0; j<6; j++)
        {
            if(tt <= t)
                vector_points2.at(i).velocities.at(j) =  a[j][1] + 2*a[j][2]*tt + 3*a[j][3]*pow(tt,2) + 4*a[j][4]*pow(tt,3) + 5*a[j][5]*pow(tt,4);

            else
            {
                if((tt > t) && (tt <= t + t_b1))
                    vector_points2.at(i).velocities.at(j) = initial_velocity[j] + max_acc_matrix[j]*(tt - t);

                else
                {
                    if((tt > t + t_b1) && (tt <= t + t_b1 + s_t[temp_j]))
                    {
                        vector_points2.at(i).velocities.at(j) = initial_velocity[j] + max_acc_matrix[j]*t_b1;
                    }
                    else
                    {
                        vector_points2.at(i).velocities.at(j) = (initial_velocity[j] + max_acc_matrix[j]*t_b1) - max_acc_matrix[j]*(tt - t - t_b1 - s_t[temp_j]);

                    }

                }
            }

        }
    }

    for(int i = 0; i<total_no_of_points+1; i++)
    {
        float tt = i*total_time/total_no_of_points;

        vector_points2.at(i).accelerations.resize(6);

        for(int j=0; j<6; j++)
        {
            if(tt <= t)
                vector_points2.at(i).accelerations.at(j) = 2*a[j][2] + 6*a[j][3]*tt + 12*a[j][4]*pow(tt,2) + 20*a[j][5]*pow(tt,3);

            else
            {
                if((tt > t) && (tt <= t + t_b1))
                    vector_points2.at(i).accelerations.at(j) = max_acc_matrix[j];

                else
                {
                    if((tt > t + t_b1) && (tt <= t + t_b1 + s_t[temp_j]))
                    {
                        vector_points2.at(i).accelerations.at(j) = 0;
                    }
                    else
                    {
                        vector_points2.at(i).accelerations.at(j) = -max_acc_matrix[j];

                    }

                }
            }

        }
    }

    return vector_points2;

}

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}

void chatterCallback(const geometry_msgs::WrenchStamped &force)
{

    force_x =  force.wrench.force.x;
    force_y =  force.wrench.force.y;
    force_z =  force.wrench.force.z;

//    std::cout << "force(" << count_<< ", :" <<") = ["<< force_x << " " << force_y << " " << force_z <<"]\n";
    write_data_in_file(count_, force_x, force_y, force_z);
    count_ = count_ + 1;

    return;
}

void *next_move(void *threadid)
{


    geometry_msgs::PoseStamped current_state;

    moveit::planning_interface::MoveGroup group("robo_arm");

    std::vector<double>* tid;
    tid = (std::vector<double>*)threadid;

    float dis_threshold = 0.001;
    float force_threshold = 50;


    while(next_step == false)
    {

        current_state = group.getCurrentPose();
        if( pow(current_state.pose.position.x - tid->at(0),2) + pow(current_state.pose.position.y - tid->at(1),2) + pow(current_state.pose.position.z - tid->at(2),2) < pow(dis_threshold,2) )
        {
            std::cout <<"\nrobot has reached the desired state\n";
            next_step = true;
            break;
        }

        if ( ((force_x) > force_threshold)|| ((force_y) > force_threshold) || ((force_z) > force_threshold))
        {
            std::cout <<"\nexperiencing more force\n";
            group.stop();
            next_step = true;
            break;
        }

    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_actuator45");
    ros::AsyncSpinner spinner(1);
    spinner.start();


    ros::NodeHandle nh;

    ros::Subscriber force_sub = nh.subscribe("/wrench",  2, chatterCallback);

    moveit::planning_interface::MoveGroup group("robo_arm");

    group.setPlannerId("RRTConnectkConfigDefault");

    moveit::planning_interface::MoveGroup::Plan plan;

    geometry_msgs::Pose final_pose;

    geometry_msgs::PoseStamped current_state;

    std::vector <double> joint_angles;

    joint_angles.resize(6);

//    double tot_view_angles[6] = {-0.10676700273622686, -1.7864249388324183, 1.8607382774353027, -1.4938681761371058, -1.551873509083883, -0.04021817842592412};

    double tot_view_angles[6] = {-0.21302682558168584, -1.67084247270693, 1.1082968711853027, -0.829651180897848, -1.6112468878375452, -0.23895198503603154};

    std::vector<geometry_msgs::Pose> waypoints;

    tf::Quaternion q;

    char aa;

    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_iptp_5_poly;

    int step = 1;

    float acc = 3.0 ;
    float T = 2.0;
    float t_b1 = 0.2;

    pthread_t threads;

    std::vector<double> final_xyz;
    final_xyz.resize(3);


    while (ros::ok())
    {

        signal(SIGINT, signal_callback_handler);

        if(step == 1)
        {

            for(int i = 0; i<6; i++)
                joint_angles.at(i) = tot_view_angles[i];

            group.setJointValueTarget(joint_angles);

            group.plan(plan);

            trajectory = plan.trajectory_;

            std::cout <<"\n\n\n-----trajectory iptp 5th poly data------\n\n";
            trajectory_iptp_5_poly = trajectory;

            trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T, t_b1);

            plan.trajectory_ = trajectory_iptp_5_poly;

            std::cout << "\n\n\n\n\n-----Run-----\n";
            std::cin >> aa;

            group.execute(plan);
        }

        if(step == 2)
        {

            current_state = group.getCurrentPose();

            final_pose.position.x = current_state.pose.position.x;
            final_pose.position.y = current_state.pose.position.y;
            final_pose.position.z = current_state.pose.position.z - 0.5;

            final_pose.orientation = current_state.pose.orientation;

            waypoints.push_back(final_pose);

            final_xyz.at(0) = final_pose.position.x;
            final_xyz.at(1) = final_pose.position.y;
            final_xyz.at(2) = final_pose.position.z;


        }

        if(step == 2)
        {


            usleep(100000);

            double fraction = group.computeCartesianPath(waypoints,
                                                         0.01,  // eef_step
                                                         0.0,   // jump_threshold
                                                         trajectory);

            std::cout<<"\n---> "<< fraction * 100.0 <<" Path computed \n";

            robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "robo_arm");

            rt.setRobotTrajectoryMsg(group.getCurrentState()->getRobotModel(), trajectory);

            std::cout <<"\n\n\n-----trajectory iptp 5th poly data------\n\n";

            trajectory_iptp_5_poly = trajectory;
            plan.trajectory_ = trajectory;

            trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T, t_b1);

            plan.trajectory_ = trajectory_iptp_5_poly;

            std::cout << "\n\n\n\n\n-----Run-----\n";
            std::cin >> aa;

            group.asyncExecute(plan);

            pthread_create(&threads, NULL, next_move, &final_xyz);

            while(next_step == false)
            {
            };

            next_step = false;


            usleep(100000);

            rt.clear();
            waypoints.clear();
            plan.trajectory_.joint_trajectory.points.clear();
            trajectory.joint_trajectory.points.clear();
            trajectory_iptp_5_poly.joint_trajectory.points.clear();


        }



        step = step + 1;

        if(step == 3)
            step = 1;
    }




    return 0;
}
