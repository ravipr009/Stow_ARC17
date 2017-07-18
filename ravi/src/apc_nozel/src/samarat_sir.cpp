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
//#include <ManipulatorHandler.h>


#include <iostream>


using namespace Eigen;

void write_data_in_file(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points, int k)
{

    std::fstream myfile;
    if(k == 2)
        myfile.open ("/home/mohit/Desktop/ros_ws/src/apc_nozel/way_point_data.txt", std::ios::out | std::ios::app);
    else
        myfile.open ("/home/mohit/Desktop/ros_ws/src/apc_nozel/way_point_data.txt", std::ios::out);

    int len = vector_points.size();

    myfile << "\n\ntime_"<< k << " = [" ;
    for(int i = 0; i<len; i++)
    {

        trajectory_msgs::JointTrajectoryPoint point_ = vector_points.at(i);
        if(i == len-1)
            myfile << point_.time_from_start.toSec() << "]; ";
        else
            myfile << point_.time_from_start.toSec() << ", ";
    }


    for(int j = 0; j<6; j++)
    {
        myfile << "\n\nacc_" << j <<"_"<< k << " = [" ;
        for(int i = 0; i<len; i++)
        {

            trajectory_msgs::JointTrajectoryPoint point_ = vector_points.at(i);
            if(i == len-1)
                myfile << point_.accelerations.at(j) << "]; ";
            else
                myfile << point_.accelerations.at(j) << ", ";
        }
    }


    for(int j = 0; j<6; j++)
    {
        myfile << "\n\nvel_" << j <<"_"<< k << " = [" ;
        for(int i = 0; i<len; i++)
        {

            trajectory_msgs::JointTrajectoryPoint point_ = vector_points.at(i);
            if(i == len-1)
                myfile << point_.velocities.at(j) << "]; ";
            else
                myfile << point_.velocities.at(j) << ", ";
        }
    }


    for(int j = 0; j<6; j++)
    {
        myfile << "\n\npos_" << j <<"_"<< k << " = [" ;
        for(int i = 0; i<len; i++)
        {

            trajectory_msgs::JointTrajectoryPoint point_ = vector_points.at(i);
            if(i == len-1)
                myfile << point_.positions.at(j) << "]; ";
            else
                myfile << point_.positions.at(j) << ", ";
        }
    }


    myfile.close();

    return;

}


template <class myType>
myType my_sign (myType a)
{
    if(a >= 0)
        return 1;
    else
        return -1;
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

void *broadcast_kf(void *threadid)
{


    tf::TransformBroadcaster br;

    tf::StampedTransform transform_kf;

    ros::Rate rate(10.0);

    while (1)
    {

        float tx_data[] = {  0.0307645,   0.0275343,    0.999147,   0.0380546,
                             -0.999527, 0.000730582,    0.030756,  -0.0649059,
                             0.000116929,    -0.99962,   0.0275439,    0.206411,
                             0,           0,           0,           1};





        Eigen::Matrix4f K_T_wr2;
        Eigen::Affine3d affine_K_T_wr2;



        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
            {

                K_T_wr2(i,j) = tx_data[i*4+j];
                affine_K_T_wr2.matrix()(i,j) = tx_data[i*4+j];
            }


        geometry_msgs::Pose kinect_frame;
        tf::poseEigenToMsg(affine_K_T_wr2, kinect_frame);


        transform_kf.setOrigin( tf::Vector3(kinect_frame.position.x, kinect_frame.position.y, kinect_frame.position.z) );
        transform_kf.setRotation( tf::Quaternion(kinect_frame.orientation.x, kinect_frame.orientation.y
                                                 , kinect_frame.orientation.z, kinect_frame.orientation.w) );


        br.sendTransform(tf::StampedTransform(transform_kf, ros::Time::now(), "wrist_2_link", "kf"));

        rate.sleep();
    }

    long tid;
    tid = (long)threadid;
    std::cout << "Hello World! Thread ID, " << tid;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_actuator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroup group("robo_arm");

    group.setPlannerId("RRTConnectkConfigDefault");

    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    moveit::planning_interface::MoveGroup::Plan plan;

    visualization_msgs::MarkerArray markers;

    geometry_msgs::Pose final_pose;


    std::vector <double> joint_angles;

    joint_angles.resize(6);

    double tot_view_angles[6] = {-0.21402150789369756, -1.670279328023092, 1.1081533432006836, -0.8296750227557581, -1.6121814886676233, -0.23902398744692022};

    std::vector<geometry_msgs::Pose> waypoints;

    pthread_t threads;

    pthread_create(&threads, NULL, broadcast_kf, NULL);

    char aa;

    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_iptp_5_poly;

    int step = 1;

    float acc = 3.0 ;
    float T = 5.0;
    float t_b1 = 0.1;

    fstream myfile;

    char c;
    float a;

    int feedback = 0;






    myfile.open ("/home/mohit/mohit.txt", std::ios::in);
    int num_points = 0;

    while (myfile.get(c))
    {
        if(c == '\n')
            num_points = num_points + 1;
    }

    myfile.close();


    MatrixXf trajectory_points(num_points, 7);

    myfile.open ("/home/mohit/mohit.txt", std::ios::in);
    int row = 0;
    int column = 0;

    do
    {

        myfile >> a;
        trajectory_points(row, column) = a;

        column = column + 1;
        column = column % 7;
        if(column == 0)
        {
            row = row + 1;
        }

        if(row == num_points)
            break;

    }while (myfile.get(c));

    myfile.close();





    while (ros::ok())
    {

        signal(SIGINT, signal_callback_handler);

        if(step == 1)
        {
            for (int i = 0; i<2; i++)
            {
                visualization_msgs::Marker marker;

                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time::now();
                marker.ns = "my_namespace";
                marker.id = i;

                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                marker.color.a = 1.0; // Don't forget to set the alpha!

                if(i == 0)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;

                    marker.pose.position.x = trajectory_points(0, 0);
                    marker.pose.position.y = trajectory_points(0, 1);
                    marker.pose.position.z = trajectory_points(0, 2);

                }
                if(i == 1)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;

                    marker.pose.position.x = trajectory_points(num_points - 1, 0);
                    marker.pose.position.y = trajectory_points(num_points - 1, 1);
                    marker.pose.position.z = trajectory_points(num_points - 1, 2);

                }

                marker.lifetime = ros::Duration();

                markers.markers.push_back(marker);

            }

            pub.publish(markers);

            markers.markers.clear();




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

            int row = 0;
            int column = 0;

            while(1)
            {

                column = column + 1;
                column = column % 7;
                if(column == 0)
                {
                    final_pose.position.x = trajectory_points(row, 0);
                    final_pose.position.y = trajectory_points(row, 1);
                    final_pose.position.z = trajectory_points(row, 2);

                    final_pose.orientation.w = trajectory_points(row, 3);
                    final_pose.orientation.x = trajectory_points(row, 3);
                    final_pose.orientation.y = trajectory_points(row, 4);
                    final_pose.orientation.z = trajectory_points(row, 6);

                    waypoints.push_back(final_pose);

                    row = row + 1;
                }

                if(row == num_points)
                    break;
            }

        }

        if(step == 2)
        {


            usleep(100000);

            double fraction = group.computeCartesianPath(waypoints,
                                                         0.01,  // eef_step
                                                         0.0,   // jump_threshold
                                                         trajectory);

            std::cout<<"\n---> "<< fraction * 100.0 <<" Path computed \n";

            std::cout << "\nstep = " << step <<"\n\nfeedback = " << feedback << "\n\n";


            if(fraction > 0.9)
            {


                robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "robo_arm");

                rt.setRobotTrajectoryMsg(group.getCurrentState()->getRobotModel(), trajectory);

                if(step == 3)
                {

                    trajectory_iptp_5_poly = trajectory;
                    plan.trajectory_ = trajectory;


                    std::cout << "Enter T\n";
                    std::cin >> T;

                    std::cout << "Enter t_b1\n";
                    std::cin >> t_b1;


                    trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T, t_b1);
                    write_data_in_file(trajectory_iptp_5_poly.joint_trajectory.points, 1);


                    plan.trajectory_ = trajectory_iptp_5_poly;


                }
                else
                {
                    // Thrid create a IterativeParabolicTimeParameterization object
                    trajectory_processing::IterativeParabolicTimeParameterization iptp;

                    // Fourth compute computeTimeStamps
                    bool success = iptp.computeTimeStamps(rt);
                    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

                    // Get RobotTrajectory_msg from RobotTrajectory
                    rt.getRobotTrajectoryMsg(trajectory);

                    // Finally plan and execute the trajectory
                    plan.trajectory_ = trajectory;

                }


                std::cout << "\n\n\n\n\n-----Run-----\n";
                std::cin >> a;

                group.execute(plan);

                rt.clear();

                waypoints.clear();

                plan.trajectory_.joint_trajectory.points.clear();
                trajectory.joint_trajectory.points.clear();
                trajectory_iptp_5_poly.joint_trajectory.points.clear();

            }

        }

        step = step + 1;

        if(step == 3)
            step = 1;

    }


    return 0;
}
