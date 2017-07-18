#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <pthread.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <arc_controller/points.h>

#include <caliberation/apc_simulation.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Eigen;



#define KEY "KBB1V–A386–EA49–2294–A708"


void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}

std::vector<trajectory_msgs::JointTrajectoryPoint> slow_compute_cartesian_path(std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points, float slow_factor)
{
    int len = vector_points.size();

    std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points2;

    vector_points2.resize(len);
    for (int i = 0; i<len; i++)
    {
        vector_points2.at(i).time_from_start = ros::Duration(vector_points.at(i).time_from_start.toSec() * slow_factor);

        vector_points2.at(i).positions.resize(6);
        vector_points2.at(i).velocities.resize(6);
        vector_points2.at(i).accelerations.resize(6);

        for(int j=0; j<6; j++)
        {
            vector_points2.at(i).positions.at(j) = vector_points.at(i).positions.at(j);
            vector_points2.at(i).velocities.at(j) = vector_points.at(i).velocities.at(j)/slow_factor;
            vector_points2.at(i).accelerations.at(j) = vector_points.at(i).accelerations.at(j)/(slow_factor*slow_factor);
        }

    }

    return vector_points2;

}

void *broadcast_kf(void *threadid)
{


    fstream myfile;

    char c;
    float a;

    tf::TransformBroadcaster br;

    tf::StampedTransform transform_kf;

    ros::Rate rate(10.0);

    MatrixXf transformation_matrix(4, 4);


    myfile.open ("/home/mohit/Desktop/ros_ws/src/caliberation/data_files/final_R_and_T.txt", std::ios::in);

    int row = 0;
    int column = 0;

    do
    {

        myfile >> a;
        transformation_matrix(row, column) = a;

        column = column + 1;
        column = column % 4;
        if(column == 0)
            row = row + 1;

        if(row == 4)
            break;

    }while (myfile.get(c));

    myfile.close();


    while (1)
    {



        Eigen::Matrix4f K_T_wr2;
        Eigen::Affine3d affine_K_T_wr2;



        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
            {

                K_T_wr2(i,j) = transformation_matrix(i,j);
                affine_K_T_wr2.matrix()(i,j) = transformation_matrix(i,j);
            }




        geometry_msgs::Pose kinect_frame;
        tf::poseEigenToMsg(affine_K_T_wr2, kinect_frame);


        transform_kf.setOrigin( tf::Vector3(kinect_frame.position.x, kinect_frame.position.y, kinect_frame.position.z) );
        transform_kf.setRotation( tf::Quaternion(kinect_frame.orientation.x, kinect_frame.orientation.y
                                                 , kinect_frame.orientation.z, kinect_frame.orientation.w) );


        br.sendTransform(tf::StampedTransform(transform_kf, ros::Time::now(), "ee_link", "kf"));

        rate.sleep();
    }

    long tid;
    tid = (long)threadid;
    std::cout << "Hello World! Thread ID, " << tid;
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_caliberation");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("robo_arm");

    ros::NodeHandle n;

    ros::ServiceClient kinect_points_calib = n.serviceClient<arc_controller::points>("/calib/test_point");

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    visualization_msgs::MarkerArray markers;

    pthread_t threads;

    pthread_create(&threads, NULL, broadcast_kf, NULL);

    geometry_msgs::PointStamped point_wrt_kinect;
    geometry_msgs::PointStamped point_wrt_world;
    geometry_msgs::PointStamped normal_wrt_kinect;
    geometry_msgs::PointStamped normal_wrt_world;

    geometry_msgs::Pose final_pose;

    geometry_msgs::PoseStamped current_state;

    std::vector<geometry_msgs::Pose> waypoints;

    tf::TransformListener listener;

    moveit::planning_interface::MoveGroup::Plan plan;


    std::vector <double> joint_angles;

    joint_angles.resize(6);

    char aa;

    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_iptp_5_poly;

    float acc = 3.0 ;
    float T = 4.0;
    float t_b1 = 0.1;


    tf::Quaternion q, q1, q2, q3, q4;

    int step = 1;


    while (ros::ok())
    {

        signal(SIGINT, signal_callback_handler);


        if(step == 1)
        {
            joint_angles = group.getCurrentJointValues();

            double tot_view_angles[6] = {1.826417326927185, -1.7489278952227991, 1.9267487525939941, -1.7509844938861292, -1.6167572180377405, 1.2903261184692383};
            for(int i =0; i<6; i++)
                joint_angles.at(i) = tot_view_angles[i];


            group.setJointValueTarget(joint_angles);

            group.plan(plan);

            trajectory = plan.trajectory_;

            trajectory_iptp_5_poly = trajectory;

            trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T, t_b1);

            plan.trajectory_ = trajectory_iptp_5_poly;

            std::cout << "\n\n\n\n\n-----Run-----\n";
            std::cin >> aa;

            group.execute(plan);

            trajectory.joint_trajectory.points.clear();
            plan.trajectory_.joint_trajectory.points.clear();
            trajectory_iptp_5_poly.joint_trajectory.points.clear();





            //         ******* calling kinect service to get marker points wrt kinect frame *******

//            std::cout <<"press any key to call kinect service\n\n";
//            std::cin >> aa;

//            arc_controller::points srv;

//            ros::service::waitForService("/calib/test_point");

//            srv.request.key.data = KEY;

//            if(kinect_points_calib.call(srv))
//            {
//                point_wrt_kinect.point.x = srv.response.points_3d.data.at(0);
//                point_wrt_kinect.point.y = srv.response.points_3d.data.at(1);
//                point_wrt_kinect.point.z = srv.response.points_3d.data.at(2);

//                normal_wrt_kinect.point.x = srv.response.points_3d.data.at(3);
//                normal_wrt_kinect.point.y = srv.response.points_3d.data.at(4);
//                normal_wrt_kinect.point.z = srv.response.points_3d.data.at(5);
//            }

//            std::cout << point_wrt_kinect.point.x <<", "
//                      << point_wrt_kinect.point.y <<", "
//                      << point_wrt_kinect.point.z <<"\n\n";



//            std::cout << normal_wrt_kinect.point.x <<", "
//                      << normal_wrt_kinect.point.y <<", "
//                      << normal_wrt_kinect.point.z <<"\n\n\n";

                        point_wrt_kinect.point.x = 0.153573;
                        point_wrt_kinect.point.y = -0.181846;
                        point_wrt_kinect.point.z = 0.74;

                        normal_wrt_kinect.point.x = -0.291463;
                        normal_wrt_kinect.point.y = 0.515083;
                        normal_wrt_kinect.point.z = -0.806064;

            //            float m = sqrt(pow(normal_wrt_kinect.point.x, 2) + pow(normal_wrt_kinect.point.y, 2) + pow(normal_wrt_kinect.point.z, 2));




            point_wrt_kinect.header.frame_id = "kf";

            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

            listener.transformPoint("world", point_wrt_kinect, point_wrt_world);

            normal_wrt_kinect.header.frame_id = "kf";

            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

            listener.transformPoint("world", normal_wrt_kinect, normal_wrt_world);


            std::cout << point_wrt_world.point.x <<", "
                      << point_wrt_world.point.y <<", "
                      << point_wrt_world.point.z <<"\n\n";
            std::cout << normal_wrt_world.point.x <<", "
                      << normal_wrt_world.point.y <<", "
                      << normal_wrt_world.point.z <<"\n\n\n";

            float n_x = (normal_wrt_world.point.x - point_wrt_world.point.x);
            float n_y = (normal_wrt_world.point.y - point_wrt_world.point.y);
            float n_z = (normal_wrt_world.point.z - point_wrt_world.point.z);

            float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

            n_x = n_x/m;
            n_y = n_y/m;
            n_z = n_z/m;

            float phi = atan2(n_y, n_x);
            float thetha = acos(n_z);

            q.setRPY( 0, 0, phi - M_PI);
            q1.setRPY( 0, M_PI/2 - thetha, 0);


            q2.setRotation(tf::Vector3(cos(phi + M_PI/2), sin(M_PI/2 + phi), 0), thetha - M_PI/2 );

            std::cout <<"\nthetha = " << thetha*180/M_PI;
            std::cout <<"\nphi = " << phi*180/M_PI;


            q4 = q2*q;



            visualization_msgs::Marker line_list;

            line_list.header.frame_id = "/world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = "my_namespace";

            line_list.id = 3;

            line_list.type = visualization_msgs::Marker::LINE_LIST;

            line_list.scale.x = 0.01;

            line_list.color.b = 1.0;
            line_list.color.g = 1.0;
            line_list.color.a = 1.0;

            line_list.points.push_back(point_wrt_world.point);
            line_list.points.push_back(normal_wrt_world.point);


            marker_pub.publish(line_list);


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
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;

                    marker.pose.position.x = point_wrt_world.point.x;
                    marker.pose.position.y = point_wrt_world.point.y;
                    marker.pose.position.z = point_wrt_world.point.z;

                }

                if(i == 1)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;

                    marker.pose.position.x = normal_wrt_world.point.x;
                    marker.pose.position.y = normal_wrt_world.point.y;
                    marker.pose.position.z = normal_wrt_world.point.z;
                }

                if(i == 2)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;

                    marker.pose.position.x = normal_wrt_world.point.x;
                    marker.pose.position.y = normal_wrt_world.point.y;
                    marker.pose.position.z = point_wrt_world.point.z;
                }

                if(i == 3)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;

                    marker.pose.position.x = point_wrt_world.point.x + cos(phi + M_PI/2);
                    marker.pose.position.y = point_wrt_world.point.y + sin(phi + M_PI/2);
                    marker.pose.position.z = point_wrt_world.point.z;
                }

                if(i == 4)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;

                    marker.pose.position.x = cos(phi + M_PI/2);
                    marker.pose.position.y = sin(phi + M_PI/2);
                    marker.pose.position.z = 0;
                }

                marker.lifetime = ros::Duration();

                markers.markers.push_back(marker);

            }

            pub.publish(markers);

            markers.markers.clear();

        }





        final_pose.position.x = point_wrt_world.point.x;
        final_pose.position.y = point_wrt_world.point.y;
        final_pose.position.z = point_wrt_world.point.z;

        final_pose.orientation.x = q4.getX();
        final_pose.orientation.y = q4.getY();
        final_pose.orientation.z = q4.getZ();
        final_pose.orientation.w = q4.getW();

        waypoints.push_back(final_pose);





        usleep(100000);

        double fraction = group.computeCartesianPath(waypoints,
                                                     0.01,  // eef_step
                                                     0.0,   // jump_threshold
                                                     trajectory);

        std::cout<<"\n---> "<< fraction * 100.0 <<" Path computed \n";

        robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "robo_arm");

        rt.setRobotTrajectoryMsg(group.getCurrentState()->getRobotModel(), trajectory);

        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        // Fourth compute computeTimeStamps
        bool success = iptp.computeTimeStamps(rt);
        ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);

        // Finally plan and execute the trajectory
        trajectory_iptp_5_poly = trajectory;
        plan.trajectory_ = trajectory;

        trajectory_iptp_5_poly.joint_trajectory.points = slow_compute_cartesian_path(trajectory.joint_trajectory.points, 2);

        plan.trajectory_ = trajectory_iptp_5_poly;


        std::cout << "\n\n\n\n\n-----Run-----\n";
        std::cin >> aa;

        group.execute(plan);


        rt.clear();
        waypoints.clear();
        plan.trajectory_.joint_trajectory.points.clear();
        trajectory.joint_trajectory.points.clear();
        trajectory_iptp_5_poly.joint_trajectory.points.clear();


    }

    return 0;
}
