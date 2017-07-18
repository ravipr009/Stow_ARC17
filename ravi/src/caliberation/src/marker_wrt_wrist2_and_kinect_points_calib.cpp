#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <arc_controller/points.h>

#include <caliberation/apc_simulation.h>


#include <math.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigenvalues>
#include <complex>




#define KEY "KBB1V–A386–EA49–2294–A708"


using namespace Eigen;

template <class myType>
myType my_sign (myType a)
{
    if(a >= 0)
        return 1;
    else
        return -1;
}

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
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
    ros::init(argc, argv, "markers_wrt_ee_link");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("robo_arm");

    moveit::planning_interface::MoveGroup::Plan plan;

    ros::NodeHandle n;

    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_iptp_5_poly;

    float acc = 3.0 ;
    float T = 4.0;
    float t_b1 = 0.1;


    ros::ServiceClient kinect_points_calib = n.serviceClient<arc_controller::points>("/calib_aruco/ur10");

    geometry_msgs::PointStamped tip_point_wrt_world;
    geometry_msgs::PointStamped tip_point_wrt_ee_link;

    std::fstream myfile;

    tf::TransformListener listener;

    char c;

    char aa;


    // ******* To count total number of joint angles *******

    int num_of_joint_angles = 0;


    std::cout<<"\n before jt files \n";


    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/joint_angles.txt", std::ios::in);
//                  /home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files
    std::cout<<"\n after jt files \n";

    while (myfile.get(c))
    {
        if(c == '\n')
            num_of_joint_angles = num_of_joint_angles + 1;
    }

    myfile.close();



    // ******* To store all joint angles in a matrix *******


    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/joint_angles.txt", std::ios::in);

    MatrixXf joint_angles(num_of_joint_angles, 6);

    float a;

    int row = 0;
    int column = 0;

    do
    {
        myfile >> a;
        joint_angles(row, column) = a;

        column = column + 1;
        column = column % 6;
        if(column == 0)
            row = row +1;

        if(row == num_of_joint_angles)
            break;
    }while(myfile.get(c));

    myfile.close();






    // ******* To count total number of markers *******


    myfile.open("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates.txt", std::ios::in);

    int total_number_of_markers = 0;

    while(myfile.get(c))
    {
        if(c == '\n')
            total_number_of_markers = total_number_of_markers +1;
    }

    myfile.close();




    // ******* To store all markers in a matrix *******


    myfile.open("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates.txt", std::ios::in);

    MatrixXf markers_wrt_world(total_number_of_markers, 3);

    row = 0;
    column = 0;

    do
    {
        myfile >> a;
        markers_wrt_world(row, column) = a;

        column = column + 1;
        column = column % 3;
        if(column == 0)
            row = row +1;

        if(row == total_number_of_markers)
            break;
    }while(myfile.get(c));

    myfile.close();



    std::cout <<"total_markers = " << total_number_of_markers;
    std::cout <<"\n\nmarkers " << markers_wrt_world;
    std::cout <<"\n\ntotal_number_of_joint_angles = " << num_of_joint_angles;
    std::cout <<"\n\njoint_angles " << joint_angles << "\n";




    std::fstream myfile3;

    myfile3.open("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates_wrt_kinect.txt", std::ios::out);

    std::fstream myfile4;

    myfile4.open("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates_wrt_ee_link.txt", std::ios::out);






    std::vector<double> current_joint_angles;     // To store current joint angles in a while loop
    current_joint_angles.resize(6);

    int temp_num_of_joint_angles = 0;      // variable to represent the current joint angles in a while loop

    while (temp_num_of_joint_angles < num_of_joint_angles)
    {

        signal(SIGINT, signal_callback_handler);

        for(int i = 0; i<6; i++)
            current_joint_angles.at(i) = joint_angles(temp_num_of_joint_angles, i);

        temp_num_of_joint_angles = temp_num_of_joint_angles + 1;

        group.setJointValueTarget(current_joint_angles);

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




        std::cout <<"\n\nkinect service\n\n";

        // ******* calling kinect service to get marker points wrt kinect frame *******


        int total_no_of_valid_markers = 0;       // To store total number of valid marker points from kinect

        std::vector<int> marker_ID;        // To store ID of valid marker points from kinect


        do
        {

            usleep(500000);

            arc_controller::points srv;

            ros::service::waitForService("/calib_aruco/ur10");

            srv.request.key.data = KEY;

            if(kinect_points_calib.call(srv))
            {

                total_no_of_valid_markers = srv.response.points_3d.data.size()/4;

                marker_ID.resize(total_no_of_valid_markers);

                std::cout << "\nMarker ID \n";

                for (int i = 0; i< total_no_of_valid_markers; i++)
                {
                    marker_ID.at(i) = srv.response.points_3d.data.at(4*i);
                    std::cout << srv.response.points_3d.data.at(4*i) <<"\n";

                }




                MatrixXf points_wrt_kinect(total_no_of_valid_markers, 3);        // To store valid marker points from kinect in a matrix

                for (int i = 0; i< total_no_of_valid_markers; i++)
                {
                    points_wrt_kinect.block(i, 0, 1, 3) << srv.response.points_3d.data.at( 4*i + 1 ),
                            srv.response.points_3d.data.at( 4*i + 2 ),
                            srv.response.points_3d.data.at( 4*i + 3 );

                }


                myfile3 << std::setprecision(15) << points_wrt_kinect << "\n";

            }
            else
                total_no_of_valid_markers = 0;


        }while(total_no_of_valid_markers == 0);





        // ******* To store valid marker points in a matrix *******


        MatrixXf valid_markers_wrt_world(total_no_of_valid_markers, 3);

        for(int i = 0; i< total_no_of_valid_markers; i++)
            valid_markers_wrt_world.block(i, 0, 1, 3) = markers_wrt_world.block(marker_ID.at(i), 0, 1, 3);







        // ******* store valid markers wrt ee_link *******


        std::vector<double> marker_coordinates;     // To store current valid markers in a while loop
        marker_coordinates.resize(3);

        int temp_total_no_of_valid_markers = 0;      // variable to represent the current valid_marker in a while loop

        while(temp_total_no_of_valid_markers < total_no_of_valid_markers)
        {

            for(int i = 0; i<3 ;i++)
                marker_coordinates.at(i) = valid_markers_wrt_world(temp_total_no_of_valid_markers, i);

            temp_total_no_of_valid_markers = temp_total_no_of_valid_markers + 1;

            tip_point_wrt_world.header.frame_id = "world";

            tip_point_wrt_world.point.x = marker_coordinates.at(0);
            tip_point_wrt_world.point.y = marker_coordinates.at(1);
            tip_point_wrt_world.point.z = marker_coordinates.at(2);

            listener.waitForTransform( "ee_link", "world", ros::Time(0), ros::Duration(3));

            listener.transformPoint("ee_link", tip_point_wrt_world, tip_point_wrt_ee_link);

            myfile4 << std::setprecision(15) << tip_point_wrt_ee_link.point.x <<", "
                    << tip_point_wrt_ee_link.point.y <<", "
                    << tip_point_wrt_ee_link.point.z <<"\n";

        }

    }


    myfile3.close();

    myfile4.close();



    return 0;
}
