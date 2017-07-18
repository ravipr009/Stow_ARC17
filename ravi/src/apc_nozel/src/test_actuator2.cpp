#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/JointLimits.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <math.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>


#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigenvalues>
#include <complex>

#include <apc_nozel/apc_simulation.h>
#include <pthread.h>


#include <iostream>



using namespace Eigen;








template <class myType>
myType my_sign (myType a)
{
    if(a >= 0)
        return 1;
    else
        return -1;
}




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

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}


std::vector <trajectory_msgs::JointTrajectoryPoint> generate_data(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points)
{
    //    return vector_points;

    int len = vector_points.size();


    std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points2;

    for(int i = 0; i<len; i++)
    {
        trajectory_msgs::JointTrajectoryPoint point_ = vector_points.at(i);


        float v0, v1, v2, v3, v4, v5;

        if(i==4)
        {
            v0 = point_.velocities.at(0);
            v1 = point_.velocities.at(1);
            v2 = point_.velocities.at(2);
            v3 = point_.velocities.at(3);
            v4 = point_.velocities.at(4);
            v5 = point_.velocities.at(5);
        }

        if((i>4) && (i<len - 4))
        {
            point_.velocities.at(0) = v0;
            point_.velocities.at(1) = v1;
            point_.velocities.at(2) = v2;
            point_.velocities.at(3) = v3;
            point_.velocities.at(4) = v4;
            point_.velocities.at(5) = v5;
        }




        vector_points2.push_back(point_);


    }



    return vector_points2;

}

std::vector <trajectory_msgs::JointTrajectoryPoint> generate_waypoints(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points)
{
    //    return vector_points;
    int len = vector_points.size();


    std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points2;

    for(int i = 0; i<len; i++)
    {
        trajectory_msgs::JointTrajectoryPoint point_ = vector_points.at(i);

        //        if (((i>0) && (i<4)) || ((i>5) && (i<len)))
        if(i < 0)
        {
            for(int j = 0; j<5; j++)
                point_.positions.at(j) = (vector_points.at(i-1).positions.at(j) + vector_points.at(i).positions.at(j))/2;



            vector_points2.push_back(point_);

        }

        point_ = vector_points.at(i);


        vector_points2.push_back(point_);

    }

    return vector_points2;

}

std::vector <trajectory_msgs::JointTrajectoryPoint> generate_waypoints_cubic(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points, float slow_factor)
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points2;

    int len = vector_points.size();

    float time[len];
    double joint_angles[len][6];
    double velocity[len][6];
    double acceleration[len][6];


    for (int i = 0; i<len; i++)
    {
        time[i] = slow_factor*vector_points.at(i).time_from_start.toSec() ;
        vector_points2.push_back(vector_points.at(i));
    }


    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            joint_angles[i][j] = vector_points.at(i).positions.at(j);
        }

    }


    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            velocity[i][j] = vector_points.at(i).velocities.at(j);
        }

    }


    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            acceleration[i][j] = vector_points.at(i).accelerations.at(j);
        }

    }




    double joint_angles2[len][6];
    double velocity2[len][6];
    double acceleration2[len][6];



    for (int i = 0; i<len; i++)
    {
        vector_points2.at(i).time_from_start = ros::Duration(time[i]);
    }




    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            joint_angles2[i][j] = joint_angles[0][j]
                    + (3/(time[len-1]*time[len-1]))*(joint_angles[len - 1][j] - joint_angles[0][j])*(time[i]*time[i])
                    - (2/(time[len-1]*time[len-1]*time[len-1]))*(joint_angles[len - 1][j] - joint_angles[0][j])*(time[i]*time[i]*time[i]);

            while(joint_angles2[i][j] < -2*M_PI || joint_angles2[i][j] > 2*M_PI)
            {
                if(joint_angles2[i][j] > 2*M_PI)
                    joint_angles2[i][j] = joint_angles2[i][j] - 2*M_PI;
                else
                    joint_angles2[i][j] = joint_angles2[i][j] + 2*M_PI;
            }
            vector_points2.at(i).positions.at(j) = joint_angles2[i][j];
        }
    }


    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            velocity2[i][j] = (6/(time[len-1]*time[len-1]))*(joint_angles[len - 1][j] - joint_angles[0][j])*(time[i])
                    - (6/(time[len-1]*time[len-1]*time[len-1]))*(joint_angles[len - 1][j] - joint_angles[0][j])*(time[i]*time[i]);


            vector_points2.at(i).velocities.at(j) = velocity2[i][j];
        }
    }


    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            acceleration2[i][j] = 6/(time[len-1]*time[len-1])*(joint_angles[len - 1][j] - joint_angles[0][j])
                    - 12/(time[len-1]*time[len-1]*time[len-1])*(joint_angles[len - 1][j] - joint_angles[0][j])*(time[i]);


            vector_points2.at(i).accelerations.at(j) =  acceleration2[i][j];
        }
    }




    return vector_points2;


}

std::vector <trajectory_msgs::JointTrajectoryPoint> generate_waypoints_5_polynomial(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points, float slow_factor)
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points2;

    int len = vector_points.size();

    float time[len];
    double joint_angles[len][6];
    double velocity[len][6];
    double acceleration[len][6];


    float initial_joints[6];
    float final_joints[6];

    for(int j=0; j<6; j++)
    {
        initial_joints[j] = vector_points.at(0).positions.at(j);
        final_joints[j] = vector_points.at(len - 1).positions.at(j);
    }




    for (int i = 0; i<len; i++)
    {
        time[i] = slow_factor*vector_points.at(i).time_from_start.toSec() ;
        vector_points2.push_back(vector_points.at(i));
    }

    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            joint_angles[i][j] = vector_points.at(i).positions.at(j);
        }

    }

    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            velocity[i][j] = vector_points.at(i).velocities.at(j);
        }


    }

    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            acceleration[i][j] = vector_points.at(i).accelerations.at(j);
        }


    }




    double joint_angles2[len][6];
    double velocity2[len][6];
    double acceleration2[len][6];


    double T = time[len - 1] - time[0];

    MatrixXf a0(6, 1);
    MatrixXf a1(6, 1);
    MatrixXf a2(6, 1);
    MatrixXf a3(6, 1);
    MatrixXf a4(6, 1);
    MatrixXf a5(6, 1);




    for( int i = 0; i<6; i++)
    {
        a0(i, 0) = joint_angles[0][i];
        a1(i, 0) = 0;
        a2(i, 0) = 0;

        a3(i, 0) = 20*(joint_angles[len - 1][i] - joint_angles[0][i])/(2*T*T*T);

        a4(i, 0) =  -30*(joint_angles[len - 1][i] - joint_angles[0][i])/(2*T*T*T*T);

        a5(i, 0) = 12*(joint_angles[len - 1][i] - joint_angles[0][i])/(2*T*T*T*T*T);
    }


    for (int i = 0; i<len; i++)
    {
        vector_points2.at(i).time_from_start = ros::Duration(time[i]);
        //vector_points2.at(i).time_from_start = ros::Duration(float(i)*2/(len-1));
    }



    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            joint_angles2[i][j] = a0(j, 0) + a1(j, 0)*time[i] + a2(j, 0)*time[i]*time[i]
                    + a3(j, 0)*time[i]*time[i]*time[i] + a4(j, 0)*time[i]*time[i]*time[i]*time[i]
                    + a5(j, 0)*time[i]*time[i]*time[i]*time[i]*time[i];


            while(joint_angles2[i][j] < -2*M_PI || joint_angles2[i][j] > 2*M_PI)
            {
                if(joint_angles2[i][j] > 2*M_PI)
                    joint_angles2[i][j] = joint_angles2[i][j] - 2*M_PI;
                else
                    joint_angles2[i][j] = joint_angles2[i][j] + 2*M_PI;
            }
            vector_points2.at(i).positions.at(j) = joint_angles2[i][j];
        }
    }


    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            velocity2[i][j] = a1(j, 0)+ 2*a2(j, 0)*time[i]
                    + 3*a3(j, 0)*time[i]*time[i] + 4*a4(j, 0)*time[i]*time[i]*time[i]
                    + 5*a5(j, 0)*time[i]*time[i]*time[i]*time[i];


            vector_points2.at(i).velocities.at(j) = velocity2[i][j];
        }
    }


    for (int i = 0; i<len; i++)
    {
        for(int j = 0; j<6; j++)
        {
            acceleration2[i][j] = 2*a2(j, 0)
                    + 6*a3(j, 0)*time[i] + 12*a4(j, 0)*time[i]*time[i]
                    + 20*a5(j, 0)*time[i]*time[i]*time[i];


            vector_points2.at(i).accelerations.at(j) =  acceleration2[i][j];
        }
    }




    return vector_points2;



}

std::vector <trajectory_msgs::JointTrajectoryPoint> parabolic_blend(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points,
                                                                    float max_acc, float slow_factor, int no_of_way_points)
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points2;

    int len = vector_points.size();


    double initial_joint_angles[6];
    double final_joint_angles[6];

    for(int j = 0; j<6; j++)
        initial_joint_angles[j] = vector_points.at(0).positions.at(j);

    for(int j = 0; j<6; j++)
        final_joint_angles[j] = vector_points.at(len-1).positions.at(j);



    float max_time_array[6];

    float blend_time[6];

    float max_acc_array[6];

    float max_time = 0;

    float max_blend_time = 0;

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

    for(int j = 0; j<6; j++)
    {

        max_acc_array[j] = my_sign<float>(diff[j])*max_acc*abs(diff[j]/max_diff);

        max_time_array[j] = (7/sqrt(10))*slow_factor*sqrt(4*diff[j]/max_acc_array[j]);

        blend_time[j] = max_time_array[j]/2 - 0.5*sqrt((max_time_array[j]*max_time_array[j]*max_acc_array[j] - 4*diff[j])/max_acc_array[j]);


        if(max_time_array[j] > max_time)
            max_time = max_time_array[j];

        if(blend_time[j] > max_blend_time)
            max_blend_time = blend_time[j];

    }

    std::cout <<"\n\nblend time = " << max_blend_time;
    std::cout <<"\n\ntotla time = " << max_time;

    std::cout << "\n\nration = " << max_time/max_blend_time;



    no_of_way_points = int((7/sqrt(10))*3 + 1);


    float positions_array[no_of_way_points][6];
    float time_from_start_array[no_of_way_points];
    float velocitys_array[no_of_way_points][6];
    float accelerations_array[no_of_way_points][6];

    vector_points2.resize(no_of_way_points+1);



    for(int i = 0; i<no_of_way_points+1; i++)

        time_from_start_array[i] = i*max_time/no_of_way_points;






    for(int i = 0; i<no_of_way_points+1; i++)
    {

        for(int j = 0; j<6; j++)
        {
            if(time_from_start_array[i] < blend_time[j])
            {
                positions_array[i][j] = initial_joint_angles[j] + 0.5*max_acc_array[j]*time_from_start_array[i]*time_from_start_array[i];
            }
            else
            {
                if((time_from_start_array[i] >= blend_time[j]) && (time_from_start_array[i] < max_time_array[j] - blend_time[j]))
                {
                    positions_array[i][j] = initial_joint_angles[j] +
                            max_acc_array[j]*blend_time[j]*(time_from_start_array[i] - 0.5*blend_time[j]);
                }
                else
                {
                    positions_array[i][j] = final_joint_angles[j] - 0.5*max_acc_array[j]*
                            (max_time_array[j] - time_from_start_array[i])*(max_time_array[j] - time_from_start_array[i]);
                }

            }

            while(positions_array[i][j] < -2*M_PI || positions_array[i][j] > 2*M_PI)
            {
                if(positions_array[i][j] > 2*M_PI)
                    positions_array[i][j] = positions_array[i][j] - 2*M_PI;
                else
                    positions_array[i][j] = positions_array[i][j] + 2*M_PI;
            }

        }

    }


    for(int i = 0; i<no_of_way_points+1; i++)                                                                                                                                                                   for(int i = 0; i<no_of_way_points; i++)
    {
        for(int j = 0; j<6; j++)
        {
            if(time_from_start_array[i] < blend_time[j])
                velocitys_array[i][j] = max_acc_array[j]*time_from_start_array[i];
            else
            {
                if((time_from_start_array[i] >= blend_time[j]) && (time_from_start_array[i] < max_time_array[j] - blend_time[j]))
                    velocitys_array[i][j] = max_acc_array[j]*blend_time[j];
                else
                    velocitys_array[i][j] = max_acc_array[j]*(max_time_array[j] - time_from_start_array[i]);

            }

        }
    }


    for(int i = 0; i<no_of_way_points+1; i++)
    {
        for(int j = 0; j<6; j++)
        {
            if(time_from_start_array[i] < blend_time[j])
                accelerations_array[i][j] = max_acc_array[j];
            else
            {
                if((time_from_start_array[i] >= blend_time[j]) && (time_from_start_array[i] < max_time_array[j] - blend_time[j]))
                    accelerations_array[i][j] = 0;
                else
                    accelerations_array[i][j] = -max_acc_array[j];

            }

        }
    }


    for(int i = 0; i<no_of_way_points+1; i++)
    {
        vector_points2.at(i).time_from_start = ros::Duration(time_from_start_array[i]);

        vector_points2.at(i).positions.resize(6);
        vector_points2.at(i).velocities.resize(6);
        vector_points2.at(i).accelerations.resize(6);

        for(int j = 0; j<6; j++)
        {
            vector_points2.at(i).positions.at(j) = positions_array[i][j];
            vector_points2.at(i).velocities.at(j) = velocitys_array[i][j];
            vector_points2.at(i).accelerations.at(j) = accelerations_array[i][j];


        }
    }



    return vector_points2;

}

std::vector<trajectory_msgs::JointTrajectoryPoint> quintic_blend(std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points,
                                                                 float max_acc, int total_no_of_points)
{
    int len = vector_points.size();

    float initial_joint_angles[6];
    float final_joint_angles[6];

    for(int j=0; j<6; j++)
    {
        initial_joint_angles[j] = vector_points.at(0).positions.at(j);
        final_joint_angles[j] = vector_points.at(len - 1).positions.at(j);
    }





    float max_acc_matrix[6];

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

    for(int j = 0; j<6; j++)
        max_acc_matrix[j] = my_sign<float>(diff[j])*max_acc*abs(diff[j]/max_diff);





    float T = 2.00;

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




    MatrixXf companion_matrix(3,3);

    companion_matrix << 0, 0 ,-(2*a[0][2]-max_acc_matrix[0])/(20*a[0][5]),
            1, 0, -(6*a[0][3])/(20*a[0][5]),
            0, 1, -(12*a[0][4])/(20*a[0][5]);


    EigenSolver<MatrixXf> myobject;

    myobject.compute(companion_matrix).eigenvalues();

    std::complex<double> mycomplex(myobject.compute(companion_matrix).eigenvalues().coeff(0));

    float t = sqrt(pow(mycomplex.real(),2) + pow(mycomplex.imag(),2));

    float thetha[6];
    float speed[6];
    float change[6];
    float thetha_f[6];
    float straight_time[6];

    for(int i = 0; i<6; i++)
    {
        thetha[i] = a[i][0] + a[i][1]*t + a[i][2]*pow(t,2) + a[i][3]*pow(t,3) + a[i][4]*pow(t,4) + a[i][5]*pow(t,5);
        speed[i] =  a[i][1] + 2*a[i][2]*pow(t,1) + 3*a[i][3]*pow(t,2) + 4*a[i][4]*pow(t,3) + 5*a[i][5]*pow(t,4);
        change[i] = thetha[i] - initial_joint_angles[i];
        thetha_f[i] = final_joint_angles[i] - change[i];
        straight_time[i] = (thetha_f[i] - thetha[i])/speed[i];
    }

    float shift = 2*t + straight_time[1] - T;


    float total_time = 2*t + straight_time[1];


    std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points2;

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
                if((tt > t) && (tt <= t + straight_time[1]))
                    vector_points2.at(i).positions.at(j) = thetha[j] + speed[j]*(tt - t);

                else
                {
                    tt = tt - shift;
                    vector_points2.at(i).positions.at(j) = a[j][0]+ a[j][1]*tt + a[j][2]*pow(tt,2) +
                            a[j][3]*pow(tt,3) + a[j][4]*pow(tt,4) + a[j][5]*pow(tt,5);
                    tt = tt + shift;
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
            if(tt < t)
                vector_points2.at(i).velocities.at(j) =  a[j][1] + 2*a[j][2]*pow(tt,1) +
                        3*a[j][3]*pow(tt,2) + 4*a[j][4]*pow(tt,3) + 5*a[j][5]*pow(tt,4);

            else
            {
                if((tt > t) && (tt < t + straight_time[1]))
                    vector_points2.at(i).velocities.at(j) = speed[j];

                else
                {
                    tt = tt - shift;
                    vector_points2.at(i).velocities.at(j) =  a[j][1] + 2*a[j][2]*pow(tt,1) +
                            3*a[j][3]*pow(tt,2) + 4*a[j][4]*pow(tt,3) + 5*a[j][5]*pow(tt,4);
                    tt = tt + shift;
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
            if(tt < t)
                vector_points2.at(i).accelerations.at(j) = 2*a[j][2] +
                        6*a[j][3]*pow(tt,1) + 12*a[j][4]*pow(tt,2) + 20*a[j][5]*pow(tt,3);

            else
            {
                if((tt > t) && (tt < t + straight_time[1]))
                    vector_points2.at(i).accelerations.at(j) = 0;

                else
                {
                    tt = tt - shift;
                    vector_points2.at(i).accelerations.at(j) = 2*a[j][2] +
                            6*a[j][3]*pow(tt,1) + 12*a[j][4]*pow(tt,2) + 20*a[j][5]*pow(tt,3);
                    tt = tt + shift;
                }
            }

        }
    }




    //    std::cout << "\n\nblend_time\n\n";
    //    std::cout << sqrt(pow(mycomplex.real(),2) + pow(mycomplex.imag(),2));

    //    std::cout <<"\n\nshift \n\n";
    //    std::cout <<shift;

    //    std::cout << "\n\nthetha \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << thetha[i] <<" ";

    //    std::cout << "\n\nspeed \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << speed[i] <<" ";

    //    std::cout << "\n\nchange \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << change[i] <<" ";

    //    std::cout << "\n\nthetha_f \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << thetha_f[i] <<" ";

    //    std::cout << "\n\nstraight_time \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << straight_time[i] <<" ";

    //    std::cout << "\n\nInitial_angles \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << initial_joint_angles[i] <<" ";

    //    std::cout << "\n\nfinal_joint_angles \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << final_joint_angles[i] <<" ";

    //    std::cout << "\n\nmax_acc_array \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << max_acc_matrix[i] <<" ";

    //    std::cout << "\n\ndiff \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << diff[i] <<" ";

    //    std::cout << "\n\na \n\n ";
    //    for (int i = 0; i<6; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << a[i][j] <<" ";
    //        std::cout <<"\n";
    //    }

    //    std::cout <<"\n\ncompanion matrix \n\n ";
    //    std::cout <<companion_matrix;

    //    std::cout <<"\n\ntotal time \n\n ";
    //    std::cout <<total_time;

    //    std::cout << "\n\npositions \n\n ";
    //    for (int i = 0; i<total_no_of_points+1; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << vector_points2.at(i).positions.at(j) <<" ";
    //        std::cout <<"\n";
    //    }

    //    std::cout << "\n\nvelocity \n\n ";
    //    for (int i = 0; i<total_no_of_points+1; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << vector_points2.at(i).velocities.at(j) <<" ";
    //        std::cout <<"\n";
    //    }

    //    std::cout << "\n\naccelerations \n\n ";
    //    for (int i = 0; i<total_no_of_points+1; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << vector_points2.at(i).accelerations.at(j) <<" ";
    //        std::cout <<"\n";
    //    }




    return vector_points2;


}

std::vector<trajectory_msgs::JointTrajectoryPoint> quintic_cubic_blend(std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points, float max_acc, float T)
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



    float t = (-2*12*a[0][4] + sqrt(pow(2*12*a[0][4],2) - 4*3*20*a[0][5]*6*a[0][3]))/(2*2*12*a[0][4]);
    if(t > T/2)
        t = (-2*12*a[0][4] - sqrt(pow(2*12*a[0][4],2) - 4*3*20*a[0][5]*6*a[0][3]))/(2*2*12*a[0][4]);


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

    companion_matrix << 0, 0 ,-(2*a[0][2]-max_acc_matrix[0])/(20*a[0][5]),
            1, 0, -(6*a[0][3])/(20*a[0][5]),
            0, 1, -(12*a[0][4])/(20*a[0][5]);


    EigenSolver<MatrixXf> myobject;

    myobject.compute(companion_matrix).eigenvalues();

    std::complex<double> mycomplex(myobject.compute(companion_matrix).eigenvalues().coeff(0));

    t = sqrt(pow(mycomplex.real(),2) + pow(mycomplex.imag(),2));

    float thetha[6];
    float speed[6];
    float change[6];
    float thetha_f[6];
    float acc[6];

    for(int i = 0; i<6; i++)
    {
        thetha[i] = a[i][0] + a[i][1]*t + a[i][2]*pow(t,2) + a[i][3]*pow(t,3) + a[i][4]*pow(t,4) + a[i][5]*pow(t,5);
        speed[i] =  a[i][1] + 2*a[i][2]*pow(t,1) + 3*a[i][3]*pow(t,2) + 4*a[i][4]*pow(t,3) + 5*a[i][5]*pow(t,4);
        change[i] = thetha[i] - initial_joint_angles[i];
        thetha_f[i] = final_joint_angles[i] - change[i];
        acc[i] = 2*a[i][2] + 6*a[i][3]*t + 12*a[i][4]*pow(t,2) + 20*a[i][5]*pow(t,3);
    }


    float t_f = (-(6*speed[0]) + sqrt(pow((6*speed[0]),2) + 24*acc[0]*(thetha_f[0] - thetha[0])))/(2*acc[0]);

    if(t_f<0)
        t_f = (-(6*speed[0]) - sqrt(pow((6*speed[0]),2) + 24*acc[0]*(thetha_f[0] - thetha[0])))/(2*acc[0]);



    float cubic_a[6][4];

    for (int joint_angle = 0; joint_angle<6; joint_angle++)
    {
        for (int variable = 0; variable<4; variable++)
        {
            if (variable == 0)
                cubic_a[joint_angle][variable] = thetha[joint_angle];


            if (variable == 1)
                cubic_a[joint_angle][variable] = speed[joint_angle];


            if (variable == 2)
                cubic_a[joint_angle][variable] = (3*thetha_f[joint_angle] - 3*thetha[joint_angle] -
                                                  speed[joint_angle]*t_f - 2*speed[joint_angle]*t_f)/(pow(t_f,2));


            if (variable == 3)
                cubic_a[joint_angle][variable] = (2*thetha[joint_angle] + t_f*speed[joint_angle] -
                                                  2*thetha_f[joint_angle] + t_f*speed[joint_angle])/(pow(t_f,3));
        }
    }

    float shift = 2*t + t_f - T;


    float total_time = 2*t + t_f;


    std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points2;

    int total_no_of_points = 15;

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
                if((tt > t) && (tt <= t + t_f))
                    vector_points2.at(i).positions.at(j) = cubic_a[j][0]+ cubic_a[j][1]*(tt-t) + cubic_a[j][2]*pow(tt-t,2) + cubic_a[j][3]*pow(tt-t,3);

                else
                {
                    tt = tt - shift;
                    vector_points2.at(i).positions.at(j) = a[j][0]+ a[j][1]*tt + a[j][2]*pow(tt,2) +
                            a[j][3]*pow(tt,3) + a[j][4]*pow(tt,4) + a[j][5]*pow(tt,5);
                    tt = tt + shift;
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
            if(tt < t)
                vector_points2.at(i).velocities.at(j) =  a[j][1] + 2*a[j][2]*pow(tt,1) +
                        3*a[j][3]*pow(tt,2) + 4*a[j][4]*pow(tt,3) + 5*a[j][5]*pow(tt,4);

            else
            {
                if((tt > t) && (tt < t + t_f))
                    vector_points2.at(i).velocities.at(j) = cubic_a[j][1] + 2*cubic_a[j][2]*pow(tt-t,1) + 3*cubic_a[j][3]*pow(tt-t,2);

                else
                {
                    tt = tt - shift;
                    vector_points2.at(i).velocities.at(j) =  a[j][1] + 2*a[j][2]*pow(tt,1) +
                            3*a[j][3]*pow(tt,2) + 4*a[j][4]*pow(tt,3) + 5*a[j][5]*pow(tt,4);
                    tt = tt + shift;
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
            if(tt < t)
                vector_points2.at(i).accelerations.at(j) = 2*a[j][2] +
                        6*a[j][3]*pow(tt,1) + 12*a[j][4]*pow(tt,2) + 20*a[j][5]*pow(tt,3);

            else
            {
                if((tt > t) && (tt < t + t_f))
                    vector_points2.at(i).accelerations.at(j) = 2*cubic_a[j][2] + 6*cubic_a[j][3]*pow(tt-t,1);

                else
                {
                    tt = tt - shift;
                    vector_points2.at(i).accelerations.at(j) = 2*a[j][2] +
                            6*a[j][3]*pow(tt,1) + 12*a[j][4]*pow(tt,2) + 20*a[j][5]*pow(tt,3);
                    tt = tt + shift;
                }
            }

        }
    }




    //    std::cout << "\n\nblend_time\n\n";
    //    std::cout << sqrt(pow(mycomplex.real(),2) + pow(mycomplex.imag(),2));

    //    std::cout <<"\n\nshift \n\n";
    //    std::cout <<shift;

    //    std::cout << "\n\nthetha \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << thetha[i] <<" ";

    //    std::cout << "\n\nspeed \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << speed[i] <<" ";

    //    std::cout << "\n\nchange \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << change[i] <<" ";

    //    std::cout << "\n\nthetha_f \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << thetha_f[i] <<" ";

    //    std::cout << "\n\nstraight_time \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << straight_time[i] <<" ";

    std::cout << "\n\nInitial_angles \n\n ";
    for (int i = 0; i<6; i++)
        std::cout << initial_joint_angles[i] <<" ";

    std::cout << "\n\nfinal_joint_angles \n\n ";
    for (int i = 0; i<6; i++)
        std::cout << final_joint_angles[i] <<" ";

    //    std::cout << "\n\nmax_acc_array \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << max_acc_matrix[i] <<" ";

    //    std::cout << "\n\ndiff \n\n ";
    //    for (int i = 0; i<6; i++)
    //        std::cout << diff[i] <<" ";

    //    std::cout << "\n\na \n\n ";
    //    for (int i = 0; i<6; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << a[i][j] <<" ";
    //        std::cout <<"\n";
    //    }

    //    std::cout <<"\n\ncompanion matrix \n\n ";
    //    std::cout <<companion_matrix;

    //    std::cout <<"\n\ntotal time \n\n ";
    //    std::cout <<total_time;

    //    std::cout << "\n\npositions \n\n ";
    //    for (int i = 0; i<total_no_of_points+1; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << vector_points2.at(i).positions.at(j) <<" ";
    //        std::cout <<"\n";
    //    }

    //    std::cout << "\n\nvelocity \n\n ";
    //    for (int i = 0; i<total_no_of_points+1; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << vector_points2.at(i).velocities.at(j) <<" ";
    //        std::cout <<"\n";
    //    }

    //    std::cout << "\n\naccelerations \n\n ";
    //    for (int i = 0; i<total_no_of_points+1; i++)
    //    {
    //        for (int j = 0; j<6; j++)
    //            std::cout << vector_points2.at(i).accelerations.at(j) <<" ";
    //        std::cout <<"\n";
    //    }


    //    std::cout << "\n\ncubic_a \n\n ";
    //    for (int i = 0; i<6; i++)
    //    {
    //        for (int j = 0; j<4; j++)
    //            std::cout << cubic_a[i][j] <<" ";
    //        std::cout <<"\n";
    //    }

    //    std::cout <<"\n\nt_f\n\n";
    //    std::cout <<t_f;




    return vector_points2;


}


std::vector<trajectory_msgs::JointTrajectoryPoint> quintic_parabolic_blend(std::vector<trajectory_msgs::JointTrajectoryPoint> vector_points, float max_acc, float T)
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
        {
            diff[j] = my_sign<float>(final_joint_angles[j])*(abs(initial_joint_angles[j]) + abs(final_joint_angles[j]));
            std::cout <<"im in diff else\n";
        }

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




    float t = (-2*12*a[0][4] + sqrt(pow(2*12*a[0][4],2) - 4*3*20*a[0][5]*6*a[0][3]))/(2*3*20*a[0][5]);
    if(t > T/2)
        t = (-2*12*a[0][4] - sqrt(pow(2*12*a[0][4],2) - 4*3*20*a[0][5]*6*a[0][3]))/(2*3*20*a[0][5]);



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

    companion_matrix << 0, 0 ,-(2*a[0][2]-max_acc_matrix[0])/(20*a[0][5]),
            1, 0, -(6*a[0][3])/(20*a[0][5]),
            0, 1, -(12*a[0][4])/(20*a[0][5]);


    EigenSolver<MatrixXf> myobject;

    myobject.compute(companion_matrix).eigenvalues();

    std::complex<double> mycomplex(myobject.compute(companion_matrix).eigenvalues().coeff(0));

    t = sqrt(pow(mycomplex.real(),2) + pow(mycomplex.imag(),2));


    float initial_thetha[6];
    float initial_velocity[6];


    for(int i = 0; i<6; i++)
    {
        initial_thetha[i] = a[i][0] + a[i][1]*t + a[i][2]*pow(t,2) + a[i][3]*pow(t,3) + a[i][4]*pow(t,4) + a[i][5]*pow(t,5);
        initial_velocity[i] =  a[i][1] + 2*a[i][2]*pow(t,1) + 3*a[i][3]*pow(t,2) + 4*a[i][4]*pow(t,3) + 5*a[i][5]*pow(t,4);
    }

    float t_b1 = 0.3;

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
                (initial_velocity[i] + max_acc_matrix[i]*t_b1)*s_t[i];
    }


    float t_b2[6];
    float t_b22[6];

    for(int i = 0; i<6; i++)
    {
        t_b2[i] = sqrt(2*(final_joint_angles[i] - sudo_thetha[i])/max_acc_matrix[i]);
        t_b22[i] = (initial_velocity[i] + max_acc_matrix[i]*t_b1)/max_acc_matrix[i];
    }


    float total_time = t + t_b1 + s_t[0] + t_b2[0];

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
                    if((tt > t + t_b1) && (tt <= t + t_b1 + s_t[j]))
                    {
                        vector_points2.at(i).positions.at(j) = initial_thetha[j] + initial_velocity[j]*t_b1 + 0.5*max_acc_matrix[j]*pow(t_b1,2) +
                                (initial_velocity[j] + max_acc_matrix[j]*t_b1)*(tt - t - t_b1);
                    }
                    else
                    {
                        vector_points2.at(i).positions.at(j) = sudo_thetha[j] + (initial_velocity[j] + max_acc_matrix[j]*t_b1)*(tt - t - t_b1 - s_t[j]) -
                                0.5*max_acc_matrix[j]*pow((tt - t - t_b1 - s_t[j]),2);

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
                    if((tt > t + t_b1) && (tt <= t + t_b1 + s_t[j]))
                    {
                        vector_points2.at(i).velocities.at(j) = initial_velocity[j] + max_acc_matrix[j]*t_b1;
                    }
                    else
                    {
                        vector_points2.at(i).velocities.at(j) = (initial_velocity[j] + max_acc_matrix[j]*t_b1) - max_acc_matrix[j]*(tt - t - t_b1 - s_t[j]);

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
                    if((tt > t + t_b1) && (tt <= t + t_b1 + s_t[j]))
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

    std::cout << "\n\nInitial_angles \n\n ";
    for (int i = 0; i<6; i++)
        std::cout << initial_joint_angles[i] <<" ";

    std::cout << "\n\nfinal_joint_angles \n\n ";
    for (int i = 0; i<6; i++)
        std::cout << final_joint_angles[i] <<" ";


    return vector_points2;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_actuator2");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroup group("robo_arm");

    moveit::planning_interface::MoveGroup::Plan plan;

    group.setPlannerId("RRTConnectkConfigDefault");

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }


    geometry_msgs::Pose final_pose;

    geometry_msgs::PoseStamped current_state;

    std::vector <double> joint_angles;

    joint_angles.resize(6);

    double tot_view_angles[6] = {-0.06113034883608037, -1.4722874800311487, 1.956202507019043, -2.073355023060934, -0.00863057771791631, -1.2897523085223597};
    double motion_angles[6] = {-0.028253857289449513, -1.4589341322528284, 1.7086734771728516, -1.8530500570880335, -1.5225256125079554, -1.289692227040426};
    double final_xyz[3];


    std::vector<geometry_msgs::Pose> waypoints;

    int place_step = 0;

    group.allowReplanning(true);
    group.setNumPlanningAttempts(5);


    char a;


    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_iptp;
    moveit_msgs::RobotTrajectory trajectory_iptp_cubic;
    moveit_msgs::RobotTrajectory trajectory_iptp_5_poly;
    moveit_msgs::RobotTrajectory trajectory_cubic;
    moveit_msgs::RobotTrajectory trajectory_5_poly;

    int step = 1;

    float acc = 3.0 ;
    float T = 2.0;




    while (ros::ok())
    {

        signal(SIGINT, signal_callback_handler);

        std::cout <<"\n\nstep\n\n"<<step;


        if(step == 1)
        {

            for(int i = 0; i<6; i++)
                joint_angles.at(i) = tot_view_angles[i];


            group.setJointValueTarget(joint_angles);

            group.plan(plan);

            trajectory = plan.trajectory_;

            std::cout <<"\n\n\n-----trajectory iptp 5th poly data------\n\n";
            trajectory_iptp_5_poly = trajectory;

            //            std::cout << "Enter the acc\n";
            //            std::cin >> acc;

            std::cout << "Enter T\n";
            std::cin >> T;

            trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T);
            write_data_in_file(trajectory_iptp_5_poly.joint_trajectory.points, 1);

            plan.trajectory_ = trajectory_iptp_5_poly;


            std::cout << "\n\n\n\n\n-----Run-----\n";
            std::cin >> a;


            group.execute(plan);




        }

        if(step == 2)
        {

            for(int i = 0; i<6; i++)
                joint_angles.at(i) = motion_angles[i];

            group.setJointValueTarget(joint_angles);

            group.plan(plan);

            trajectory = plan.trajectory_;

            std::cout <<"\n\n\n-----trajectory iptp 5th poly data------\n\n";
            trajectory_iptp_5_poly = trajectory;

            //            std::cout << "Enter the acc\n";
            //            std::cin >> acc;

            std::cout << "Enter T\n";
            std::cin >> T;

            trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T);
            write_data_in_file(trajectory_iptp_5_poly.joint_trajectory.points, 1);

            plan.trajectory_ = trajectory_iptp_5_poly;


            std::cout << "\n\n\n\n\n-----Run-----\n";
            std::cin >> a;


            group.execute(plan);



        }


        if(step == 3)
        {
            current_state = group.getCurrentPose();

            final_pose.orientation = current_state.pose.orientation;

            final_pose.position.x = current_state.pose.position.x + 0.10;
            final_pose.position.y = current_state.pose.position.y - 0.15;
            final_pose.position.z = current_state.pose.position.z - 0.45;

            waypoints.push_back(final_pose);
        }

        if(step == 4)
        {


            final_pose.position.x = current_state.pose.position.x + 0.10;
            final_pose.position.y = current_state.pose.position.y - 0.15;
            final_pose.position.z = current_state.pose.position.z - 0.25;

            waypoints.push_back(final_pose);
        }

        if(step == 5)
        {
            final_pose.position.x = 0.351;
            final_pose.position.y = -0.773;
            final_pose.position.z = 0.674;

            waypoints.push_back(final_pose);
        }

        if(step == 6)
        {

            switch(place_step)
            {
            case 0:
                final_pose.position.x = 0.495;
                final_pose.position.y = -0.941;
                final_pose.position.z = 0.347;
                break;

            case 1:
                final_pose.position.x = 0.506;
                final_pose.position.y = -0.670;
                final_pose.position.z = 0.368;
                break;

            case 2:
                final_pose.position.x = 0.192;
                final_pose.position.y = -0.962;
                final_pose.position.z = 0.379;
                break;

            default:
                final_pose.position.x = 0.178;
                final_pose.position.y = -0.665;
                final_pose.position.z = 0.338;
            }

            place_step = place_step + 1;
            place_step = place_step % 4;



            waypoints.push_back(final_pose);

        }



        if(step>2)
        {
            final_xyz[0] = final_pose.position.x;
            final_xyz[1] = final_pose.position.y;
            final_xyz[2] = final_pose.position.z;

            usleep(100000);

            double fraction = group.computeCartesianPath(waypoints,
                                                         0.01,  // eef_step
                                                         0.0,   // jump_threshold
                                                         trajectory);

            std::cout<<"\n---> "<< fraction * 100.0 <<" Path computed \n";

            if(fraction > 0.9)
            {

                robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "robo_arm");

                rt.setRobotTrajectoryMsg(group.getCurrentState()->getRobotModel(), trajectory);

                std::cout <<"\n\n\n-----trajectory iptp 5th poly data------\n\n";
                trajectory_iptp_5_poly = trajectory;

                //                std::cout << "Enter the acc\n";
                //                std::cin >> acc;

                std::cout << "Enter T\n";
                std::cin >> T;

                trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T);
                write_data_in_file(trajectory_iptp_5_poly.joint_trajectory.points, 1);

                plan.trajectory_ = trajectory_iptp_5_poly;


                std::cout << "\n\n\n\n\n-----Run-----\n";
                std::cin >> a;


                group.execute(plan);

                rt.clear();

            }

            waypoints.clear();
        }


        trajectory.joint_trajectory.points.clear();
        plan.trajectory_.joint_trajectory.points.clear();
        trajectory_5_poly.joint_trajectory.points.clear();
        trajectory_cubic.joint_trajectory.points.clear();
        trajectory_iptp.joint_trajectory.points.clear();
        trajectory_iptp_5_poly.joint_trajectory.points.clear();
        trajectory_iptp_cubic.joint_trajectory.points.clear();




        step = step + 1;
        if(step == 7)
            step = 1;



    }



    return 0;
}


