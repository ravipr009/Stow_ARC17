/*
 * speed control
 *
 * joint thread
 *
 * step 2 Asyn execute state_thread
 *
 * all steps are group.execute
 *
 */

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


#include <iostream>

/*
 * 1. Toilet_Brush, 2. Avery_Binder, 3. Balloons, 4. Band_Aid_Tape, 5. Bath_Sponge, 6. Black_Fashion_Gloves, 7. Burts_Bees_Baby_Wipes
   8. Colgate_Toothbrush_4PK, 9. Composition_Book, 10. Crayons, 11. Duct_Tape, 12. Epsom_Salts, 13. Expo_Eraser, 14. Fiskars_Scissors
   15. Flashlight, 16. Glue_Sticks, 17. Hand_Weight, 18. Hanes_Socks, 19. Hinged_Ruled_Index_Cards, 20. Ice_Cube_Tray, 21. Irish_Spring_Soap
   22. Laugh_Out_Loud_Jokes, 23. Marbles, 24. Measuring_Spoons, 25. Mesh_Cup, 26. Mouse_Traps, 27. Pie_Plates, 28. Plastic_Wine_Glass
   29. Poland_Spring_Water, 30. Reynolds_Wrap, 31. Robots_DVD, 32. Robots_Everywhere, 33. Scotch_Sponges, 34. Speed_Stick, 35. White_Facecloth
   36. Table_Cloth, 37. Tennis_Ball_Container, 38. Ticonderoga_Pencils, 39. Tissue_Box, 40.Windex
*/



using namespace Eigen;
float force_x, force_y, force_z;
bool next_step = false;
bool next_joint_step = false;
int count_ = 1;

float object_speed_S2(int object_ID)
{
    float t[40] = {2,2,2,2,2,2,2,2,2,2,
                   2,4,2,2,2,2,4,2,2,2,
                   2,2,2,2,2,2,2,4,4,2,
                   2,2,2,2,2,2,2,2,2,2};

    return t[object_ID - 1];
}

float object_speed_S3(int object_ID)
{
    float t[40] = {2,2,2,2,2,2,2,2,2,2,
                   2,4,2,2,2,2,4,2,2,2,
                   2,2,2,2,2,2,2,2,2,2,
                   2,2,2,2,2,2,2,2,2,2};

    return t[object_ID - 1];
}

float object_speed_S5(int object_ID)
{
    float t[40] = {2,2,2,2,2,2,2,2,2,2,
                   2,4,2,2,2,2,4,2,2,2,
                   2,2,2,2,2,2,2,4,4,2,
                   2,2,2,2,2,2,2,2,2,2};

    return t[object_ID - 1];
}

void object_selected(std_msgs::UInt32 object_ID)
{

    const std::string object_names[]=
    {
        "Toilet_Brush"
        ,"Avery_Binder"
        ,"Balloons"
        ,"Band_Aid_Tape"
        ,"Bath_Sponge"
        ,"Black_Fashion_Gloves"
        ,"Burts_Bees_Baby_Wipes"
        ,"Colgate_Toothbrush_4PK"
        ,"Composition_Book"
        ,"Crayons"
        ,"Duct_Tape"
        ,"Epsom_Salts"
        ,"Expo_Eraser"
        ,"Fiskars_Scissors"
        ,"Flashlight"
        ,"Glue_Sticks"
        ,"Hand_Weight"
        ,"Hanes_Socks"
        ,"Hinged_Ruled_Index_Cards"
        ,"Ice_Cube_Tray"
        ,"Irish_Spring_Soap"
        ,"Laugh_Out_Loud_Jokes"
        ,"Marbles"
        ,"Measuring_Spoons"
        ,"Mesh_Cup"
        ,"Mouse_Traps"
        ,"Pie_Plates"
        ,"Plastic_Wine_Glass"
        ,"Poland_Spring_Water"
        ,"Reynolds_Wrap"
        ,"Robots_DVD"
        ,"Robots_Everywhere"
        ,"Scotch_Sponges"
        ,"Speed_Stick"
        ,"White_Facecloth"
        ,"Table_Cloth"
        ,"Tennis_Ball_Container"
        ,"Ticonderoga_Pencils"
        ,"Tissue_Box"
        ,"Windex"
    };

    std::cout <<"Object detected " << object_names[object_ID.data - 1] << "\n";

    return;
}

void write_data_in_file2(int k, float fx, float fy, float fz)
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

std::vector<trajectory_msgs::JointTrajectoryPoint> quintic_parabolic_blend2(double initial_angles[], double final_angles[], float max_acc, float T, float t_b1)
{

    float initial_joint_angles[6];
    float final_joint_angles[6];

    for(int j=0; j<6; j++)
    {
        initial_joint_angles[j] = (float)initial_angles[j];
        final_joint_angles[j] = (float)final_angles[j];
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

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
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

void *next_move(void *threadid)
{


    geometry_msgs::PoseStamped current_state;

    moveit::planning_interface::MoveGroup group("robo_arm");

    std::vector<double>* tid;
    tid = (std::vector<double>*)threadid;

    float dis_threshold = 0.005;
    float force_threshold = 60;

    float dis_error = 0;
    float force_error = 0;

    char a;

    while(next_step == false)
    {
        //        std::cin >> a;

        std::cout <<"\nim in state thread";

        current_state = group.getCurrentPose();

        dis_error = pow(current_state.pose.position.x - tid->at(0), 2) + pow(current_state.pose.position.y - tid->at(1), 2) + pow(current_state.pose.position.z - tid->at(2), 2);

        std::cout <<"dis_error = " << sqrt(dis_error) << "\n\n";

        if( sqrt(dis_error) < dis_threshold )
        {
            std::cout <<"\nrobot has reached the desired state\n";
            group.stop();
            next_step = true;
            break;
        }

        force_error = pow(force_x, 2) + pow(force_y, 2) + pow(force_z, 2) ;

        if( sqrt(force_error) > force_threshold )
        {
            std::cout <<"\nexperiencing more force\n";
            group.stop();
            next_step = true;
            break;
        }

    }

}

void *next_joint_move(void *threadid)
{


    std::vector<double> current_angles;

    moveit::planning_interface::MoveGroup group("robo_arm");

    std::vector<double>* tid;
    tid = (std::vector<double>*)threadid;

    float angle_threshold = 0.05;

    float error = 0;

    char a;


    while(next_joint_step == false)
    {

        //        std::cin >> a;

        current_angles = group.getCurrentJointValues();

        std::cout <<"\nim in joint thread";

        //        std::cout << current_angles.at(0) <<", "
        //                  << current_angles.at(1) <<", "
        //                  << current_angles.at(2) <<", "
        //                  << current_angles.at(3) <<", "
        //                  << current_angles.at(4) <<", "
        //                  << current_angles.at(5) <<"\n "
        //                  << tid->at(0) <<", "
        //                  << tid->at(1) <<", "
        //                  << tid->at(2) <<", "
        //                  << tid->at(3) <<", "
        //                  << tid->at(4) <<", "
        //                  << tid->at(5) <<"\n\n ";



        error = pow((current_angles.at(0) - tid->at(0)), 2) +
                pow((current_angles.at(1) - tid->at(1)), 2) +
                pow((current_angles.at(2) - tid->at(2)), 2) +
                pow((current_angles.at(3) - tid->at(3)), 2) +
                pow((current_angles.at(4) - tid->at(4)), 2) +
                pow((current_angles.at(5) - tid->at(5)), 2) ;

        //        std:: cout << sqrt(error) <<"\n\n";

        if(sqrt(error) < angle_threshold)
        {
            std::cout <<"\nrobot has reached the desired joint_state\n";
            group.stop();
            next_joint_step = true;
            break;
        }



    }

}

int object_bin_position(int object_ID)
{
    float t[40] = {2, 2, 1, 3, 3, 1, 1, 1, 2, 3,
                   3, 1, 3, 3, 3, 1, 3, 1, 3, 2,
                   3, 3, 3, 3, 2, 3, 1, 2, 1, 2,
                   1, 1, 3, 3, 2, 2, 2, 1, 1, 2};

    return t[object_ID - 1];
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

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    moveit::planning_interface::MoveGroup::Plan plan;

    visualization_msgs::MarkerArray markers;

    geometry_msgs::Pose final_pose;

    geometry_msgs::PoseStamped current_state;

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

    float rack_length = 0.9;
    float rack_width = 0.57;

    float row_offset ;
    float column_offset ;

    int rows_1 = 2;
    int columns_1 = 1;

    int rows_2 = 1;
    int columns_2 = 1;

    int rows_3 = 2;
    int columns_3 = 2;

    int bin_number = 1;

    int bin_1_number = 0;
    int bin_2_number = 0;
    int bin_3_number = 0;

    float bin_1_position[rows_1*columns_1][3];
    float bin_2_position[rows_2*columns_2][3];
    float bin_3_position[rows_3*columns_3][3];

    float rack_length_1 = rack_length/4;

    for(int i = 0; i< rows_1; i++)
    {
        row_offset = 0;
        if(i == 0)
            row_offset = 0.01;
        if(i == rows_1 - 1)
            row_offset = row_offset - 0.01;

        for(int k = 0; k< columns_1; k++)
        {
            column_offset = 0;
            if(k == 0)
                column_offset = 0.01;
            if(k == columns_1 - 1)
                column_offset = column_offset - 0.01;

            for(int j = 0; j< 3; j++)
            {
                if(j == 0)
                    bin_1_position[i*columns_1 + k][j] = row_offset + rack_width/(2*rows_1) + i*rack_width/(rows_1);
                if(j == 1)
                    bin_1_position[i*columns_1 + k][j] = column_offset + rack_length_1/(2*columns_1) + k*rack_length_1/(columns_1);
                if(j == 2)
                    bin_1_position[i*columns_1 + k][j] = 0.50;
            }

            std::cout << bin_1_position[i*columns_1 + k][0] <<", "<< bin_1_position[i*columns_1 + k][1] <<", "<< bin_1_position[i*columns_1 + k][2] <<"\n ";

        }
    }

    float rack_length_2 = rack_length/4;

    for(int i = 0; i< rows_2; i++)
    {
        row_offset = 0;
        if(i == 0)
            row_offset = 0.01;
        if(i == rows_2 - 1)
            row_offset = row_offset - 0.01;

        for(int k = 0; k< columns_2; k++)
        {
            column_offset = 0;
            if(k == 0)
                column_offset = 0.01;
            if(k == columns_2 - 1)
                column_offset = column_offset - 0.01;

            for(int j = 0; j< 3; j++)
            {
                if(j == 0)
                    bin_2_position[i*columns_2 + k][j] = row_offset + rack_width/(2*rows_2) + i*rack_width/(rows_2);
                if(j == 1)
                    bin_2_position[i*columns_2 + k][j] = column_offset + rack_length_1 +  rack_length_2/(2*columns_2) + k*rack_length_2/(columns_2);
                if(j == 2)
                    bin_2_position[i*columns_2 + k][j] = 0.50;
            }

            std::cout << bin_2_position[i*columns_2 + k][0] <<", "<< bin_2_position[i*columns_2 + k][1] <<", "<< bin_2_position[i*columns_2 + k][2] <<"\n ";

        }
    }

    float rack_length_3 = rack_length/2;

    for(int i = 0; i< rows_3; i++)
    {
        row_offset = 0;
        if(i == 0)
            row_offset = 0.01;
        if(i == rows_3 - 1)
            row_offset = row_offset - 0.01;

        for(int k = 0; k< columns_3; k++)
        {
            column_offset = 0;
            if(k == 0)
                column_offset = 0.01;
            if(k == columns_3 - 1)
                column_offset = column_offset - 0.01;

            for(int j = 0; j< 3; j++)
            {
                if(j == 0)
                    bin_3_position[i*columns_3 + k][j] = row_offset + rack_width/(2*rows_3) + i*rack_width/(rows_3);
                if(j == 1)
                    bin_3_position[i*columns_3 + k][j] = column_offset + rack_length_1 + rack_length_2 + rack_length_3/(2*columns_3) + k*rack_length_3/(columns_3);
                if(j == 2)
                    bin_3_position[i*columns_3 + k][j] = 0.50;
            }

            std::cout << bin_3_position[i*columns_3 + k][0] <<", "<< bin_3_position[i*columns_3 + k][1] <<", "<< bin_3_position[i*columns_3 + k][2] <<"\n ";

        }
    }

    std::vector <double> joint_angles;

    joint_angles.resize(6);

    double tot_view_angles[6] = {-1.8936517874347132, -1.8472698370562952, 1.888345718383789, -1.385820213948385, -1.6484110991107386, 0.48918482661247253};

    std::vector<geometry_msgs::Pose> waypoints;

    tf::TransformListener listener;

    tf::Quaternion q, q1, q2, q3, q4;

    pthread_t threads;

    pthread_create(&threads, NULL, broadcast_kf, NULL);

    char a;

    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_iptp_5_poly;

    int step = 1;

    float acc = 3.0 ;
    float T = 2.0;
    float t_b1 = 0.2;

    int feedback = 0;

    std::vector<double> final_xyz;
    final_xyz.resize(3);

    std::vector<double> final_angles;
    final_angles.resize(6);

    int object_ID;

    int total_number_of_objects = 40;

    int object_IDs[total_number_of_objects];

    for (int i = 0; i<total_number_of_objects; i++)
        object_IDs[i] = 1;

    while (ros::ok())

    {

        signal(SIGINT, signal_callback_handler);

        if(step == 1)
        {
            std::cout << "\n\n*************** im in step 1 ***************\n\n";

            T = 2.0;
            t_b1 = 0.2;

            for(int i = 0; i<6; i++)
                joint_angles.at(i) = tot_view_angles[i];

            double initial_joint_angles[6];
            for (int i = 0; i<6; i++)
                initial_joint_angles[i] = group.getCurrentJointValues().at(i);

            group.setJointValueTarget(joint_angles);

            group.plan(plan);

            trajectory = plan.trajectory_;

            trajectory_iptp_5_poly = trajectory;

            trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend2(initial_joint_angles, tot_view_angles, acc, 2.0, 0.2);

            write_data_in_file(trajectory_iptp_5_poly.joint_trajectory.points, 1);

            plan.trajectory_ = trajectory_iptp_5_poly;

            std::cout << "\n\n\n\n\n-----Run-----\n";
            //            std::cin >> a;


            for(int i = 0; i<6; i++)
                final_angles.at(i) = tot_view_angles[i];

            group.asyncExecute(plan);

            pthread_create(&threads, NULL, next_joint_move, &final_angles);

            while(next_joint_step == false)
            {
            };

            next_joint_step = false;


            //*******call service for object centroid and object normal

            ros::ServiceClient client = nh.serviceClient<apc_nozel::get_points2>("get_kinect_points_topic");

            apc_nozel::get_points2 srv;

            std::cout<<"\n\ncalling service to get points wrt kinect\n";

            ros::service::waitForService("get_kinect_points_topic");

            srv.request.remaining_objects.data.resize(40);

            for(int i = 0; i<total_number_of_objects; i++)
                srv.request.remaining_objects.data.at(i) = object_IDs[i];


            if (client.call(srv))
                std::cout<<"point_service successful\n";
            else
                std::cout<<"point_service failed\n";

            object_ID = srv.response.object_ID.data;

            object_selected(srv.response.object_ID);

            object_IDs[srv.response.object_ID.data - 1] = 0;

            centroid_point_wrt_kinect.point.x = srv.response.centroid_wrt_kinect.x;
            centroid_point_wrt_kinect.point.y = srv.response.centroid_wrt_kinect.y;
            centroid_point_wrt_kinect.point.z = srv.response.centroid_wrt_kinect.z;

            centroid_point_wrt_kinect.header.frame_id = "kf";

            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

            listener.transformPoint("world", centroid_point_wrt_kinect, centroid_point_wrt_world);

            normal_point_wrt_kinect.point.x = srv.response.normal_wrt_kinect.x;
            normal_point_wrt_kinect.point.y = srv.response.normal_wrt_kinect.y;
            normal_point_wrt_kinect.point.z = srv.response.normal_wrt_kinect.z;

            normal_point_wrt_kinect.header.frame_id = "kf";

            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

            listener.transformPoint("world", normal_point_wrt_kinect, normal_point_wrt_world);


            axis_point_wrt_kinect.point.x = srv.response.axis_wrt_kinect.x;
            axis_point_wrt_kinect.point.y = srv.response.axis_wrt_kinect.y;
            axis_point_wrt_kinect.point.z = srv.response.axis_wrt_kinect.z;

            axis_point_wrt_kinect.header.frame_id = "kf";

            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

            listener.transformPoint("world", axis_point_wrt_kinect, axis_point_wrt_world);


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

            line_list.points.push_back(centroid_point_wrt_world.point);
            line_list.points.push_back(normal_point_wrt_world.point);

            line_list.points.push_back(centroid_point_wrt_world.point);
            line_list.points.push_back(axis_point_wrt_world.point);

            marker_pub.publish(line_list);

            normal_point_wrt_world.point.z = normal_point_wrt_world.point.z + 0.07;


            if((object_ID == 12)||(object_ID == 17) ||(object_ID == 29))
            {
                t_b1 = 0.2;
                T = 4.0;
            }

            if ((object_ID == 28)||(object_ID == 17)||(object_ID == 23)||(object_ID == 1)||(object_ID == 24)||(object_ID == 25)||(object_ID == 14)||(object_ID == 5)||(object_ID == 15))
            {
                normal_point_wrt_world.point.x = centroid_point_wrt_world.point.x;
                normal_point_wrt_world.point.y = centroid_point_wrt_world.point.y;
                normal_point_wrt_world.point.z = centroid_point_wrt_world.point.z + 0.25;
            }

            float n_x = (normal_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
            float n_y = (normal_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
            float n_z = (normal_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

            float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

            n_x = n_x/m;
            n_y = n_y/m;
            n_z = n_z/m;

            float phi = atan2(-n_y, -n_x);

            q.setRPY( 0, M_PI/2, 0);
            q1.setRPY( 0, 0, phi);

            q4 = q1*q;

            final_pose.position.x = normal_point_wrt_world.point.x;
            final_pose.position.y = normal_point_wrt_world.point.y;
            final_pose.position.z = normal_point_wrt_world.point.z;

            final_pose.orientation.x = q4.getX();
            final_pose.orientation.y = q4.getY();
            final_pose.orientation.z = q4.getZ();
            final_pose.orientation.w = q4.getW();


            for (int i = 0; i<3; i++)
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

                    marker.pose.position.x = centroid_point_wrt_world.point.x;
                    marker.pose.position.y = centroid_point_wrt_world.point.y;
                    marker.pose.position.z = centroid_point_wrt_world.point.z;

                }

                if(i == 1)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;

                    marker.pose.position.x = normal_point_wrt_world.point.x;
                    marker.pose.position.y = normal_point_wrt_world.point.y;
                    marker.pose.position.z = normal_point_wrt_world.point.z;
                }

                if(i == 2)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;

                    marker.pose.position.x = axis_point_wrt_world.point.x;
                    marker.pose.position.y = axis_point_wrt_world.point.y;
                    marker.pose.position.z = axis_point_wrt_world.point.z;
                }

                marker.lifetime = ros::Duration();

                markers.markers.push_back(marker);

            }

            pub.publish(markers);

            markers.markers.clear();

            waypoints.push_back(final_pose);


            final_xyz.at(0) = final_pose.position.x;
            final_xyz.at(1) = final_pose.position.y;
            final_xyz.at(2) = final_pose.position.z;



        }

        if(step == 2)
        {
            //*******call service for object centroid and object normal wrt suction frame


            ros::ServiceClient client = nh.serviceClient<apc_nozel::actuator_frame2>("get_suction_points_topic");

            apc_nozel::actuator_frame2 srv2;

            std::cout<<"calling service to get points wrt suction2\n";

            ros::service::waitForService("get_suction_points_topic");

            srv2.request.centroid_wrt_world = centroid_point_wrt_world.point;
            srv2.request.normal_wrt_world = normal_point_wrt_world.point;

            if (client.call(srv2))
                std::cout<<"suction_point_service successful\n";
            else
                std::cout<<"suction_point_service failed\n";


            float x = srv2.response.centroid_wrt_suction_point.x;
            float y = srv2.response.centroid_wrt_suction_point.y;
            float z = srv2.response.centroid_wrt_suction_point.z;

            float thetha = atan2(sqrt(x*x + y*y), z);
            float phi = atan2(y, x);


            float a_y = -M_PI/2 + thetha;


            std::cout << "nozel need to be rotated by " << a_y*180/M_PI <<" in degrees\n";
            std::cout << "nozel need to be rotated by " << a_y <<" in radians\n";

            std::cout <<"Press any key if you have entered the nozel angle\n";
            //            std::cin >> a;

            float r = sqrt(x*x + y*y + z*z);

            float nozel_length;

            if((object_ID == 17) || (object_ID == 5))
                nozel_length = 0.05;
            else
                nozel_length = 0.07;

            final_pos_wrt_suction_point.header.frame_id = "iitk_link";

            final_pos_wrt_suction_point.point.x = (r-nozel_length)*cos(phi)*sin(thetha);
            final_pos_wrt_suction_point.point.y = (r-nozel_length)*sin(phi)*sin(thetha);
            final_pos_wrt_suction_point.point.z = (r-nozel_length)*cos(thetha);

            listener.waitForTransform("world", "iitk_link",  ros::Time(0), ros::Duration(3));

            listener.transformPoint("world", final_pos_wrt_suction_point, final_pos_wrt_world);

            final_pose.position.x = final_pos_wrt_world.point.x;
            final_pose.position.y = final_pos_wrt_world.point.y;
            final_pose.position.z = final_pos_wrt_world.point.z;

            current_state = group.getCurrentPose();

            final_pose.orientation = current_state.pose.orientation;

            waypoints.push_back(final_pose);


            final_xyz.at(0) = final_pose.position.x;
            final_xyz.at(1) = final_pose.position.y;
            final_xyz.at(2) = final_pose.position.z;

        }

        if(step == 3)
        {

            current_state = group.getCurrentPose();

            final_pose.orientation = current_state.pose.orientation;

            final_pose.position.x = current_state.pose.position.x;
            final_pose.position.y = current_state.pose.position.y;
            final_pose.position.z = current_state.pose.position.z + 0.5;

            waypoints.push_back(final_pose);


            final_xyz.at(0) = final_pose.position.x;
            final_xyz.at(1) = final_pose.position.y;
            final_xyz.at(2) = final_pose.position.z;

        }

        if(step == 4)
        {


            float x = (axis_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
            float y = (axis_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
            float z = (axis_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

            float m = sqrt(pow(x,2) + pow(y,2) + pow(z,2) );

            z = z/m;
            y = y/m;
            x = x/m;

            float phi = atan2(y, x);

            std::cout << "\n axis phi: = " << phi*180/M_PI;

            if(phi > M_PI/2)
                phi = phi - M_PI;
            if(phi < -M_PI/2)
                phi = phi + M_PI;

            std::cout << "\n axis phi: = " << phi*180/M_PI <<"\n";

            bin_number = object_bin_position(object_ID);

            switch(bin_number)
            {
            case 1:
                std::cout <<"bin_1_number = " << bin_1_number <<"\n";
                point_wrt_rack.point.x = bin_1_position[bin_1_number][1];
                point_wrt_rack.point.y = bin_1_position[bin_1_number][0];
                point_wrt_rack.point.z = bin_1_position[bin_1_number][2];

                bin_1_number = bin_1_number + 1;
                bin_1_number = bin_1_number % (rows_1 * columns_1);
                std::cout <<"bin_1_number = " << bin_1_number <<"\n";

                break;

            case 2:
                std::cout <<"bin_2_number = " << bin_2_number <<"\n";
                point_wrt_rack.point.x = bin_2_position[bin_2_number][1];
                point_wrt_rack.point.y = bin_2_position[bin_2_number][0];
                point_wrt_rack.point.z = bin_2_position[bin_2_number][2];

                bin_2_number = bin_2_number + 1;
                bin_2_number = bin_2_number % (rows_2 * columns_2);
                std::cout <<"bin_2_number = " << bin_2_number <<"\n";

                break;

            case 3:
                std::cout <<"bin_3_number = " << bin_3_number <<"\n";
                point_wrt_rack.point.x = bin_3_position[bin_3_number][1];
                point_wrt_rack.point.y = bin_3_position[bin_3_number][0];
                point_wrt_rack.point.z = bin_3_position[bin_3_number][2];

                bin_3_number = bin_3_number + 1;
                bin_3_number = bin_3_number %(rows_3 * columns_3);
                std::cout <<"bin_3_number = " << bin_3_number <<"\n";

                break;

            }

            bin_number = bin_number + 1;
            if(bin_number == 4)
                bin_number = 1;


            point_wrt_rack.header.frame_id = "rack_link";

            listener.waitForTransform( "world", "rack_link", ros::Time(0), ros::Duration(3));

            listener.transformPoint("world", point_wrt_rack, point_wrt_world);

            std::cout << "\n" << point_wrt_rack.point.x << ", "
                      << point_wrt_rack.point.y << ", "
                      << point_wrt_rack.point.z << "\n\n ";

            std::cout << "\n" << point_wrt_world.point.x << ", "
                      << point_wrt_world.point.y << ", "
                      << point_wrt_world.point.z << "\n\n ";

            for (int i = 0; i<1; i++)
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

                marker.lifetime = ros::Duration();

                markers.markers.push_back(marker);

            }

            pub.publish(markers);

            markers.markers.clear();


            final_pose.position.x = point_wrt_world.point.x;
            final_pose.position.y = point_wrt_world.point.y;
            final_pose.position.z = point_wrt_world.point.z;

            current_state = group.getCurrentPose();

            q1.setX(current_state.pose.orientation.x);
            q1.setY(current_state.pose.orientation.y);
            q1.setZ(current_state.pose.orientation.z);
            q1.setW(current_state.pose.orientation.w);

            q2.setRPY(0,0,-phi);

            q4 = q2*q1;

            final_pose.orientation.x = q4.getX();
            final_pose.orientation.y = q4.getY();
            final_pose.orientation.z = q4.getZ();
            final_pose.orientation.w = q4.getW();

            waypoints.push_back(final_pose);


            final_xyz.at(0) = final_pose.position.x;
            final_xyz.at(1) = final_pose.position.y;
            final_xyz.at(2) = final_pose.position.z;

        }

        if(step == 5)
        {

            current_state = group.getCurrentPose();

            final_pose.orientation = current_state.pose.orientation;

            final_pose.position.x = current_state.pose.position.x;
            final_pose.position.y = current_state.pose.position.y;
            final_pose.position.z = current_state.pose.position.z - 0.2;

            waypoints.push_back(final_pose);


            final_xyz.at(0) = final_pose.position.x;
            final_xyz.at(1) = final_pose.position.y;
            final_xyz.at(2) = final_pose.position.z;

        }

        if(step == 6)
        {
            std::cout <<"press any key to Turn Off the suction\n";
            //            std::cin >> a;

        }

        if(step == 7)
        {

            current_state = group.getCurrentPose();

            q.setRPY(0, M_PI/2, 0);
            q2.setRPY(0, 0, 0*M_PI/180);

            q3 = q2*q;

            //            final_pose.orientation.x = q3.getX();
            //            final_pose.orientation.y = q3.getY();
            //            final_pose.orientation.z = q3.getZ();
            //            final_pose.orientation.w = q3.getW();

            final_pose.orientation = current_state.pose.orientation;

            final_pose.position.x = current_state.pose.position.x;
            final_pose.position.y = current_state.pose.position.y;
            final_pose.position.z = current_state.pose.position.z + 0.15;

            waypoints.push_back(final_pose);


            final_xyz.at(0) = final_pose.position.x;
            final_xyz.at(1) = final_pose.position.y;
            final_xyz.at(2) = final_pose.position.z;

        }

        if(step!=6 && step!=0)
        {

            while(1)
            {

                usleep(100000);

                double fraction = group.computeCartesianPath(waypoints,
                                                             0.01,  // eef_step
                                                             0.0,   // jump_threshold
                                                             trajectory);

                std::cout<<"\n---> "<< fraction * 100.0 <<" Path computed \n";

                std::cout << "\nstep = " << step <<"\n\nfeedback = " << feedback << "\n\n";


                if(fraction > 0.999)
                {


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


                    if (step == 1)
                    {
                        trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, 2.0, 0.2);
                    }

                    if (step == 2)
                    {

                        if(feedback == 0)
                        {
                            float slow_factor = object_speed_S2(object_ID);
                            trajectory_iptp_5_poly.joint_trajectory.points = slow_compute_cartesian_path(trajectory.joint_trajectory.points, slow_factor);
                        }
                        else
                            trajectory_iptp_5_poly.joint_trajectory.points = slow_compute_cartesian_path(trajectory.joint_trajectory.points, 1);
                    }

                    if (step == 3)
                    {

                        float slow_factor = object_speed_S3(object_ID);
                        trajectory_iptp_5_poly.joint_trajectory.points = slow_compute_cartesian_path(trajectory.joint_trajectory.points, slow_factor);
                    }

                    if (step == 4)
                    {
                        trajectory_iptp_5_poly.joint_trajectory.points = quintic_parabolic_blend(trajectory.joint_trajectory.points, acc, T, t_b1);
                    }

                    if (step == 5)
                    {

                        float slow_factor = object_speed_S5(object_ID);
                        trajectory_iptp_5_poly.joint_trajectory.points = slow_compute_cartesian_path(trajectory.joint_trajectory.points, slow_factor);
                    }

                    if (step == 7)
                    {
                        trajectory_iptp_5_poly.joint_trajectory.points = slow_compute_cartesian_path(trajectory.joint_trajectory.points, 1);
                    }

                    plan.trajectory_ = trajectory_iptp_5_poly;


                    std::cout << "\n\n\n\n\n-----Run2-----\n";
//                    std::cin >> a;

                    if((step == 2)&&(feedback == 0))
                    {
                        std::cout << "Turn on the suction\n";

                        group.asyncExecute(plan);

                        pthread_create(&threads, NULL, next_move, &final_xyz);

                        while(next_step == false)
                        {
                        };

                        next_step = false;

                        std::cout <<"Async_executed\n";
                    }
                    else
                    {
                        group.execute(plan);
                    }



                    std::cout << "Turn on the suction\n";

                    //                    group.asyncExecute(plan);

                    //                    pthread_create(&threads, NULL, next_move, &final_xyz);

                    //                    while(next_step == false)
                    //                    {
                    //                    };

                    //                    next_step = false;

                    //                    std::cout <<"Async_executed\n";



                    rt.clear();

                    if((feedback == 0)||(step == 1))
                    {
                        waypoints.clear();
                        plan.trajectory_.joint_trajectory.points.clear();
                        trajectory.joint_trajectory.points.clear();
                        trajectory_iptp_5_poly.joint_trajectory.points.clear();


                        std::cout <<"\nplan.trajectory_.joint_trajectory.points.size = " << plan.trajectory_.joint_trajectory.points.size();
                        std::cout <<"\ntrajectory.joint_trajectory.points.size = " << trajectory.joint_trajectory.points.size();
                        std::cout <<"\ntrajectory_iptp_5_poly.joint_trajectory.points.size = " << trajectory_iptp_5_poly.joint_trajectory.points.size();

                        feedback = 0;
                        break;
                    }

                    if((step == 2) && (feedback == 1))
                    {
                        waypoints.clear();

                        tote_reference_point.header.frame_id = "tote_link";

                        tote_reference_point.point.x = 0;
                        tote_reference_point.point.y = 0;
                        tote_reference_point.point.z = 0;

                        listener.waitForTransform( "world", "tote_link", ros::Time(0), ros::Duration(3));

                        listener.transformPoint("world", tote_reference_point, tote_wrt_world);

                        normal_point_wrt_world.point.x = (centroid_point_wrt_world.point.x + tote_wrt_world.point.x)/2;
                        normal_point_wrt_world.point.y = (centroid_point_wrt_world.point.y + tote_wrt_world.point.y)/2;
                        normal_point_wrt_world.point.z = centroid_point_wrt_world.point.z + 0.15;

                        float n_x = (normal_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
                        float n_y = (normal_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
                        float n_z = (normal_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

                        float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

                        n_x = n_x/m;
                        n_y = n_y/m;
                        n_z = n_z/m;

                        float phi = atan2(-n_y, -n_x);

                        q.setRPY( 0, M_PI/2, 0);
                        q1.setRPY( 0, 0, phi);

                        q4 = q1*q;

                        final_pose.orientation.x = q4.getX();
                        final_pose.orientation.y = q4.getY();
                        final_pose.orientation.z = q4.getZ();
                        final_pose.orientation.w = q4.getW();

                        final_pose.position.x = normal_point_wrt_world.point.x;
                        final_pose.position.y = normal_point_wrt_world.point.y;
                        final_pose.position.z = normal_point_wrt_world.point.z;

                        waypoints.push_back(final_pose);


                        final_xyz.at(0) = final_pose.position.x;
                        final_xyz.at(1) = final_pose.position.y;
                        final_xyz.at(2) = final_pose.position.z;


                        feedback = 0;
                        step = 1;

                    }

                }
                else
                {
                    waypoints.clear();

                    if( ((step == 2) && (feedback == 0)) || ((step == 1) && (feedback == 0)))
                    {
                        current_state = group.getCurrentPose();

                        final_pose.orientation = current_state.pose.orientation;

                        if(step == 1)
                        {
                            tote_reference_point.header.frame_id = "tote_link";

                            tote_reference_point.point.x = 0;
                            tote_reference_point.point.y = 0;
                            tote_reference_point.point.z = 0;

                            listener.waitForTransform( "world", "tote_link", ros::Time(0), ros::Duration(3));

                            listener.transformPoint("world", tote_reference_point, tote_wrt_world);

                            normal_point_wrt_world.point.x = (centroid_point_wrt_world.point.x + tote_wrt_world.point.x)/2;
                            normal_point_wrt_world.point.y = (centroid_point_wrt_world.point.y + tote_wrt_world.point.y)/2;
                            normal_point_wrt_world.point.z = centroid_point_wrt_world.point.z + 0.15;

                            float n_x = (normal_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
                            float n_y = (normal_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
                            float n_z = (normal_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

                            float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

                            n_x = n_x/m;
                            n_y = n_y/m;
                            n_z = n_z/m;

                            float phi = atan2(-n_y, -n_x);

                            q.setRPY( 0, M_PI/2, 0);
                            q1.setRPY( 0, 0, phi);

                            q4 = q1*q;

                            final_pose.orientation.x = q4.getX();
                            final_pose.orientation.y = q4.getY();
                            final_pose.orientation.z = q4.getZ();
                            final_pose.orientation.w = q4.getW();

                            final_pose.position.x = normal_point_wrt_world.point.x;
                            final_pose.position.y = normal_point_wrt_world.point.y;
                            final_pose.position.z = normal_point_wrt_world.point.z;
                        }
                        else
                        {
                            final_pose.position.x = current_state.pose.position.x;
                            final_pose.position.y = current_state.pose.position.y;
                            final_pose.position.z = current_state.pose.position.z + 0.2;
                        }

                        waypoints.push_back(final_pose);


                        final_xyz.at(0) = final_pose.position.x;
                        final_xyz.at(1) = final_pose.position.y;
                        final_xyz.at(2) = final_pose.position.z;

                        feedback = 1;

                    }
                    else
                    {
                        step = 0;
                        break;
                    }

                }

            }

        }

        //        usleep(100000);

        step = step + 1;

        if(step == 8)
            step = 1;

    }


    return 0;
}

