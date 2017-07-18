/*
        No speed control
        all steps are group.execute
 */


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <tf/transform_listener.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

#include <apc_nozel/get_points2.h>
#include <apc_nozel/actuator_frame2.h>

#include <pthread.h>

#include <eigen_conversions/eigen_msg.h>
#include <apc_nozel/apc_simulation.h>

#include <iostream>

#include <apc_nozel/CheckClearProtectiveStop.h>

/*
 * 1. Toilet_Brush, 2. Avery_Binder, 3. Balloons, 4. Band_Aid_Tape, 5. Bath_Sponge, 6. Black_Fashion_Gloves, 7. Burts_Bees_Baby_Wipes
   8. Colgate_Toothbrush_4PK, 9. Composition_Book, 10. Crayons, 11. Duct_Tape, 12. Epsom_Salts, 13. Expo_Eraser, 14. Fiskars_Scissors
   15. Flashlight, 16. Glue_Sticks, 17. Hand_Weight, 18. Hanes_Socks, 19. Hinged_Ruled_Index_Cards, 20. Ice_Cube_Tray, 21. Irish_Spring_Soap
   22. Laugh_Out_Loud_Jokes, 23. Marbles, 24. Measuring_Spoons, 25. Mesh_Cup, 26. Mouse_Traps, 27. Pie_Plates, 28. Plastic_Wine_Glass
   29. Poland_Spring_Water, 30. Reynolds_Wrap, 31. Robots_DVD, 32. Robots_Everywhere, 33. Scotch_Sponges, 34. Speed_Stick, 35. White_Facecloth
   36. Table_Cloth, 37. Tennis_Ball_Container, 38. Ticonderoga_Pencils, 39. Tissue_Box, 40.Windex
*/



using namespace Eigen;
bool next_step = false;
bool next_joint_step = false;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_actuator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroup group("robo_arm");

    group.setPlannerId("RRTConnectkConfigDefault");

    ros::ServiceClient protective_clear_srv = nh.serviceClient <apc_nozel::CheckClearProtectiveStop>("iitktcs/motion_planner/check_clear_protective_stop");

    apc_nozel::CheckClearProtectiveStop check_srv;

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

    geometry_msgs::PointStamped point_wrt_world;

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

    group.setMaxVelocityScalingFactor(1);
    group.setMaxAccelerationScalingFactor(1);

    check_srv.request.check.data = true;




//    while (ros::ok())

//    {

//        signal(SIGINT, signal_callback_handler);

////        if(protective_clear_srv.call(check_srv))
////        {
////             std::cout  << "\n flag value = "<< check_srv.response.flag_protective_stopped.data
////                          << "\n success value = "<< check_srv.response.success.data <<"\n\n";
////        }
////        else
////            std::cout << "\nGHANTA...\n";

////        std::cin >> a;

//        if(step == 1)
//        {
//            std::cout << "\n\n*************** im in step 1 ***************\n\n";

//            usleep(1000000);

//            T = 2.0;
//            t_b1 = 0.2;


//            for(int i = 0; i<6; i++)
//                joint_angles.at(i) = tot_view_angles[i];

//            double initial_joint_angles[6];
//            for (int i = 0; i<6; i++)
//                initial_joint_angles[i] = group.getCurrentJointValues().at(i);

//            group.setJointValueTarget(joint_angles);

//            group.plan(plan);


//            std::cout << "\n\n\n\n\n-----Run-----\n";
////                        std::cin >> a;




//            group.execute(plan);

//            usleep(1000000);



//            //*******call service for object centroid and object normal

//            //            ros::ServiceClient client = nh.serviceClient<apc_nozel::get_points2>("get_kinect_points_topic");

//            //            apc_nozel::get_points2 srv;

//            //            std::cout<<"\n\ncalling service to get points wrt kinect\n";

//            //            ros::service::waitForService("get_kinect_points_topic");

//            //            srv.request.remaining_objects.data.resize(40);

//            //            for(int i = 0; i<total_number_of_objects; i++)
//            //                srv.request.remaining_objects.data.at(i) = object_IDs[i];


//            //            if (client.call(srv))
//            //                std::cout<<"point_service successful\n";
//            //            else
//            //                std::cout<<"point_service failed\n";

//            //            object_ID = srv.response.object_ID.data;

//            //            object_selected(srv.response.object_ID);

//            //            object_IDs[srv.response.object_ID.data - 1] = 0;

//            //            centroid_point_wrt_kinect.point.x = srv.response.centroid_wrt_kinect.x;
//            //            centroid_point_wrt_kinect.point.y = srv.response.centroid_wrt_kinect.y;
//            //            centroid_point_wrt_kinect.point.z = srv.response.centroid_wrt_kinect.z;

//            object_ID = 2;

//            axis_point_wrt_kinect.point.x = 0;
//            axis_point_wrt_kinect.point.y = 0;
//            axis_point_wrt_kinect.point.z = 0;

//            centroid_point_wrt_kinect.point.x = 0;
//            centroid_point_wrt_kinect.point.y = 0;
//            centroid_point_wrt_kinect.point.z = 0.6;

//            normal_point_wrt_kinect.point.x = 0;
//            normal_point_wrt_kinect.point.y = 0;
//            normal_point_wrt_kinect.point.z = 0.4;

//            centroid_point_wrt_kinect.header.frame_id = "kf";

//            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

//            listener.transformPoint("world", centroid_point_wrt_kinect, centroid_point_wrt_world);

//            //            normal_point_wrt_kinect.point.x = srv.response.normal_wrt_kinect.x;
//            //            normal_point_wrt_kinect.point.y = srv.response.normal_wrt_kinect.y;
//            //            normal_point_wrt_kinect.point.z = srv.response.normal_wrt_kinect.z;

//            normal_point_wrt_kinect.header.frame_id = "kf";

//            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

//            listener.transformPoint("world", normal_point_wrt_kinect, normal_point_wrt_world);


//            //            axis_point_wrt_kinect.point.x = srv.response.axis_wrt_kinect.x;
//            //            axis_point_wrt_kinect.point.y = srv.response.axis_wrt_kinect.y;
//            //            axis_point_wrt_kinect.point.z = srv.response.axis_wrt_kinect.z;



//            axis_point_wrt_kinect.header.frame_id = "kf";

//            listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

//            listener.transformPoint("world", axis_point_wrt_kinect, axis_point_wrt_world);


//            visualization_msgs::Marker line_list;

//            line_list.header.frame_id = "/world";
//            line_list.header.stamp = ros::Time::now();
//            line_list.ns = "my_namespace";

//            line_list.id = 3;

//            line_list.type = visualization_msgs::Marker::LINE_LIST;

//            line_list.scale.x = 0.01;

//            line_list.color.b = 1.0;
//            line_list.color.g = 1.0;
//            line_list.color.a = 1.0;

//            line_list.points.push_back(centroid_point_wrt_world.point);
//            line_list.points.push_back(normal_point_wrt_world.point);

//            line_list.points.push_back(centroid_point_wrt_world.point);
//            line_list.points.push_back(axis_point_wrt_world.point);

//            marker_pub.publish(line_list);

//            normal_point_wrt_world.point.z = normal_point_wrt_world.point.z + 0.07;


//            if((object_ID == 12)||(object_ID == 17) ||(object_ID == 29))
//            {
//                t_b1 = 0.2;
//                T = 4.0;
//            }

//            if ((object_ID == 28)||(object_ID == 17)||(object_ID == 23)||(object_ID == 1)||(object_ID == 24)||(object_ID == 25)||(object_ID == 14)||(object_ID == 5)||(object_ID == 15))
//            {
//                normal_point_wrt_world.point.x = centroid_point_wrt_world.point.x;
//                normal_point_wrt_world.point.y = centroid_point_wrt_world.point.y;
//                normal_point_wrt_world.point.z = centroid_point_wrt_world.point.z + 0.25;
//            }

//            float n_x = (normal_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
//            float n_y = (normal_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
//            float n_z = (normal_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

//            float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

//            n_x = n_x/m;
//            n_y = n_y/m;
//            n_z = n_z/m;

//            float phi = atan2(-n_y, -n_x);

//            q.setRPY( 0, M_PI/2, 0);
//            q1.setRPY( 0, 0, phi);

//            q4 = q1*q;

//            final_pose.position.x = normal_point_wrt_world.point.x;
//            final_pose.position.y = normal_point_wrt_world.point.y;
//            final_pose.position.z = normal_point_wrt_world.point.z;

//            final_pose.orientation.x = q4.getX();
//            final_pose.orientation.y = q4.getY();
//            final_pose.orientation.z = q4.getZ();
//            final_pose.orientation.w = q4.getW();


//            for (int i = 0; i<3; i++)
//            {
//                visualization_msgs::Marker marker;

//                marker.header.frame_id = "world";
//                marker.header.stamp = ros::Time::now();
//                marker.ns = "my_namespace";
//                marker.id = i;

//                marker.type = visualization_msgs::Marker::SPHERE;
//                marker.action = visualization_msgs::Marker::ADD;

//                marker.pose.orientation.w = 1.0;
//                marker.scale.x = 0.05;
//                marker.scale.y = 0.05;
//                marker.scale.z = 0.05;

//                marker.color.a = 1.0; // Don't forget to set the alpha!

//                if(i == 0)
//                {
//                    marker.color.r = 0.0;
//                    marker.color.g = 0.0;
//                    marker.color.b = 1.0;

//                    marker.pose.position.x = centroid_point_wrt_world.point.x;
//                    marker.pose.position.y = centroid_point_wrt_world.point.y;
//                    marker.pose.position.z = centroid_point_wrt_world.point.z;

//                }

//                if(i == 1)
//                {
//                    marker.color.r = 1.0;
//                    marker.color.g = 0.0;
//                    marker.color.b = 0.0;

//                    marker.pose.position.x = normal_point_wrt_world.point.x;
//                    marker.pose.position.y = normal_point_wrt_world.point.y;
//                    marker.pose.position.z = normal_point_wrt_world.point.z;
//                }

//                if(i == 2)
//                {
//                    marker.color.r = 0.0;
//                    marker.color.g = 1.0;
//                    marker.color.b = 0.0;

//                    marker.pose.position.x = axis_point_wrt_world.point.x;
//                    marker.pose.position.y = axis_point_wrt_world.point.y;
//                    marker.pose.position.z = axis_point_wrt_world.point.z;
//                }

//                marker.lifetime = ros::Duration();

//                markers.markers.push_back(marker);

//            }

//            pub.publish(markers);

//            markers.markers.clear();

//            waypoints.push_back(final_pose);


//            final_xyz.at(0) = final_pose.position.x;
//            final_xyz.at(1) = final_pose.position.y;
//            final_xyz.at(2) = final_pose.position.z;



//        }

//        if(step == 2)
//        {
//            //*******call service for object centroid and object normal wrt suction frame


//            ros::ServiceClient client = nh.serviceClient<apc_nozel::actuator_frame2>("get_suction_points_topic");

//            apc_nozel::actuator_frame2 srv2;

//            std::cout<<"calling service to get points wrt suction2\n";

//            ros::service::waitForService("get_suction_points_topic");

//            srv2.request.centroid_wrt_world = centroid_point_wrt_world.point;
//            srv2.request.normal_wrt_world = normal_point_wrt_world.point;

//            client.call(srv2);

//            //            if (client.call(srv2))
//            //                std::cout<<"suction_point_service successful\n";
//            //            else
//            //                std::cout<<"suction_point_service failed\n";


//            float x = srv2.response.centroid_wrt_suction_point.x;
//            float y = srv2.response.centroid_wrt_suction_point.y;
//            float z = srv2.response.centroid_wrt_suction_point.z;

//            float thetha = atan2(sqrt(x*x + y*y), z);
//            float phi = atan2(y, x);


//            float a_y = -M_PI/2 + thetha;


//            //            std::cout << "nozel need to be rotated by " << a_y*180/M_PI <<" in degrees\n";
//            //            std::cout << "nozel need to be rotated by " << a_y <<" in radians\n";

//            //            std::cout <<"Press any key if you have entered the nozel angle\n";
//            //            std::cin >> a;

//            float r = sqrt(x*x + y*y + z*z);

//            float nozel_length;

//            if((object_ID == 17) || (object_ID == 5))
//                nozel_length = 0.05;
//            else
//                nozel_length = 0.07;

//            final_pos_wrt_suction_point.header.frame_id = "suction2";

//            final_pos_wrt_suction_point.point.x = (r-nozel_length)*cos(phi)*sin(thetha);
//            final_pos_wrt_suction_point.point.y = (r-nozel_length)*sin(phi)*sin(thetha);
//            final_pos_wrt_suction_point.point.z = (r-nozel_length)*cos(thetha);

//            listener.waitForTransform("world", "suction2",  ros::Time(0), ros::Duration(3));

//            listener.transformPoint("world", final_pos_wrt_suction_point, final_pos_wrt_world);

//            final_pose.position.x = final_pos_wrt_world.point.x;
//            final_pose.position.y = final_pos_wrt_world.point.y;
//            final_pose.position.z = final_pos_wrt_world.point.z;

//            current_state = group.getCurrentPose();

//            final_pose.orientation = current_state.pose.orientation;

//            waypoints.push_back(final_pose);


//            final_xyz.at(0) = final_pose.position.x;
//            final_xyz.at(1) = final_pose.position.y;
//            final_xyz.at(2) = final_pose.position.z;

//        }

//        if(step == 3)
//        {

//            current_state = group.getCurrentPose();

//            final_pose.orientation = current_state.pose.orientation;

//            final_pose.position.x = current_state.pose.position.x;
//            final_pose.position.y = current_state.pose.position.y;
//            final_pose.position.z = current_state.pose.position.z + 0.2;

//            waypoints.push_back(final_pose);


//            final_xyz.at(0) = final_pose.position.x;
//            final_xyz.at(1) = final_pose.position.y;
//            final_xyz.at(2) = final_pose.position.z;

//        }

//        if(step == 4)
//        {

//            float x = (axis_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
//            float y = (axis_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
//            float z = (axis_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

//            float m = sqrt(pow(x,2) + pow(y,2) + pow(z,2) );

//            z = z/m;
//            y = y/m;
//            x = x/m;

//            float phi = atan2(y, x);

//            std::cout << "\n axis phi: = " << phi*180/M_PI;

//            if(phi > M_PI/2)
//                phi = phi - M_PI;
//            if(phi < -M_PI/2)
//                phi = phi + M_PI;

//            std::cout << "\n axis phi: = " << phi*180/M_PI <<"\n";

//            point_wrt_world.point.x = 0.472;
//            point_wrt_world.point.y = 0.033;
//            point_wrt_world.point.z = 0.600;

//            for (int i = 0; i<1; i++)
//            {
//                visualization_msgs::Marker marker;

//                marker.header.frame_id = "world";
//                marker.header.stamp = ros::Time::now();
//                marker.ns = "my_namespace";
//                marker.id = i;

//                marker.type = visualization_msgs::Marker::SPHERE;
//                marker.action = visualization_msgs::Marker::ADD;

//                marker.pose.orientation.w = 1.0;
//                marker.scale.x = 0.05;
//                marker.scale.y = 0.05;
//                marker.scale.z = 0.05;

//                marker.color.a = 1.0; // Don't forget to set the alpha!

//                if(i == 0)
//                {
//                    marker.color.r = 0.0;
//                    marker.color.g = 0.0;
//                    marker.color.b = 1.0;

//                    marker.pose.position.x = point_wrt_world.point.x;
//                    marker.pose.position.y = point_wrt_world.point.y;
//                    marker.pose.position.z = point_wrt_world.point.z;

//                }

//                marker.lifetime = ros::Duration();

//                markers.markers.push_back(marker);

//            }

//            pub.publish(markers);

//            markers.markers.clear();


//            final_pose.position.x = point_wrt_world.point.x;
//            final_pose.position.y = point_wrt_world.point.y;
//            final_pose.position.z = point_wrt_world.point.z;

//            current_state = group.getCurrentPose();

//            q1.setX(current_state.pose.orientation.x);
//            q1.setY(current_state.pose.orientation.y);
//            q1.setZ(current_state.pose.orientation.z);
//            q1.setW(current_state.pose.orientation.w);

//            q2.setRPY(0,0,-phi);

//            q4 = q2*q1;

//            final_pose.orientation.x = q4.getX();
//            final_pose.orientation.y = q4.getY();
//            final_pose.orientation.z = q4.getZ();
//            final_pose.orientation.w = q4.getW();

//            waypoints.push_back(final_pose);


//            final_xyz.at(0) = final_pose.position.x;
//            final_xyz.at(1) = final_pose.position.y;
//            final_xyz.at(2) = final_pose.position.z;

//        }

//        if(step == 5)
//        {

//            current_state = group.getCurrentPose();

//            final_pose.orientation = current_state.pose.orientation;

//            final_pose.position.x = current_state.pose.position.x;
//            final_pose.position.y = current_state.pose.position.y;
//            final_pose.position.z = current_state.pose.position.z - 0.2;

//            waypoints.push_back(final_pose);


//            final_xyz.at(0) = final_pose.position.x;
//            final_xyz.at(1) = final_pose.position.y;
//            final_xyz.at(2) = final_pose.position.z;

//        }

//        if(step == 6)
//        {
//            std::cout <<"press any key to Turn Off the suction\n";
//            //            std::cin >> a;

//        }

//        if(step == 7)
//        {

//            current_state = group.getCurrentPose();

//            q.setRPY(0, M_PI/2, 0);
//            q2.setRPY(0, 0, 0*M_PI/180);

//            q3 = q2*q;

//            //            final_pose.orientation.x = q3.getX();
//            //            final_pose.orientation.y = q3.getY();
//            //            final_pose.orientation.z = q3.getZ();
//            //            final_pose.orientation.w = q3.getW();

//            final_pose.orientation = current_state.pose.orientation;

//            final_pose.position.x = current_state.pose.position.x;
//            final_pose.position.y = current_state.pose.position.y;
//            final_pose.position.z = current_state.pose.position.z + 0.15;

//            waypoints.push_back(final_pose);


//            final_xyz.at(0) = final_pose.position.x;
//            final_xyz.at(1) = final_pose.position.y;
//            final_xyz.at(2) = final_pose.position.z;

//        }

//        if(step!=6 && step!=0)
//        {

//            while(1)
//            {

//                usleep(100000);

//                double fraction = group.computeCartesianPath(waypoints,
//                                                             0.01,  // eef_step
//                                                             0.0,   // jump_threshold
//                                                             trajectory);

//                //                std::cout<<"\n---> "<< fraction * 100.0 <<" Path computed \n";

//                //                std::cout << "\nstep = " << step <<"\n\nfeedback = " << feedback << "\n\n";


//                if(fraction > 0.999)
//                {


//                    robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "robo_arm");

//                    rt.setRobotTrajectoryMsg(group.getCurrentState()->getRobotModel(), trajectory);

//                    // Thrid create a IterativeParabolicTimeParameterization object
//                    trajectory_processing::IterativeParabolicTimeParameterization iptp;

//                    // Fourth compute computeTimeStamps
//                    bool success = iptp.computeTimeStamps(rt, 1, 1);
//                    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

//                    // Get RobotTrajectory_msg from RobotTrajectory
//                    rt.getRobotTrajectoryMsg(trajectory);

//                    // Finally plan and execute the trajectory
//                    trajectory_iptp_5_poly = trajectory;
//                    plan.trajectory_ = trajectory;

//                    std::cout << "\n\n\n\n\n-----Run2-----\n";
//                    //                    std::cin >> a;

//                    if(step == 4)
//                    {

//                        int len = plan.trajectory_.joint_trajectory.points.size();
//                        for(int i = 0; i<6; i++)
//                            joint_angles.at(i) = plan.trajectory_.joint_trajectory.points.at(len - 1).positions.at(i);

//                        group.setJointValueTarget(joint_angles);

//                        group.move();
//                    }
//                    else
//                    {
//                        group.execute(plan);
//                    }


//                    rt.clear();

//                    if((feedback == 0)||(step == 1))
//                    {
//                        waypoints.clear();
//                        plan.trajectory_.joint_trajectory.points.clear();
//                        trajectory.joint_trajectory.points.clear();
//                        trajectory_iptp_5_poly.joint_trajectory.points.clear();


//                        std::cout <<"\nplan.trajectory_.joint_trajectory.points.size = " << plan.trajectory_.joint_trajectory.points.size();
//                        std::cout <<"\ntrajectory.joint_trajectory.points.size = " << trajectory.joint_trajectory.points.size();
//                        std::cout <<"\ntrajectory_iptp_5_poly.joint_trajectory.points.size = " << trajectory_iptp_5_poly.joint_trajectory.points.size();

//                        feedback = 0;
//                        break;
//                    }

//                    if((step == 2) && (feedback == 1))
//                    {
//                        waypoints.clear();

//                        tote_reference_point.header.frame_id = "tote_link";

//                        tote_reference_point.point.x = 0;
//                        tote_reference_point.point.y = 0;
//                        tote_reference_point.point.z = 0;

//                        listener.waitForTransform( "world", "tote_link", ros::Time(0), ros::Duration(3));

//                        listener.transformPoint("world", tote_reference_point, tote_wrt_world);

//                        normal_point_wrt_world.point.x = (centroid_point_wrt_world.point.x + tote_wrt_world.point.x)/2;
//                        normal_point_wrt_world.point.y = (centroid_point_wrt_world.point.y + tote_wrt_world.point.y)/2;
//                        normal_point_wrt_world.point.z = centroid_point_wrt_world.point.z + 0.15;

//                        float n_x = (normal_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
//                        float n_y = (normal_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
//                        float n_z = (normal_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

//                        float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

//                        n_x = n_x/m;
//                        n_y = n_y/m;
//                        n_z = n_z/m;

//                        float phi = atan2(-n_y, -n_x);

//                        q.setRPY( 0, M_PI/2, 0);
//                        q1.setRPY( 0, 0, phi);

//                        q4 = q1*q;

//                        final_pose.orientation.x = q4.getX();
//                        final_pose.orientation.y = q4.getY();
//                        final_pose.orientation.z = q4.getZ();
//                        final_pose.orientation.w = q4.getW();

//                        final_pose.position.x = normal_point_wrt_world.point.x;
//                        final_pose.position.y = normal_point_wrt_world.point.y;
//                        final_pose.position.z = normal_point_wrt_world.point.z;

//                        waypoints.push_back(final_pose);


//                        final_xyz.at(0) = final_pose.position.x;
//                        final_xyz.at(1) = final_pose.position.y;
//                        final_xyz.at(2) = final_pose.position.z;


//                        feedback = 0;
//                        step = 1;

//                    }

//                }
//                else
//                {
//                    waypoints.clear();

//                    if( ((step == 2) && (feedback == 0)) || ((step == 1) && (feedback == 0)))
//                    {
//                        current_state = group.getCurrentPose();

//                        final_pose.orientation = current_state.pose.orientation;

//                        if(step == 1)
//                        {
//                            tote_reference_point.header.frame_id = "tote_link";

//                            tote_reference_point.point.x = 0;
//                            tote_reference_point.point.y = 0;
//                            tote_reference_point.point.z = 0;

//                            listener.waitForTransform( "world", "tote_link", ros::Time(0), ros::Duration(3));

//                            listener.transformPoint("world", tote_reference_point, tote_wrt_world);

//                            normal_point_wrt_world.point.x = (centroid_point_wrt_world.point.x + tote_wrt_world.point.x)/2;
//                            normal_point_wrt_world.point.y = (centroid_point_wrt_world.point.y + tote_wrt_world.point.y)/2;
//                            normal_point_wrt_world.point.z = centroid_point_wrt_world.point.z + 0.15;

//                            float n_x = (normal_point_wrt_world.point.x - centroid_point_wrt_world.point.x);
//                            float n_y = (normal_point_wrt_world.point.y - centroid_point_wrt_world.point.y);
//                            float n_z = (normal_point_wrt_world.point.z - centroid_point_wrt_world.point.z);

//                            float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

//                            n_x = n_x/m;
//                            n_y = n_y/m;
//                            n_z = n_z/m;

//                            float phi = atan2(-n_y, -n_x);

//                            q.setRPY( 0, M_PI/2, 0);
//                            q1.setRPY( 0, 0, phi);

//                            q4 = q1*q;

//                            final_pose.orientation.x = q4.getX();
//                            final_pose.orientation.y = q4.getY();
//                            final_pose.orientation.z = q4.getZ();
//                            final_pose.orientation.w = q4.getW();

//                            final_pose.position.x = normal_point_wrt_world.point.x;
//                            final_pose.position.y = normal_point_wrt_world.point.y;
//                            final_pose.position.z = normal_point_wrt_world.point.z;
//                        }
//                        else
//                        {
//                            final_pose.position.x = current_state.pose.position.x;
//                            final_pose.position.y = current_state.pose.position.y;
//                            final_pose.position.z = current_state.pose.position.z + 0.2;
//                        }

//                        waypoints.push_back(final_pose);


//                        final_xyz.at(0) = final_pose.position.x;
//                        final_xyz.at(1) = final_pose.position.y;
//                        final_xyz.at(2) = final_pose.position.z;

//                        feedback = 1;

//                    }
//                    else
//                    {
//                        step = 0;
//                        break;
//                    }

//                }

//            }

//        }

//        //        usleep(100000);

//        step = step + 1;

//        if(step == 8)
//            step = 1;

//    }


    return 0;
}


