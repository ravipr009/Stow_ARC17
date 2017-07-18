#include "support.h"
#define SLOW 0.5
/*
 * 1. Toilet_Brush, 2. Avery_Binder, 3. Balloons, 4. Band_Aid_Tape, 5. Bath_Sponge, 6. Black_Fashion_Gloves, 7. Burts_Bees_Baby_Wipes
   8. Colgate_Toothbrush_4PK, 9. Composition_Book, 10. Crayons, 11. Duct_Tape, 12. Epsom_Salts, 13. Expo_Eraser, 14. Fiskars_Scissors
   15. Flashlight, 16. Glue_Sticks, 17. Hand_Weight, 18. Hanes_Socks, 19. Hinged_Ruled_Index_Cards, 20. Ice_Cube_Tray, 21. Irish_Spring_Soap
   22. Laugh_Out_Loud_Jokes, 23. Marbles, 24. Measuring_Spoons, 25. Mesh_Cup, 26. Mouse_Traps, 27. Pie_Plates, 28. Plastic_Wine_Glass
   29. Poland_Spring_Water, 30. Reynolds_Wrap, 31. Robots_DVD, 32. Robots_Everywhere, 33. Scotch_Sponges, 34. Speed_Stick, 35. White_Facecloth
   36. Table_Cloth, 37. Tennis_Ball_Container, 38. Ticonderoga_Pencils, 39. Tissue_Box, 40.Windex
*/

Stowing_ARC_2017::Stowing_ARC_2017(std::string group_name, std::string group_planner_ID, ros::NodeHandle& nh)
{
    this->group = new moveit::planning_interface::MoveGroup(group_name);
    this->group->setPlannerId(group_planner_ID);

    this->nh = nh;

    this->pub_points = this->nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    this->pub_lines = this->nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    nh.getParam("/tote_view_joint_angles", this->tote_view_joint_angles);
    nh.getParam("/ARC17_OBJECT_ID", this->ID);
    nh.getParam("/ARC17_OBJECT_NAMES", this->name);
    nh.getParam("/ARC17_OBJECT_NORMAL_IGNORE", this->ignore_normal);
    nh.getParam("/ARC17_OBJECT_WEIGHT", this->weight);
    nh.getParam("/ARC17_OBJECT_SIZE", this->size);
    nh.getParam("/ARC17_NO_OF_OBJECT", this->total_no_of_objects);
    nh.getParam("/ARC17_OBJECT_PUSH", this->push_object);
    nh.getParam("/ARC17_R_T_MATRIX", this->matrix_data);

std::cout<<"\n"<<this->total_no_of_objects<<"\n";

    pthread_create(&this->broadcast_ensenso_frame, NULL, reinterpret_cast<void* (*)(void *)>(&Stowing_ARC_2017::broadcast_kf), this);

}




bool Stowing_ARC_2017::read_joson_file()
{
    ros::ServiceClient read_jason_file_srv = this->nh.serviceClient<apc_nozel::stowToteContents>("/tote_contents/data");
    apc_nozel::stowToteContents tote_contents;

    if(read_jason_file_srv.call(tote_contents))
    {
        std::cout <<"jason file loaded";
        return true;
    }
    else
    {
        std::cout <<"unable to load jason file";
        return false;
    }

}



bool Stowing_ARC_2017::write_jason_file(int bin_number)
{
    ros::ServiceClient write_jason_file_srv = this->nh.serviceClient<apc_nozel::write_stow_data>("/write_stow_data_service");
    apc_nozel::write_stow_data write_stow_data;

    write_stow_data.request.bin_id.data = bin_number;
    write_stow_data.request.obj_id.data = this->object_ID;

    if(write_jason_file_srv.call(write_stow_data))
    {
        std::cout <<"\nObject " << this->name[this->object_ID - 1] << "is placed in bin number " << bin_number;
        return true;
    }
    else
        return false;
}





bool Stowing_ARC_2017::move_to_tote()
{
    //    std::cout <<"im in move_to_tote\n";

    this->group->setMaxAccelerationScalingFactor(1);
    this->group->setMaxVelocityScalingFactor(1);

    this->group->setJointValueTarget(this->tote_view_joint_angles);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    bool success = this->group->plan(my_plan);

    if(success)
        this->group->execute(my_plan);

    return verify_joint_goal_position(this->tote_view_joint_angles);




}

bool Stowing_ARC_2017::call_vision_service_bajrangi()
{
    ros::ServiceClient client = nh.serviceClient<stow_class::computer_vision_stowing>("/iitktcs/computer_vision_stowing");

}


bool Stowing_ARC_2017::call_vision_service()
{

    //    std::cout <<"im in call_vision_service\n";

    ros::ServiceClient client = nh.serviceClient<apc_nozel::get_points2>("get_kinect_points_topic");

    apc_nozel::get_points2 srv;

    std::cout<<"\n\ncalling service to get points wrt kinect\n";

    ros::service::waitForService("get_kinect_points_topic");


//    char a;
//    std::cin>>a;
    srv.request.remaining_objects.data.resize(this->total_no_of_objects);

    for(int i = 0; i<this->total_no_of_objects; i++)
    {
        srv.request.remaining_objects.data.at(i) = 1;
        if( i == object_ID - 1)
            srv.request.remaining_objects.data.at(i) = 0;
    }

    if (!client.call(srv))
        return false;

    this->object_ID = srv.response.object_ID.data;

    this->centroid_point_wrt_kinect.point.x = srv.response.centroid_wrt_kinect.x;
    this->centroid_point_wrt_kinect.point.y = srv.response.centroid_wrt_kinect.y;
    this->centroid_point_wrt_kinect.point.z = srv.response.centroid_wrt_kinect.z;

    this->centroid_point_wrt_kinect.header.frame_id = "kf";

    this->tf_listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

    this->tf_listener.transformPoint("world", this->centroid_point_wrt_kinect, this->centroid_point_wrt_world);

    this->normal_point_wrt_kinect.point.x = srv.response.normal_wrt_kinect.x;
    this->normal_point_wrt_kinect.point.y = srv.response.normal_wrt_kinect.y;
    this->normal_point_wrt_kinect.point.z = srv.response.normal_wrt_kinect.z;

    this->normal_point_wrt_kinect.header.frame_id = "kf";

    this->tf_listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

    this->tf_listener.transformPoint("world", this->normal_point_wrt_kinect, this->normal_point_wrt_world);


    this->axis_point_wrt_kinect.point.x = srv.response.axis_wrt_kinect.x;
    this->axis_point_wrt_kinect.point.y = srv.response.axis_wrt_kinect.y;
    this->axis_point_wrt_kinect.point.z = srv.response.axis_wrt_kinect.z;

    this->axis_point_wrt_kinect.header.frame_id = "kf";

    this->tf_listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

    this->tf_listener.transformPoint("world", this->axis_point_wrt_kinect, this->axis_point_wrt_world);


    std::cout << "ID: "<<this->ID[this->object_ID - 1] << "\n" <<
                 "NAME: " << this->name[this->object_ID - 1] << "\n" <<
                 "NORMAL_IGNORE: "<< this->ignore_normal[this->object_ID - 1] << "\n" <<
                 "WEIGHT: " << this->weight[this->object_ID - 1] << "\n" <<
                 "SIZE:  " << this->size[this->object_ID - 1] << "\n";


    return true;

}


bool Stowing_ARC_2017::visualize_points()
{

    //    std::cout <<"im in visualize_points\n";

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

    line_list.points.push_back(this->centroid_point_wrt_world.point);
    line_list.points.push_back(this->normal_point_wrt_world.point);

    line_list.points.push_back(this->centroid_point_wrt_world.point);
    line_list.points.push_back(this->axis_point_wrt_world.point);

    this->pub_lines.publish(line_list);




    if ( this->ignore_normal[this->object_ID - 1] )
    {
        std::cout <<"Normal Changed\n";

        this->normal_point_wrt_world.point.x = this->centroid_point_wrt_world.point.x;
        this->normal_point_wrt_world.point.y = this->centroid_point_wrt_world.point.y;
        this->normal_point_wrt_world.point.z = this->centroid_point_wrt_world.point.z + 0.25;
    }

    visualization_msgs::MarkerArray markers;

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


    this->pub_points.publish(markers);
    markers.markers.clear();

    return true;

}


bool Stowing_ARC_2017::reach_normal()
{

    //    std::cout <<"im in reach_normal\n";

    geometry_msgs::Pose final_pose = this->normal_point_with_orientation();

    return linear_movement(final_pose, 1.0);
}


bool Stowing_ARC_2017::reach_centroid()
{

    //    std::cout <<"im in reach_centroid\n";

    geometry_msgs::Pose goal_pose = this->touch_centroid_point();

    float speed_factor = 0.5;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    if(async_linear_movement(goal_pose, speed_factor))
        return true;
    else
        return false;


}


bool Stowing_ARC_2017::lift_object()
{

    //    std::cout <<"im in lift_object\n";

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    float lift;

    if(this->size[this->object_ID - 1] == "large")
        lift = 0.5;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = 0.40;
        else
            lift = 0.30;
    }

    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = current_pose.pose.position.x;
    goal_pose.position.y = current_pose.pose.position.y;
    goal_pose.position.z = current_pose.pose.position.z + lift;


    float speed_factor;

    goal_pose.orientation = current_pose.pose.orientation;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    return linear_movement(goal_pose, speed_factor);

}


bool Stowing_ARC_2017::reach_bin(double location[0])
{

    //    std::cout <<"im in reach_bin\n";

    geometry_msgs::Pose goal_pose;

    tf::Quaternion q4, q1, q2;

    float x = (this->axis_point_wrt_world.point.x - this->centroid_point_wrt_world.point.x);
    float y = (this->axis_point_wrt_world.point.y - this->centroid_point_wrt_world.point.y);
    float z = (this->axis_point_wrt_world.point.z - this->centroid_point_wrt_world.point.z);

    float m = sqrt(pow(x,2) + pow(y,2) + pow(z,2) );

    z = z/m;
    y = y/m;
    x = x/m;

    float phi = atan2(y, x);

    if(phi > M_PI/2)
        phi = phi - M_PI;
    if(phi < -M_PI/2)
        phi = phi + M_PI;


    geometry_msgs::PointStamped point_wrt_rack;

    geometry_msgs::PointStamped point_wrt_world;

    float lift = 0.4;

    if(this->size[this->object_ID - 1] == "large")
        lift = 0.6;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = 0.45;
        else
            lift = 0.35;
    }

    point_wrt_rack.point.x = location[0];
    point_wrt_rack.point.y = location[1];
    point_wrt_rack.point.z = location[2] + lift;


    point_wrt_rack.header.frame_id = "rack_link";

    this->tf_listener.waitForTransform( "world", "rack_link", ros::Time(0), ros::Duration(3));

    this->tf_listener.transformPoint("world", point_wrt_rack, point_wrt_world);

    visualization_msgs::MarkerArray markers;

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


    this->pub_points.publish(markers);
    markers.markers.clear();


    goal_pose.position.x = point_wrt_world.point.x;
    goal_pose.position.y = point_wrt_world.point.y;
    goal_pose.position.z = point_wrt_world.point.z;

    geometry_msgs ::PoseStamped current_state = this->group->getCurrentPose();

    q1.setX(current_state.pose.orientation.x);
    q1.setY(current_state.pose.orientation.y);
    q1.setZ(current_state.pose.orientation.z);
    q1.setW(current_state.pose.orientation.w);

    q2.setRPY(0,0,0);

    q4 = q2*q1;

    goal_pose.orientation.x = q4.getX();
    goal_pose.orientation.y = q4.getY();
    goal_pose.orientation.z = q4.getZ();
    goal_pose.orientation.w = q4.getW();

    std::vector<double> final_joint_angles = final_joint_angle(goal_pose);
    float speed_factor;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    return joint_angle_movement(final_joint_angles, speed_factor);
    //    return linear_movement(goal_pose, speed_factor);
}


bool Stowing_ARC_2017::drop_object()
{

    //    std::cout <<"im in drop object\n";

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    float lift;

    if(this->size[this->object_ID - 1] == "large")
        lift = 0.5;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = 0.4;
        else
            lift = 0.3;
    }

    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = current_pose.pose.position.x;
    goal_pose.position.y = current_pose.pose.position.y;
    goal_pose.position.z = current_pose.pose.position.z - 0.2;


    float speed_factor = 0.5;

    goal_pose.orientation = current_pose.pose.orientation;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    return linear_movement(goal_pose, speed_factor);

}


bool Stowing_ARC_2017::move_upwards()
{

    //    std::cout <<"im in move_upwards\n";

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = current_pose.pose.position.x;
    goal_pose.position.y = current_pose.pose.position.y;
    goal_pose.position.z = current_pose.pose.position.z + 0.2;

    goal_pose.orientation = current_pose.pose.orientation;

    return linear_movement(goal_pose, 1);

}


std::vector<double> Stowing_ARC_2017::final_joint_angle(geometry_msgs::Pose pose_goal)
{

    //    std::cout <<"im in final_joint_angle\n";

    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(pose_goal);

    double fraction = this->group->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);



    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

    rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool success = iptp.computeTimeStamps(rt);

    rt.getRobotTrajectoryMsg(trajectory);

    my_plan.trajectory_ = trajectory;

    int len = trajectory.joint_trajectory.points.size();

    return trajectory.joint_trajectory.points.at(len - 1).positions;


}


bool Stowing_ARC_2017::joint_angle_movement(std::vector<double> joint_angles, float speed_factor)
{

    //    std::cout <<"im in joint_angle_movement\n";

    this->group->setJointValueTarget(joint_angles);

    this->group->setMaxAccelerationScalingFactor(speed_factor);
    this->group->setMaxVelocityScalingFactor(speed_factor);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    bool success = this->group->plan(my_plan);

    if(success)
        this->group->execute(my_plan);
    else
        return false;


    return(verify_joint_goal_position(joint_angles));


}


void* Stowing_ARC_2017::broadcast_kf(void * threadid)
{

    std::fstream myfile;

    char c;
    float a;

    tf::TransformBroadcaster br;

    tf::StampedTransform transform_ensenso_frame;

    ros::Rate rate(10.0);

    Eigen::MatrixXf transformation_matrix(4, 4);

    for(unsigned int i=0; i<4;i++)
        for(unsigned int j=0;j<4;j++)
            transformation_matrix(i,j) = this->matrix_data.at(4*i+j);


    while(true)
    {

        Eigen::Matrix4f e_T_ee;
        Eigen::Affine3d affine_e_T_ee;


        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
            {
                e_T_ee(i,j) = transformation_matrix(i,j);
                affine_e_T_ee.matrix()(i,j) = transformation_matrix(i,j);
            }
        geometry_msgs::Pose ensenso_frame;
        tf::poseEigenToMsg(affine_e_T_ee, ensenso_frame);

        transform_ensenso_frame.setOrigin(tf::Vector3(ensenso_frame.position.x,
                                                      ensenso_frame.position.y,
                                                      ensenso_frame.position.z));
        transform_ensenso_frame.setRotation(tf::Quaternion(ensenso_frame.orientation.x,
                                                           ensenso_frame.orientation.y,
                                                           ensenso_frame.orientation.z,
                                                           ensenso_frame.orientation.w));


        br.sendTransform(tf::StampedTransform(transform_ensenso_frame, ros::Time::now(), "ee_link", "kf"));

        rate.sleep();
    }

}


bool Stowing_ARC_2017::verify_linear_position(geometry_msgs::Pose pose_goal)
{

    //    std::cout <<"im in verify_linear_position\n";

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    float linear_threshold = 0.005;

    float linear_error =  pow((current_pose.pose.position.x - pose_goal.position.x), 2) +
            pow((current_pose.pose.position.y - pose_goal.position.y), 2) +
            pow((current_pose.pose.position.z - pose_goal.position.z), 2);

    if(sqrt(linear_error) < linear_threshold)
        return true;
    else
        return false;

    //    return true;
}


bool Stowing_ARC_2017::verify_joint_goal_position(std::vector<double> &goal)
{

    //    std::cout <<"im in verify_joint_goal_position\n";

    std::vector<double> current_joint_angles = this->group->getCurrentJointValues();

    float joint_threshold = 0.01;

    float joint_error = 0;

    for(int i = 0; i<6; i++)
        joint_error = joint_error + pow((current_joint_angles.at(i) - goal.at(i)), 2);

    if(sqrt(joint_error) < joint_threshold)
        return true;
    else
        return false;
}


void Stowing_ARC_2017::modify_normal()
{

    //    std::cout <<"im in modify_normal\n";

    tote_reference_point.header.frame_id = "tote_link";

    tote_reference_point.point.x = 0;
    tote_reference_point.point.y = 0;
    tote_reference_point.point.z = 0;

    this->tf_listener.waitForTransform( "world", "tote_link", ros::Time(0), ros::Duration(3));

    this->tf_listener.transformPoint("world", tote_reference_point, tote_wrt_world);

    this->normal_point_wrt_world.point.x = (this->centroid_point_wrt_world.point.x + tote_wrt_world.point.x)/2;
    this->normal_point_wrt_world.point.y = (this->centroid_point_wrt_world.point.y + tote_wrt_world.point.y)/2;
    this->normal_point_wrt_world.point.z = this->centroid_point_wrt_world.point.z + 0.15;
}


bool Stowing_ARC_2017::linear_movement(geometry_msgs::Pose pose_goal, float speed_factor)
{

    //    std::cout <<"im in linear_movement\n";

    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(pose_goal);

    double fraction = this->group->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);

    std::cout << "\n Path Computed: " << 100*fraction <<"\n";

    if(fraction > 0.95)
    {


        robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

        rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        iptp.computeTimeStamps(rt, speed_factor, speed_factor);

        rt.getRobotTrajectoryMsg(trajectory);

        my_plan.trajectory_ = trajectory;

        this->group->execute(my_plan);

        this->write_data_in_file(my_plan.trajectory_.joint_trajectory.points);

        return verify_linear_position(pose_goal);
    }
    else
    {
        std :: cout << "linear motion not found\n";
        return false;
    }

}


bool Stowing_ARC_2017::async_linear_movement(geometry_msgs::Pose pose_goal, float speed_factor)
{

    //    std::cout <<"im in async_linear_movement\n";

    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(pose_goal);

    double fraction = this->group->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);
    if(fraction > 0.95)
    {


        robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

        rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;


        iptp.computeTimeStamps(rt, speed_factor, speed_factor);

        rt.getRobotTrajectoryMsg(trajectory);

        my_plan.trajectory_ = trajectory;

        this->write_data_in_file(my_plan.trajectory_.joint_trajectory.points);

        this->group->asyncExecute(my_plan);

        float dis_error = 0;
        float dis_threshold = 0.005;

        while(1)
        {


            geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

            dis_error = pow((current_pose.pose.position.x - pose_goal.position.x), 2) +
                    pow((current_pose.pose.position.y - pose_goal.position.y), 2) +
                    pow((current_pose.pose.position.z - pose_goal.position.z), 2);

            std::cout <<"im in while loop, error: " << sqrt(dis_error) << "\n";

            if(sqrt(dis_error) < dis_threshold)
            {
                this->group->stop();
                break;
            }

        }

        std::cout <<"\nreached_desire_position\n";


        return verify_linear_position(pose_goal);
    }
    else
        return false;

}


bool Stowing_ARC_2017::plan_to_reach_centroid_via_normal()
{

    //    std::cout <<"im in plan_to_reach_centroid_via_normal\n";

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose normal_pose = this->normal_point_with_orientation();

    waypoints.push_back(normal_pose);

    geometry_msgs::Pose centroid_pose = this->touch_centroid_point();

    waypoints.push_back(centroid_pose);

    moveit_msgs::RobotTrajectory trajectory;

    double fraction = this->group->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);


    if(fraction > 0.999)
        return true;

    else
    {
        std :: cout << "complete_plan not found\n";
        return false;
    }

}


geometry_msgs::Pose Stowing_ARC_2017::normal_point_with_orientation()
{

    //    std::cout <<"im in normal_point_with_orientation\n";

    tf::Quaternion q, q1, q2;

    geometry_msgs::Pose final_pose;


    float n_x = (this->normal_point_wrt_world.point.x - this->centroid_point_wrt_world.point.x);
    float n_y = (this->normal_point_wrt_world.point.y - this->centroid_point_wrt_world.point.y);
    float n_z = (this->normal_point_wrt_world.point.z - this->centroid_point_wrt_world.point.z);

    float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

    n_x = n_x/m;
    n_y = n_y/m;
    n_z = n_z/m;

    float phi = atan2(-n_y, -n_x);

    q.setRPY( 0, M_PI/2, 0);
    q1.setRPY( 0, 0, phi);

    q2 = q1*q;

    final_pose.position.x = normal_point_wrt_world.point.x;
    final_pose.position.y = normal_point_wrt_world.point.y;
    final_pose.position.z = normal_point_wrt_world.point.z;

    final_pose.orientation.x = q2.getX();
    final_pose.orientation.y = q2.getY();
    final_pose.orientation.z = q2.getZ();
    final_pose.orientation.w = q2.getW();

    return final_pose;
}


geometry_msgs::Pose Stowing_ARC_2017::touch_centroid_point()
{

    //    std::cout <<"im in touch_centroid_point\n";

    tf::Quaternion q, q1, q2;


    float n_x = (this->normal_point_wrt_world.point.x - this->centroid_point_wrt_world.point.x);
    float n_y = (this->normal_point_wrt_world.point.y - this->centroid_point_wrt_world.point.y);
    float n_z = (this->normal_point_wrt_world.point.z - this->centroid_point_wrt_world.point.z);

    float m = sqrt(pow(n_x,2) + pow(n_y,2) + pow(n_z,2));

    n_x = n_x/m;
    n_y = n_y/m;
    n_z = n_z/m;

    double phi = atan2(-n_y, -n_x);

    q.setRPY( 0, M_PI/2, 0);
    q1.setRPY( 0, 0, phi);

    q2 = q1*q;


    geometry_msgs::Pose goal_pose;

    std::vector<double> angles = this->nozzel_angle_calculation();

    double nozzle_angle = angles.at(0);
    phi = angles.at(1);
    float nozel_length ;
    if(this->push_object[this->object_ID -1]== 2)
        nozel_length =  0.05;
    else
        if(this->push_object[this->object_ID -1]== 1)
        nozel_length =  0.0635;
    else
            nozel_length =  0.07;

    if(this->object_ID == 28)
        nozel_length=  0.07+0.09;



    std::cout <<"nozzle_angle = " << 180/M_PI*nozzle_angle <<"\n\n";


    goal_pose.position.x = this->centroid_point_wrt_world.point.x +  nozel_length*cos(phi)*sin(nozzle_angle);
    goal_pose.position.y = this->centroid_point_wrt_world.point.y +  nozel_length*sin(phi)*sin(nozzle_angle);
    goal_pose.position.z = this->centroid_point_wrt_world.point.z +  nozel_length*cos(nozzle_angle);


    goal_pose.orientation.x = q2.getX();
    goal_pose.orientation.y = q2.getY();
    goal_pose.orientation.z = q2.getZ();
    goal_pose.orientation.w = q2.getW();

    return goal_pose;

}


bool Stowing_ARC_2017::move_towards_centroid_via_normal()
{

    //    std::cout <<"im in move_towards_centroid_via_normal\n";

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose normal_pose = this->normal_point_with_orientation();

    waypoints.push_back(normal_pose);

    geometry_msgs::Pose centroid_pose = this->touch_centroid_point();

    waypoints.push_back(centroid_pose);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    double fraction = this->group->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);

    float speed_factor;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;


    if(fraction > 0.999)
    {

        robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

        rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        iptp.computeTimeStamps(rt, speed_factor, speed_factor);

        rt.getRobotTrajectoryMsg(trajectory);

        my_plan.trajectory_ = trajectory;

        this->group->execute(my_plan);

        return true;

    }
    else
        return false;


}


std::vector<double> Stowing_ARC_2017::nozzel_angle_calculation()
{

    //    std::cout <<"im in nozzel_angle_calculation\n";

    std::vector<double> angles;

    float x = this->normal_point_wrt_world.point.x - this->centroid_point_wrt_world.point.x;
    float y = this->normal_point_wrt_world.point.y - this->centroid_point_wrt_world.point.y;
    float z = this->normal_point_wrt_world.point.z - this->centroid_point_wrt_world.point.z;

    float m = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

    x = x/m;
    y = y/m;
    z = z/m;

    angles.push_back(acos(z));
    angles.push_back(atan2(y, x));

    return angles;

}


void Stowing_ARC_2017::write_data_in_file(std::vector <trajectory_msgs::JointTrajectoryPoint> vector_points)
{

    std::fstream myfile;

    int k = 1;

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
