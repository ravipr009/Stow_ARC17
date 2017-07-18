#include "stowing.h"

/*
 * 1. Toilet_Brush, 2. Avery_Binder, 3. Balloons, 4. Band_Aid_Tape, 5. Bath_Sponge, 6. Black_Fashion_Gloves, 7. Burts_Bees_Baby_Wipes
   8. Colgate_Toothbrush_4PK, 9. Composition_Book, 10. Crayons, 11. Duct_Tape, 12. Epsom_Salts, 13. Expo_Eraser, 14. Fiskars_Scissors
   15. Flashlight, 16. Glue_Sticks, 17. Hand_Weight, 18. Hanes_Socks, 19. Hinged_Ruled_Index_Cards, 20. Ice_Cube_Tray, 21. Irish_Spring_Soap
   22. Laugh_Out_Loud_Jokes, 23. Marbles, 24. Measuring_Spoons, 25. Mesh_Cup, 26. Mouse_Traps, 27. Pie_Plates, 28. Plastic_Wine_Glass
   29. Poland_Spring_Water, 30. Reynolds_Wrap, 31. Robots_DVD, 32. Robots_Everywhere, 33. Scotch_Sponges, 34. Speed_Stick, 35. White_Facecloth
   36. Table_Cloth, 37. Tennis_Ball_Container, 38. Ticonderoga_Pencils, 39. Tissue_Box, 40.Windex
*/


#define VACUUM_SUCTION_ON 1
#define VACUUM_SUCTION_OFF 0

#define SLOW 0.1
#define LIFT_SMALL_OBJECT 0.35
#define LIFT_MEDIUM_OBJECT 0.45
#define LIFT_BIG_OBJECT 0.55
#define NORMAL_CHANGE_LENGTH 0.3
#define MODIFY_NORMAL_CHANGE_LENGTH 0.25




Stowing_ARC_2017::Stowing_ARC_2017(std::string group_name, std::string group_planner_ID, ros::NodeHandle& nh)
{
    this->group = new moveit::planning_interface::MoveGroup(group_name);
    this->group->setPlannerId(group_planner_ID);

    this->nh = nh;

    this->pub_points = this->nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    this->pub_lines = this->nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //    nh.getParam("/tote_view_joint_angles", this->tote_view_joint_angles);
    //    nh.getParam("/Object_ID", this->ID);
    //    nh.getParam("/Object_name", this->name);
    //    nh.getParam("/Object_normal_ignore", this->ignore_normal);
    //    nh.getParam("/Object_weight", this->weight);
    //    nh.getParam("/Object_size", this->size);

    nh.getParam("/tote_view_joint_angles", this->tote_view_joint_angles);

    nh.getParam("/ARC17_OBJECT_ID", this->ID);

    nh.getParam("/ARC17_OBJECT_NAMES", this->name);

    nh.getParam("/ARC17_OBJECT_NORMAL_IGNORE", this->ignore_normal);

    nh.getParam("/ARC17_OBJECT_WEIGHT", this->weight);

    nh.getParam("/ARC17_OBJECT_SIZE", this->size);

    nh.getParam("/ARC17_NO_OF_OBJECT", this->total_no_of_objects);

    nh.getParam("/ARC17_OBJECT_PUSH", this->push_object);


    pthread_create(&this->broadcast_ensenso_frame, NULL, reinterpret_cast<void* (*)(void *)>(&Stowing_ARC_2017::broadcast_kf), this);

    pthread_create(&this->rotate_nozzle_thread, NULL, reinterpret_cast<void* (*)(void *)>(&Stowing_ARC_2017::rotate_nozzle), this);

    this->nozzel_angle = 0;

}


bool Stowing_ARC_2017::move_to_tote()
{
    //    std::cout <<"im in move_to_tote\n";


    this->group->setMaxAccelerationScalingFactor(1);
    this->group->setMaxVelocityScalingFactor(1);

    this->group->setStartStateToCurrentState();

    usleep(100000);

    this->group->setJointValueTarget(this->tote_view_joint_angles);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    bool success = this->group->plan(my_plan);

    if(success)
        this->group->execute(my_plan);

    return verify_joint_goal_position(this->tote_view_joint_angles);

}


bool Stowing_ARC_2017::call_vision_service()
{

    //    std::cout <<"im in call_vision_service\n";

    ros::ServiceClient client = nh.serviceClient<apc_nozel::get_points2>("get_kinect_points_topic");

    apc_nozel::get_points2 srv;

    std::cout<<"\n\ncalling service to get points wrt kinect\n";

    ros::service::waitForService("get_kinect_points_topic");


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
        this->normal_point_wrt_world.point.z = this->centroid_point_wrt_world.point.z + NORMAL_CHANGE_LENGTH;
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

    std::vector<double> angles = this->nozzel_angle_calculation();

    this->nozzel_angle = angles[0];


    return true;

}


bool Stowing_ARC_2017::reach_normal()
{

    //    std::cout <<"im in reach_normal\n";

    return this->joint_angle_movement(this->final_joint_angle(this->normal_point_with_orientation()), 1);
}


bool Stowing_ARC_2017::reach_centroid()
{

    //    std::cout <<"im in reach_centroid\n";

    geometry_msgs::Pose goal_pose = this->touch_centroid_point();

    float speed_factor;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;


    //    std::vector<double> angles = this->nozzel_angle_calculation();

    //    double nozzle_angle = angles.at(0);

    //    ros::ServiceClient gripper_suction_controller_srv ;

    //    unsigned char a_y_digital = nozzle_angle*(180/M_PI)*(255/90);

    //    this->vacuum_suction_on_off(gripper_suction_controller_srv, true, a_y_digital, VACUUM_SUCTION_ON, VACUUM_SUCTION_OFF);


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
        lift = LIFT_BIG_OBJECT;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = LIFT_MEDIUM_OBJECT;
        else
            lift = LIFT_SMALL_OBJECT;
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

    return this->linear_movement(goal_pose, speed_factor);

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

    float lift;

    if(this->size[this->object_ID - 1] == "large")
        lift = LIFT_BIG_OBJECT;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = LIFT_MEDIUM_OBJECT;
        else
            lift = LIFT_SMALL_OBJECT;
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

    q2.setRPY(0,0,-phi);

    q4 = q2*q1;

    goal_pose.orientation.x = q4.getX();
    goal_pose.orientation.y = q4.getY();
    goal_pose.orientation.z = q4.getZ();
    goal_pose.orientation.w = q4.getW();

    this->bin_pose = goal_pose;

    std::vector<double> final_angles = this->final_joint_angle(this->bin_pose);

    float speed_factor;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    return this->joint_angle_movement(final_angles, speed_factor);


}


bool Stowing_ARC_2017::drop_object()
{

    //    std::cout <<"im in drop object\n";

    this->nozzel_angle = 0;


    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    float lift;

    if(this->size[this->object_ID - 1] == "large")
        lift = LIFT_BIG_OBJECT;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = LIFT_MEDIUM_OBJECT;
        else
            lift = LIFT_SMALL_OBJECT;
    }

    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = current_pose.pose.position.x;
    goal_pose.position.y = current_pose.pose.position.y;
    goal_pose.position.z = current_pose.pose.position.z - lift;


    float speed_factor = 1.0;

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

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);



    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

    rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt);

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
        return this->group->execute(my_plan);

    else
        return false;



}


bool Stowing_ARC_2017::verify_linear_position(geometry_msgs::Pose pose_goal)
{

    std::cout <<"im in verify_linear_position\n";

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    float linear_threshold = 0.01;

    float linear_error =  pow((current_pose.pose.position.x - pose_goal.position.x), 2) +
            pow((current_pose.pose.position.y - pose_goal.position.y), 2) +
            pow((current_pose.pose.position.z - pose_goal.position.z), 2);

    if(sqrt(linear_error) < linear_threshold)
        return true;
    else
        return false;

}


bool Stowing_ARC_2017::verify_joint_goal_position(std::vector<double> &goal)
{

    //    std::cout <<"im in verify_joint_goal_position\n";

    std::vector<double> current_joint_angles = this->group->getCurrentJointValues();

    float joint_threshold = 0.005;

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

    std::cout <<"im in modify_normal\n";

    tote_reference_point.header.frame_id = "tote_link";

    tote_reference_point.point.x = 0;
    tote_reference_point.point.y = 0;
    tote_reference_point.point.z = 0;

    this->tf_listener.waitForTransform( "world", "tote_link", ros::Time(0), ros::Duration(3));

    this->tf_listener.transformPoint("world", tote_reference_point, tote_wrt_world);

    this->normal_point_wrt_world.point.x = (this->centroid_point_wrt_world.point.x + tote_wrt_world.point.x)/2;
    this->normal_point_wrt_world.point.y = (this->centroid_point_wrt_world.point.y + tote_wrt_world.point.y)/2;
    this->normal_point_wrt_world.point.z = this->centroid_point_wrt_world.point.z + MODIFY_NORMAL_CHANGE_LENGTH;


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
}


bool Stowing_ARC_2017::linear_movement(geometry_msgs::Pose pose_goal, float speed_factor)
{

    std::cout <<"im in linear_movement\n";

    this->group->setStartStateToCurrentState();

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

        return this->group->execute(my_plan);


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

    this->group->setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(pose_goal);

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);



    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

    rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, speed_factor, speed_factor);

    rt.getRobotTrajectoryMsg(trajectory);

    my_plan.trajectory_ = trajectory;

    this->group->asyncExecute(my_plan);



    float dis_error = 0;
    float dis_threshold = 0.005;

    float force_error = 0;
    float force_threshold = 50;

    while(1)
    {


        geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();


        dis_error = pow((current_pose.pose.position.x - pose_goal.position.x), 2) +
                pow((current_pose.pose.position.y - pose_goal.position.y), 2) +
                pow((current_pose.pose.position.z - pose_goal.position.z), 2);

        if(sqrt(dis_error) < dis_threshold)
        {
            this->group->stop();
            std::cout <<"\nreached_desire_position\n";
            return verify_linear_position(pose_goal);
        }



        //        force_error = pow(this->force_x, 2) +
        //                pow(this->force_y, 2) +
        //                pow(this->force_z, 2);


        //        if(sqrt(force_error) > force_threshold)
        //        {
        //            this->group->stop();
        //            std::cout <<"\nexperiencing_more_force\n";
        //            return true;
        //        }

    }





}



bool Stowing_ARC_2017::plan_to_reach_centroid_via_normal()
{

    //    std::cout <<"im in plan_to_reach_centroid_via_normal\n";

    std::vector<double> angles = this->nozzel_angle_calculation();

    this->nozzel_angle = angles[0];

    robot_state::RobotStatePtr reference_state;

    reference_state = this->group->getCurrentState();

    double* initial_angle = new double();


    *initial_angle = M_PI/2 - angles[0];
    reference_state->setJointPositions("suction_base_joint", initial_angle);



    this->group->setStartState(*reference_state);


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

    char a;
    std::cin >> a;


    if(fraction > 0.999)
    {
        std::vector<double> angles = this->nozzel_angle_calculation();

        this->nozzel_angle = angles[0];

        return true;
    }

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

    float nozel_length;

    if(this->push_object[this->object_ID - 1] == 2)
        nozel_length = 0.05;
    else
    {
        if(this->push_object[this->object_ID - 1] == 1)
            nozel_length = 0.0635;
        else
            nozel_length = 0.07;
    }


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

    moveit_msgs::RobotTrajectory trajectory;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);



    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");


    rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt);

    rt.getRobotTrajectoryMsg(trajectory);

    my_plan.trajectory_ = trajectory;

    return this->group->execute(my_plan);

}




std::vector<double> Stowing_ARC_2017::nozzel_angle_calculation()
{

    //    std::cout <<"\nim in nozzel_angle_calculation\n";

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




std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::combine_trajectory( float speed_factor)
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_1;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_2;

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_3;

    std::vector<geometry_msgs::Pose> waypoints;

    moveit_msgs::RobotTrajectory trajectory1;

    waypoints.push_back(this->normal_point_with_orientation());

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory1);



    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

    rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory1);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, 1, 1);

    rt.getRobotTrajectoryMsg(trajectory1);

    trajectory_1 = trajectory1.joint_trajectory.points;

    char a;

    int len = trajectory_1.size();

    trajectory_2 = compute_linear_trajectory(trajectory_1.at(len - 1), this->touch_centroid_point(), speed_factor);

    trajectory_1.clear();



    double vel[] = {0,0,0,0,0,0};

    std::vector<double> initial_velocity;
    std::vector<double> initial_acc;

    initial_velocity.resize(6);
    initial_acc.resize(6);

    for(int i = 0; i<6; i++)
    {
        initial_velocity.at(i) = vel[i];
        initial_acc.at(i) = vel[i];
    }



    trajectory_1 = compute_joint_trajectory(this->tote_view_joint_angles,
                                            initial_velocity,
                                            initial_acc,
                                            trajectory_2.at(2).positions,
                                            trajectory_2.at(2).velocities,
                                            trajectory_2.at(2).accelerations, 1);


    //    std::cout <<"\npress any key to write trajectory_1\n";
    //    std::cin >>a;
    this->write_data_in_file(trajectory_1);


    //    std::cout <<"press any key to write trajectory_2\n";
    //    std::cin >>a;
    this->write_data_in_file(trajectory_2);

    len = trajectory_1.size();

    int len2 = trajectory_2.size();

    int len3 = len - 0 + len2 - 3;

    float time = trajectory_1.at(len - 1).time_from_start.toSec();

    trajectory_3.resize(len3);

    for(int i = 0; i< len - 0; i++)
        trajectory_3.at(i) = trajectory_1.at(i);

    for(int i = len - 0; i< len3; i++)
    {
        trajectory_3.at(i) = trajectory_2.at(i - (len - 0) + 3);
        trajectory_3.at(i).time_from_start = ros::Duration( time +
                                                            trajectory_2.at(i - (len - 0) + 3).time_from_start.toSec() -
                                                            trajectory_2.at(2).time_from_start.toSec());
    }


    //    std::cout <<"\npress any key to write trajectory_3\n";
    //    std::cin >>a;

    this->write_data_in_file(trajectory_3);

    //    std::cout <<"\npress any key to move\n";
    //    std::cin >>a;

    return trajectory_3;



}






std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::compute_linear_trajectory(
        trajectory_msgs::JointTrajectoryPoint initial_point,
        geometry_msgs::Pose final_pose, float speed_factor)
{


    robot_state::RobotStatePtr reference_state;

    reference_state = this->group->getCurrentState();


    double* initial_angle = new double();
    double* initial_velocity = new double();
    double* initial_efforts = new double();



    *initial_angle = initial_point.positions.at(0);
    reference_state->setJointPositions("shoulder_pan_joint", initial_angle);
    *initial_velocity = initial_point.velocities.at(0);
    reference_state->setJointVelocities(reference_state->getJointModel("shoulder_pan_joint"), initial_velocity);
    //    *initial_efforts = initial_angles.effort.at(0);
    //    reference_state->setJointEfforts(reference_state->getJointModel("shoulder_pan_joint"), initial_efforts);




    *initial_angle = initial_point.positions.at(1);
    reference_state->setJointPositions("shoulder_lift_joint", initial_angle);
    *initial_velocity = initial_point.velocities.at(1);
    reference_state->setJointVelocities(reference_state->getJointModel("shoulder_lift_joint"), initial_velocity);
    //    *initial_efforts = initial_angles.effort.at(1);
    //    reference_state->setJointEfforts(reference_state->getJointModel("shoulder_lift_joint"), initial_efforts);


    *initial_angle = initial_point.positions.at(2);
    reference_state->setJointPositions("elbow_joint", initial_angle);
    *initial_velocity = initial_point.velocities.at(2);
    reference_state->setJointVelocities(reference_state->getJointModel("elbow_joint"), initial_velocity);
    //    *initial_efforts = initial_angles.effort.at(2);
    //    reference_state->setJointEfforts(reference_state->getJointModel("elbow_joint"), initial_efforts);


    *initial_angle = initial_point.positions.at(3);
    reference_state->setJointPositions("wrist_1_joint", initial_angle);
    *initial_velocity = initial_point.velocities.at(3);
    reference_state->setJointVelocities(reference_state->getJointModel("wrist_1_joint"), initial_velocity);
    //    *initial_efforts = initial_angles.effort.at(3);
    //    reference_state->setJointEfforts(reference_state->getJointModel("wrist_1_joint"), initial_efforts);


    *initial_angle = initial_point.positions.at(4);
    reference_state->setJointPositions("wrist_2_joint", initial_angle);
    *initial_velocity = initial_point.velocities.at(4);
    reference_state->setJointVelocities(reference_state->getJointModel("wrist_2_joint"), initial_velocity);
    //    *initial_efforts = initial_angles.effort.at(4);
    //    reference_state->setJointEfforts(reference_state->getJointModel("wrist_2_joint"), initial_efforts);


    *initial_angle = initial_point.positions.at(5);
    reference_state->setJointPositions("wrist_3_joint", initial_angle);
    *initial_velocity = initial_point.velocities.at(5);
    reference_state->setJointVelocities(reference_state->getJointModel("wrist_3_joint"), initial_velocity);
    //    *initial_efforts = initial_angles.effort.at(5);
    //    reference_state->setJointEfforts(reference_state->getJointModel("wrist_3_joint"), initial_efforts);


    //    std::cout << *reference_state->getJointPositions("shoulder_pan_joint") <<"\n";
    //    std::cout << *reference_state->getJointPositions("shoulder_lift_joint") <<"\n";
    //    std::cout << *reference_state->getJointPositions("elbow_joint") <<"\n";
    //    std::cout << *reference_state->getJointPositions("wrist_1_joint") <<"\n";
    //    std::cout << *reference_state->getJointPositions("wrist_2_joint") <<"\n";
    //    std::cout << *reference_state->getJointPositions("wrist_3_joint") <<"\n";


    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(final_pose);

    moveit_msgs::RobotTrajectory trajectory;

    this->group->setStartState(*reference_state);

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);



    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

    this->group->setStartState(*reference_state);

    rt.setRobotTrajectoryMsg(*reference_state, trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, speed_factor, speed_factor);

    rt.getRobotTrajectoryMsg(trajectory);


    return trajectory.joint_trajectory.points;


}






std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::compute_joint_trajectory(std::vector<double> initial_angles,
                                                                                               std::vector<double> initial_velocity,
                                                                                               std::vector<double> initial_acc,
                                                                                               std::vector<double> final_angles,
                                                                                               std::vector<double> final_velocity,
                                                                                               std::vector<double> final_acc,
                                                                                               float speed_factor)
{


    this->group->setMaxAccelerationScalingFactor(speed_factor);
    this->group->setMaxVelocityScalingFactor(speed_factor);


    robot_state::RobotStatePtr reference_state;

    reference_state = this->group->getCurrentState();


    double* initial_angle = new double();




    *initial_angle = initial_angles.at(0);
    reference_state->setJointPositions("shoulder_pan_joint", initial_angle);





    *initial_angle = initial_angles.at(1);
    reference_state->setJointPositions("shoulder_lift_joint", initial_angle);



    *initial_angle = initial_angles.at(2);
    reference_state->setJointPositions("elbow_joint", initial_angle);



    *initial_angle = initial_angles.at(3);
    reference_state->setJointPositions("wrist_1_joint", initial_angle);



    *initial_angle = initial_angles.at(4);
    reference_state->setJointPositions("wrist_2_joint", initial_angle);



    *initial_angle = initial_angles.at(5);
    reference_state->setJointPositions("wrist_3_joint", initial_angle);


    this->group->setStartState(*reference_state);

    usleep(100000);


    this->group->setJointValueTarget(final_angles);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    this->group->plan(my_plan);

    int len = my_plan.trajectory_.joint_trajectory.points.size();

    float T = 1.0 * (my_plan.trajectory_.joint_trajectory.points.at(len - 1).time_from_start.toSec());

    this->group->setStartStateToCurrentState();


    Eigen::MatrixXf A(6,6);

    A <<  1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0,
            1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5),
            0, 1, 2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
            0, 0, 2, 6*T, 12*pow(T,2), 20*pow(T,3);



    Eigen::MatrixXf B_0(6,1);

    B_0 << initial_angles[0],
            initial_velocity[0],
            initial_acc[0],
            final_angles[0],
            final_velocity[0],
            final_acc[0];




    Eigen::MatrixXf B_1(6,1);

    B_1 << initial_angles[1],
            initial_velocity[1],
            initial_acc[1],
            final_angles[1],
            final_velocity[1],
            final_acc[1];



    Eigen::MatrixXf B_2(6,1);

    B_2 << initial_angles[2],
            initial_velocity[2],
            initial_acc[2],
            final_angles[2],
            final_velocity[2],
            final_acc[2];



    Eigen::MatrixXf B_3(6,1);

    B_3 << initial_angles[3],
            initial_velocity[3],
            initial_acc[3],
            final_angles[3],
            final_velocity[3],
            final_acc[3];



    Eigen::MatrixXf B_4(6,1);

    B_4 << initial_angles[4],
            initial_velocity[4],
            initial_acc[4],
            final_angles[4],
            final_velocity[4],
            final_acc[4];




    Eigen::MatrixXf B_5(6,1);

    B_5 << initial_angles[5],
            initial_velocity[5],
            initial_acc[5],
            final_angles[5],
            final_velocity[5],
            final_acc[5];

    Eigen::MatrixXf a_0(6,1) ;
    a_0 = A.colPivHouseholderQr().solve(B_0);

    Eigen::MatrixXf a_1(6,1) ;
    a_1 = A.colPivHouseholderQr().solve(B_1);

    Eigen::MatrixXf a_2(6,1) ;
    a_2 = A.colPivHouseholderQr().solve(B_2);

    Eigen::MatrixXf a_3(6,1) ;
    a_3 = A.colPivHouseholderQr().solve(B_3);

    Eigen::MatrixXf a_4(6,1) ;
    a_4 = A.colPivHouseholderQr().solve(B_4);

    Eigen::MatrixXf a_5(6,1) ;
    a_5 = A.colPivHouseholderQr().solve(B_5);



    Eigen::MatrixXf a(6,6);

    a.block(0,0,6,1) = a_0;
    a.block(0,1,6,1) = a_1;
    a.block(0,2,6,1) = a_2;
    a.block(0,3,6,1) = a_3;
    a.block(0,4,6,1) = a_4;
    a.block(0,5,6,1) = a_5;


    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_points;

    int number_of_waypoints = 45;

    trajectory_points.resize(number_of_waypoints);


    for(int i = 0; i< number_of_waypoints; i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;

        float t = i*T/(number_of_waypoints - 1);

        point.time_from_start = ros::Duration(t);

        point.positions.resize(6);
        for(int j = 0; j<6; j++)
            point.positions.at(j) = a(0, j) + a(1, j)*t + a(2, j)*pow(t, 2) + a(3, j)*pow(t, 3) + a(4, j)*pow(t, 4) + a(5, j)*pow(t, 5);

        point.velocities.resize(6);
        for(int j = 0; j<6; j++)
            point.velocities.at(j) = a(1, j) + 2*a(2, j)*t + 3*a(3, j)*pow(t, 2) + 4*a(4, j)*pow(t, 3) + 5*a(5, j)*pow(t, 4);

        point.accelerations.resize(6);
        for(int j = 0; j<6; j++)
            point.accelerations.at(j) =  2*a(2, j) + 6*a(3, j)*t + 12*a(4, j)*pow(t, 2) + 20*a(5, j)*pow(t, 3);

        trajectory_points.at(i) = point;

    }

    usleep(100000);

    return trajectory_points;


}




void Stowing_ARC_2017::vacuum_suction_on_off(ros::ServiceClient& gripper_suction_controller_srv,bool update_angle,
                                             int nozel_angle, int vacuum_cleaner, int suction_valve)
{

    apc_nozel::gripper_suction_controller gripper_suction_controller_obj;

    if(update_angle)
        gripper_suction_controller_obj.request.nozel_angle.data = (int8_t)nozel_angle;

    gripper_suction_controller_obj.request.vacuum_cleaner.data = (int8_t)vacuum_cleaner;
    gripper_suction_controller_obj.request.suction_valve.data = (int8_t)suction_valve;

    if(gripper_suction_controller_srv.call(gripper_suction_controller_obj))
        std::cout << "GRIPPER SUCTION CONTROLLER STATUS = " << gripper_suction_controller_obj.response.status.data << std::endl;
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




bool Stowing_ARC_2017::call_vision_service_bajrangi()
{

    std::vector<int> available_IDs;

    this->nh.getParam("/ARC17_AVAILABLE_IDS", available_IDs);

    std::cout <<"objects_in_tote\n";
    for(int i = 0; i< available_IDs.size(); i++)
        std::cout << available_IDs.at(i) <<", ";
    std::cout << "\n\n";

    ros::ServiceClient obj_detection_srv = this->nh.serviceClient<stow_class::computer_vision_stowing>("/iitktcs/computer_vision_stowing");

    stow_class::computer_vision_stowing detect_object;

    ros::service::waitForService("/iitktcs/computer_vision_stowing");

    usleep(1000000);


    std::cout<<"\n\ncalling service to get ROI\n";

    if(!obj_detection_srv.call(detect_object))
    {
        std::cout << "OBJ POINT CLOUD SERVICE NOT DONE\n";
        return false;
    }
    std::cout << "OBJ POINT CLOUD SERVICE DONE\n";


    std::vector<stow_class::objects_info> object_info_vector;

    for(int i = 0; i< detect_object.response.object_info.size(); i++)
        object_info_vector.push_back(detect_object.response.object_info.at(i));


    this->object_ID = (int)detect_object.response.object_info.at(0).id.data;


    ros::ServiceClient get_centroid_srv = nh.serviceClient<stow_class::pose>("/iitktcs/estimate_pose");

    stow_class::pose get_centroid;

    ros::service::waitForService("/iitktcs/estimate_pose");

    usleep(1000000);


    std::cout<<"\n\ncalling service to get pose\n";


    if(!get_centroid_srv.call(get_centroid))
    {
        std::cout << "OBJ POSE  SERVICE NOT DONE\n";
        return false;
    }
    std::cout << "OBJ POSE SERVICE DONE\n";

    for(int i = 0; i< detect_object.response.object_info.size(); i++)
        get_centroid.request.object_info.push_back(detect_object.response.object_info.at(i));


    this->centroid_point_wrt_world.point = get_centroid.response.centroid;
    this->normal_point_wrt_world.point = get_centroid.response.normal;
    this->axis_point_wrt_world.point = get_centroid.response.axis;



    geometry_msgs::Point normalized_axis;
    geometry_msgs::Point normalized_normal;


    normalized_axis.x = this->centroid_point_wrt_world.point.x - this->axis_point_wrt_world.point.x;
    normalized_axis.y = this->centroid_point_wrt_world.point.y - this->axis_point_wrt_world.point.y;
    normalized_axis.z = this->centroid_point_wrt_world.point.z - this->axis_point_wrt_world.point.z;

    float m = sqrt( pow(normalized_axis.x, 2) + pow(normalized_axis.y, 2) + pow(normalized_axis.z, 2) );

    normalized_axis.x = normalized_axis.x/m;
    normalized_axis.y = normalized_axis.y/m;
    normalized_axis.z = normalized_axis.z/m;


    normalized_normal.x = this->centroid_point_wrt_world.point.x - this->normal_point_wrt_world.point.x;
    normalized_normal.y = this->centroid_point_wrt_world.point.y - this->normal_point_wrt_world.point.y;
    normalized_normal.z = this->centroid_point_wrt_world.point.z - this->normal_point_wrt_world.point.z;

    m = sqrt( pow(normalized_normal.x, 2) + pow(normalized_normal.y, 2) + pow(normalized_normal.z, 2) );

    normalized_normal.x = normalized_normal.x/m;
    normalized_normal.y = normalized_normal.y/m;
    normalized_normal.z = normalized_normal.z/m;


    float magnitude = 0.15;

    geometry_msgs::Point axis;

    geometry_msgs::Point normal;


    axis.x = this->centroid_point_wrt_world.point.x +
            magnitude*(normalized_axis.x - this->centroid_point_wrt_world.point.x);

    axis.y = this->centroid_point_wrt_world.point.y +
            magnitude*(normalized_axis.y - this->centroid_point_wrt_world.point.y);

    axis.z = this->centroid_point_wrt_world.point.z +
            magnitude*(normalized_axis.z - this->centroid_point_wrt_world.point.z);


    normal.x = this->centroid_point_wrt_world.point.x +
            magnitude*(normalized_normal.x - this->centroid_point_wrt_world.point.x);

    normal.y = this->centroid_point_wrt_world.point.y +
            magnitude*(normalized_normal.y - this->centroid_point_wrt_world.point.y);

    normal.z = this->centroid_point_wrt_world.point.z +
            magnitude*(normalized_normal.z - this->centroid_point_wrt_world.point.z);






    this->centroid_point_wrt_world.point = get_centroid.response.centroid;
    this->normal_point_wrt_world.point = normal;
    this->axis_point_wrt_world.point = axis;






    std::cout << "ID: "<<this->ID[this->object_ID - 1] << "\n" <<
                 "NAME: " << this->name[this->object_ID - 1] << "\n" <<
                 "NORMAL_IGNORE: "<< this->ignore_normal[this->object_ID - 1] << "\n" <<
                 "WEIGHT: " << this->weight[this->object_ID - 1] << "\n" <<
                 "SIZE:  " << this->size[this->object_ID - 1] << "\n";


    return true;

}




bool Stowing_ARC_2017::move_towards_centroid_via_normal_mohit()
{

    //    std::cout <<"im in move_towards_centroid_via_normal\n";

    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(this->normal_point_with_orientation());

    waypoints.push_back(this->touch_centroid_point());

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);


    float speed_factor = 1.0;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    std::cout << "\nspeed_factor: " << speed_factor <<"\n\n";


    trajectory.joint_trajectory.points.clear();

    trajectory.joint_trajectory.points = this->combine_trajectory(speed_factor);

    my_plan.trajectory_ = trajectory;

    this->group->execute(my_plan);

    return true;



}




std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::move_towards_centroid_via_normal_mohit2()
{

    //    std::cout <<"im in move_towards_centroid_via_normal\n";

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_1;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_2;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_3;

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    float lift;

    if(this->size[this->object_ID - 1] == "large")
        lift = LIFT_BIG_OBJECT;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = LIFT_MEDIUM_OBJECT;
        else
            lift = LIFT_SMALL_OBJECT;
    }

    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = current_pose.pose.position.x;
    goal_pose.position.y = current_pose.pose.position.y;
    goal_pose.position.z = current_pose.pose.position.z + lift;


    float speed_factor = 1.0;

    goal_pose.orientation = current_pose.pose.orientation;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;



    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    moveit_msgs::RobotTrajectory trajectory2;

    waypoints.push_back(goal_pose);

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);


    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

    rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, speed_factor, speed_factor);

    rt.getRobotTrajectoryMsg(trajectory);

    my_plan.trajectory_ = trajectory;

    trajectory_1 = trajectory.joint_trajectory.points;

    int len = trajectory_1.size();




    //    waypoints.push_back(this->bin_pose);
    //    goal_pose = this->bin_pose;

    //    goal_pose.position.z = this->bin_pose.position.z - 0.2;

    //    waypoints.push_back(goal_pose);

    //    this->group->computeCartesianPath(waypoints,
    //                                      0.01,  // eef_step
    //                                      0.0,   // jump_threshold
    //                                      trajectory2);



    //    rt.setRobotTrajectoryMsg(this->group->getCurrentState()->getRobotModel(), trajectory2);

    //    iptp.computeTimeStamps(rt, speed_factor, speed_factor);

    //    rt.getRobotTrajectoryMsg(trajectory2);

    //    my_plan.trajectory_ = trajectory2;



    //    trajectory_3 = trajectory2.joint_trajectory.points;

    //    int len_ = trajectory_3.size();


    //    trajectory_2 = compute_joint_trajectory(trajectory_1.at(len - 3).positions,
    //                                            trajectory_1.at(len - 3).velocities,
    //                                            trajectory_1.at(len - 3).accelerations,
    //                                            trajectory_3.at(len_ - 3).positions,
    //                                            trajectory_3.at(len_ - 3).velocities,
    //                                            trajectory_3.at(len_ - 3).accelerations,
    //                                            speed_factor);




    std::vector<double> final_joint_angles = final_joint_angle(this->bin_pose);



    double vel[] = {0,0,0,0,0,0};

    std::vector<double> final_velocity;
    std::vector<double> final_acc;

    final_velocity.resize(6);
    final_acc.resize(6);

    for(int i = 0; i<6; i++)
    {
        final_velocity.at(i) = vel[i];
        final_acc.at(i) = vel[i];
    }


    trajectory_2 = compute_joint_trajectory(trajectory_1.at(len - 3).positions,
                                            trajectory_1.at(len - 3).velocities,
                                            trajectory_1.at(len - 3).accelerations,
                                            final_joint_angles,
                                            final_velocity,
                                            final_acc,
                                            speed_factor);





    int len2 = trajectory_2.size();

    int len3 = len - 2 + len2;

    float time = trajectory_1.at(len - 3).time_from_start.toSec();

    trajectory_3.resize(len3);

    for(int i = 0; i< len - 2; i++)
        trajectory_3.at(i) = trajectory_1.at(i);

    for(int i = len - 2; i<len3; i++)
    {
        trajectory_3.at(i) = trajectory_2.at( i - (len - 2) );
        trajectory_3.at(i).time_from_start = ros::Duration( time + trajectory_2.at( i - (len - 2) ).time_from_start.toSec());
    }


    return trajectory_3;



}




std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::reach_bin2(double location[0])
{

    this->reach_bin(location);

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();


    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = current_pose.pose.position.x;
    goal_pose.position.y = current_pose.pose.position.y;
    goal_pose.position.z = current_pose.pose.position.z + 0.4;


    std::vector<geometry_msgs::Pose> waypoints;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(goal_pose);

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);


    trajectory.joint_trajectory.points.clear();

    trajectory.joint_trajectory.points = this->move_towards_centroid_via_normal_mohit2();

    return trajectory.joint_trajectory.points;
}




std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::compute_linear_trajectory2(std::vector<double> initial_angles,
                                                                                                 std::vector<double> initial_velocities,
                                                                                                 geometry_msgs::Pose final_pose,
                                                                                                 float speed_factor)
{

    robot_state::RobotStatePtr reference_state;

    reference_state = this->group->getCurrentState();


    double* initial_angle = new double();
    double* initial_velocity = new double();



    *initial_angle = initial_angles.at(0);
    reference_state->setJointPositions("shoulder_pan_joint", initial_angle);
    *initial_velocity = initial_velocities.at(0);
    reference_state->setJointVelocities(reference_state->getJointModel("shoulder_pan_joint"), initial_velocity);





    *initial_angle = initial_angles.at(1);
    reference_state->setJointPositions("shoulder_lift_joint", initial_angle);
    *initial_velocity = initial_velocities.at(1);
    reference_state->setJointVelocities(reference_state->getJointModel("shoulder_lift_joint"), initial_velocity);



    *initial_angle = initial_angles.at(2);
    reference_state->setJointPositions("elbow_joint", initial_angle);
    *initial_velocity = initial_velocities.at(2);
    reference_state->setJointVelocities(reference_state->getJointModel("elbow_joint"), initial_velocity);



    *initial_angle = initial_angles.at(3);
    reference_state->setJointPositions("wrist_1_joint", initial_angle);
    *initial_velocity = initial_velocities.at(3);
    reference_state->setJointVelocities(reference_state->getJointModel("wrist_1_joint"), initial_velocity);



    *initial_angle = initial_angles.at(4);
    reference_state->setJointPositions("wrist_2_joint", initial_angle);
    *initial_velocity = initial_velocities.at(4);
    reference_state->setJointVelocities(reference_state->getJointModel("wrist_2_joint"), initial_velocity);



    *initial_angle = initial_angles.at(5);
    reference_state->setJointPositions("wrist_3_joint", initial_angle);
    *initial_velocity = initial_velocities.at(5);
    reference_state->setJointVelocities(reference_state->getJointModel("wrist_3_joint"), initial_velocity);





    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(final_pose);

    moveit_msgs::RobotTrajectory trajectory;

    this->group->setStartState(*reference_state);

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.00,   // jump_threshold
                                      trajectory);



    robot_trajectory::RobotTrajectory rt (this->group->getCurrentState()->getRobotModel(), "robo_arm");

    this->group->setStartState(*reference_state);

    rt.setRobotTrajectoryMsg(*reference_state, trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, speed_factor, speed_factor);

    rt.getRobotTrajectoryMsg(trajectory);

    return trajectory.joint_trajectory.points;

}





std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::combine_trajectory2( float speed_factor, double location[0])
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_1;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_2;

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_3;

    trajectory_1 = this->reach_bin2(location);

    int len = trajectory_1.size();

    geometry_msgs::Pose pose_goal = this->bin_pose;

    pose_goal.position.z = pose_goal.position.z - 0.2;

    trajectory_2 = compute_linear_trajectory(trajectory_1.at(len - 2), pose_goal, speed_factor);


    len = trajectory_1.size();

    int len2 = trajectory_2.size();

    int len3 = len - 1 + len2 - 1;

    float time = trajectory_1.at(len - 2).time_from_start.toSec();

    trajectory_3.resize(len3);

    for(int i = 0; i< len - 1; i++)
        trajectory_3.at(i) = trajectory_1.at(i);

    for(int i = len - 1; i< len3; i++)
    {
        trajectory_3.at(i) = trajectory_2.at(i - (len - 1) + 1);
        trajectory_3.at(i).time_from_start = ros::Duration( time +
                                                            trajectory_2.at(i - (len - 1) + 1).time_from_start.toSec() -
                                                            trajectory_2.at(0).time_from_start.toSec());
    }



    this->write_data_in_file(trajectory_3);


    return trajectory_3;



}




bool Stowing_ARC_2017::reach_bin3(double location[0])
{

    this->reach_bin(location);

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();


    geometry_msgs::Pose goal_pose;

    goal_pose.position.x = current_pose.pose.position.x;
    goal_pose.position.y = current_pose.pose.position.y;
    goal_pose.position.z = current_pose.pose.position.z + 0.4;


    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(goal_pose);

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);



    float speed_factor = 1.0;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    trajectory.joint_trajectory.points.clear();

    trajectory.joint_trajectory.points = this->combine_trajectory2(speed_factor, location);

    my_plan.trajectory_ = trajectory;

    this->group->execute(my_plan);

    return true;

}





std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::compute_trajectory_from_tote_to_object()
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_from_tote_to_normal;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_from_normal_to_object;

    std::vector <trajectory_msgs::JointTrajectoryPoint> complete_trajectory;

    double temp[] = {0,0,0,0,0,0};

    std::vector<double> initial_velocity;
    std::vector<double> initial_acc;

    initial_velocity.resize(6);
    initial_acc.resize(6);

    for(int i = 0; i<6; i++)
    {
        initial_velocity.at(i) = temp[i];
        initial_acc.at(i) = temp[i];

    }

    trajectory_from_tote_to_normal = this->compute_linear_trajectory2(this->tote_view_joint_angles,
                                                                      initial_velocity,
                                                                      this->normal_point_with_orientation(),
                                                                      1);

    int len = trajectory_from_tote_to_normal.size();

    std::vector<double> angles = trajectory_from_tote_to_normal.at(len - 1).positions;

    float speed_factor = 1.0;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    trajectory_from_normal_to_object = this->compute_linear_trajectory2(angles,
                                                                        initial_velocity,
                                                                        this->touch_centroid_point(),
                                                                        speed_factor);

    int len2 = trajectory_from_normal_to_object.size();

    trajectory_from_tote_to_normal.clear();

    trajectory_from_tote_to_normal = this->compute_joint_trajectory(this->tote_view_joint_angles,
                                                                    initial_velocity,
                                                                    initial_acc,
                                                                    trajectory_from_normal_to_object.at(2).positions,
                                                                    trajectory_from_normal_to_object.at(2).velocities,
                                                                    trajectory_from_normal_to_object.at(2).accelerations,
                                                                    1);

    len = trajectory_from_tote_to_normal.size();


    int len3 = len - 0 + len2 - 3;

    float time = trajectory_from_tote_to_normal.at(len - 1).time_from_start.toSec();

    complete_trajectory.resize(len3);

    for(int i = 0; i< len - 0; i++)
        complete_trajectory.at(i) = trajectory_from_tote_to_normal.at(i);

    for(int i = len - 0; i< len3; i++)
    {
        complete_trajectory.at(i) = trajectory_from_normal_to_object.at(i - (len - 0) + 3);
        complete_trajectory.at(i).time_from_start = ros::Duration( time +
                                                                   trajectory_from_normal_to_object.at(i - (len - 0) + 3).time_from_start.toSec() -
                                                                   trajectory_from_normal_to_object.at(2).time_from_start.toSec());
    }




    return complete_trajectory;


}




std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::compute_trajectory_from_object_to_bin(double location[0])
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_to_lift_object;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_to_reach_bin;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_to_drop_object;

    std::vector <trajectory_msgs::JointTrajectoryPoint> complete_trajectory;

    float speed_factor = 1.0;

    if(this->weight[this->object_ID - 1] == "heavy")
        speed_factor = SLOW;
    else
        speed_factor = 1.0;

    std::vector<double> angles = this->group->getCurrentJointValues();

    geometry_msgs::PoseStamped current_pose = this->group->getCurrentPose();

    geometry_msgs::Pose lift_pose = current_pose.pose;

    float lift = 0.6;

    if(this->size[this->object_ID - 1] == "large")
        lift = LIFT_BIG_OBJECT;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = LIFT_MEDIUM_OBJECT;
        else
            lift = LIFT_SMALL_OBJECT;
    }


    double temp[] = {0,0,0,0,0,0};

    std::vector<double> initial_velocity;
    std::vector<double> initial_acc;

    initial_velocity.resize(6);
    initial_acc.resize(6);

    for(int i = 0; i<6; i++)
    {
        initial_velocity.at(i) = temp[i];
        initial_acc.at(i) = temp[i];
    }

    lift_pose.position.z = lift_pose.position.z + lift;

    trajectory_to_lift_object = this->compute_linear_trajectory2(angles,
                                                                 initial_velocity,
                                                                 lift_pose,
                                                                 speed_factor);

    int len = trajectory_to_lift_object.size();

    this->reach_bin(location);

    trajectory_to_reach_bin = this->compute_linear_trajectory2(trajectory_to_lift_object.at(len-1).positions,
                                                               initial_velocity,
                                                               this->bin_pose,
                                                               speed_factor);

    int len2 = trajectory_to_reach_bin.size();

    geometry_msgs::Pose final_pose = this->bin_pose;

    final_pose.position.z = final_pose.position.z - lift;

    trajectory_to_drop_object = this->compute_linear_trajectory2(trajectory_to_reach_bin.at(len2 - 1).positions,
                                                                 initial_velocity,
                                                                 final_pose,
                                                                 speed_factor);

    int len3 = trajectory_to_drop_object.size();

    trajectory_to_reach_bin.clear();

    trajectory_to_reach_bin = this->compute_joint_trajectory(trajectory_to_lift_object.at(len-3).positions,
                                                             trajectory_to_lift_object.at(len-3).velocities,
                                                             trajectory_to_lift_object.at(len-3).accelerations,
                                                             trajectory_to_drop_object.at(2).positions,
                                                             trajectory_to_drop_object.at(2).velocities,
                                                             trajectory_to_drop_object.at(2).accelerations,
                                                             speed_factor);

    len2 = trajectory_to_reach_bin.size();

    int len4 = len - 2 + len2 + len3 - 3;


    float time1 = trajectory_to_lift_object.at(len-3).time_from_start.toSec();
    float time2 = trajectory_to_reach_bin.at(len2-1).time_from_start.toSec();


    complete_trajectory.resize(len4);

    for(int i = 0; i< len - 2; i++)
        complete_trajectory.at(i) = trajectory_to_lift_object.at(i);


    for(int i = len - 2; i< len - 2 + len2; i++)
    {
        complete_trajectory.at(i) = trajectory_to_reach_bin.at(i - (len - 2));
        complete_trajectory.at(i).time_from_start = ros::Duration(trajectory_to_reach_bin.at(i - (len - 2)).time_from_start.toSec()
                                                                  + time1);
    }


    for(int i = len - 2 + len2; i< len4; i++)
    {
        complete_trajectory.at(i) = trajectory_to_drop_object.at(i - (len - 2 + len2) + 3);
        complete_trajectory.at(i).time_from_start =
                ros::Duration( time1 + time2 +
                               trajectory_to_drop_object.at(i - (len - 2 + len2) + 3).time_from_start.toSec()
                               - trajectory_to_drop_object.at(2).time_from_start.toSec());
    }

    return complete_trajectory;


}





std::vector <trajectory_msgs::JointTrajectoryPoint> Stowing_ARC_2017::compute_trajectory_from_bin_to_tote()
{

    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_to_reach_tote;
    std::vector <trajectory_msgs::JointTrajectoryPoint> trajectory_to_move_up;

    std::vector <trajectory_msgs::JointTrajectoryPoint> complete_trajectory;

    std::vector<double> angles = this->group->getCurrentJointValues();

    geometry_msgs::Pose move_up_pose = this->bin_pose;

    double temp[] = {0,0,0,0,0,0};

    std::vector<double> initial_velocity;
    std::vector<double> initial_acc;

    std::vector<double> final_velocity;
    std::vector<double> final_acc;

    initial_velocity.resize(6);
    initial_acc.resize(6);
    final_velocity.resize(6);
    final_acc.resize(6);

    for(int i = 0; i<6; i++)
    {
        initial_velocity.at(i) = temp[i];
        initial_acc.at(i) = temp[i];

        final_velocity.at(i) = temp[i];
        final_acc.at(i) = temp[i];
    }

    trajectory_to_move_up = this->compute_linear_trajectory2(angles,
                                                             initial_velocity,
                                                             move_up_pose,
                                                             1);


    int len = trajectory_to_move_up.size();

    trajectory_to_reach_tote = this->compute_joint_trajectory(trajectory_to_move_up.at(len - 4).positions,
                                                              trajectory_to_move_up.at(len - 4).velocities,
                                                              trajectory_to_move_up.at(len - 4).accelerations,
                                                              this->tote_view_joint_angles,
                                                              final_velocity,
                                                              final_acc,
                                                              1);

    int len2 = trajectory_to_reach_tote.size();

    float time = trajectory_to_move_up.at(len - 4).time_from_start.toSec();

    int len3 = len - 3 + len2;

    complete_trajectory.resize(len3);

    for(int i = 0; i<len - 3; i++)
        complete_trajectory.at(i) = trajectory_to_move_up.at(i);


    for(int i = len - 3; i<len3; i++)
    {
        complete_trajectory.at(i) = trajectory_to_reach_tote.at(i - (len - 3));
        complete_trajectory.at(i).time_from_start = ros::Duration(trajectory_to_reach_tote.at(i - (len - 3)).time_from_start.toSec() + time);
    }


    return complete_trajectory;
}






bool Stowing_ARC_2017::suck_object()
{

    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(this->normal_point_with_orientation());

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);


    trajectory.joint_trajectory.points.clear();

    trajectory.joint_trajectory.points = this->compute_trajectory_from_tote_to_object();

    my_plan.trajectory_ = trajectory;

    return(this->group->execute(my_plan));
}





bool Stowing_ARC_2017::put_object(double location[0])
{

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose test_pose = this->bin_pose;

    float lift = 0.6;

    if(this->size[this->object_ID - 1] == "large")
        lift = LIFT_BIG_OBJECT;
    else
    {
        if(this->size[this->object_ID - 1] == "medium")
            lift = LIFT_MEDIUM_OBJECT;
        else
            lift = LIFT_SMALL_OBJECT;
    }

    test_pose.position.z = test_pose.position.z - lift;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(test_pose);

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);


    trajectory.joint_trajectory.points.clear();

    trajectory.joint_trajectory.points = this->compute_trajectory_from_object_to_bin(location);

    my_plan.trajectory_ = trajectory;

    return(this->group->execute(my_plan));

    //        return true;

    //    return this->verify_linear_position(test_pose);

}




bool Stowing_ARC_2017::go_to_tote()
{

    this->nozzel_angle = 0;

    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(this->normal_point_with_orientation());

    this->group->computeCartesianPath(waypoints,
                                      0.01,  // eef_step
                                      0.0,   // jump_threshold
                                      trajectory);


    trajectory.joint_trajectory.points.clear();

    trajectory.joint_trajectory.points = this->compute_trajectory_from_bin_to_tote();

    my_plan.trajectory_ = trajectory;

    return(this->group->execute(my_plan));

    //        return true;


    //    return this->verify_joint_goal_position(this->tote_view_joint_angles);
}





void* Stowing_ARC_2017::rotate_nozzle()
{

    ros::Publisher nozzle_angle_publisher =
            this->nh.advertise<std_msgs::Float32MultiArray>("/iitktcs/motion_planner/planning/suction_gripper_pos",1);

    std_msgs::Float32MultiArray data;

    data.data.resize(4);

    data.data[0] = 0;
    data.data[1] = 0;

    while(1)
    {


        //        data.data[2] = (float)real;
        data.data[2] = 0;
        data.data[3] = M_PI/2 - this->nozzel_angle;

        ros::Rate loop_rate(500);

        // Just publish for few times
        for(int i = 0; i < 20; i++)
        {
            nozzle_angle_publisher.publish(data);
            loop_rate.sleep();
        }


    }



}





void* Stowing_ARC_2017::broadcast_kf(void * threadid)
{

    tf::TransformBroadcaster br;

    tf::StampedTransform transform_ensenso_frame;

    std::vector<double> data;

    Eigen::MatrixXf transformation_matrix(4, 4);


    this->nh.getParam("/ARC17_R_T_MATRIX", data);

    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            transformation_matrix(i,j) = data.at(4*i + j);



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


        ros::Rate loop_rate(100);

        loop_rate.sleep();

    }

}




void Stowing_ARC_2017::chatterCallback(const geometry_msgs::WrenchStamped &force)
{

    this->force_x =  force.wrench.force.x;
    this->force_y =  force.wrench.force.y;
    this->force_z =  force.wrench.force.z;

    if(this->force_z < 0)
        this->force_z = 0;


    return;
}



