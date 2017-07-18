#ifndef APC_CONTROLLER_H
#define APC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <unistd.h>
#include <fstream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

//#include <lines_rack_det/DetectShelf.h>

//#include <json_maker/write_pick_status.h>
//#include <json_maker/write_stow_data.h>
//#include <json_maker/get_bin_object.h>
//#include <json_maker/getBinContents.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <robotarm.h>
//#include <ur5_control/UR5Goal.h>



#include<pcl_ros/transforms.h>

#define ROBOT_ANGLE_SIMULATION  1
#define ROBOT      1
#define NO_OF_BINS  12
#define NO_OF_JOINTS    16
#define NO_CLASSES 13

#define VACCUM_SUCTION  1//  Vaccum suction flag

#define SHOW_SIMULATION_MARKER  1

#define OBJECT_BIN          1 // To enable object bin

#define STOW_TASK           1 // To pick object from tote

#define WEBCAM_TOTE_VIDEO_RECORD 0

#define AUTO_OP 1 // Autonomous operation of robot

#define TRAJECTORY_RPT 1

#define USE_RGB_OPTICAL_FRAME 1

#if(STOW_TASK)
#define OBJECT_REC_RAJ_SURF 0
#define OBJECT_REC_NISHANT_PC 1

#endif

#if(SHOW_SIMULATION_MARKER)
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif

//#include <ik_test_service/IKTest.h>
//#include <ik_test_service/GetFK.h>
//#include <ik_test_service/GetIK.h>

//#if(TRAJECTORY_RPT)
//#include <trajectory_rpt/TrajRpt.h>
//#include <trajectory_rpt/TrajStow.h>
//#endif

// OBJECT DETECTION APPROACHES
//#define RCNN 1

//#if(RCNN)
//#include <apc_controller/objectDetect.h>
//#include <json_maker/stowToteContents.h>
//#endif

//#include <apc_controller/CropBin.h>

#define RACK_REGISTRATION 1
#define SEND_END_CORNERS 1
#if(RACK_REGISTRATION)
//#include <apc_controller/rack_registration.h>
#endif


#define PICKING_TASK    1

using namespace std;

std::string model_names[] = {
    "barbie_book",      "camlin_color_pencils",     "cave_mate_handtowel",      "cave_mate_tissues",    "cotton_socks",
    "devi_coffee",      "fevicol",                  "fevikwik",                 "floor_brush_yellow",   "garnet_bulb",
    "jump_rope",        "oralb_pink_toothbrush",    "staples_scissor_black"
};

void publishJointStateInSimulation(vector<double> &jointVector, ros::Publisher &simJointPublisher)
{
    sensor_msgs::JointState simJointState;
    simJointState.name.resize(int(NO_OF_JOINTS));
    simJointState.position.resize(int(NO_OF_JOINTS));

    ros::Rate loop_rate(10);

    simJointState.name[0] ="joint_front_right_steer";
    simJointState.name[1] ="joint_front_right_wheel";
    simJointState.name[2] ="joint_front_left_steer";
    simJointState.name[3] ="joint_front_left_wheel";
    simJointState.name[4] ="joint_back_left_steer";
    simJointState.name[5] ="joint_back_left_wheel";
    simJointState.name[6] ="joint_back_right_steer";
    simJointState.name[7] ="joint_back_right_wheel";
    simJointState.name[8] ="scissor_prismatic_joint";
    simJointState.name[9] ="j1_joint";
    simJointState.name[10] ="j2_joint";
    simJointState.name[11] ="j3_joint";
    simJointState.name[12] ="j4_joint";
    simJointState.name[13] ="j5_joint";
    simJointState.name[14] ="j6_joint";
    simJointState.name[15] ="j7_joint";

    for(int i=0;i<8;i++)
        simJointState.position[i] = 0.0;

    simJointState.position[8] = 0.5;

    for(int i=9;i<16;i++)
        simJointState.position[i] = jointVector[i-jointVector.size()-2];

    for(int i = 0; i < 10; i++)
    {
        simJointPublisher.publish(simJointState);
        loop_rate.sleep();

    }

    return ;
}

#if(SHOW_SIMULATION_MARKER)
void addMarker(double *centroid, double *color, visualization_msgs::Marker &marker, std::string &frame_id, int id)
{
    //    static int id = 0;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    //    id++;
}

void getBaseFrameToKinect(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    //    tf::StampedTransform transform_kinect_j1

    transform_listener.waitForTransform("/camera_rgb_optical_frame", "/base_link", ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform("/camera_rgb_optical_frame", "/base_link", ros::Time(0), transform);
    return;
}

void wamBaseToKinect(vector<double> &point, vector<double> &tf_point)
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/camera_rgb_optical_frame", "/wam_base_link", ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform("/camera_rgb_optical_frame", "/wam_base_link", ros::Time(0), transform_kinect_j1);

    {
        tf::Vector3 pt(point[0], point[1], point[2]);
        tf::Vector3 tf_pt = transform_kinect_j1*pt;
        tf_point[0] = tf_pt.getX();
        tf_point[1] = tf_pt.getY();
        tf_point[2] = tf_pt.getZ();
    }

    return;
}


extern const char* file_name_tx;

void kinectToBaseFrame(geometry_msgs::Point &point, geometry_msgs::Point &tf_point)
{
//    tf::TransformListener transform_listener1;
//    tf::StampedTransform transform_kinect_j2;

//    transform_listener1.waitForTransform("/wrist_3_link", "/from_kinect2", ros::Time(0), ros::Duration(3));
//    transform_listener1.lookupTransform("/wrist_3_link", "/from_kinect2", ros::Time(0), transform_kinect_j2);


//    tf::Matrix3x3 rot = transform_kinect_j2.getBasis();
//    tf::Vector3 t = transform_kinect_j2.getOrigin();

//    for(int i=0;i<3;i++)
//    {
//        for(int j=0;j<3;j++)
//            std::cout << rot[i][j] << " ";
//        std::cout << std::endl;
//    }

//    for(int i=0;i<3;i++)
//        std::cout << t[i] << " ";
//    std::cout << std::endl;


    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/world", "/kinect2_link", ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform("/world", "/kinect2_link", ros::Time(0), transform_kinect_j1);

std::cout << point.x << " " << point.y << " " << point.z << "\n";

    tf::Vector3 pt(point.x,point.y,point.z);
    tf::Vector3 tf_pt = transform_kinect_j1*pt;

   std::cout << tf_pt.getX() << " " << tf_pt.getY() << " " << tf_pt.getZ() << std::endl;


    tf_point.x = tf_pt.getX();
    tf_point.y = tf_pt.getY();
    tf_point.z = tf_pt.getZ();

    return;
}

// Function to transform a vector from kinect frame to Robot base frame
void kinectToBaseFrameVector(geometry_msgs::Point &point, geometry_msgs::Point &tf_point)
{

    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/world", "/kinect2_link", ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform("/world", "/kinect2_link", ros::Time(0), transform_kinect_j1);

    tf::Vector3 pt(point.x,point.y,point.z);
    tf::Vector3 tf_pt = transform_kinect_j1*pt;

//    tf::Vector3 tr_origin = transform_kinect_j1.getOrigin();


//    tf_point.x = tf_pt.getX()-tr_origin.getX();
//    tf_point.y = tf_pt.getY()-tr_origin.getY();
//    tf_point.z = tf_pt.getZ()-tr_origin.getZ();

    tf_point.x = tf_pt.getX();
    tf_point.y = tf_pt.getY();
    tf_point.z = tf_pt.getZ();

    return;
}

void sphereMarker(vector<tf::Vector3> &points, ros::Publisher &marker_pub, string frame_id)
{
    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker sphere_marker;

    double color[3] = {1.0, 0.0, 0.0};

    double centroid[3];

    cout << "ref color: red" << endl;
    for(int j=0; j<points.size(); j++)
    {
        tf::Vector3 v = points[j];
        centroid[0] = v.getX(); centroid[1] = v.getY(); centroid[2] = v.getZ();
        string frame = frame_id;

        addMarker(centroid,color,sphere_marker,frame, j);
        markerArray.markers.push_back(sphere_marker);
    }

    //    while(marker_pub.getNumSubscribers() < 1)
    //    {
    //        ros::Duration(3).sleep();
    //        cout << "Put subscriber for topic: \bin_corner"  << endl;
    //    }

    marker_pub.publish(markerArray);

    return;
}

void sphereMarker_green(vector<tf::Vector3> &points, ros::Publisher &marker_pub, string frame_id)
{
    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker sphere_marker;

    double color[3] = {0.0, 1.0, 0.0};

    double centroid[3];

    cout << "ref color: red" << endl;
    for(int j=0; j<points.size(); j++)
    {
        tf::Vector3 v = points[j];
        centroid[0] = v.getX(); centroid[1] = v.getY(); centroid[2] = v.getZ();
        string frame = frame_id;

        addMarker(centroid,color,sphere_marker,frame, j);
        markerArray.markers.push_back(sphere_marker);
    }

    //    while(marker_pub.getNumSubscribers() < 1)
    //    {
    //        ros::Duration(3).sleep();
    //        cout << "Put subscriber for topic: \bin_corner"  << endl;
    //    }

    marker_pub.publish(markerArray);

    return;
}

void sphereMarker_blue(vector<tf::Vector3> &points, ros::Publisher &marker_pub, string frame_id)
{
    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker sphere_marker;

    double color[3] = {0.0, 0.0, 1.0};

    double centroid[3];

    cout << "ref color: red" << endl;
    for(int j=0; j<points.size(); j++)
    {
        tf::Vector3 v = points[j];
        centroid[0] = v.getX(); centroid[1] = v.getY(); centroid[2] = v.getZ();
        string frame = frame_id;

        addMarker(centroid,color,sphere_marker,frame, j);
        markerArray.markers.push_back(sphere_marker);
    }

    //    while(marker_pub.getNumSubscribers() < 1)
    //    {
    //        ros::Duration(3).sleep();
    //        cout << "Put subscriber for topic: \bin_corner"  << endl;
    //    }

    marker_pub.publish(markerArray);

    return;
}

void showMarkerInSimulation(vector<double> kinectCentroid,vector<double> wamCentroid,vector<double> fkCentroid, ros::Publisher &marker_pub)
{
    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker kinect_centroid_marker;

    double color[3] = {1.0, 0.0, 0.0};

    double centroid[3];

    ros::Rate loop_rate(10);

    for(int i=0;i<3;i++)
        centroid[i]=kinectCentroid[i];
    string frame = "/camera_rgb_optical_frame";

    cout << "Kinect ref color: red" << endl;
    addMarker(centroid,color,kinect_centroid_marker,frame, 0);
    markerArray.markers.push_back(kinect_centroid_marker);

    for(int i=0;i<3;i++)
        centroid[i]=wamCentroid[i];

    color[0] = 0;
    color[1] = 1;
    color[2] = 0;
    frame = "/wam_base_link";

    cout << "wam ref color: green" << endl;
    addMarker(centroid,color,kinect_centroid_marker,frame, 1);
    markerArray.markers.push_back(kinect_centroid_marker);

    //    for(int i=0;i<3;i++)
    //        centroid[i]=fkCentroid[i];
    centroid[0]=fkCentroid[0]-0.22;
    centroid[1]=fkCentroid[1]-0.14;
    centroid[2]=fkCentroid[2]-0.346;

    color[0] = 0;
    color[1] = 0;
    color[2] = 1;

    cout << "FK ref color: blue" << endl;
    addMarker(centroid,color,kinect_centroid_marker,frame, 2);
    markerArray.markers.push_back(kinect_centroid_marker);

    while(marker_pub.getNumSubscribers() < 1)
    {
        ros::Duration(3).sleep();
        cout << "Put subscriber for kinect and WAM base marker" << endl;
    }

    marker_pub.publish(markerArray);

    return;
}

#endif

#if(AUTO_OP)
void wamJointStateCallback(const sensor_msgs::JointState::ConstPtr &jt_state_msg, vector<double> &jt_state)
{
    for(int i = 0; i < jt_state_msg->position.size(); i++)
        jt_state[i] = jt_state_msg->position[i];

    return;
}

bool checkJointsReached(vector<double> &desired_joints_state, vector<double> &current_joints_state)
{
    bool goal_achieved = false;
    double current_jts[7];
    double time = ros::Time::now().toSec();
    double duration;

    int start_jt = 1;
    while(!goal_achieved)
    {
        ros::spinOnce();
        for(int i = 0; i < current_joints_state.size(); i++)
            current_jts[i] = current_joints_state[i];

        double avg_pos_diff = 0;
        for(int i = start_jt; i < desired_joints_state.size(); i++)
            avg_pos_diff += fabs((current_jts[i] - desired_joints_state[i]));
        avg_pos_diff /= desired_joints_state.size();

        // wait until avg joint position diff is approx. 5.7 degrees
        if(avg_pos_diff < 0.025)
            goal_achieved = true;
        else
        {
            goal_achieved = false;

        }

        duration = ros::Time::now().toSec() - time;
        if(duration > 5) // if in loop for more than 60 secs then break out
        {
            ROS_INFO("Time limit exceeded to reach desired position");
            goal_achieved = true;
            break;
        }
        //        cout << "Desired: [";
        //        for(int i=0; i<7 ; i++)
        //            cout << desired_joints_state[i] << " ";
        //        cout << "]\n";
        //        cout << "Current: [";
        //        for(int i=0; i<7 ; i++)
        //            cout << current_joints_state[i] << " ";
        //        cout << "]";
    }
    if(goal_achieved)
    {
        //        cout << "WAM desired position achieved in: " << duration << "seconds" << endl;
        return true;
    }
    else
        return false;
}

#endif

// centroidVector: contains bin centroids data wrt base_link
// bin_corners: contains bin corners data wrt base_link
vector< vector<double> > centroidVector = vector< vector<double> >(int(NO_OF_BINS),vector<double>(3,0));
vector<tf::Vector3> bin_corners(4*(int (NO_OF_BINS)));
bool got_shelf_data = false;
//bool shelfDataCallback(lines_rack_det::DetectShelf::Request &req, lines_rack_det::DetectShelf::Response &res)
//{
//    if(got_shelf_data)
//    {
//        cout << "/***** At shelf data transfer service *****/";
//        for(int i = 0; i < centroidVector.size(); i++)
//        {
//            vector<double> v = centroidVector[i];
//            for(int j = 0; j < v.size(); j++)
//                res.centroids.data.push_back(v[j]);
//        }
//        for(int i = 0; i < bin_corners.size(); i++)
//        {
//            tf::Vector3 v = bin_corners[i];
//            res.bin_corners.data.push_back(v.getX());
//            res.bin_corners.data.push_back(v.getY());
//            res.bin_corners.data.push_back(v.getZ());
//        }
//        cout << "Sent the shelf centroid and corner data" << endl;
//        return true;
//    }
//    else
//    {
//        cout << "Shelf centroid and corner data not obtained" << endl;
//        return false;
//    }
//}

// Function to modify centroids and corners in case the robot moves in x,y,z direction
// x,y,z all should be provided
// values should be positive in case moved along the positive direction of the axis
// values should be negative in case moved along the negative direction of the axis
void translateCentroidCorners(double x, double y, double z)
{
    for(int i=0; i<centroidVector.size(); i++)
    {
        //        cout << "Centroid " << i << ": " << centroidVector[i][0] << "\t" << centroidVector[i][1] << "\t" << centroidVector[i][2] << "\t" << endl;
        centroidVector[i][0] -= x;
        centroidVector[i][1] -= y;
        centroidVector[i][2] -= z;
        //        cout << "M Centroid " << i << ": " << centroidVector[i][0] << "\t" << centroidVector[i][1] << "\t" << centroidVector[i][2] << "\t" << endl;
    }
    tf::Vector3 v(x,y,z);
    for(int i=0; i<bin_corners.size(); i++)
    {
        bin_corners[i] -= v;
    }
    //    if(x < 0)
    //        cout << "Centroid are shifted right: " << x;
    //    else if(x > 0)
    //        cout << "Centroid are shifted left: " << x;
    //    if(y < 0)
    //        cout << "Centroid are shifted forward: " << y;
    //    else if(y > 0)
    //        cout << "Centroid are shifted backward: " << y;
    //    if(z < 0)
    //        cout << "Centroid are shifted up: " << z;
    //    else if(z > 0)
    //        cout << "Centroid are shifted down: " << z;

    //    char c;
    //    cout << "\nPress any character ..." << endl;
    //    cin >> c;
    //    getchar();
    return;
}

void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr, bool &copy)
{
    if(copy)
    {
        pcl::fromROSMsg(*input, *cloud_ptr);
        if(!cloud_ptr->empty())
        {
            copy = false;
        }
    }
    return;
}

void irSensorCallback(const std_msgs::Int16::ConstPtr &ir_msg, int &ir_data, bool &flag_available)
{
    if(!flag_available)
    {
        ir_data = ir_msg->data;
        flag_available = true;
    }
    return;
}

void metalSensorCallback(const std_msgs::Int16::ConstPtr &metal_msg, int &metal_data, bool &flag_available)
{
    metal_data = metal_msg->data;
    flag_available = true;
    return;
}

void addCylinderMarker(double *centroid, geometry_msgs::Quaternion &q, double &cyl_radius, double *color, visualization_msgs::Marker &marker, std::string &frame_id, int id)
{
    //    static int id = 0;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_cylinder";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::DELETE;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = q.x;
    marker.pose.orientation.y = q.y;
    marker.pose.orientation.z = q.z;
    marker.pose.orientation.w = q.w;
    marker.scale.x = cyl_radius;
    marker.scale.y = cyl_radius;
    marker.scale.z = 2*cyl_radius;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    //    id++;
}

void addLineMarker(double *centroid, string &ns, geometry_msgs::Point &start_pt, geometry_msgs::Point &end_pt,
                   double *color, visualization_msgs::Marker &marker, std::string &frame_id, int id)
{
    //    static int id = 0;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::DELETE;

    marker.points.push_back(start_pt);
    marker.points.push_back(end_pt);


    //    marker.pose.position.x = centroid[0];
    //    marker.pose.position.y = centroid[1];
    //    marker.pose.position.z = centroid[2];
    //    marker.pose.orientation.x = q.x;
    //    marker.pose.orientation.y = q.y;
    //    marker.pose.orientation.z = q.z;
    //    marker.pose.orientation.w = q.w;
    marker.scale.x = 0.1;
    //    marker.scale.y = cyl_radius;
    //    marker.scale.z = 2*cyl_radius;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    //    id++;
}

// Function to accept rotation matrix and return Quaternion
geometry_msgs::Quaternion rotationMatrixtoQuaternion(tf::Matrix3x3 &matrix)
{
    tf::Quaternion q;
    geometry_msgs::Quaternion gm_qt;

    matrix.getRotation(q);
    gm_qt.x = q.getX();
    gm_qt.y = q.getY();
    gm_qt.z = q.getZ();
    gm_qt.w = q.getW();

    return gm_qt;

}

void addCylinderMarkerArray(vector<tf::Vector3> &centroids, vector<tf::Matrix3x3> orientation, double &cyl_radius, ros::Publisher &marker_pub)
{
    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker cyl_marker;

    double color[3] = {0.0, 1.0, 1.0};

    double centroid[3];

    cout << "ref color: cyan" << endl;
    for(int j=0; j<centroids.size(); j++)
    {
        tf::Vector3 v = centroids[j];
        centroid[0] = v.getX(); centroid[1] = v.getY(); centroid[2] = v.getZ();
        string frame = "/camera_rgb_optical_frame";
        geometry_msgs::Quaternion quat = rotationMatrixtoQuaternion(orientation[j]);

        addCylinderMarker(centroid, quat, cyl_radius, color, cyl_marker, frame, j);
        markerArray.markers.push_back(cyl_marker);
    }

    while(marker_pub.getNumSubscribers() < 1)
    {
        ros::Duration(3).sleep();
        cout << "Put subscriber for topic: cylinder_marker"  << endl;
    }

    marker_pub.publish(markerArray);

    return;
}

void addArrowMarker(tf::Vector3 &start_pt, tf::Vector3 &end_pt, double *color, visualization_msgs::Marker &marker,
                    std::string &frame_id, int id)
{
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "apc_orient_vector";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point pt;
    pt.x = start_pt.getX(); pt.y = start_pt.getY(); pt.z = start_pt.getZ();

    marker.points.push_back(pt);// start_pt

    pt.x = end_pt.getX(); pt.y = end_pt.getY(); pt.z = end_pt.getZ();
    marker.points.push_back(pt);// end_pt


    marker.scale.x = 0.0075;
    marker.scale.y = 0.01;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
}

void addArrowMarkerArray(vector<tf::Vector3> &start_points, vector<tf::Vector3> &end_points,
                         ros::Publisher &marker_pub, string ns, double *line_color, string frame)
{
    static int id = 0;
    std::cout << "Publishing Markers: " << ns << " lines" << std::endl;

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker arrow_marker;

    //    string frame = "/camera_rgb_optical_frame";

    for(int n_i = 0; n_i < start_points.size(); n_i++)
    {
        addArrowMarker(start_points[n_i], end_points[n_i], line_color, arrow_marker, frame, id);
        markerArray.markers.push_back(arrow_marker);
    }
    while(marker_pub.getNumSubscribers() < 1)
    {
        ros::Duration(3).sleep();
        cout << "Put subscriber for topic: " << ns << "_marker"  << endl;
    }

    marker_pub.publish(markerArray);

    return;
}

// Function to get the end point a vector at distance 7cm from the start point
tf::Vector3 getEndpt(tf::Vector3 vector, tf::Vector3 &start_pt)
{
    vector.normalize();

    vector = vector - start_pt;

    vector.normalize();

    vector *= 0.30;
    tf::Vector3 end_pt(vector);
    end_pt += start_pt;

    return end_pt;
}


#endif // APC_CONTROLLER_H
