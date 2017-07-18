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



#define KEY "KBB1V–A386–EA49–2294–A708"


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


        //        float tx_data[] = { 0.0613685,-0.0748391,0.995306,0.0628091,
        //                            -0.997221,-0.0468059,0.0579672,-0.0493841,
        //                            0.0422479,-0.996097,-0.0775035,0.252622,
        //                            0,0,0,1,};


        float tx_data[] = { 0.0153707, 0.0256545, 0.999553, 0.0341811,
                            -0.999878, 0.00320517, 0.0152935, -0.0652545,
                            -0.00281141, -0.999666, 0.0257006, 0.213271,
                            0, 0, 0,  1};


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
    ros::init(argc, argv, "test_caliberation");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("robo_arm");

    ros::NodeHandle n;

    ros::ServiceClient kinect_points_calib = n.serviceClient<arc_controller::points>("/calib/test_point");

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    pthread_t threads;

    pthread_create(&threads, NULL, broadcast_kf, NULL);

    geometry_msgs::PointStamped point_wrt_kinect;
    geometry_msgs::PointStamped point_wrt_world;

    geometry_msgs::Pose final_pose;

    geometry_msgs::PoseStamped current_state;

    std::vector<geometry_msgs::Pose> waypoints;

    tf::TransformListener listener;

    moveit::planning_interface::MoveGroup::Plan plan;

    std::vector <double> joint_angles;

    joint_angles.resize(6);

    while (ros::ok())
    {

        signal(SIGINT, signal_callback_handler);


        //         ******* calling kinect service to get marker points wrt kinect frame *******

        arc_controller::points srv;

        ros::service::waitForService("/calib/test_point");

        srv.request.key.data = KEY;

        if(kinect_points_calib.call(srv))
        {
            point_wrt_kinect.point.x = srv.response.points_3d.data.at(0);
            point_wrt_kinect.point.y = srv.response.points_3d.data.at(1);
            point_wrt_kinect.point.z = srv.response.points_3d.data.at(2);
        }

        std::cout << point_wrt_kinect.point.x <<", "
                  << point_wrt_kinect.point.y <<", "
                  << point_wrt_kinect.point.z <<"\n\n\n";


        point_wrt_kinect.header.frame_id = "kf";

        listener.waitForTransform( "world", "kf", ros::Time(0), ros::Duration(3));

        listener.transformPoint("world", point_wrt_kinect, point_wrt_world);

        //        std::cin >> point_wrt_world.point.x
        //                 >> point_wrt_world.point.y
        //                 >> point_wrt_world.point.z;


        final_pose.position.x =  point_wrt_world.point.x;
        final_pose.position.y =  point_wrt_world.point.y;
        final_pose.position.z =  point_wrt_world.point.z;

        std::cout << "\n" << point_wrt_world.point.x <<", "
                  << point_wrt_world.point.y <<", "
                  << point_wrt_world.point.z <<"\n\n\n";


        visualization_msgs::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0; // Don't forget to set the alpha!


        marker.color.r = 1.0;

        marker.pose.position.x = point_wrt_world.point.x;
        marker.pose.position.y = point_wrt_world.point.y;
        marker.pose.position.z = point_wrt_world.point.z;

        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);

        joint_angles = group.getCurrentJointValues();

        double tot_view_angles[6] = {1.5141866207122803, -1.6036017576800745, 1.4953527450561523, -1.5201142469989222, -1.6038177649127405, -1.711933437977926};


        //        for(int i = 0; i<6; i++)
        //            joint_angles.at(i) = tot_view_angles[i];

        joint_angles.at(4) = tot_view_angles[4];


        group.setJointValueTarget(joint_angles);

        group.plan(plan);

        std::cout << "Press any key to reach at start position\n";

        char a;
        std :: cin >> a;



        group.move();



        current_state = group.getCurrentPose();

        final_pose.orientation = current_state.pose.orientation;

        waypoints.push_back(final_pose);




        usleep(100000);

        moveit_msgs::RobotTrajectory trajectory;

        double fraction = group.computeCartesianPath(waypoints,
                                                     0.01,  // eef_step
                                                     0.0,   // jump_threshold
                                                     trajectory);

        std::cout<<"\n---> "<< fraction * 100.0 <<" Path computed \n";



        robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "robo_arm");

        rt.setRobotTrajectoryMsg(group.getCurrentState()->getRobotModel(), trajectory);

        trajectory_processing :: IterativeParabolicTimeParameterization iptp;

        bool success = iptp.computeTimeStamps(rt);

        ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");


        rt.getRobotTrajectoryMsg(trajectory);

        if(success)
        {

            plan.trajectory_ = trajectory;

            std::cout << "Press any key to move\n";
            char a;
            std :: cin >> a;

            group.execute(plan);

        }


        waypoints.clear();
        trajectory.joint_trajectory.points.clear();
        plan.trajectory_.joint_trajectory.points.clear();
        rt.clear();


       double home[] = {0.06296025216579437, -1.7456653753863733, 1.4136481285095215, -1.2450502554522913, 0.004579097498208284, -2.0302422682391565};


        for(int i = 0; i<6; i++)
            joint_angles.at(i) = home[i];


        group.setJointValueTarget(joint_angles);

        group.plan(plan);

        std::cout << "Press any key to reach at home position\n";

        std :: cin >> a;


        group.move();




    }
    return 0;
}
