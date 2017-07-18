#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
//#include <arc_controller/camera_robot_calibration.h>
#include <arc_controller/points.h>

using namespace Eigen;

#define KEY "KBB1V–A386–EA49–2294–A708"

int main(int argc, char **argv) {

    ros::init(argc,argv,"arc_real_robot");
    ros::NodeHandle nh;

    //ros::ServiceClient kinect_points_calib = nh.serviceClient<arc_controller::camera_robot_calibration>("/arc_2017_computer_vision/camera_robot_calibration");

    ros::ServiceClient kinect_points_calib = nh.serviceClient<arc_controller::points>("/calib_aruco/ur10");

    //arc_controller::camera_robot_calibration srv;
    arc_controller::points srv;

    int total_no_of_points ;

    srv.request.key.data = KEY;

    ros::service::waitForService("/calib_aruco/ur10");

    if(kinect_points_calib.call(srv))
    {

        total_no_of_points = srv.response.points_3d.data.size()/4;

        std::cout << total_no_of_points << "\n\n";

        std::vector<int> points_array;
        points_array.resize(total_no_of_points);

        for (int i = 0; i< total_no_of_points; i++)
            points_array.at(i) = srv.response.points_3d.data.at(4*i);

        for (int i = 0; i< total_no_of_points; i++)
            std::cout << points_array[i] << "\n\n";

        MatrixXf points_wrt_kinect(total_no_of_points, 3);

        for (int i = 0; i < total_no_of_points; i++)
        {
            points_wrt_kinect.block(i, 0, 1, 3) << srv.response.points_3d.data.at( 4*i + 1 ),
                    srv.response.points_3d.data.at( 4*i + 2 ),
                    srv.response.points_3d.data.at( 4*i + 3 );
        }

        std::cout << points_wrt_kinect;

    }


    return 0;
}
