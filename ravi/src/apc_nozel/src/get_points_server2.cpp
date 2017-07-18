#include <ros/ros.h>
#include <apc_nozel/get_points2.h>

bool set_val(apc_nozel::get_points2::Request &req, apc_nozel::get_points2::Response &res)
{


    std::cout <<"\nFolloing are the object remaining in the tot\n";

    for(int i = 0; i<40; i++)
    {
        if(i%10 == 0)
            std::cout << "\n";

        std::cout << req.remaining_objects.data.at(i) << " ";
    }


    std::cout << "\nEnter the object ID\n";

    std::cin >> res.object_ID.data;


    std::cout <<"\nEnter centroid(x, y, z) of object wrt kinect\n";

    std::cin >> res.centroid_wrt_kinect.x
             >> res.centroid_wrt_kinect.y
             >> res.centroid_wrt_kinect.z;


    std::cout <<"\nEnter normal(x, y, z) of object wrt kinect\n";

    std::cin >> res.normal_wrt_kinect.x
             >> res.normal_wrt_kinect.y
             >> res.normal_wrt_kinect.z;


    std::cout <<"\nEnter axis(x, y, z) of object wrt kinect\n";

    std::cin >> res.axis_wrt_kinect.x
             >> res.axis_wrt_kinect.y
             >> res.axis_wrt_kinect.z;


    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_points");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_kinect_points_topic", set_val);

    ros::spin();

    return 0;
}
