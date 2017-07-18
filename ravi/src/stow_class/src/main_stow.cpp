#include <iostream>

#include "support.h"
#include "bin.h"

#include <signal.h>


void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << "\n";

    exit(0);
}

int main(int argc, char** argv) {

    ROS_INFO("Initializing IITK Stow controller");
    ROS_DEBUG("DEBUG");

    ros::init(argc, argv, "test_actuator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    Stowing_ARC_2017 move_planner("robo_arm", "RRTConnectkConfigDefault", nh);

    char a;
    Bin_ARC_2017 bin(0.85, 0.56, 0.00, 0.00);

    bool success = false;

    int max_attempts = 10;

    int attempts = 5;

    move_planner.read_joson_file();

    int bin_number = 1;
    int bin_location1 = 0;
    int bin_location2 = 0;

    double bin_location[3];
    double *bin_location_pointer;






    while(ros::ok())
    {
        signal(SIGINT, signal_callback_handler);


        //                std::cin >> a;


        attempts = max_attempts;
        std::cout <<"\n\n*****************moving towards tote*****************\n\n";
        do
        {
            success = move_planner.move_to_tote();
            if(success)
                std::cout << "tote_motion_done\n";
            else
                std::cout <<"*****GHANTA TOTE MOTION!!!******\n";

            attempts = attempts - 1;

            std::cout <<attempts<<"\n";

        }while(!success && attempts);


        //                std::cin >> a;



        attempts = max_attempts;
        std::cout <<"\n\n*****************calling vision service*****************\n\n";
        do
        {
            success = move_planner.call_vision_service();
            if(success)
                std::cout << "vision_service_completed\n";
            else
                std::cout <<"*****GHANTA VISION SERVICE!!!******\n";

            attempts = attempts - 1;

        }while(!success && attempts);


        //                std::cin >> a;



        attempts = max_attempts;
        std::cout <<"\n\n*****************visualizing points*****************\n\n";
        do
        {
            success = move_planner.visualize_points();
            if(success)
                std::cout << "visualize_completed\n";
            else
                std::cout <<"*****GHANTA VISUALIZE!!!******\n";

            attempts = attempts - 1;

        }while(!success && attempts);



        //                std::cin >> a;



        attempts = max_attempts;
        std::cout <<"\n\n*****************reaching centroid via normal*****************\n\n";
        do
        {
            success = move_planner.plan_to_reach_centroid_via_normal();
            if(success)
                move_planner.move_towards_centroid_via_normal();
            else
                move_planner.modify_normal();

            attempts = attempts - 1;

        }while(!success && attempts);



        usleep(100000);



        attempts = max_attempts;
        std::cout <<"\n\n*****************lifting object*****************\n\n";
        do
        {
            success = move_planner.lift_object();
            if(success)
                std::cout <<"\n object_lifted\n";
            else
                std::cout <<"*****GHANTA LIFT!!!******\n";

            attempts = attempts - 1;

        }while(!success && attempts);



        //        std::cin >> a;



        usleep(100000);


        switch (bin_number)
        {
        case 1:
            bin_location_pointer = bin.set_bin_number_and_bin_location(bin_number, bin_location1);
            bin_location1 = bin_location1 + 1;
            bin_location1 = bin_location1 % 4;

            break;

        case 2:
            bin_location_pointer = bin.set_bin_number_and_bin_location(bin_number, bin_location2);
            bin_location2 = bin_location2 + 1;
            bin_location2 = bin_location2 % 4;

            break;
        }

        bin_number = bin_number + 1;
        if(bin_number == 3)
            bin_number = 1;


        bin_location[0] = *bin_location_pointer;
        bin_location[1] = *(bin_location_pointer + 1);
        bin_location[2] = *(bin_location_pointer + 2);




        attempts = max_attempts;
        std::cout <<"\n\n*****************reaching bin*****************\n\n";
        do
        {
            success = move_planner.reach_bin(bin_location);
            if(success)
                std::cout <<"\n bin_reached\n";
            else
                std::cout <<"*****GHANTA BIN REACHED!!!******\n";

            attempts = attempts - 1;

        }while(!success && attempts);


        //                std::cin >> a;



        usleep(100000);


        attempts = max_attempts;
        std::cout <<"\n\n*****************droping object*****************\n\n";
        do
        {
            success = move_planner.drop_object();
            if(success)
                std::cout <<"\n object_dropped\n";
            else
                std::cout <<"*****GHANTA OBJECT DROPPED!!!******\n";

            attempts = attempts - 1;

        }while(!success && attempts);


        //                std::cin >> a;



        usleep(100000);


        attempts = max_attempts;
        std::cout <<"\n\n*****************moving upwards*****************\n\n";
        do
        {
            success = move_planner.move_upwards();
            if(success)
                std::cout <<"\n moved up";
            else
                std::cout <<"*****GHANTA MOVED UP!!!******\n";

            attempts = attempts - 1;

        }while(!success && attempts);


        usleep(100000);

        move_planner.write_jason_file(bin_number -1);
    }


    return 0;
}
