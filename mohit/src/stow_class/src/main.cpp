#include <iostream>

#include "stowing.h"
#include "bin.h"

#include <signal.h>


void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << "\n";

    exit(0);
}

int main(int argc, char** argv)
{

    ROS_INFO("Initializing IITK Stow controller");
    ROS_DEBUG("DEBUG");

    ros::init(argc, argv, "test_actuator");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    Stowing_ARC_2017 move_planner("robo_arm", "RRTConnectkConfigDefault", nh);

    Bin_ARC_2017 bin(0.85, 0.56, 0.08, 0.08);

    bool success;

    int max_attempts = 10;

    int attempts;

    move_planner.read_joson_file();

    int bin_number = 1;
    int bin_location1 = 0;
    int bin_location2 = 0;

    double bin_location[3];
    double *bin_location_pointer;



    move_planner.nh.getParam("/ARC17_AVAILABLE_IDS" , move_planner.available_IDs);
    //move_planner.nh.getParam("/ARC17_AVAILABLE_IDS" , move_planner.temp_available_IDs);


    ros::Subscriber force_sub = nh.subscribe("/wrench",  2, &Stowing_ARC_2017::force_Call_back, &move_planner);

    ros::Subscriber flow_sensor_sub = nh.subscribe<rosserial_arduino::Adc>
                ("/iitktcs/arduino/flow_Sensor", 100, &Stowing_ARC_2017::flow_sensor_call_back, &move_planner);



    move_planner.tote_corner_points();




    while(ros::ok())
    {




        signal(SIGINT, signal_callback_handler);




        attempts = max_attempts;
        std::cout <<"\n\n*****************moving towards tote*****************\n\n";
        do
        {
            usleep(100000);
            success = move_planner.move_to_tote();
            if(success)
                std::cout << "tote_motion_done\n";
            else
            {
                move_planner.clear_protective_stop();
                std::cout <<"*****GHANTA TOTE MOTION!!!******\n";
            }

            attempts = attempts - 1;

        }while(!success && attempts);





        attempts = max_attempts;
        std::cout <<"\n\n*****************calling vision service*****************\n\n";
        do
        {
            usleep(100000);
            success = move_planner.call_vision_service_bajrangi();
            if(success)
                std::cout << "vision_service_completed\n";
            else
                std::cout <<"*****GHANTA VISION SERVICE!!!******\n";

            attempts = attempts - 1;

        }while(!success && attempts);


        if(success == false)
            continue;





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





        attempts = max_attempts;
        std::cout <<"\n\n*****************planning to reach centroid via normal*****************\n\n";
        do
        {
            usleep(100000);
            success = move_planner.plan_to_reach_centroid_via_normal();
            if(success)
                std::cout <<"plan_found\n";
            else
                move_planner.modify_normal();

            attempts = attempts - 1;

        }while(!success && attempts);





        if(move_planner.weight[move_planner.object_ID - 1] == "heavy" )
        {

            attempts = max_attempts;
            std::cout <<"\n\n*****************reaching normal*****************\n\n";
            do
            {
                usleep(100000);
                success = move_planner.reach_normal();
                if(success)
                    std::cout <<"normal_reached\n";
                else
                    move_planner.clear_protective_stop();

                attempts = attempts - 1;

            }while(!success && attempts);





            attempts = max_attempts;
            std::cout <<"\n\n*****************reaching centroid*****************\n\n";
            do
            {
                usleep(100000);
                success = move_planner.reach_centroid();
                if(success)
                    std::cout <<"centroid_reached\n";
                else
                    move_planner.clear_protective_stop();

                attempts = attempts - 1;

            }while(!success && attempts);

        }
        else
        {

            attempts = max_attempts;
            std::cout <<"\n\n*****************reaching centroid via normal*****************\n\n";
            do
            {
                usleep(100000);
                success = move_planner.move_towards_centroid_via_normal();
                if(success)
                    std::cout <<"centroid_reached\n";
                else
                    move_planner.clear_protective_stop();

                attempts = attempts - 1;

            }while(!success && attempts);

        }




        bool success = move_planner.check_feedback();

        if(success == false)
            continue;




        attempts = max_attempts;
        std::cout <<"\n\n*****************lifting object*****************\n\n";
        do
        {
            usleep(100000);
            success = move_planner.lift_object();
            if(success)
                std::cout <<"object_lifted\n";
            else
                move_planner.clear_protective_stop();

            attempts = attempts - 1;

        }while(!success && attempts);





        //        double bin_location[3] = {0.313, 0.453, 0.10};

        int centre = 0;

        if(move_planner.object_ID == 2)
            centre = 1;

        switch (bin_number)
        {
        case 1:
            bin_location_pointer = bin.set_bin_number_and_bin_location(bin_number, bin_location1, centre);
            bin_location1 = bin_location1 + 1;
            bin_location1 = bin_location1 % 4;

            break;

        case 2:
            bin_location_pointer = bin.set_bin_number_and_bin_location(bin_number, bin_location2, centre);
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
        std::cout <<"\n\n*****************reaching_bin*****************\n\n";
        do
        {
            usleep(100000);
            success = move_planner.reach_bin(bin_location);
            if(success)
                std::cout <<"bin_lifted\n";
            else
                move_planner.clear_protective_stop();

            attempts = attempts - 1;

        }while(!success && attempts);






        attempts = max_attempts;
        std::cout <<"\n\n*****************dropping object*****************\n\n";
        do
        {
            usleep(100000);
            success = move_planner.drop_object();
            if(success)
                std::cout <<"object_droped\n";
            else
                move_planner.clear_protective_stop();

            attempts = attempts - 1;

        }while(!success && attempts);





        attempts = max_attempts;
        std::cout <<"\n\n*****************moving_up*****************\n\n";
        do
        {
            usleep(100000);
            success = move_planner.move_upwards();
            if(success)
                std::cout <<"moved_up\n";
            else
                move_planner.clear_protective_stop();

            attempts = attempts - 1;

        }while(!success && attempts);





        move_planner.write_jason_file(bin_number - 1);





    }


    return 0;
}
