#include <iostream>
#include<ros/ros.h>
#include<signal.h>
#include<apc_nozel/write_stow_data.h>
#include<apc_nozel/stowToteContents.h>
#include<apc_nozel/CheckClearProtectiveStop.h>



#define NO_OF_TOTE_OBJS 20

using namespace std;

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"stow_controller");

    ros::NodeHandle nh;





    ros::ServiceClient stowToteContentsService = nh.serviceClient<apc_nozel::stowToteContents>("/tote_contents/data");
    ros::ServiceClient write_stow_data_service = nh.serviceClient<apc_nozel::write_stow_data>("/write_stow_data_service");
ros::ServiceClient protective_clear_service = nh.serviceClient<apc_nozel::CheckClearProtectiveStop>("/iitktcs/ur_modern_driver/check_protective_stop");

    vector<int> tote_obj(NO_OF_TOTE_OBJS);
    vector<bool> tote_obj_picked(NO_OF_TOTE_OBJS);

    for(int i=0; i<tote_obj.size(); i++)
    {

            tote_obj_picked[i] = false;

    }


    std::vector<string> model_names;
    ros::param::get("/ARC17_OBJECT_NAMES",model_names);


    std::string bin_names[] = {"bin_A", "bin_B"};


    bool service_status_flag = false;

    int bin_iteration = 0;

    while(ros::ok())
    {
        signal(SIGINT, signal_callback_handler);
        int input;
        cout << "\n Press 2 to read the tote contents"
             << "\n Press 3 to test protective clear"


             << "\n Press 6 to test writing of stow task json file"

             << endl;
        cin >> input;
        getchar();

        switch(input)
        {

        case 2:
        {
            // call service to get the tote contents
            apc_nozel::stowToteContents tote_contents;
            if(stowToteContentsService.call(tote_contents))
            {
                cout << "Tote contents:\n";
                for(int i=0; i<tote_contents.response.tote_contents.data.size(); i++)
                {
                    tote_obj[i] = tote_contents.response.tote_contents.data[i];
                    tote_obj_picked[i] = false;

                    cout << tote_obj[i] << "    :      " << model_names[tote_obj[i]-1] << endl;
                }
                cout << "Got the tote contents" << endl;
            }
            else
                cout << "Failed to call tote object content service " << endl;
        }
            break;


        case 3:
        {
            // call service to get the tote contents
            apc_nozel::CheckClearProtectiveStop test_clear;
            test_clear.request.check.data = true;
            if(protective_clear_service.call(test_clear))
            {
                cout<<"\n cleared status : "<<int(test_clear.response.success.data)<<"\n";
            }
            else
            {
                ROS_ERROR("failed to clear protective stop");
            }


        }
            break;



        case 6:
        {
            for(int i=0; i<tote_obj.size(); i++)
            {
                if(!tote_obj_picked[i])
                    cout <<tote_obj[i] << "    :      "<< model_names[tote_obj[i]-1].c_str() << endl;
            }
            cout << "Enter the index of obj: " << endl;
            int idx;
            cin >> idx;

            for(int i=0; i<tote_obj.size(); i++)
            {
                if(tote_obj[i] == idx)
                {
                    tote_obj_picked[i] = true;
                    cout << model_names[tote_obj[i]-1] << " is picked" << endl;
                    apc_nozel::write_stow_data write_stow_data;
                    std::cout<<"enter the destination bin : \n";
                    int d;
                    std::cin>>d;
                    write_stow_data.request.bin_id.data = d;//............TODO  --------> valley detection   bin_idx;
                    write_stow_data.request.obj_id.data = tote_obj[i];


                    if(write_stow_data_service.call(write_stow_data))
                    {
                        cout << "Gollum Wrote into file " << tote_obj[i] << " is put into " << (i%2) << " bin" << endl;
                    }
                    else
                        cout << "Failed to call write tote object data service" << endl;

                    break;
                }
            }
        }
            break;




        }// switch bracket close
    }

    return 0;
}
