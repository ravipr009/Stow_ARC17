
#include <json_maker/json_helper.h>
#include <json_maker/get_bin_object.h>
#include <json_maker/write_pick_status.h>
#include <json_maker/write_stow_data.h>
#include <json_maker/stowToteContents.h>

class JSON_MAKER{


private:
protected:
public:

    vector< vector<string> > bin_contents;
    vector<string> rp_items;
    vector<string> tote_contents;
    int iteration ;

#if(PICKING_TASK)
    vector<bool> rp_picked;
    ros::ServiceServer pick_obj_service;
    ros::ServiceServer pick_status_write_service;
    bool pick_object_callback(json_maker::get_bin_objectRequest &req,json_maker::get_bin_objectResponse &res);
    bool pick_status_write_callback(json_maker::write_pick_statusRequest &req,json_maker::write_pick_statusResponse &res);

#endif

#if(STOWING_TASK)
    vector<bool> st_picked;
    vector<int> st_bin;
    ros::ServiceServer write_stow_data_service;
    ros::ServiceServer tote_contents_service;
    bool write_stow_data_service_callback(json_maker::write_stow_dataRequest &req,json_maker::write_stow_dataResponse &res);
    bool tote_contents_callback(json_maker::stowToteContents::Request &req, json_maker::stowToteContents::Response &res);
#endif

    JSON_MAKER();
    ~JSON_MAKER();

};

JSON_MAKER::JSON_MAKER()
{

    iteration = 0;
    ros::NodeHandle nh;
    bin_contents = vector< vector<string> >(NO_BINS);

#if(PICKING_TASK)
    rp_items = vector<string>(NO_BINS);
    rp_picked = vector<bool>(NO_BINS, false);
    string path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/rp_input.json";
    pick_obj_service = nh.advertiseService("/pick_object_service",&JSON_MAKER::pick_object_callback,this);
    pick_status_write_service = nh.advertiseService("/pick_object_status_service",&JSON_MAKER::pick_status_write_callback,this);
#endif

#if(STOWING_TASK)
    st_picked =  vector<bool> (int(20), false);
    st_bin = vector<int>(int(20), -1);
    string path = "/home/ravi/Desktop/APC_stow/stow_ws/src/json_maker/data/examples/item_location_file.json";
    write_stow_data_service = nh.advertiseService("/write_stow_data_service",&JSON_MAKER::write_stow_data_service_callback,this);
    tote_contents_service = nh.advertiseService("/tote_contents/data",&JSON_MAKER::tote_contents_callback,this);
#endif


    readJsonFile(path, bin_contents, rp_items, tote_contents);

}
#if(PICKING_TASK)
bool JSON_MAKER::pick_object_callback(json_maker::get_bin_objectRequest &req,json_maker::get_bin_objectResponse &res)
{

    int bin_map_array[12] = {1,4,7,10,11,8,5,2,0,3,6,9};

    cout << "Iteration number \t" << iteration << endl;
    if(iteration == 12)
        iteration = 0;

    int priority_obj_id = 0 ;

    for(int i=0;i<39;i++)
    {
//        cout << rp_items[iteration] << "\t" << model_names[i] << endl;
        if(rp_items[bin_map_array[iteration]].compare(model_names[i]) == 0)
        {
            priority_obj_id = i+1;
//            cout << "priority_obejct id \t" << priority_obj_id << "\t" << endl;
            break;
        }
    }

    int priority_bin_id= bin_map_array[iteration];

    res.bin_num.data = priority_bin_id;
    res.obj_id.data = priority_obj_id;

    iteration++;

    return true;
}

bool JSON_MAKER::pick_status_write_callback(json_maker::write_pick_statusRequest &req, json_maker::write_pick_statusResponse &res)
{
// in write_pick_status service call obj_id refers to the bin number, WE HAVE RETAINED obj_id TO REPRESENT BIN NUMBER TO AVOID MAKING CHANGES TO SRV FILE
    if(req.pick_status.data)
        rp_picked[req.obj_id.data] = true;
    else
        rp_picked[req.obj_id.data] = false;

    string rp_out_path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/rp_out_test.json";
    writeRPJsonFile(rp_out_path,bin_contents,rp_items,rp_picked);

    return true;
}

#endif

#if(STOWING_TASK)
bool JSON_MAKER::write_stow_data_service_callback(json_maker::write_stow_dataRequest &req, json_maker::write_stow_dataResponse &res)
{

    // Compare object id returned by the services with the objects in the json and set the index accordingly.

    int obj_id_in_json;
    int obj_id = req.obj_id.data-1;
    bool found_id = false;
    // Run a for loop and the object returned from apc_controller lies in req.obj_id.data. Compare it with array in json file data
    // and store its index in obj_id_in_json variable
    for(int i=0; i<tote_contents.size(); i++)
    {
        if(strcmp(tote_contents[i].c_str(),model_names[obj_id].c_str()) == 0)
        {
            obj_id_in_json = i;
            found_id = true;
            break;
        }
    }

    if(found_id)
    {
        st_picked[obj_id_in_json] = true;
        st_bin[obj_id_in_json] = req.bin_id.data;

        string st_out_path = "/home/ravi/Desktop/APC_stow/stow_ws/src/json_maker/data/examples/hiiiiiiiiiii.json";
        writeSTJsonFile(st_out_path,bin_contents,tote_contents,st_picked,st_bin);
    }
    return true;
}

// Service call back function to send the tote contents object ids
bool JSON_MAKER::tote_contents_callback(json_maker::stowToteContents::Request &req, json_maker::stowToteContents::Response &res)
{
    cout << "Entered tote contents return call back " << endl;
    for(int i=0; i<this->tote_contents.size(); i++)
    {
        for(int j=0;j<39;j++)
        {
            if(tote_contents[i].compare(model_names[j]) == 0)
            {
                int priority_obj_id = j+1;
                res.tote_contents.data.push_back(priority_obj_id);// push the object index
            }
        }
    }
    return true;
}

#endif

JSON_MAKER::~JSON_MAKER()
{

}


int main(int argc, char **argv)
{

    ros::init(argc,argv,"json_reader_Writer");

    JSON_MAKER *json_maker = new JSON_MAKER();

    ros::spin();



    return 0;
}
