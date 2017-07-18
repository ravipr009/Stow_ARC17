
#include <json_maker/json_helper_17.h>
#include <json_maker/get_bin_object.h>
#include <json_maker/write_pick_status.h>
#include <json_maker/write_stow_data.h>
#include <json_maker/stowToteContents.h>
#include <boost/algorithm/string.hpp>

class JSON_MAKER{


private:
protected:
public:

    vector< vector<string> > bin_contents;
    vector<string> rp_items;
    vector<string> tote_contents;
    int iteration ;
    std::vector<string> model_names;



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

    ros::param::get("/ARC17_OBJECT_NAMES", model_names);




#if(STOWING_TASK)
    st_picked =  vector<bool> (int(20), false);
    st_bin = vector<int>(int(20), -1);
    string path = "/home/ravi/Desktop/APC_stow/stow_ws/src/json_maker/data/examples/item_location_file.json";
    write_stow_data_service = nh.advertiseService("/write_stow_data_service",&JSON_MAKER::write_stow_data_service_callback,this);
    tote_contents_service = nh.advertiseService("/tote_contents/data",&JSON_MAKER::tote_contents_callback,this);
#endif


    readJsonFile(path, bin_contents, rp_items, tote_contents);

}


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
        std::string A;std::string B;
//        if(strcmp(tote_contents[i].c_str(),model_names[obj_id].c_str()) == 0)
        A=tote_contents[i];B=model_names[obj_id];
         boost::to_lower(A);boost::to_lower(B);


        if(A.compare(B) == 0)

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
    std::vector<int> competetion_set;


    cout << "Entered tote contents return call back " << endl;
    for(int i=0; i<this->tote_contents.size(); i++)
    {
        for(int j=0;j<model_names.size();j++)
        {
            std::string A;std::string B;
            A=this->tote_contents[i];B=model_names[j];
             boost::to_lower(A);boost::to_lower(B);
            if(A.compare(B) == 0)
            {
                int priority_obj_id = j+1;
//                std::cout<<"tote objects id : "<<priority_obj_id<<"\n";
                competetion_set.push_back(priority_obj_id);
                res.tote_contents.data.push_back(priority_obj_id);// push the object index
            }
        }
    }
    std::sort(competetion_set.begin(),competetion_set.end());
    ros::param::set("/ARC17_AVAILABLE_IDS",competetion_set);
    competetion_set.clear();


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
