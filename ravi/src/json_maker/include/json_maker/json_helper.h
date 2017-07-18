#ifndef JSON_HELPER_H
#define JSON_HELPER_H

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "json/json.h"
#include <std_msgs/String.h>
#include <ros/package.h>


using namespace std;
#define NO_BINS 2
#define PICKING_TASK    0
#define STOWING_TASK    1

string bin_names[] = {"bin_A","bin_B"};

std::string model_names[] = {
    "Toilet_Brush"             , "Avery_Binder",
    "Balloons"                 , "Band_Aid_Tape",
    "Bath_Sponge"              , "Black_Fashion_Gloves",
    "Burts_Bees_Baby_Wipes"    , "Colgate_Toothbrush_4PK",
    "Composition_Book"         , "Crayons",
    "Duct_Tape"                , "Epsom_Salts",
    "Expo_Eraser"              , "Fiskars_Scissors",
    "Flashlight"               , "Glue_Sticks",
    "Hand_Weight"              , "Hanes_Socks",
    "Hinged_Ruled_Index_Cards" , "Ice_Cube_Tray",
    "Irish_Spring_Soap"         , "Laugh_Out_Loud_Jokes",
    "Marbles"                   , "Measuring_Spoons",
    "Mesh_Cup"                  , "Mouse_Traps"	,
    "Pie_Plates"                , "Plastic_Wine_Glass",
    "Poland_Spring_Water"       , "Reynolds_Wrap",
    "Robots_DVD"                , "Robots_Everywhere",
    "Scotch_Sponges"            , "Speed_Stick",
    "White_Facecloth"           , "Table_Cloth",
    "Tennis_Ball_Container"     , "Ticonderoga_Pencils",
    "Tissue_Box"                ,"Windex"
};

bool readJsonFile(string &file_path, vector< vector<string> > &rack_contents, vector<string> &rp_objects,
                  vector<string> &tote_items)
{

    std::cout<<"helllllllllllllllo\n";
    std::ifstream ifile(file_path.c_str());
    Json::Reader reader;
    Json::Value root;
    if (ifile != NULL && reader.parse(ifile, root))
    {
        if(root.isMember("bin_contents"))
        {
            const Json::Value bin_contents = root["bin_contents"];

            for(int j=0; j<rack_contents.size(); j++)
            {
                string bin = bin_names[j];
                if(bin_contents.isMember(bin.c_str()))
                {
                    //                    cout << bin.c_str() << ":\n";
                    vector<string> object_names;
                    for (unsigned int i = 0; i < bin_contents[bin.c_str()].size(); i++)
                    {
                        //                        cout << bin_contents[bin.c_str()][i].asString() << endl;
                        object_names.push_back(bin_contents[bin.c_str()][i].asString());
                    }
                    //                    cout << endl;
                    rack_contents[j] = object_names;
                    std::cout<<"ravi   \n"<<object_names.size()<<"\n";

                }
            }
        }

        if(root.isMember("work_order"))
        {
            const Json::Value order = root["work_order"];
            //            cout << "work order\n";

            for(int j=0; j<order.size(); j++)
            {
                string bin = order[j]["bin"].asString();
                string item = order[j]["item"].asString();

                for(int k=0; k<NO_BINS; k++)
                {
                    if(strcmp(bin.c_str(),bin_names[k].c_str()) == 0)
                    {
                        //                        cout << bin.c_str() << ": ";
                        //                        cout << item.c_str() << endl;
                        rp_objects[k] = item;
                    }
                }
            }
        }
        if(root.isMember("tote_contents"))
        {
            for(int i=0; i<root["tote_contents"].size(); i++)
            {
                tote_items.push_back(root["tote_contents"][i].asString());
            }
        }
        return true;
    }
    else
    {
        if(ifile == NULL)
            cerr << "Failed to open JSON file: " << file_path << endl;
        else
            cerr << "Failed to parse JSON file: " << file_path << endl;
        return false;
    }
}

// file_path: path to the output file
// bin_items: array of bins(12) with each containing array of names of items in the respective bin
// tote_items: array of items to be picked from tote
// picked: array of bool indicating whether tote object as indexed in tote_items is picked or not
// dest_bin: array of bin index in which the tote object is placed
bool writeSTJsonFile(string &file_path, vector< vector<string> > &bin_items, vector<string> &tote_items,
                     vector<bool> &picked, vector<int> &dest_bin)
{
    Json::Value root;
    vector<bool> bin_empty(2, true);// In case bin is empty, flag is required to write empty array to json file

    if(bin_items.size() == NO_BINS)
    {
        // First put all the existing items in the rack into root
        for(int i=0; i<NO_BINS; i++)
        {
            for(int j=0; j<bin_items[i].size(); j++)
            {
                root["bin_contents"][bin_names[i]].append(bin_items[i][j]);
                bin_empty[i] = false;
            }
        }


            for(int i=0; i<tote_items.size(); i++)
            {
                // if the tote item is picked then push the item into bin contents else tote contents
                if(picked[i])
                {
                    int bin = dest_bin[i];
                    root["bin_contents"][bin_names[bin]].append(tote_items[i]);
                    bin_empty[bin] = false;
                }
                else
                {
                    root["tote_contents"].append(tote_items[i]);
                }
            }

            // put empty array if any bin content is empty
            for(int i=0; i<NO_BINS; i++)
            {
                if(bin_empty[i])
                    root["bin_contents"][bin_names[i]] = Json::Value (Json::arrayValue);
            }



    }
    else
    {
        cerr << "No of bins given is less that 2" << endl;
        return false;
    }

    bool all_picked = true; // to check whether all tote contents are picked, tote is empty
    for(int i=0; i<picked.size(); i++)
    {
        if(!picked[i])
        {
            all_picked = false;
            break;
        }
    }
    if(all_picked)
        root["tote_contents"] = Json::Value (Json::arrayValue);

    std::ofstream file_id;
    file_id.open(file_path.c_str());

    if(!file_id.is_open())
    {
        cerr << "Error opening file in json" << endl;
        exit(0);
    }

    file_id << root;

    file_id.close();


    return true;
}

// file_path: path to the output file
// bin_items: array of bins(12) with each containing array of names of items in the respective bin
// pick_items: array of items to be picked from each bin. Indexed with the bin number.
// picked: array of bool indicating whether desired object from the respective bin is picked or not
bool writeRPJsonFile(string &file_path, vector< vector<string> > &bin_items, vector<string> &pick_items, vector<bool> &picked)
{
    Json::Value root;

    if(bin_items.size() == NO_BINS && pick_items.size() == NO_BINS)
    {
        int tote_size = 0;

        for(int i=0; i<NO_BINS; i++)
        {
            int bin_size = 0;
            bool item_found = false;// Because multiple copies of same item could be there
            for(int j=0; j<bin_items[i].size(); j++)
            {
                // if bin item is the to be picked item then put to tote if picked
                if(strcmp(bin_items[i][j].c_str(), pick_items[i].c_str()) == 0 && picked[i] && !item_found)
                {
                    item_found = true;
                    root["tote_contents"][tote_size] = bin_items[i][j];
                    tote_size++;
//                    cout << "tote " << i << ": " << bin_items[i][j] << endl;
                }
                else
                {
                    root["bin_contents"][bin_names[i]][bin_size] = bin_items[i][j];
                    bin_size++;
//                    cout << "bin " << i << ": " << bin_items[i][j] << endl;
                }
            }
            if(bin_size == 0)
                root["bin_contents"][bin_names[i]] = Json::Value (Json::arrayValue);

        }
    }
    else
    {
        cerr << "No of bins not equal to 2" << endl;
        return false;
    }

    std::ofstream file_id;
    file_id.open(file_path.c_str());

    if(!file_id.is_open())
    {
        cerr << "Error opening file in json" << endl;
        exit(0);
    }

    file_id << root;

    file_id.close();


    return true;
}



#endif // JSON_HELPER_H
