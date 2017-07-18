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

string bin_id[] = {"A", "B"};



bool readJsonFile(string &file_path, vector< vector<string> > &bin_contents, vector<string> &rp_objects,
                  vector<string> &tote_items)
{
    std::ifstream ifile(file_path.c_str());
    Json::Reader reader;
    Json::Value root;


    if (ifile != NULL && reader.parse(ifile, root))
    {
        if(root.isMember("bins"))
        {
            const Json::Value bins = root["bins"];


            for(unsigned int i=0; i<bins.size();i++)
            {


                //                std::cout<<"Bin name in" << i<<"th bin : "<<bins[i]["bin_id"].asString()<<std::endl;

                vector<string> strings;
                for(unsigned int index = 0; index<bins[i]["contents"].size(); ++index)
                {

                    //                    std::cout<<"Element "<<index<<" in contents: "<<bins[i]["contents"][index].asString()<<std::endl;

                    strings.push_back(bins[i]["contents"][index].asString());
                }

                bin_contents.push_back(strings);

            }

        }

        //std::cout<<"\n"<<bin_contents.size()<<"\n"<<bin_contents[0].size()<<"\n"<<bin_contents[1].size()<<"\n";
        const Json::Value bins = root["bins"];

//        for(unsigned t=0; t<bin_contents.size();t++)
//        {
//            std::cout<<"Contents in bin_id : "<<bins[t]["bin_id"].asString()<<std::endl;
//            for(int j=0; j<bin_contents[t].size(); j++){
//                cout << bin_contents[t][j].c_str() << "\n ";}

//            cout << endl;
//        }

        std::cout<<"\n Contents in Tote : \n";
        if(root.isMember("tote"))
        {
            const Json::Value tote = root["tote"];

            for(int i=0; i<tote["contents"].size(); i++)
            {
                tote_items.push_back(tote["contents"][i].asString());
            }
        }

        for (unsigned i=0; i<tote_items.size();i++)
            cout<<tote_items[i].c_str()<<"\n";


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
    vector<bool> bin_empty(2,true);

    root["bins"]= Json::Value (Json::arrayValue);
    root["bins"][0]["bin_id"]="A";
//    root["bins"][0]["contents"]=Json::Value (Json::arrayValue);

    root["bins"][1]["bin_id"]="B";
//    root["bins"][1]["contents"]=Json::Value (Json::arrayValue);

        // First put all the existing items in the rack into root
        for(int i=0; i<NO_BINS; i++)
        {
            for(int j=0; j<bin_items[i].size(); j++)
            {
                root["bins"][i]["contents"].append(bin_items[i][j]);
                bin_empty[i] = false;
            }
        }


        for(int i=0; i<tote_items.size(); i++)
        {
            // if the tote item is picked then push the item into bin contents else tote contents
            if(picked[i])
            {
                int bin = dest_bin[i];
                root["bins"][bin]["contents"].append(tote_items[i]);
                bin_empty[bin] = false;
            }
            else
            {
                root["tote"]["contents"].append(tote_items[i]);
            }
        }

        // put empty array if any bin content is empty
        for(int i=0; i<NO_BINS; i++)
        {
            if(bin_empty[i])
                 root["bins"][i]["contents"] = Json::Value (Json::arrayValue);
        }


    root["boxes"]=Json::Value (Json::arrayValue);

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
     root["tote"]["contents"]=Json::Value (Json::arrayValue);

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
