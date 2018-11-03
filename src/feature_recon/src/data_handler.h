#ifndef DATA_HANDLER
#define DATA_HANDLER

#include "ros/ros.h"
#include "human_structs.h"
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>

const string TXT_EXTENSION = ".txt";

bool save_human_data(human_data human, string path){
  string line;

  // Writing human id to file
  line = "Human_id:";
  line.append(to_string(human.id));
  line.append("\n");

  // Writing encoding to file
  line.append("Encoding:");
  for (int i = 0; i < 128; i++) {
    line.append(to_string(human.encoding[i]));
    line.append(",");
  }
  line.append("\n");

  // Saving all the limbs
  ROS_INFO("Human %d - Saving %d limbs", human.id, (int)human.limbs.size());
  for (int i = 0; i < human.limbs.size(); i++) {
    // Writing limb id to file
    line.append("Limb_id:");
    line.append(to_string(human.limbs[i].id));
    line.append("\n");

    //Writing limb name to file
    line.append("Limb_name:");
    line.append(human.limbs[i].name);
    line.append("\n");

    //Writing info list to files
    line.append("Info_list:");
    for (int j = 0; j < 5; j++) {
      if(j < human.limbs[i].info_list.size()){
        line.append(";");
        line.append(to_string(human.limbs[i].info_list[j].length));
        line.append(",");
        line.append(to_string(human.limbs[i].info_list[j].joint_confidence));
      }else{
        line.append(";0,0");
      }
    }
    line.append("\n");

    // Writing avg info to file
    line.append("Avg_info:");
    line.append(to_string(human.limbs[i].avg_info.length));
    line.append(",");
    line.append(to_string(human.limbs[i].avg_info.joint_confidence));
    line.append("\n");
  }

  ofstream human_data;
  path.append(to_string(human.id));
  path.append(TXT_EXTENSION);
  human_data.open(path, ios::trunc);

  if(!human_data.is_open()){
    return false;
  }

  human_data << line;
  human_data.close();
  return true;
}


bool load_human_data(string path){
  ifstream human_file;
  human_data human;
  path.append(TXT_EXTENSION);

  human_file.open(path);

  if(!human_file.is_open()){
    return false;
  }

    string line;
    // Getting the name of the human
    getline(human_file, line, ':');
    human_file >> human.id;
    ROS_INFO("Human(%d) - Loadning human", human.id);
    human_file.get();

    // Getting the encodings of the human
    getline(human_file, line, ':');
    ROS_INFO("Human(%d) - Loadning encodings", human.id);
    for (int i = 0; i < 5; i++) {
      getline(human_file, line, ',');
      human.encoding.push_back(stod(line));
    }
    human_file.get();
    ROS_INFO("Human(%d) - Encodings loaded", human.id);

    // Getting limb info
    while(human_file){
    ROS_INFO("Human(%d) - Adding bodypart", human.id);
    body_limb limb;
      // Getting limb id
      getline(human_file, line, ':');
      human_file >> limb.id;
      human_file.get();

      //Getting limb name
      getline(human_file, line, ':');
      human_file >> limb.name;
      human_file.get();
      human.limbs.push_back(limb);

      // Getting list of limb info
      body_limb_info limb_info;
      getline(human_file, line, ':');
      for (int i = 0; i < 5; i++) {
        getline(human_file, line, ';');
        getline(human_file, line, ',');
        limb_info.length = stod(line);
        human_file >> limb_info.joint_confidence;
        if(limb_info.length && limb_info.joint_confidence){
          human.limbs[human.limbs.size()-1].info_list.push_back(limb_info);
        }
      }
      ROS_INFO("Human(%d) - Info list of size %d was added",human.id, (int)human.limbs[human.limbs.size()-1].info_list.size());
      human_file.get();


      //Getting avg info;
      getline(human_file, line, ':');
      getline(human_file, line, ',');
      limb_info.length = stod(line);
      human_file >> limb_info.joint_confidence;human.limbs.size()-1;
      human.limbs[human.limbs.size()-1].avg_info = limb_info;
      ROS_INFO("Human(%d) - Avg info with length: %f and confidence: %f was added", human.id, limb_info.length, limb_info.joint_confidence);
      human_file.get();
      human_file.get();
      ROS_INFO("Human(%d) - Added body limb: %s", human.id, human.limbs[human.limbs.size()-1].name.c_str());

      // Breaks if no more limbs to add
      if(!human_file){
        ROS_INFO("Human(%d) - Total of %d limb(s) were added", human.id, (int)human.limbs.size());
        ROS_INFO("Human(%d) - Human done loadning", human.id);
        break;
      }
    }
  human_file.close();
  return true;
}

#endif
