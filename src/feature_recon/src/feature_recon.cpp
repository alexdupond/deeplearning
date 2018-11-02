#include "ros/ros.h"
#include "std_msgs/String.h"
#include "human_structs.h"
#include "feature_extractor/feature_extractor.h"
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>


using namespace std;

void load_human_data(string path);

int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle nh;
FeatureExtractor FeatExt(nh);

ros::Rate loop_rate(10);
string path = "/home/alexdupond/deeplearning/src/feature_recon/src/human_data/human_1.txt";
load_human_data(path);

while (ros::ok()){


  /*
  if((int)FeatExt.getFacelessHumans().size())
  ROS_INFO("Number of humans without faces: %d", (int)FeatExt.getFacelessHumans().size());

  if((int)FeatExt.getCompleteHumans().size())
    ROS_INFO("Number of humans WHIT faces: %d", (int)FeatExt.getCompleteHumans().size());
*/
  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}

void load_human_data(string path){
  ifstream infile(path);
  human_data human;

  int human_id;
  double encoding_val;
  if(infile.is_open()){
    if(infile >> human_id){
      ROS_INFO("ID is %d", human_id);
      human.id = human_id;
    }

    for (int i = 0; i < 4; i++) {
      if(infile >> encoding_val){
        ROS_INFO("Enconding value %f added", encoding_val);
        human.encoding.push_back(encoding_val);
      }
    }
    ROS_INFO("A total of %d, encodings were loaded", human.encoding.size());

    int no_body_parts;
    if(infile >> no_body_parts){
      ROS_INFO("Total of %d body parts for human with ID: %d", no_body_parts, human_id);
    }
    body_limb_info body_info;
    int limb_id;
    string limb_name;
    for (int i = 0; i < no_body_parts; i++) {

      if(infile >> body_info.length >> body_info.joint_confidence){
        ROS_INFO("Body info %d with len: %f and conf: %f", i,  body_info.length, body_info.joint_confidence);
      //  human.limbs.push_back(body_info);
      }
    }
    ROS_INFO("Total number of %d limb info added", human.limbs.size());

  }else{
    ROS_INFO("Could not load file");
  }
}
