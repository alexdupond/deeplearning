#include "data_handler.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "human_structs.h"
#include "feature_extractor/feature_extractor.h"
#include <string>

const int MAX_HUMANS = 100;

using namespace std;

bool load_human_data(string path);
bool save_human_data(human_data human, string path);

int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle nh;
FeatureExtractor FeatExt(nh);

ros::Rate loop_rate(10);

// The path for the folder where to load and save human data
string path = "/home/alexdupond/deeplearning/src/feature_recon/src/human_data/";

// Loading the humans already found.
for (int i = 0; i < MAX_HUMANS; i++) {
  string full_path = path;
  full_path.append("human_");
  full_path.append(to_string(i+1));
  if(!load_human_data(full_path)){
    ROS_INFO("No more human data files to process");
    break;
  }
}

bool face = true;

while (ros::ok()){
  if(FeatExt.getCompleteHumans().size() && face){
    human_data human_1 = FeatExt.getCompleteHumans()[0];
    human_1.id = 1;
    if(save_human_data(human_1, path)){
      ROS_INFO("Human %d - Saved", human_1.id);
    }else{
      ROS_INFO("Human %d - Could not save", human_1.id);
    }
    face = false;
  }

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
