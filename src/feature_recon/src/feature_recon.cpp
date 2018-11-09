#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_extractor/feature_extractor.h"
#include "Persons.h"
#include <string>

using namespace std;

bool load_human_data(string path);
bool save_human_data(human_data human, string path);

int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle nh;
FeatureExtractor FeatExt(nh);
Persons persons;
ros::Rate loop_rate(100);

// The path for the folder where to load and save human data

ros::Time last_stamp;

while (ros::ok()){
  if(last_stamp != FeatExt.getCurrentStamp()){
    last_stamp = FeatExt.getCurrentStamp();
    for (int i = 0; i < FeatExt.getCompleteHumans().size(); i++) {
      if(persons.updatePerson(FeatExt.getCompleteHumans()[i])){
          ROS_INFO("Person updated");
      }
    }
  }

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
