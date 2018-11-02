#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_recon/Persons.h"
#include "feature_recon/Person.h"
#include "feature_recon/BodyPartElm.h"
#include "human_structs.h"
#include "feature_extractor/feature_extractor.h"
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle nh;
FeatureExtractor FeatExt(nh);

ros::Rate loop_rate(10);


while (ros::ok()){
  if((int)FeatExt.getFacelessHumans().size())
    ROS_INFO("Number of humans without faces: %d", (int)FeatExt.getFacelessHumans().size());

  if((int)FeatExt.getCompleteHumans().size())
    ROS_INFO("Number of humans WHIT faces: %d", (int)FeatExt.getCompleteHumans().size());

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
