#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_extractor/feature_extractor.h"
#include "Person.h"
#include <string>

using namespace std;

bool load_human_data(string path);
bool save_human_data(human_data human, string path);

int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle nh;
FeatureExtractor FeatExt(nh);
Person persons;
ros::Rate loop_rate(10);

// The path for the folder where to load and save human data



while (ros::ok()){

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
