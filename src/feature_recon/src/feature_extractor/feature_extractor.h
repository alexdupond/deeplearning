#ifndef FEATURE_EXTRACTOR
#define FEATURE_EXTRACTOR

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "feature_recon/Persons.h"
#include "feature_recon/Person.h"
#include "feature_recon/BodyPartElm.h"
#include "../human_structs.h"
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <fstream>

using namespace std;

#define MAX_SMA_SIZE 5

class FeatureExtractor
{
public:
  FeatureExtractor(ros::NodeHandle &node_handle);

  void callback(const feature_recon::Persons::ConstPtr& msg);
  double calDistance(feature_recon::BodyPartElm first, feature_recon::BodyPartElm secound);
  body_limb isBodyPair(feature_recon::BodyPartElm first, feature_recon::BodyPartElm secound);
  //void printHuman();
  void saveLimbToFile(body_limb limb);
  vector<vector<body_limb>> getFacelessHumans();
  vector<human_data> getCompleteHumans();
  ros::Time getCurrentStamp(); 


private:
  ros::Time current_stamp;
  ros::NodeHandle nh;
  ros::Subscriber extract_features_sub;
  vector<vector<body_limb>> humans_faceless;
  vector<human_data> humans_complete;
};

#endif
