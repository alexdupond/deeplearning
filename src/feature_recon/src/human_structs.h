#ifndef HUMAN_STRUCTS
#define HUMAN_STRUCTS

#include <string>
#include <vector>
#include "ros/ros.h"

using namespace std;

struct body_limb_info{
  double length;
  double joint_confidence;
};

struct body_limb{
  int id;
  string name;
  vector<body_limb_info> info_list{};
  body_limb_info avg_info;
};

struct part_to_limb{
  int compare_id;
  int limb_id;
  string name;
};

struct human_data
{
  vector<double> encoding;
  vector<body_limb> limbs;
  int id;
  string name = "Unknown";
  double confidence = 0.0; 
  ros::Time t;
};

#endif
