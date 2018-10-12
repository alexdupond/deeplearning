#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_recon/Persons.h"
#include "feature_recon/Person.h"
#include "feature_recon/BodyPartElm.h"
#include <sstream>
#include <string>
#include <math.h>

using namespace std;

class FeatureExtractor
{
public:
  FeatureExtractor(ros::NodeHandle &node_handle);

  void callback(const feature_recon::Persons::ConstPtr& msg);
  double calDistance(feature_recon::BodyPartElm first, feature_recon::BodyPartElm secound);
  bool isBodyPair(int id1, int id2);

private:
  ros::NodeHandle nh;
  ros::Subscriber extract_features_sub;
};

FeatureExtractor::FeatureExtractor(ros::NodeHandle &node_handle): nh (node_handle){
  extract_features_sub =  nh.subscribe("broadcaster/poses", 1, &FeatureExtractor::callback, this);
}

void FeatureExtractor::callback(const feature_recon::Persons::ConstPtr& msg){
  if(msg->persons.size()){
    if(msg->persons[0].body_part.size()){
      for (int i = 0; i < msg->persons[0].body_part.size(); i++) {
        if(msg->persons[0].body_part[i].part_id == 0){
          cout << "Body part location - x: " << msg->persons[0].body_part[i].x << ", y: " << msg->persons[0].body_part[i].y  << ", z: " << msg->persons[0].body_part[i].z<< endl;
        }
      }
    }
  }
}

double FeatureExtractor::calDistance(feature_recon::BodyPartElm first, feature_recon::BodyPartElm second){
  return sqrt(pow((second.x-first.x), 2) + pow((second.y-first.y), 2) + pow((second.z-first.z),2));
}

bool FeatureExtractor::isBodyPair(int id1, int id2){
  int idPAirs[18][5] = {{1, 14, 15, -1, -1},
                        {0, 2, 5, 8, 11},
                        {1, 3, -1, -1, -1},
                        {2, 4, -1, -1, -1},
                        {3, -1, -1, -1, -1},
                        {1, 6, -1, -1, -1},
                        {5, 7, -1, -1, -1},
                        {6, -1, -1, -1, -1},
                        {1, 9, -1, -1, -1},
                        {8, 10, -1, -1, -1},
                        {9, -1, -1, -1, -1},
                        {1, 12, -1, -1, -1},
                        {11, 13, -1, -1, -1},
                        {12, -1, -1, -1 ,-1},
                        {0, 16, -1, -1, -1},
                        {0, 17, -1, -1, -1},
                        {14, -1, -1, -1, -1},
                        {15, -1, -1, -1, -1}}
}


int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle nh;
FeatureExtractor FeatExt(nh);

ros::Rate loop_rate(10);


while (ros::ok()){

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
