#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_recon/Persons.h"
#include "feature_recon/Person.h"
#include "feature_recon/BodyPartElm.h"
#include <sstream>
#include <string>

using namespace std;

void callback(const feature_recon::Persons::ConstPtr& msg){
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


int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle n;

ros::Subscriber get_features = n.subscribe("broadcaster/poses", 1, callback);

ros::Rate loop_rate(10);


while (ros::ok()){

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
