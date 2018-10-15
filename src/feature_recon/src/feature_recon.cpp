#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_recon/Persons.h"
#include "feature_recon/Person.h"
#include "feature_recon/BodyPartElm.h"
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>

using namespace std;

struct body_limb{
  int id;
  string name;
  feature_recon::BodyPartElm body_part_1;
  feature_recon::BodyPartElm body_part_2;
  double length;
  double joint_confidence;
};

struct part_to_limb{
  int compare_id;
  int limb_id;
  string name;
};

class FeatureExtractor
{
public:
  FeatureExtractor(ros::NodeHandle &node_handle);

  void callback(const feature_recon::Persons::ConstPtr& msg);
  double calDistance(feature_recon::BodyPartElm first, feature_recon::BodyPartElm secound);
  body_limb isBodyPair(feature_recon::BodyPartElm first, feature_recon::BodyPartElm secound);
  void printHuman();
  void saveLimbToFile(body_limb limb);

private:
  ros::NodeHandle nh;
  ros::Subscriber extract_features_sub;
  body_limb human[21]{};
};

FeatureExtractor::FeatureExtractor(ros::NodeHandle &node_handle): nh (node_handle){
  extract_features_sub =  nh.subscribe("broadcaster/poses", 1, &FeatureExtractor::callback, this);

  for (int i = 0; i < 21; i++) {
    human[i].id = -1;
  }
}

void FeatureExtractor::callback(const feature_recon::Persons::ConstPtr& msg){
  body_limb current_limb;
  if(msg->persons.size()){                                                    // If the message contains any persons then
    if(msg->persons[0].body_part.size()){                                     // If the person contains any body parts then
      for (int i = 0; i < msg->persons[0].body_part.size(); i++) {            // Go through each body part
        for (int j = 0; j+i < msg->persons[0].body_part.size(); j++) {        // And compare to each body part
          current_limb = isBodyPair(msg->persons[0].body_part[i], msg->persons[0].body_part[i+j]);
          if(current_limb.id != -1){
            saveLimbToFile(current_limb);
            if(human[current_limb.id].id == -1){
            //ROS_INFO("ADDED: Body limb %s(%d) of length %f found with conficence of %f", current_limb.name.c_str(),current_limb.id, current_limb.length, current_limb.joint_confidence);
            human[current_limb.id] = current_limb;
          }else if(human[current_limb.id].joint_confidence < current_limb.joint_confidence){
            human[current_limb.id] = current_limb;
            //ROS_INFO("UPDATED: Body limb %s(%d) of length %f found with conficence of %f", current_limb.name.c_str(),current_limb.id, current_limb.length, current_limb.joint_confidence);
          }
          }
        }
      }
      printHuman();
    }
  }
}

double FeatureExtractor::calDistance(feature_recon::BodyPartElm first, feature_recon::BodyPartElm second){
  return sqrt(pow((second.x-first.x), 2) + pow((second.y-first.y), 2) + pow((second.z-first.z),2));
}

body_limb FeatureExtractor::isBodyPair(feature_recon::BodyPartElm first, feature_recon::BodyPartElm second){
  body_limb new_limb{};
  new_limb.id = -1;

  part_to_limb idPairs[18][4] = {{{14, 0, "nose_to_r_eye"}, {15, 1, "nose_to_l_eye"}, {16, 2, "nose_to_r_ear"}, {17, 3, "nose_to_l_ear"}},
                        {{2, 4, "midt_to_r_shoulder"}, {5, 5, "midt_to_l_shoulder"}, {8, 6, "midt_to_r_hip"}, {11, 7, "midt_to_l_hip"}},
                        {{3,8, "right_upper_arm"}, {5, 9,"db_shoulders"}, {-1, -1,""}, {-1, -1,""}},
                        {{4,10, "right_lower_arm"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{6,11,"left_upper_arm"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{7,12, "left_lower_arm"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{9,13, "right_thigh"}, {11, 14, "db_hips"}, {-1, -1,""}, {-1, -1,""}},
                        {{10, 15,"right_calf"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{12, 16,"left_thigh"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{13, 17,"left_calf"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""},{-1, -1,""}, {-1, -1,""}},
                        {{15, 18,"db_eyes"}, {16,19,"eye_to_ear_r"}, {-1, -1,""}, {-1, -1,""}},
                        {{17, 20,"eye_to_ear_l"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}}
                      };

  for (int i = 0; i < 4; i++) {
    if(idPairs[first.part_id][i].compare_id == second.part_id){
      new_limb.id = idPairs[first.part_id][i].limb_id;
      new_limb.name = idPairs[first.part_id][i].name;
      new_limb.body_part_1 = first;
      new_limb.body_part_2 = second;
      if(!isnan(calDistance(first, second))){
        new_limb.joint_confidence = min(first.confidence, second.confidence);
        new_limb.length = calDistance(first, second);
      }
      return new_limb;
    }

  }
  return new_limb;
}

void FeatureExtractor::printHuman(){
  for (int i = 0; i < 21; i++) {
    if(human[i].id != -1)
      ROS_INFO("ID: %d, Length %f , Conficence: %f, Name: %s",human[i].id, human[i].length, human[i].joint_confidence, human[i].name.c_str());
  }
  ROS_INFO("--");
  ROS_INFO("--");
  ROS_INFO("--");

}

void FeatureExtractor::saveLimbToFile(body_limb limb){
  if(limb.length && limb.joint_confidence){
  ofstream newfile;
  string str;
  str.append("/home/alexdupond/data/");
  str.append(limb.name);
  str.append(".txt");
  newfile.open(str, ios_base::app);
  newfile << limb.length << ", " << limb.joint_confidence << "\n";
  newfile.close();
}
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
