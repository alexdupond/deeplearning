#include "feature_extractor.h"

FeatureExtractor::FeatureExtractor(ros::NodeHandle &node_handle): nh (node_handle){
  extract_features_sub =  nh.subscribe("broadcaster/poses", 1, &FeatureExtractor::callback, this);
}

vector<vector<body_limb>> FeatureExtractor::getFacelessHumans(){
  return humans_faceless;
}

vector<human_data> FeatureExtractor::getCompleteHumans(){
  return humans_complete;
}

ros::Time FeatureExtractor::getCurrentStamp(){
  return current_stamp;
}


void FeatureExtractor::callback(const feature_recon::Persons::ConstPtr& msg){
  vector<vector<body_limb>> humans_faceless_temp;
  vector<human_data> humans_complete_temp;

  current_stamp = msg->header.stamp;

  body_limb current_limb;
  for (int k = 0; k < msg->persons.size(); k++){                            // Go through all persons
    vector<body_limb> limbs_temp;
    for (int i = 0; i < msg->persons[k].body_part.size(); i++) {            // Go through each body part
      for (int j = 0; j+i < msg->persons[k].body_part.size(); j++) {      // And compare to each body part
        current_limb = isBodyPair(msg->persons[k].body_part[i], msg->persons[k].body_part[i+j]);
        if((current_limb.id != -1) && current_limb.avg_info.length && current_limb.avg_info.joint_confidence){         // If the current limb is a body pair, then ...
          limbs_temp.push_back(current_limb);
          limbs_temp[limbs_temp.size()-1].info_list.push_back(current_limb.avg_info);
        }
      }
    }

    if(msg->persons[k].encoding.size() == 128){
      human_data human_temp;
      human_temp.encoding = msg->persons[k].encoding;
      human_temp.limbs = limbs_temp;
      human_temp.id = -1;
      human_temp.t = current_stamp;
      humans_complete_temp.push_back(human_temp);
    }else{
      humans_faceless_temp.push_back(limbs_temp);
    }
  }
  humans_faceless = humans_faceless_temp;
  humans_complete = humans_complete_temp;
}


double FeatureExtractor::calDistance(feature_recon::BodyPartElm first, feature_recon::BodyPartElm second){
  return sqrt(pow((second.x-first.x), 2) + pow((second.y-first.y), 2) + pow((second.z-first.z),2));
}

body_limb FeatureExtractor::isBodyPair(feature_recon::BodyPartElm first, feature_recon::BodyPartElm second){
  body_limb new_limb{};
  new_limb.id = -1;

  part_to_limb idPairs[18][4] = {{{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{2, 1, "midt_to_r_shoulder"}, {5, 2, "midt_to_l_shoulder"}, {8, 3, "midt_to_r_hip"}, {11, 4, "midt_to_l_hip"}},
                        {{3, 5, "right_upper_arm"}, {5, 6,"db_shoulders"}, {-1, -1,""}, {-1, -1,""}},
                        {{4,7, "right_lower_arm"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{6, 8,"left_upper_arm"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{7, 9, "left_lower_arm"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{9, 10, "right_thigh"}, {11, 11, "db_hips"}, {-1, -1,""}, {-1, -1,""}},
                        {{10, 12,"right_calf"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{12, 13,"left_thigh"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{13, 14,"left_calf"}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""},{-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}},
                        {{-1, -1,""}, {-1, -1,""}, {-1, -1,""}, {-1, -1,""}}
                      };

  for (int i = 0; i < 4; i++) {
    if(idPairs[first.part_id][i].compare_id == second.part_id){
      new_limb.id = idPairs[first.part_id][i].limb_id;
      new_limb.name = idPairs[first.part_id][i].name;
      if(!isnan(calDistance(first, second))){
        new_limb.avg_info.joint_confidence = min(first.confidence, second.confidence);
        new_limb.avg_info.length = calDistance(first, second);
      }
      return new_limb;
    }

  }
  return new_limb;
}


/*
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
*/

void FeatureExtractor::saveLimbToFile(body_limb limb){
  if(limb.avg_info.length && limb.avg_info.joint_confidence){
    ofstream newfile;
    string str;
    str.append("/home/alexdupond/data/");
    str.append(limb.name);
    str.append(".txt");
    newfile.open(str, ios_base::app);
    newfile << limb.avg_info.length << ", " << limb.avg_info.joint_confidence << "\n";
    newfile.close();
  }
}
