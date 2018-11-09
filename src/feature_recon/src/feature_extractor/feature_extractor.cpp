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

    if(msg->persons[k].encoding.size()){
      human_data human_temp;
      human_temp.encoding = msg->persons[k].encoding;
      human_temp.limbs = limbs_temp;
      human_temp.id = -1;
      humans_complete_temp.push_back(human_temp);
    }else{
      humans_faceless_temp.push_back(limbs_temp);
    }
  }
  humans_faceless = humans_faceless_temp;
  humans_complete = humans_complete_temp;
}

/*
void FeatureExtractor::callback(const feature_recon::Persons::ConstPtr& msg){
  body_limb current_limb;
  if(msg->persons.size()){                                                    // If the message contains any persons then
    if(msg->persons[0].body_part.size()){                                     // If the person contains any body parts then
      for (int i = 0; i < msg->persons[0].body_part.size(); i++) {            // Go through each body part
        for (int j = 0; j+i < msg->persons[0].body_part.size(); j++) {        // And compare to each body part
          current_limb = isBodyPair(msg->persons[0].body_part[i], msg->persons[0].body_part[i+j]);
          // If the current limb is a body pair, then ...
          if((current_limb.id != -1) && current_limb.avg_info.length && current_limb.avg_info.joint_confidence){
            //saveLimbToFile(current_limb);
            // If the info list is empty and the ID is not set jet. Else if check if the max size is reached
            if(!human[current_limb.id].info_list.size()){
              human[current_limb.id] = current_limb;
              human[current_limb.id].info_list.push_back(current_limb.avg_info);
              //ROS_INFO("1: was added as a limb and pushed to the vector and the size should be 1 = %d", human[current_limb.id].info_list.size() );
              //ROS_INFO("1: The length = %f, and conf = %f", current_limb.avg_info.length, current_limb.avg_info.joint_confidence);
              saveLimbToFile(human[current_limb.id]);
            }else if(human[current_limb.id].info_list.size() < MAX_SMA_SIZE){
              human[current_limb.id].info_list.push_back(current_limb.avg_info);
              //ROS_INFO("2 - The length = %f, and conf = %f was added", current_limb.avg_info.length, current_limb.avg_info.joint_confidence);

              double sum = 0;
              body_limb_info min_body_info = {0, 1.0};
              for (int k = 0; k < human[current_limb.id].info_list.size(); k++) {
                if(min_body_info.joint_confidence > human[current_limb.id].info_list[k].joint_confidence){
                  min_body_info = human[current_limb.id].info_list[k];
                }
                sum += human[current_limb.id].info_list[k].length;
              }
              human[current_limb.id].avg_info.length = sum/human[current_limb.id].info_list.size();
              human[current_limb.id].avg_info.joint_confidence = min_body_info.joint_confidence;
              //ROS_INFO("2: Added as a limb and pushed to the vector - Size: %d", human[current_limb.id].info_list.size() );
              //ROS_INFO("2: New calculated avg of %f from %d number of values", human[current_limb.id].avg_info.length, human[current_limb.id].info_list.size() );
              saveLimbToFile(human[current_limb.id]);
            }else if(current_limb.avg_info.joint_confidence > human[current_limb.id].avg_info.joint_confidence){
              double sum = 0;
              body_limb_info min_body_info = {0, 1.0};
              //ROS_INFO("BEFORE - Conf: %f, %f, %f, %f, %f", human[current_limb.id].info_list[0].joint_confidence, human[current_limb.id].info_list[1].joint_confidence, human[current_limb.id].info_list[2].joint_confidence, human[current_limb.id].info_list[3].joint_confidence, human[current_limb.id].info_list[4].joint_confidence, human[current_limb.id].info_list[5].joint_confidence);
              //ROS_INFO("BEFORE - Length: %f, %f, %f, %f, %f", human[current_limb.id].info_list[0].length, human[current_limb.id].info_list[1].length, human[current_limb.id].info_list[2].length, human[current_limb.id].info_list[3].length, human[current_limb.id].info_list[4].length, human[current_limb.id].info_list[5].length);

              for (int k = 0; k < human[current_limb.id].info_list.size(); k++) {
                if(human[current_limb.id].avg_info.joint_confidence == human[current_limb.id].info_list[k].joint_confidence){
                  human[current_limb.id].info_list[k] = current_limb.avg_info;
                  //cout << "The current joint confidence (" << human[current_limb.id].avg_info.joint_confidence << ") is found in vector index k = " << k << endl;
                }

                if(min_body_info.joint_confidence > human[current_limb.id].info_list[k].joint_confidence){
                  min_body_info = human[current_limb.id].info_list[k];
                }
                sum += human[current_limb.id].info_list[k].length;
              }
              //ROS_INFO("AFTER - Conf: %f, %f, %f, %f, %f", human[current_limb.id].info_list[0].joint_confidence, human[current_limb.id].info_list[1].joint_confidence, human[current_limb.id].info_list[2].joint_confidence, human[current_limb.id].info_list[3].joint_confidence, human[current_limb.id].info_list[4].joint_confidence, human[current_limb.id].info_list[5].joint_confidence);
              //ROS_INFO("AFTER - Length: %f, %f, %f, %f, %f", human[current_limb.id].info_list[0].length, human[current_limb.id].info_list[1].length, human[current_limb.id].info_list[2].length, human[current_limb.id].info_list[3].length, human[current_limb.id].info_list[4].length, human[current_limb.id].info_list[5].length);

              human[current_limb.id].avg_info.length = sum/human[current_limb.id].info_list.size();
              human[current_limb.id].avg_info.joint_confidence = min_body_info.joint_confidence;
              //ROS_INFO("3 - The size new length = %f, and confidence = %f", human[current_limb.id].id, human[current_limb.id].avg_info.length, human[current_limb.id].avg_info.joint_confidence);
              saveLimbToFile(human[current_limb.id]);
            }
          }
        }
      }
      printHuman();
    }
  }
}
*/

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
    //  new_limb.body_part_1 = first;
      //new_limb.body_part_2 = second;
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
void FeatureExtractor::printHuman(){
  for (int i = 0; i < 21; i++) {
    if(human[i].id != -1){
      saveLimbToFile(human[i]);
      ROS_INFO("ID: %d, Length %f , Conficence: %f, Name: %s",human[i].id, human[i].avg_info.length, human[i].avg_info.joint_confidence, human[i].name.c_str());
    }
  }
  ROS_INFO("--");
  ROS_INFO("--");
  ROS_INFO("--");

}

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
