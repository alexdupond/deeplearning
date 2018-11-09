#include "Persons.h"

Persons::Persons()
{
  if(loadPersons(PATH)){
    ROS_INFO( "%d number of persons loaded", (int)PersonsList.size());
  }else{
    ROS_INFO("No persons was loaded");
  }
}


bool Persons::faceVerification(human_data &person1, human_data &person2){
  int sum = 0;
  for (int i = 0; i < person1.encoding.size(); i++)
  {
    sum += (person1.encoding[i] - person2.encoding[i])*(person1.encoding[i] - person2.encoding[i]);
  }
  sum = sqrt(sum);
  return (sum <= face_comp_thresh);
}

bool Persons::updateLimb(body_limb &new_limb, body_limb &old_limb){
  double min_confidence = 1.0;

  // Check if there are less then 5 elements in the list
  if(old_limb.info_list.size() < MAX_SMM){
    // Add new value to list
    old_limb.info_list.push_back(new_limb.avg_info);
    // Find the smallest value of joint confidence
    for (int i = 0; i < old_limb.info_list.size(); i++) {
      if(min_confidence > old_limb.info_list[i].joint_confidence){
        min_confidence = old_limb.info_list[i].joint_confidence;
      }
    }
  // Else check if the new limb conf is bigger then the smallest conf
  }else if(old_limb.avg_info.joint_confidence < new_limb.avg_info.joint_confidence){
    // Find the smallest confidence and replace with new info.
    for (int i = 0; i < old_limb.info_list.size(); i++) {
      if(old_limb.info_list[i].joint_confidence == old_limb.avg_info.joint_confidence){
        old_limb.info_list[i] = new_limb.avg_info;
        //ROS_INFO("Lowest confidence is: %f and will be replaced by: %f", old_limb.avg_info.joint_confidence, new_limb.avg_info.joint_confidence);
      }
      if(min_confidence > old_limb.info_list[i].joint_confidence){
        min_confidence = old_limb.info_list[i].joint_confidence;
      }
    }
  }else{
    return false;
  }
  double sum = 0;
  for (int i = 0; i < old_limb.info_list.size(); i++) {
    sum += old_limb.info_list[i].length;
  }
  old_limb.avg_info.length = sum/old_limb.info_list.size();
  old_limb.avg_info.joint_confidence = min_confidence;

  return true;
}

bool Persons::updatePerson(human_data &person){
  bool limb_updated = false;
  for (int i = 0; i < PersonsList.size(); i++) {
    if(faceVerification(PersonsList[i], person)){
      ROS_INFO("---");
      ROS_INFO("---");
      ROS_INFO("---");
      ROS_INFO("Person with ID: %d was a match to unknown", PersonsList[i].id);
      for (int j = 0; j < person.limbs.size(); j++) {
        bool limb_exist = false;
        for (int k = 0; k < PersonsList[i].limbs.size(); k++) {
          if(person.limbs[j].id == PersonsList[i].limbs[k].id){
            limb_exist = true;
            if(updateLimb(person.limbs[j], PersonsList[i].limbs[k])){
              ROS_INFO("Limb ID: %d was updated!", PersonsList[i].limbs[k].id);
              limb_updated = true;
            }
          }
        }
        if(!limb_exist){
          PersonsList[i].limbs.push_back(person.limbs[j]);
          ROS_INFO("New limb added");
        }
      }

      if(limb_updated){
        if(saveToFile(PersonsList[i], PATH)){
          ROS_INFO("Person with ID: %d got updatede", PersonsList[i].id);
        }else{
          ROS_INFO("Could not update person with ID: %d!", PersonsList[i].id);
        }
        return true;
      }
      return false;
    }
  }
  person.id = PersonsList.size()+1;
  PersonsList.push_back(person);
  cout << "Person is = " << PersonsList.size() << " + 1 = " << person.id << endl;
  if(saveToFile(person, PATH)){
    ROS_INFO("Person with ID: %d got saved", person.id);
  }else{
    ROS_INFO("Could not save person with ID: %d!", person.id);
  }

  return false;
}

vector<human_data> Persons::getPersons(){
  return PersonsList;
}

bool Persons::saveToFile(human_data &person, string path){
  string line;

  // Writing human id to file
  line = "Human_id:";
  line.append(to_string(person.id));
  line.append("\n");

  // Writing encoding to file
  line.append("Encoding:");
  for (int i = 0; i < 128; i++) {
    line.append(to_string(person.encoding[i]));
    line.append(",");
  }
  line.append("\n");

  // Saving all the limbs
  ROS_INFO("Human %d - Saving %d limbs", person.id, (int)person.limbs.size());
  for (int i = 0; i < person.limbs.size(); i++) {
    // Writing limb id to file
    line.append("Limb_id:");
    line.append(to_string(person.limbs[i].id));
    line.append("\n");

    //Writing limb name to file
    line.append("Limb_name:");
    line.append(person.limbs[i].name);
    line.append("\n");

    //Writing info list to files
    line.append("Info_list:");
    for (int j = 0; j < 5; j++) {
      if(j < person.limbs[i].info_list.size()){
        line.append(";");
        line.append(to_string(person.limbs[i].info_list[j].length));
        line.append(",");
        line.append(to_string(person.limbs[i].info_list[j].joint_confidence));
      }else{
        line.append(";0,0");
      }
    }
    line.append("\n");

    // Writing avg info to file
    line.append("Avg_info:");
    line.append(to_string(person.limbs[i].avg_info.length));
    line.append(",");
    line.append(to_string(person.limbs[i].avg_info.joint_confidence));
    line.append("\n");
  }

  ofstream human_data;
  path.append("human_");
  path.append(to_string(person.id));
  path.append(TXT_EXTENSION);
  human_data.open(path, ios::trunc);

  if(!human_data.is_open()){
    return false;
  }

  human_data << line;
  human_data.close();
  return true;
}



bool Persons::loadPersons(string path){
  for (int i = 0; i < MAX_PERSONS; i++) {
    string full_path = path;
    full_path.append("human_");
    full_path.append(to_string(i+1));

    ifstream human_file;
    human_data human;
    full_path.append(TXT_EXTENSION);
    ROS_INFO("Trying the following path: %s", full_path.c_str());

    human_file.open(full_path);

    if(!human_file.is_open()){
      return true;
    }

      string line;
      // Getting the name of the human
      getline(human_file, line, ':');
      human_file >> human.id;
      ROS_INFO("Human(%d) - Loadning human", human.id);
      human_file.get();

      // Getting the encodings of the human
      getline(human_file, line, ':');
      ROS_INFO("Human(%d) - Loadning encodings", human.id);
      for (int i = 0; i < 5; i++) {
        getline(human_file, line, ',');
        human.encoding.push_back(stod(line));
      }
      human_file.get();
      ROS_INFO("Human(%d) - Encodings loaded", human.id);

      // Getting limb info
      while(human_file){
      ROS_INFO("Human(%d) - Adding bodypart", human.id);
      body_limb limb;
        // Getting limb id
        getline(human_file, line, ':');
        human_file >> limb.id;
        human_file.get();

        //Getting limb name
        getline(human_file, line, ':');
        human_file >> limb.name;
        human_file.get();
        human.limbs.push_back(limb);

        // Getting list of limb info
        body_limb_info limb_info;
        getline(human_file, line, ':');
        for (int i = 0; i < 5; i++) {
          getline(human_file, line, ';');
          getline(human_file, line, ',');
          limb_info.length = stod(line);
          human_file >> limb_info.joint_confidence;
          if(limb_info.length && limb_info.joint_confidence){
            human.limbs[human.limbs.size()-1].info_list.push_back(limb_info);
          }
        }
        ROS_INFO("Human(%d) - Info list of size %d was added",human.id, (int)human.limbs[human.limbs.size()-1].info_list.size());
        human_file.get();


        //Getting avg info;
        getline(human_file, line, ':');
        getline(human_file, line, ',');
        limb_info.length = stod(line);
        human_file >> limb_info.joint_confidence;human.limbs.size()-1;
        human.limbs[human.limbs.size()-1].avg_info = limb_info;
        ROS_INFO("Human(%d) - Avg info with length: %f and confidence: %f was added", human.id, limb_info.length, limb_info.joint_confidence);
        human_file.get();
        human_file.get();
        ROS_INFO("Human(%d) - Added body limb: %s", human.id, human.limbs[human.limbs.size()-1].name.c_str());

        // Breaks if no more limbs to add
        if(!human_file)
        {
          ROS_INFO("Human(%d) - Total of %d limb(s) were added", human.id, (int)human.limbs.size());
          ROS_INFO("Human(%d) - Human done loadning", human.id);
          break;
        }
      }
    PersonsList.push_back(human);
    human_file.close();
  }
  return false;
}

void Persons::initializeKnownPersons(string knownPersonsPath)
{
  std::string command = "python ";
  command += knownPersonsPath;
  system(command.c_str());
}