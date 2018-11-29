#include "Persons.h"

Persons::Persons()
{
  if(loadPersons(PATH)){
    ROS_INFO( "%d number of persons loaded", (int)PersonsList.size());
  }else{
    ROS_INFO("No persons was loaded");
  }
}

double Persons::distanceBetween(human_data human_known, vector<body_limb> human_unknown){
  double sum = 0;
  for (int i = 0; i < human_unknown.size(); i++) {
    for (int j = 0; j < human_known.limbs.size(); j++) {
      sum += pow(human_unknown[i].avg_info.length - human_known.limbs[i].avg_info.length,2);
    }
  }
  return sqrt(sum);
}


double Persons::faceVerification(human_data &person1, human_data &person2){
  double sum = 0;
  for (int i = 0; i < person1.encoding.size(); i++)
  {
    sum += (person1.encoding[i] - person2.encoding[i])*(person1.encoding[i] - person2.encoding[i]);
  }
  sum = sqrt(sum);
  ROS_INFO("Distance between face[%d] and face[%d] = %f", person1.id, person2.id, sum);
  return sum;
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
  if (tempPersons.size() >= MAX_SMM)
  {
    bubbleSort(tempPersons);
    shrinkVector(tempPersons);
  }
  double faceScore = 2.0;
  int minID = 0;
  bool isKnownPerson = false;
  // Temp person list update
  for (int i = 0; i < tempPersons.size(); i++) {
    double tempScore = faceVerification(tempPersons[i], person);
    if(tempScore < faceScore){
      faceScore = tempScore;
      minID = i;
    }
  }

  for (int i = 0; i < PersonsList.size(); i++) {
    double tempScore = faceVerification(PersonsList[i], person);
    if(tempScore < faceScore){
      faceScore = tempScore;
      minID = i;
      isKnownPerson = true;
    }
  }

  if((faceScore < face_comp_thresh && !isKnownPerson)){

    ROS_INFO("---");
    ROS_INFO("---");
    ROS_INFO("---");
    ROS_INFO("Temp person with ID: %d was a match to unknown", tempPersons[minID].id);
    bool checkIfSave = false;
    for (int j = 0; j < person.limbs.size(); j++) {
      bool limb_exist = false;
      for (int k = 0; k < tempPersons[minID].limbs.size(); k++) {
        if(person.limbs[j].id == tempPersons[minID].limbs[k].id){
          limb_exist = true;
          if(updateLimb(person.limbs[j], tempPersons[minID].limbs[k])){
            tempPersons[minID].t = person.t;
            if(tempPersons[minID].limbs[k].id == 18 && tempPersons[minID].limbs[k].info_list.size() == MAX_SMM){
              checkIfSave = true;
            }
          //  ROS_INFO("Temp - Limb ID: %d was updated!", tempPersons[minID].limbs[k].id);
          }
        }
      }
      if(!limb_exist){
        tempPersons[minID].limbs.push_back(person.limbs[j]);
      //  ROS_INFO("New temp limb added");
      }
    }
    if(checkIfSave){
      human_data tempP = tempPersons[minID];
      tempP.id = PersonsList.size()+1;
      PersonsList.push_back(tempP);
      tempPersons.erase(tempPersons.begin()+minID);
      if(saveToFile(tempP, PATH)){
        ROS_INFO("Temp person with ID: %d got saved in human list!", tempPersons[minID].id);
      }else{
      //  ROS_INFO("Could not update person with ID: %d!", PersonsList[minID].id);
      }
      return true;
    }
    return false;
  }


  if(faceScore < face_comp_thresh && isKnownPerson){
//    ROS_INFO("---");
//    ROS_INFO("---");
    ROS_INFO("---");
    ROS_INFO("Person with ID: %d was a match to unknown", PersonsList[minID].id);
    bool limb_updated = false;
    for (int j = 0; j < person.limbs.size(); j++) {
      bool limb_exist = false;
      for (int k = 0; k < PersonsList[minID].limbs.size(); k++) {
        if(person.limbs[j].id == PersonsList[minID].limbs[k].id){
          limb_exist = true;
          if(updateLimb(person.limbs[j], PersonsList[minID].limbs[k])){
          //  ROS_INFO("Limb ID: %d was updated!", PersonsList[minID].limbs[k].id);
            limb_updated = true;
          }
        }
      }
      if(!limb_exist){
        PersonsList[minID].limbs.push_back(person.limbs[j]);
      }
    }

    if(limb_updated){
      if(saveToFile(PersonsList[minID], PATH)){
        ROS_INFO("Person with ID: %d got updatede and resaved", PersonsList[minID].id);
      }else{
      //  ROS_INFO("Could not update person with ID: %d!", PersonsList[minID].id);
      }
      return true;
    }
    return false;
  }

  tempPersons.push_back(person);
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
    for (int j = 0; j < MAX_SMM; j++) {
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
      for (int i = 0; i < 128; i++) {
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
        for (int i = 0; i < MAX_SMM; i++) {
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

void Persons::bubbleSort(vector<human_data> &array)
{
    bubbleSort(array, 0, array.size()-1);
}

void Persons::bubbleSort(vector<human_data> &array, int left, int right)
{
    bool swapped = false;
    for (int i = left; i <= right; ++i)
    {
        for (int j = left; j <= right - i - 1 + left; ++j)
        {
            if (array[j].t < array[j + 1].t)
            {
                swap(array, j, j + 1);
                swapped = true;
            }
        }
        if(!swapped)
            return;
    }
}

inline void Persons::swap(vector<human_data> &array, int i, int j)
{
    human_data indexI = array[i];
    array[i] = array[j];
    array[j] = indexI;
}


void Persons::shrinkVector(vector<human_data> &array)
{
    while (array.size() > MAX_SMM)
    {
      array.pop_back();
    }
}
