#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_extractor/feature_extractor.h"
#include "Persons.h"
#include <string>

using namespace std;

bool load_human_data(string path);
bool save_human_data(human_data human, string path);
double comfidenceFunction(human_data &h1, vector<body_limb> &h2);

int main(int argc, char **argv)
{

ros::init(argc, argv, "deature_recon");

ros::NodeHandle nh;
FeatureExtractor FeatExt(nh);
Persons persons;
ros::Rate loop_rate(100);

// The path for the folder where to load and save human data

ros::Time last_stamp;

while (ros::ok()){
  if(last_stamp != FeatExt.getCurrentStamp()){
    last_stamp = FeatExt.getCurrentStamp();
    for (int i = 0; i < FeatExt.getCompleteHumans().size(); i++) {
      if(persons.updatePerson(FeatExt.getCompleteHumans()[i])){
      }
    }

    for (int j = 0; j < FeatExt.getFacelessHumans().size(); j++) {
      double maxConf = -1;
      int index = -1;
      for (int i = 0; i < persons.getPersons().size(); i++) {
        double conf = comfidenceFunction(persons.getPersons()[i], FeatExt.getFacelessHumans()[j]);
        if(conf > maxConf){
            maxConf = conf;
            index = i;
        }

      }

      double confOfConf = maxConf*persons.getPersons()[index].confidence;//1 - abs(conf - persons.getPersons()[i].confidence)/persons.getPersons()[i].confidence;
      if(maxConf != -1 && confOfConf > 0.01){
          ROS_INFO("Human with ID[%d](%s) has a confidence to unknown person of %f", persons.getPersons()[index].id, persons.getPersons()[index].name.c_str(), confOfConf);
      }
    }

  }

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}


double comfidenceFunction(human_data &h1, vector<body_limb> &h2)
{
    int minLimbs = 8;

    int h1Size = h1.limbs.size();
    int h2Size = h2.size();

    double length = 0.0;
    double confidence = 0.0;

    int comparedLimbs = 0;

    if (h1Size <= h2Size && h1Size >= minLimbs)
    {
        for (size_t i = 0; i < h1Size; i++)
        {
            for (size_t j = 0; j < h2Size; j++)
            {
                if(h1.limbs[i].id == h2[j].id)
                {
                  if(h1.limbs[i].avg_info.length <= h2[j].avg_info.length)
                    confidence += (1-abs(h1.limbs[i].avg_info.length - h2[j].avg_info.length)/h2[j].avg_info.length)*(h1.limbs[i].avg_info.joint_confidence * h2[j].avg_info.joint_confidence);
                  if(h1.limbs[i].avg_info.length > h2[j].avg_info.length)
                    confidence += (1-abs(h2[j].avg_info.length - h1.limbs[i].avg_info.length)/ h1.limbs[i].avg_info.length)*(h1.limbs[i].avg_info.joint_confidence * h2[j].avg_info.joint_confidence);
                    comparedLimbs++;
                }
            }
        }
        if (comparedLimbs >= minLimbs)
          return confidence/14;
    }
    else if (h2Size < h1Size && h2Size >= minLimbs)
    {
        for (size_t i = 0; i < h2Size; i++)
        {
            for (size_t j = 0; j < h1Size; j++)
            {
                if(h1.limbs[i].id == h2[j].id)
                {
                  if(h2[i].avg_info.length <= h1.limbs[j].avg_info.length)
                    confidence += (1-abs(h2[i].avg_info.length - h1.limbs[j].avg_info.length)/h1.limbs[j].avg_info.length)*(h1.limbs[i].avg_info.joint_confidence * h2[i].avg_info.joint_confidence);
                  if(h2[i].avg_info.length > h1.limbs[j].avg_info.length)
                    confidence += (1-abs(h1.limbs[j].avg_info.length - h2[i].avg_info.length)/h2[i].avg_info.length)*(h1.limbs[i].avg_info.joint_confidence * h2[i].avg_info.joint_confidence);

                  comparedLimbs++;
                }
            }
        }
        if (comparedLimbs >= minLimbs)
          return confidence/14;
    }
    return -1;
}


/*
h1size < h2size
length += abs((h1.limbs[i].avg_info.length - h2[j].avg_info.length));
confidence += (h1.limbs[i].avg_info.joint_confidence + h2[j].avg_info.joint_confidence);
comparedLimbs++;

(length/h1Size)*(confidence/(h1Size*2))

h2size < h1 size

length += (h2[i].avg_info.length / h1.limbs[j].avg_info.length);
confidence += (h2[i].avg_info.joint_confidence + h1.limbs[j].avg_info.joint_confidence);
comparedLimbs++;

(length/h2Size)*(confidence/(h2Size*2))
*/
