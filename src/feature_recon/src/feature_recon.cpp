#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feature_extractor/feature_extractor.h"
#include "Persons.h"
#include <string>

using namespace std;

bool load_human_data(string path);
bool save_human_data(human_data human, string path);
double comfidenceFunction(human_data &h1, human_data &h2);

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
          ROS_INFO("Person updated");
      }
    }

    for (int j = 0; j < FeatExt.getFacelessHumans().size(); j++) {
      for (int i = 0; i < persons.getPersons().size(); i++) {
        ROS_INFO("Distance between %d and unknown is: %f", persons.getPersons()[i].id, persons.distanceBetween(persons.getPersons()[i], FeatExt.getFacelessHumans()[j]));
      }
    }

  }

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}


double comfidenceFunction(human_data &h1, human_data &h2)
{
    int minLimbs = 6;

    int h1Size = h1.limbs.size();
    int h2Size = h2.limbs.size();

    double length = 0.0;
    double confidence = 0.0;

    int comparedLimbs = 0;

    if (h1Size <= h2Size && h1Size >= minLimbs)
    {
        for (size_t i = 0; i < h1Size; i++)
        {
            for (size_t j = 0; j < h2Size; j++)
            {
                if(h1.limbs[i].id == h2.limbs[j].id)
                {
                    length += abs(h1.limbs[i].avg_info.length - h2.limbs[j].avg_info.length);
                    confidence += (h1.limbs[i].avg_info.joint_confidence + h2.limbs[j].avg_info.joint_confidence);
                    comparedLimbs++;
                }
            }
        }
        if (comparedLimbs >= minLimbs)
          return (length/h1Size)*(confidence/(h1Size*2));
    }
    else if (h2Size < h1Size && h2Size >= minLimbs)
    {
        for (size_t i = 0; i < h2Size; i++)
        {
            for (size_t j = 0; j < h1Size; j++)
            {
                if(h1.limbs[i].id == h2.limbs[j].id)
                {
                    length += (h2.limbs[i].avg_info.length / h1.limbs[j].avg_info.length);
                    confidence += (h2.limbs[i].avg_info.joint_confidence + h1.limbs[j].avg_info.joint_confidence);
                    comparedLimbs++;
                }
            }
        }
        if (comparedLimbs >= minLimbs)
          return (length/h2Size)*(confidence/(h2Size*2));
    }
    return -1;
}
