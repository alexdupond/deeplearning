#pragma once

#include "human_structs.h"
#include <vector>
#include "ros/ros.h"
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>

const string TXT_EXTENSION = ".txt";
const string PATH = "/home/alexdupond/deeplearning/src/feature_recon/src/human_data/";

const int MAX_PERSONS = 100;
const int MAX_SMM = 5;
using namespace std;

class Persons
{
private:
    // PLACEHOLDERS:
    vector<human_data> PersonsList;

    float face_comp_thresh = 0.6;

    void initializeKnownPersons(string knownPersonsPath);

public:
    Persons();
    bool faceVerification(human_data &person1, human_data &person2);
    vector<human_data> getPersons();

    bool updatePerson(human_data &person);
    bool updateLimb(body_limb &limb1, body_limb &limb2);
    bool saveToFile(human_data &person, string path);
    bool loadPersons(string path);

};