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
using namespace std;

class Persons
{
private:
    // PLACEHOLDERS:
    vector<human_data> Persons;

    float face_comp_thresh = 0.6;

    void initializeKnownPersons(string knownPersonsPath);

public:
    Persons();

    vector<double> getFaceEncoding();
    vector<int> getLibs();
    bool faceVerification(human_data &person);

    bool saveToFile(human_data &person, string path);
    bool loadPersons(string path);

};
