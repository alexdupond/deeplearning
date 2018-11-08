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

class Person
{
private:
    // PLACEHOLDERS:
    vector<human_data> Persons;
    vector<double> faceEncoding;
    vector<int> limbs;

public:
    Person();
    //Person(vector<double> &faceEncoding, vector<int> &limbs);

    vector<double> getFaceEncoding();
    vector<int> getLibs();
    bool faceVerification(human_data &person);

    bool updatePerson(human_data &person);
    bool saveToFile(human_data &person, string path);
    bool loadPersons(string path);

};
