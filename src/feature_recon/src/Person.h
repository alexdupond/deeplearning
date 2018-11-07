#pragma once

#include <vector>

using namespace std;

class Person
{
private:
    // PLACEHOLDERS:
    vector<double> faceEncoding;
    vector<int> limbs;


public:
    Person();
    Person(vector<double> &faceEncoding, vector<int> &limbs);

    void updatePerson(vector<double> &faceEncoding, vector<int> &limbs);
    vector<double> getFaceEncoding();
    vector<int> getLibs();

};
