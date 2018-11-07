#include "Person.h"

Person::Person()
{

}

Person::Person(vector<double> &faceEncoding, vector<int> &limbs)
{
    this->faceEncoding = faceEncoding;
    this->limbs = limbs;
}

void Person::updatePerson(vector<double> &faceEncoding, vector<int> &limbs)
{
    if (faceEncoding.size() > 0)
        this->faceEncoding = faceEncoding;

    if (limbs.size() > 0)
        this->limbs = limbs;
}

vector<double> Person::getFaceEncoding()
{
    return faceEncoding;
}

vector<int> Person::getLibs()
{
    return limbs;
}



