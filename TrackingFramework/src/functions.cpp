#include <iostream>
#include <cmath>
#include "functions.h"
using namespace std;

#define pi 3.14159265358979323846

//Find Gamma angle
float Gamma(float ScanningTime_G_D, float period)
{
    return (ScanningTime_G_D * pi / period);
}

// Find Theta angles
float Theta(float scanningTime_B_H, float period)
{
    return (-scanningTime_B_H * pi / period) + pi;
}


//Box limit control function
std::vector<float> controLimit(vector<float> XYZ, float width, float heigth, float depth)
{
    if (XYZ[0] > width / 2)
    {
        XYZ[0] = width / 2; cout << "out of the box X" << endl;
    }
    else if (XYZ[0]<-width / 2)
    {
        XYZ[0] = -width / 2; cout << "out of the box -X" << endl;
    }
    if (XYZ[1] > heigth / 2)
    {
        XYZ[1] = heigth / 2; cout << "out of the box Y" << endl;
    }
    else if (XYZ[1]<-heigth / 2)
    {
        XYZ[1] = -heigth / 2; cout << "out of the box -Y" << endl;
    }
    if (XYZ[2] > depth / 2)
    {
        XYZ[2] = depth / 2; cout << "out of the box Z" << endl;
    }
    else if (XYZ[2]<-depth / 2)
    {
        XYZ[2] = -depth / 2; cout << "out of the box -Z" << endl;
    }
    return XYZ;
}


// Find the object position from the Light House
vector<float> posFromLightHouse(float gamma, float theta)
{
    vector<float> pos;
    pos.push_back( sin(gamma) * cos(theta));
    pos.push_back(sin(gamma) * sin(theta));
    pos.push_back(cos(gamma));
    return  pos;
}

// Find solution of equation system
vector<float> equationSystem(float a, float b, float c, float ap, float bp, float cp)
{
    vector<float>  solution;
    solution.push_back( (b * cp - bp * c) / (b * ap - a * bp));
    solution.push_back( (c - a * solution[0]) / b);
    return solution;
}
