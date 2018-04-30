#pragma once
#ifndef functions
#define functions
#include <vector>


std::vector<float> posFromLightHouse(float gamma, float theta);
std::vector<float> equationSystem(float a, float b, float c, float ap, float bp, float cp);
float Gamma(float scanning_G_D, float period);
float Theta(float scanning_B_H, float period);
std::vector<float> controLimit(std::vector<float> XYZ, float width, float heigth, float depth);

#endif
