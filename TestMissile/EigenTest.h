#pragma once
#include <Eigen/Dense>

inline float TestEigenH(float input)
{
    Eigen::Vector3f v(input, input * 2.0f, input * 3.0f);
    return v.norm(); // input * sqrt(14) ≈ input * 3.7417
}