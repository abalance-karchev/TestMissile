//SEEKER
#include "pch.h"
#include <Eigen/Dense>
#include <cmath>
using Eigen::Vector3f;
using Eigen::Quaternionf;
using std::cos;
using std::sin;

class TargetData
{
private:
    float azimuth = 0.0f; // horizontal angle from IR seeker (radians)
    float elevation = 0.0f; // vertical angle from IR seeker (radians)

    //exported data
    Vector3f los_body; // line of sight in body frame (relative to missile nose)

    void askIRSeeker() {
        // TODO: read from sensor
        // Unity API: set azimuth/elevation directly via setAngles()
        azimuth = 0.05f; // placeholder
        elevation = 0.03f; // placeholder
    }

public:
    TargetData() {}
    ~TargetData() {}

    // call once per tick
    void update() {
        askIRSeeker();

        los_body = Vector3f(
            cos(elevation) * sin(azimuth), // right x in unity
            sin(elevation),                 // up y in unity
            cos(elevation) * cos(azimuth) // forward z in unity
        );
    }

    // Unity API - call this before update() to inject seeker angles
    void setAngles(float az, float el) {
        azimuth = az;
        elevation = el;
    }

    Vector3f getLineOfSight() { return los_body; }
};
