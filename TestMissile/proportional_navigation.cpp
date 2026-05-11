//CONTROL
#include "pch.h"
#include <Eigen/Dense>
using Eigen::Vector3f;

class ProportionalNavigation
{
private:
    const float N = 4.0f; // navigation constant - tune between 3 and 5

    //exported data
    Vector3f gimbal_direction;

public:
    ProportionalNavigation() {}
    ~ProportionalNavigation() {}

    // call once per tick
    // los_rate    = how fast the line of sight angle is changing (per second)
    // closing_spd = how fast missile is approaching target (m/s)
    void update(Vector3f los_rate, float closing_spd) {

        // step 10 - PN law: acceleration command proportional to LOS rate
        Vector3f accel_cmd = N * closing_spd * los_rate;

        // normalize to get a direction for the gimbal to point at
        if (accel_cmd.norm() > 0.001f)
            gimbal_direction = accel_cmd.normalized();
        else
            gimbal_direction = Vector3f(0.0f, 0.0f, 1.0f);
    }

    Vector3f getGimbalDirection() { return gimbal_direction; }
};
