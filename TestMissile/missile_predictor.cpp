//GUIDANCE
#include "pch.h"
#include <Eigen/Dense>
#include <cmath>
using Eigen::Vector3f;

class MissilePredictor
{
private:
    //exported data
    Vector3f optimal_direction;
    Vector3f error;
    float    gimbal_pitch = 0.0f;
    float    gimbal_yaw = 0.0f;

public:
    MissilePredictor() {}
    ~MissilePredictor() {}

    // call once per tick
    void update(Vector3f missile_position, Vector3f missile_velocity, Vector3f target_predicted_position) {

        // step 8 - vector pointing missile -> predicted target
        Vector3f v_opt = target_predicted_position - missile_position;
        optimal_direction = v_opt.normalized();

        // step 9 - difference between where we want to go and where we are going
        Vector3f current_direction = missile_velocity.norm() > 0.001f
            ? missile_velocity.normalized()
            : Vector3f(0.0f, 0.0f, 1.0f);

        error = optimal_direction - current_direction;

        // convert error to gimbal angles
        gimbal_pitch = std::atan2(error.z(), error.x());
        gimbal_yaw = std::atan2(error.y(), error.x());
    }

    Vector3f getOptimalDirection() { return optimal_direction; }
    Vector3f getError() { return error; }
    float    getGimbalPitch() { return gimbal_pitch; }
    float    getGimbalYaw() { return gimbal_yaw; }
};
