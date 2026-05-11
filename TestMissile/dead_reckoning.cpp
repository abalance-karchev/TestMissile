//PREDICTION
#include "pch.h"
#include <Eigen/Dense>
using Eigen::Vector3f;
using Eigen::Quaternionf;

class DeadReckoning
{
private:
    const float delta_time = 0.020f;
    float range = 500.0f; // metres - updated by HAL_SetRange() each tick

    Vector3f prev_los_world = Vector3f(0.0f, 0.0f, 1.0f);

    //exported data
    Vector3f target_velocity;
    Vector3f target_predicted_position;

public:
    DeadReckoning() {}
    ~DeadReckoning() {}

    void setRange(float r) { range = r; }

    // call once per tick
    void update(Vector3f missile_position, Quaternionf orientation, Vector3f los_body) {

        // step 5 - rotate LOS from body frame to world frame
        Vector3f los_world = orientation * los_body;

        // step 6 - estimate target velocity from change in LOS angle
        Vector3f delta_los = los_world - prev_los_world;
        target_velocity = delta_los / delta_time;
        prev_los_world = los_world;

        // scale by real range to get actual target position
        Vector3f target_now = missile_position + los_world * range;

        // step 7 - predict where target will be in 1 second
        target_predicted_position = target_now + target_velocity * 1.0f;
    }

    Vector3f getTargetVelocity() { return target_velocity; }
    Vector3f getPredictedPosition() { return target_predicted_position; }
};
