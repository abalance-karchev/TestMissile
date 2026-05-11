//GNC MASTER
#include "pch.h"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include "dead_reckoning.cpp"
#include "missile_predictor.cpp"
#include "proportional_navigation.cpp"
using Eigen::Vector3f;
using Eigen::Quaternionf;

class GNCMaster
{
private:
    const float delta_time = 0.020f;

    DeadReckoning          dead_reckoning;
    MissilePredictor       missile_predictor;
    ProportionalNavigation proportional_navigation;

    Vector3f    missile_position;
    Vector3f    missile_velocity;
    Quaternionf missile_orientation;
    Vector3f    los_body;

    Vector3f prev_los_world = Vector3f(0.0f, 0.0f, 1.0f);

    //exported data
    Vector3f optimal_direction;
    float    gimbal_pitch = 0.0f;
    float    gimbal_yaw = 0.0f;

    int tick = 0;

public:
    GNCMaster() {}
    ~GNCMaster() {}

    void setMissileState(Vector3f pos, Vector3f vel, Quaternionf orient) {
        missile_position = pos;
        missile_velocity = vel;
        missile_orientation = orient;
    }

    void setLOSBody(Vector3f los) { los_body = los; }

    // passthrough to dead reckoning
    void setRange(float range) { dead_reckoning.setRange(range); }

    void update() {
        // PREDICTION
        dead_reckoning.update(missile_position, missile_orientation, los_body);
        Vector3f predicted_pos = dead_reckoning.getPredictedPosition();
        Vector3f target_vel = dead_reckoning.getTargetVelocity();

        // GUIDANCE
        missile_predictor.update(missile_position, missile_velocity, predicted_pos);
        optimal_direction = missile_predictor.getOptimalDirection();
        gimbal_pitch = missile_predictor.getGimbalPitch();
        gimbal_yaw = missile_predictor.getGimbalYaw();

        // CONTROL
        proportional_navigation.update(target_vel, missile_velocity.norm());

        if (++tick % 25 == 0) {
            std::cout << std::fixed << std::setprecision(2)
                << "[GNC tick " << tick << "]"
                << " optimal (" << optimal_direction.x() << ", "
                << optimal_direction.y() << ", "
                << optimal_direction.z() << ")"
                << " pitch=" << gimbal_pitch
                << " yaw=" << gimbal_yaw
                << "\n";
        }
    }

    Vector3f getOptimalDirection() { return optimal_direction; }
    float    getGimbalPitch() { return gimbal_pitch; }
    float    getGimbalYaw() { return gimbal_yaw; }
    Vector3f getPNDirection() { return proportional_navigation.getGimbalDirection(); }
};
