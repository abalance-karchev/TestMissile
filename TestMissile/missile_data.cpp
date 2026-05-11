//NAVIGATION
#include "pch.h"
#include <Eigen/Dense>
#include <cmath>
using Eigen::Vector3f;
using Eigen::Quaternionf;

class MissileData
{
private:
    const float delta_time = 0.016f;

    Vector3f acceleration;
    Vector3f angular_velocity;

    //exported data
    Vector3f velocity;
    Vector3f position;
    Quaternionf orientation;

    void askAccelerometer() {
        // TODO: read from sensor
        // Unity input: set acceleration directly via setAcceleration()
        acceleration = Vector3f(0.0f, 0.0f, 50.0f); // placeholder: full thrust forward
    }

    void askGyroscope() {
        // TODO: read from sensor
        // Unity input: set angular_velocity directly via setAngularVelocity()
        angular_velocity = Vector3f(0.0f, 0.0f, 0.0f); // placeholder: no rotation
    }

    void initialize() {
        acceleration = Vector3f(0.0f, 0.0f, 0.0f);
        velocity = Vector3f(0.0f, 0.0f, 0.0f);
        position = Vector3f(0.0f, 0.0f, 0.0f);
        orientation = Quaternionf::Identity();
    }

public:
    MissileData() { initialize(); }
    ~MissileData() {}

    // call once per tick
    void update() {
        askAccelerometer();
        velocity += acceleration * delta_time;

        askGyroscope();
        position += velocity * delta_time;

        // integrate angular velocity into orientation quaternion
        Quaternionf dq(
            1.0f,
            0.5f * angular_velocity.x() * delta_time,
            0.5f * angular_velocity.y() * delta_time,
            0.5f * angular_velocity.z() * delta_time
        );
        orientation.coeffs() += (orientation * dq).coeffs();
        orientation.normalize();
    }
    void setPosition(Vector3f p) { position = p; }
    void setVelocity(Vector3f v) { velocity = v; }
    void setOrientation(Quaternionf q) { orientation = q; }
    // Unity API - call these before update() to inject engine data
    void setAcceleration(Vector3f a) { acceleration = a; }
    void setAngularVelocity(Vector3f av) { angular_velocity = av; }

    Vector3f    getVelocity() { return velocity; }
    Vector3f    getPosition() { return position; }
    Quaternionf getOrientation() { return orientation; }
};
