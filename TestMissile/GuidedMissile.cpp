#include "pch.h"
#include "missile_data.cpp"
#include "target_data.cpp"
#include "master.cpp"
//#include "missile_data.h"
//#include "target_data.h"
//#include "master.h"

// ── Global instances (mirrors main.cpp locals) ────────────────────────────────
static MissileData g_missile;
static TargetData  g_target;
static GNCMaster   g_gnc;

extern "C" {

    // ── HAL: Unity feeds missile state (replaces missile_data.update()) ───────

    __declspec(dllexport) void HAL_SetMissileState(
        float pos_x, float pos_y, float pos_z,
        float vel_x, float vel_y, float vel_z,
        float q_w, float q_x, float q_y, float q_z)
    {
        g_missile.setPosition(Eigen::Vector3f(pos_x, pos_y, pos_z));
        g_missile.setVelocity(Eigen::Vector3f(vel_x, vel_y, vel_z));
        g_missile.setOrientation(Eigen::Quaternionf(q_w, q_x, q_y, q_z));
    }

    // ── HAL: Unity feeds seeker angles (replaces target_data.update()) ────────

    __declspec(dllexport) void HAL_SetSeekerAngles(float azimuth, float elevation)
    {
        g_target.setAngles(azimuth, elevation);
    }

    __declspec(dllexport) void HAL_SetRange(float range)
    {
        //g_target.setRange(range);
    }

    // ── GNC: mirrors gnc.set*() + gnc.update() from main's for loop ──────────

    __declspec(dllexport) void GNC_Update(float dt)
    {
        // mirrors main.cpp loop body exactly:
        g_gnc.setMissileState(
            g_missile.getPosition(),
            g_missile.getVelocity(),
            g_missile.getOrientation()
        );
        g_gnc.setLOSBody(g_target.getLineOfSight());
        g_gnc.update();

        (void)dt; // reserved for future proportional navigation
    }

    // ── GNC output: steer direction as float[3] ───────────────────────────────

    __declspec(dllexport) void GNC_GetOptimalDirection(float* out)
    {
        Eigen::Vector3f steer = g_gnc.getOptimalDirection();
        out[0] = steer.x();
        out[1] = steer.y();
        out[2] = steer.z();
    }
}