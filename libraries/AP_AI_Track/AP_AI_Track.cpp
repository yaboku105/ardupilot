/*
 * AP_AI_Track.cpp - AI-assisted target tracking implementation
 * Using AP_Mount for gimbal control
 * Fixed for ArduPilot build system
 */

#include "AP_AI_Track.h"

#include <AP_Mount/AP_Mount.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;

// Helper clamp
static inline float clamp_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// Singleton
AP_AI_Track *AP_AI_Track::_singleton = nullptr;

// Parameters
const AP_Param::GroupInfo AP_AI_Track::var_info[] = {

    AP_GROUPINFO("MODE", 0, AP_AI_Track, _mode, 1),
    AP_GROUPINFO("FOLLOW_DIST", 1, AP_AI_Track, _follow_distance, 10.0f),
    AP_GROUPINFO("FOLLOW_ANGLE", 2, AP_AI_Track, _follow_angle, 0.0f),
    AP_GROUPINFO("GIMBAL_P", 3, AP_AI_Track, _gimbal_p_gain, 1.0f),
    AP_GROUPINFO("GIMBAL_EN", 4, AP_AI_Track, _gimbal_enabled, 1),
    AP_GROUPINFO("FOLLOW_ALT", 5, AP_AI_Track, _follow_alt_offset, 5.0f),
    AP_GROUPINFO("GIMBAL_YAW_LK", 6, AP_AI_Track, _gimbal_yaw_lock, 1),
    AP_GROUPINFO("PREDICT_TIME", 7, AP_AI_Track, _predict_time, 0.15f),
    AP_GROUPINFO("VEL_GAIN", 8, AP_AI_Track, _velocity_gain, 0.5f),
    AP_GROUPINFO("MAX_SPEED", 9, AP_AI_Track, _max_speed, 5.0f),
    AP_GROUPINFO("MIN_DIST", 10, AP_AI_Track, _min_distance, 0.5f),
    AP_GROUPINFO("ENABLE", 11, AP_AI_Track, _enable, 0),
    AP_GROUPEND
};

// Constructor
AP_AI_Track::AP_AI_Track()
{
    AP_Param::setup_object_defaults(this, var_info);

    _singleton = this;

    _ahrs = nullptr;
    _mount = nullptr;
    _vehicle = nullptr;

    _tracking_enabled = false;
    _is_tracking = false;

    _target_pos.zero();
    _target_vel.zero();
    _prev_target_pos.zero();
    _gimbal_angles.zero();
    _drone_pos_target.zero();

    _last_update_us = 0;
    _frame_count = 0;
    _gimbal_initialized = false;
}

// Singleton
AP_AI_Track *AP_AI_Track::get_singleton()
{
    return _singleton;
}

// Update
void AP_AI_Track::update()
{    
    if (_enable <= 0) {
        _is_tracking = false;
        return;
    }

    if (_ahrs == nullptr || !_ahrs->initialised()) {
        return;
    }
    if (!_tracking_enabled || !_is_tracking) {
        return;
    }

    if (_ahrs == nullptr) {
        _ahrs = AP_AHRS::get_singleton();
        if (_ahrs == nullptr) return;
    }

    if (_gimbal_enabled && _mount == nullptr) {
        _mount = AP_Mount::get_singleton();

        if (_mount != nullptr && !_gimbal_initialized) {
            _mount->set_angle_target(0.0f, 0.0f, 0.0f, true);
            _gimbal_initialized = true;
        }
    }

    _frame_count++;

    if (_gimbal_enabled && _mount != nullptr) {
        update_gimbal_control();
    }

    if (_mode == TRACK_MODE_DRONE_FOLLOW) {
        update_drone_control();
    }
    hal.console->printf("AI_TRACK RUN\n");
}

// Gimbal control
void AP_AI_Track::update_gimbal_control()
{
    if (_mount == nullptr || _ahrs == nullptr) {
        return;
    }

    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return;
    }

    Vector3f target_pos = _target_pos;

    if (!is_zero(_predict_time) && _target_vel.length() > 0.1f) {
        target_pos += _target_vel * _predict_time;
    }

    Vector3f target_vector = target_pos - drone_pos;

    float dist = target_vector.length();
    if (dist < 0.1f) return;

    float pitch = degrees(atan2f(-target_vector.z,
                      sqrtf(sq(target_vector.x) + sq(target_vector.y))));
    float yaw = degrees(atan2f(target_vector.y, target_vector.x));

    pitch *= _gimbal_p_gain;
    yaw *= _gimbal_p_gain;

    pitch = clamp_float(pitch, -45.0f, 45.0f);
    yaw = clamp_float(yaw, -180.0f, 180.0f);

    bool earth_frame = (_gimbal_yaw_lock == 1);

    _mount->set_angle_target(0.0f, pitch, yaw, earth_frame);

    _gimbal_angles = Vector3f(0, radians(pitch), radians(yaw));

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write("AITG",
            "TimeUS,TX,TY,TZ,P,Y,D,PTX,PTY,PTZ",
            "Qfffffffff",
            AP_HAL::micros64(),
            (double)_target_pos.x,
            (double)_target_pos.y,
            (double)_target_pos.z,
            (double)pitch,
            (double)yaw,
            (double)dist,
            (double)target_pos.x,
            (double)target_pos.y,
            (double)target_pos.z);
    }
}

// Drone follow
void AP_AI_Track::update_drone_control()
{
    Vector3f drone_pos;

    if (!_ahrs || !_ahrs->get_relative_position_NED_home(drone_pos)) {
        return;
    }

    Vector3f offset;

    float ang = radians(_follow_angle);

    offset.x = _follow_distance * cosf(ang);
    offset.y = _follow_distance * sinf(ang);
    offset.z = -_follow_alt_offset;

    _drone_pos_target = _target_pos + offset;

    Vector3f error = _drone_pos_target - drone_pos;

    if (error.length() < _min_distance) {
        return;
    }

    Vector3f vel = error * _velocity_gain;

    if (vel.length() > _max_speed) {
        vel = vel.normalized() * _max_speed;
    }

    if (_target_vel.length() > 0.1f) {
        vel += _target_vel * 0.3f;
    }

    send_velocity_command(vel);
}

// Velocity command
void AP_AI_Track::send_velocity_command(const Vector3f &vel)
{
#ifdef AP_COPTER_H
    AC_PosControl *pos = AP::pos_control();
    if (pos != nullptr) {
        pos->set_velocity_target(vel, false);
    }
#endif

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write("AITV", "TimeUS,VX,VY,VZ",
                     "Qfff",
                     AP_HAL::micros64(),
                     vel.x, vel.y, vel.z);
    }
}

// Gimbal aim
bool AP_AI_Track::point_gimbal_at_target(const Vector3f &target_pos, float yaw_override)
{
    if (!_mount || !_ahrs) return false;

    Vector3f drone_pos;

    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return false;
    }

    Vector3f v = target_pos - drone_pos;

    if (v.length() < 0.1f) return false;

    float pitch = degrees(atan2f(-v.z, sqrtf(sq(v.x) + sq(v.y))));
    float yaw = degrees(atan2f(v.y, v.x));

    // FIXED FLOAT CHECK
    if (!is_zero(yaw_override)) {
        yaw = yaw_override;
    }

    pitch = clamp_float(pitch, -45.0f, 45.0f);
    yaw = clamp_float(yaw, -180.0f, 180.0f);

    _mount->set_angle_target(0.0f, pitch, yaw, true);
    return true;
}

// Enable tracking
void AP_AI_Track::set_tracking_enabled(bool en)
{
    _tracking_enabled = en;

    if (!en) {
        _is_tracking = false;

        if (_mount) {
            _mount->set_angle_target(0, 0, 0, true);
        }

        send_velocity_command(Vector3f(0,0,0));
    }
}

// Target update
void AP_AI_Track::set_target_position(const Vector3f &pos_target)
{
    if (_enable <= 0) {
        _is_tracking = false;
        return;
    }

    uint64_t current_time = AP_HAL::micros64();
    float dt = (current_time - _last_update_us) / 1000000.0f;

    if (_last_update_us > 0 && dt > 0.01f) {
        Vector3f new_vel = (pos_target - _prev_target_pos) / dt;

        const float filter_alpha = 0.3f;
        _target_vel = _target_vel * (1.0f - filter_alpha) + new_vel * filter_alpha;

        if (_target_vel.length() > 30.0f) {
            _target_vel = _target_vel.normalized() * 30.0f;
        }
    }

    _prev_target_pos = _target_pos;
    _target_pos = pos_target;
    _last_update_us = current_time;

    // ONLY start tracking when enabled
    _is_tracking = true;
}
// helpers
void AP_AI_Track::run_tracking() { update(); }

Vector3f AP_AI_Track::get_target_position() const { return _target_pos; }
Vector3f AP_AI_Track::get_target_velocity() const { return _target_vel; }
Vector3f AP_AI_Track::get_drone_target() const { return _drone_pos_target; }

void AP_AI_Track::reset()
{
    _target_pos.zero();
    _target_vel.zero();
    _is_tracking = false;

    if (_mount) {
        _mount->set_angle_target(0,0,0,true);
    }

    send_velocity_command(Vector3f(0,0,0));
}