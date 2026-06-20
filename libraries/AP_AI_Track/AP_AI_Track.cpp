/*
 * AP_AI_Track.cpp - AI-assisted target tracking
 * Based on AP_Follow pattern
 * Fixed: Loop Overrun (0x100000) and Controller Checks
 */

#include "AP_AI_Track.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;

// Singleton instance
AP_AI_Track *AP_AI_Track::_singleton = nullptr;

// Parameter table
const AP_Param::GroupInfo AP_AI_Track::var_info[] = {
    // @Param: MODE
    // @DisplayName: AI Tracking Mode
    // @Description: Selects the tracking mode
    // @Values: 0:Gimbal Only, 1:Drone Follow
    // @User: Standard
    AP_GROUPINFO("MODE", 0, AP_AI_Track, _mode, 1),

    // @Param: FOLLOW_DIST
    // @DisplayName: Follow Distance
    // @Description: Distance in meters to follow behind target
    // @Range: 1 50
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("FOLLOW_DIST", 1, AP_AI_Track, _follow_distance, 10.0f),

    // @Param: FOLLOW_ANGLE
    // @DisplayName: Follow Angle
    // @Description: Angle in degrees for follow position
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("FOLLOW_ANGLE", 2, AP_AI_Track, _follow_angle, 0.0f),

    // @Param: GIMBAL_P
    // @DisplayName: Gimbal P Gain
    // @Description: Proportional gain for gimbal control
    // @Range: 0.1 2.0
    // @User: Standard
    AP_GROUPINFO("GIMBAL_P", 3, AP_AI_Track, _gimbal_p_gain, 1.0f),

    // @Param: GIMBAL_EN
    // @DisplayName: Gimbal Enable
    // @Description: Enable/disable gimbal control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("GIMBAL_EN", 4, AP_AI_Track, _gimbal_enabled, 1),

    // @Param: FOLLOW_ALT
    // @DisplayName: Follow Altitude Offset
    // @Description: Altitude offset in meters for follow position
    // @Range: -20 20
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("FOLLOW_ALT", 5, AP_AI_Track, _follow_alt_offset, 5.0f),

    // @Param: GIMBAL_YAW_LK
    // @DisplayName: Gimbal Yaw Lock
    // @Description: Lock gimbal yaw to earth frame
    // @Values: 0:Body Frame, 1:Earth Frame
    // @User: Standard
    AP_GROUPINFO("GIMBAL_YAW_LK", 6, AP_AI_Track, _gimbal_yaw_lock, 1),

    // @Param: PREDICT_TIME
    // @DisplayName: Prediction Time
    // @Description: Time in seconds to predict target movement
    // @Range: 0 1.0
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("PREDICT_TIME", 7, AP_AI_Track, _predict_time, 0.15f),

    // @Param: VEL_GAIN
    // @DisplayName: Velocity Gain
    // @Description: Gain for velocity control
    // @Range: 0.1 2.0
    // @User: Standard
    AP_GROUPINFO("VEL_GAIN", 8, AP_AI_Track, _velocity_gain, 0.5f),

    // @Param: MAX_SPEED
    // @DisplayName: Maximum Speed
    // @Description: Maximum speed in m/s for drone following
    // @Range: 1 20
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("MAX_SPEED", 9, AP_AI_Track, _max_speed, 5.0f),

    // @Param: MIN_DIST
    // @DisplayName: Minimum Distance
    // @Description: Minimum distance in meters before moving
    // @Range: 0.1 5.0
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MIN_DIST", 10, AP_AI_Track, _min_distance, 0.5f),

    // @Param: ENABLE
    // @DisplayName: AI Tracking Enable
    // @Description: Master enable for AI tracking
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 11, AP_AI_Track, _enabled, 0),

    AP_GROUPEND
};

// Constructor
AP_AI_Track::AP_AI_Track()
{
    AP_Param::setup_object_defaults(this, var_info);
    
    _singleton = this;
    _ahrs = nullptr;
    _mount = nullptr;
    _tracking_active = false;
    _gimbal_initialized = false;
    _last_update_us = 0;
    
    _target_pos.zero();
    _target_vel.zero();
    _prev_target_pos.zero();
    _drone_pos_target.zero();
}

// Get singleton
AP_AI_Track *AP_AI_Track::get_singleton()
{
    return _singleton;
}

// Set enabled state
void AP_AI_Track::set_enabled(bool enabled)
{
    if (!enabled) {
        _tracking_active = false;
        _target_pos.zero();
        _target_vel.zero();
        // Stop any velocity commands
        send_velocity_command(Vector3f(0, 0, 0));
    }
    _enabled.set(enabled ? 1 : 0);
}

// Set target position
void AP_AI_Track::set_target_position(const Vector3f &pos)
{
    if (_enabled <= 0) {
        return;
    }
    
    const uint64_t now_us = AP_HAL::micros64();
    const float dt = (now_us - _last_update_us) * 1e-6f;
    
    // Estimate velocity if we have a valid time delta
    if (_last_update_us > 0 && dt > 0.01f) {
        const Vector3f vel_est = (pos - _prev_target_pos) / dt;
        const float alpha = 0.3f;  // Low-pass filter
        _target_vel = _target_vel * (1.0f - alpha) + vel_est * alpha;
        
        // Limit velocity
        const float max_vel = 30.0f;
        if (_target_vel.length() > max_vel) {
            _target_vel = _target_vel.normalized() * max_vel;
        }
    }
    
    _prev_target_pos = _target_pos;
    _target_pos = pos;
    _last_update_us = now_us;
    _tracking_active = true;
}

// Reset tracking
void AP_AI_Track::reset()
{
    _target_pos.zero();
    _target_vel.zero();
    _tracking_active = false;
    send_velocity_command(Vector3f(0, 0, 0));
    
    if (_mount != nullptr) {
        _mount->set_angle_target(0.0f, 0.0f, 0.0f, true);
    }
}

// Main update loop
void AP_AI_Track::update()
{
    // Check if enabled and active
    if (_enabled <= 0 || !_tracking_active) {
        return;
    }

   // Rate Limiter: Limit update rate to 50Hz (20ms) using class member
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < 20) { 
        return; 
    }
    _last_update_ms = now_ms;
    
    // Get AHRS instance
    if (_ahrs == nullptr) {
        _ahrs = AP_AHRS::get_singleton();
        if (_ahrs == nullptr) {
            return;
        }
    }
    
    if (!_ahrs->initialised()) {
        return;
    }
    
    // Initialize mount if needed
    if (_gimbal_enabled && _mount == nullptr) {
        _mount = AP_Mount::get_singleton();
        if (_mount != nullptr && !_gimbal_initialized) {
            _mount->set_angle_target(0.0f, 0.0f, 0.0f, true);
            _gimbal_initialized = true;
        }
    }
    
    // Update gimbal
    if (_gimbal_enabled && _mount != nullptr) {
        update_gimbal();
    }
    
    // Update drone follow
    if (_mode == 1) { // TRACK_MODE_DRONE_FOLLOW
        update_drone_follow();
    }
}

// Update gimbal control
void AP_AI_Track::update_gimbal()
{
    if (_mount == nullptr || _ahrs == nullptr) {
        return;
    }
    
    // Get drone position
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return;
    }
    
    // Predict target position
    Vector3f target_pos = _target_pos;
    if (!is_zero(_predict_time) && _target_vel.length() > 0.1f) {
        target_pos += _target_vel * _predict_time;
    }
    
    // Calculate vector from drone to target
    const Vector3f target_vector = target_pos - drone_pos;
    const float dist = target_vector.length();
    
    if (dist < 0.1f) {
        return;
    }
    
    // Calculate pitch and yaw
    float pitch = degrees(atan2f(-target_vector.z,
                                 sqrtf(sq(target_vector.x) + sq(target_vector.y))));
    float yaw = degrees(atan2f(target_vector.y, target_vector.x));
    
    // Apply gains and limits
    pitch = constrain_float(pitch * _gimbal_p_gain, -45.0f, 45.0f);
    yaw = constrain_float(yaw * _gimbal_p_gain, -180.0f, 180.0f);
    
    // Set gimbal target
    const bool earth_frame = (_gimbal_yaw_lock == 1);
    _mount->set_angle_target(0.0f, pitch, yaw, earth_frame);
}

// Update drone following
void AP_AI_Track::update_drone_follow()
{
    if (_ahrs == nullptr) {
        return;
    }
    
    // Get drone position
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return;
    }
    
    // Calculate desired follow position
    const float ang_rad = radians(_follow_angle);
    Vector3f offset;
    offset.x = _follow_distance * cosf(ang_rad);
    offset.y = _follow_distance * sinf(ang_rad);
    offset.z = -_follow_alt_offset;
    
    _drone_pos_target = _target_pos + offset;
    
    // Calculate error
    const Vector3f error = _drone_pos_target - drone_pos;
    
    // [FIX 2] Safe math for distance check
    if (error.length() < _min_distance || is_zero(error.length())) {
        send_velocity_command(Vector3f(0, 0, 0)); // Stop moving if close enough
        return;
    }
    
    // Calculate velocity command
    Vector3f vel = error * _velocity_gain;
    
    // [FIX 3] Safe normalization for speed limit
    float vel_len = vel.length();
    if (vel_len > _max_speed) {
        vel = vel * (_max_speed / vel_len); 
    }
    
    // Add target velocity feedforward
    if (_target_vel.length() > 0.1f) {
        vel += _target_vel * 0.3f;
    }
    
    // Send velocity command
    send_velocity_command(vel);
}

// Send velocity command
void AP_AI_Track::send_velocity_command(const Vector3f &vel)
{
#ifdef AP_COPTER_H
    AC_PosControl *pos = AP::pos_control();
    
    // [FIX 4] Controller Safety Check: Prevent crash if pos_control is not active
    if (pos != nullptr && pos->is_active_xy() && pos->is_active_z()) {
        pos->set_velocity_target(vel, false);
    }
#endif
    
    // [FIX 5] Removed Dynamic String Logging in Fast Loop to prevent 0x100000
    /*
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        // dynamic string logging ("Qfff") is CPU intensive and blocks the main thread
        // logger->Write("AITV", "TimeUS,VX,VY,VZ", "Qfff", AP_HAL::micros64(), vel.x, vel.y, vel.z);
    }
    */
}