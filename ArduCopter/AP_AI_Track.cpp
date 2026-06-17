/*
 * AP_AI_Track.cpp - AI Target Tracking Implementation
 */

#include "AP_AI_Track.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

AP_AI_Track *AP_AI_Track::_singleton = nullptr;

const AP_Param::GroupInfo AP_AI_Track::var_info[] = {
    AP_GROUPINFO("MODE", 0, AP_AI_Track, _mode, 1),
    AP_GROUPINFO("FOLLOW_DIST", 1, AP_AI_Track, _follow_distance, 10.0f),
    AP_GROUPINFO("FOLLOW_ANGLE", 2, AP_AI_Track, _follow_angle, 0.0f),
    AP_GROUPINFO("GIMBAL_P", 3, AP_AI_Track, _gimbal_p_gain, 1.0f),
    AP_GROUPINFO("GIMBAL_EN", 4, AP_AI_Track, _gimbal_enabled, 1),
    AP_GROUPINFO("FOLLOW_ALT", 5, AP_AI_Track, _follow_alt_offset, 5.0f),
    AP_GROUPINFO("GIMBAL_YAW_LOCK", 6, AP_AI_Track, _gimbal_yaw_lock, 1),
    AP_GROUPEND
};

AP_AI_Track::AP_AI_Track()
{
    AP_Param::setup_object_defaults(this, var_info);
    _tracking_enabled = false;
    _target_pos.zero();
    _last_update_ms = 0;
    _ahrs = nullptr;
    _mount = nullptr;
    
    if (_singleton == nullptr) {
        _singleton = this;
    }
    
    _ahrs = AP_AHRS::get_singleton();
    _mount = AP_Mount::get_singleton();
    
    hal.console->printf("AP_AI_Track: Initialized\n");
}

AP_AI_Track *AP_AI_Track::get_singleton()
{
    if (_singleton == nullptr) {
        _singleton = new AP_AI_Track();
    }
    return _singleton;
}

void AP_AI_Track::update()
{
    if (!_tracking_enabled) {
        return;
    }
    
    if (_ahrs == nullptr) {
        _ahrs = AP_AHRS::get_singleton();
        if (_ahrs == nullptr) {
            return;
        }
    }
    
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return;
    }
    
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write("AITK", "TimeUS,TX,TY,TZ",
                     "Qfff",
                     AP_HAL::micros64(),
                     (double)_target_pos.x,
                     (double)_target_pos.y,
                     (double)_target_pos.z);
    }
}

void AP_AI_Track::set_target_position(const Vector3f &pos_target)
{
    _target_pos = pos_target;
    _tracking_enabled = true;
    _last_update_ms = AP_HAL::millis();
    
    hal.console->printf("AI Target: X=%.2f Y=%.2f Z=%.2f\n", 
                        (double)pos_target.x, 
                        (double)pos_target.y, 
                        (double)pos_target.z);
}

void AP_AI_Track::set_tracking_enabled(bool enable)
{
    _tracking_enabled = enable;
    hal.console->printf("AI Tracking: %s\n", enable ? "Enabled" : "Disabled");
}
