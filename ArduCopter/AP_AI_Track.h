/*
 * AP_AI_Track.h - AI Target Tracking for ArduPilot
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mount/AP_Mount.h>

class AP_AI_Track {
public:
    AP_AI_Track();
    static AP_AI_Track *get_singleton();
    void update();
    void set_target_position(const Vector3f &pos_target);
    void set_tracking_enabled(bool enable);
    bool is_tracking() const { return _tracking_enabled; }
    Vector3f get_target_position() const { return _target_pos; }
    static const struct AP_Param::GroupInfo var_info[];
    
private:
    bool _tracking_enabled;
    Vector3f _target_pos;
    uint32_t _last_update_ms;
    AP_AHRS *_ahrs;
    AP_Mount *_mount;
    
    AP_Int8 _mode;
    AP_Float _follow_distance;
    AP_Float _follow_angle;
    AP_Float _gimbal_p_gain;
    AP_Int8 _gimbal_enabled;
    AP_Float _follow_alt_offset;
    AP_Int8 _gimbal_yaw_lock;
    
    static AP_AI_Track *_singleton;
};
