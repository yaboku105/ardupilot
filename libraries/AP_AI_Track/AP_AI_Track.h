#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Mount/AP_Mount.h>

class AP_AI_Track {
public:
    AP_AI_Track();

    // Singleton access
    static AP_AI_Track *get_singleton();
    
    // Parameter table
    static const struct AP_Param::GroupInfo var_info[];
    
    // Main update - call in fast loop
    void update();
    
    // Enable/disable tracking
    void set_enabled(bool enabled);
    bool enabled() const { return _enabled > 0; }
    
    // Set target position in NED frame
    void set_target_position(const Vector3f &pos);
    
    // Get target position
    Vector3f get_target_position() const { return _target_pos; }
    Vector3f get_target_velocity() const { return _target_vel; }
    
    // Reset tracking state
    void reset();
    
    // Check if tracking is active
    bool is_tracking() const { return _tracking_active; }

private:
    // Tracking modes
    enum TrackMode {
        TRACK_MODE_GIMBAL_ONLY = 0,
        TRACK_MODE_DRONE_FOLLOW = 1
    };
    
    // Internal update methods
    void update_gimbal();
    void update_drone_follow();
    void send_velocity_command(const Vector3f &vel);
    
    // Singleton instance
    static AP_AI_Track *_singleton;
    
    // External references
    AP_AHRS *_ahrs;
    AP_Mount *_mount;
    
    // Parameters
    AP_Int8     _mode;              // Tracking mode
    AP_Float    _follow_distance;   // Follow distance in meters
    AP_Float    _follow_angle;      // Follow angle in degrees
    AP_Float    _gimbal_p_gain;     // Gimbal P gain
    AP_Int8     _gimbal_enabled;    // Gimbal enable
    AP_Float    _follow_alt_offset; // Altitude offset in meters
    AP_Int8     _gimbal_yaw_lock;   // Yaw lock enable
    AP_Float    _predict_time;      // Prediction time in seconds
    AP_Float    _velocity_gain;     // Velocity gain
    AP_Float    _max_speed;         // Maximum speed in m/s
    AP_Float    _min_distance;      // Minimum distance in meters
    AP_Int8     _enabled;           // Master enable
    
    // State variables
    bool        _tracking_active;
    Vector3f    _target_pos;        // Current target position (NED)
    Vector3f    _target_vel;        // Target velocity (NED)
    Vector3f    _prev_target_pos;   // Previous target position
    Vector3f    _drone_pos_target;  // Drone position target
    uint64_t    _last_update_us;    // Last target position update time (micros)
    uint32_t    _last_update_ms;    // [ADDED] Last main loop update time for rate limiting (millis)
    bool        _gimbal_initialized;
};