/*
 * AP_AI_Track.h - API for AI-assisted target tracking using AP_Mount
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>


// Forward declaration for AP_Mount
class AP_Mount;
class AP_Vehicle;

class AP_AI_Track {
public:
    friend class Parameters;
    friend class ParametersG2;
    
    AP_AI_Track();

    /* Singleton accessor */
    static AP_AI_Track *get_singleton();
    bool enabled() const { return _enable.get() > 0; }
    /* Main update function - call at 50Hz */
    void update();

    /* Set a new 3D target position (in local NED frame) */
    void set_target_position(const Vector3f &pos_target);

    /* Get current target position */
    Vector3f get_target_position() const;

    /* Get current target velocity */
    Vector3f get_target_velocity() const;

    /* Check if a valid target is currently being tracked */
    bool is_tracking() const { return _is_tracking; }

    /* Enable or disable the tracking system */
    void set_tracking_enabled(bool enable);

    /* Run tracking with prediction */
    void run_tracking();

    /* Point gimbal at target using AP_Mount */
    bool point_gimbal_at_target(const Vector3f &target_pos, float yaw_override = 0);

    /* Get current gimbal angles (radians) */
    Vector3f get_gimbal_angles() const;

    /* Get current drone target position */
    Vector3f get_drone_target() const;

    /* Get current status */
    void get_status(bool &tracking, Vector3f &target_pos, Vector3f &gimbal_angles) const;

    /* Reset tracking system */
    void reset();

    /* Check if gimbal is initialized */
    bool is_gimbal_initialized() const;

    /* Get current distance to target */
    float get_target_distance() const;

    /* Get target relative position in body frame */
    Vector3f get_target_body_frame() const;

    // Enum for tracking control modes
    enum TrackingMode {
        TRACK_MODE_GIMBAL_ONLY = 0,
        TRACK_MODE_DRONE_FOLLOW = 1
    };

    // Parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    /* Gimbal and drone controllers */
    void update_gimbal_control();
    void update_drone_control();

    /* Calculate errors and control vectors */
    void calculate_errors(const Vector3f &pos_target, const Vector3f &pos_drone,
                          Vector3f &gimbal_error, Vector3f &drone_error);

    /* Get target vector for gimbal */
    Vector3f get_target_vector_for_gimbal(const Vector3f &pos_target, const Vector3f &pos_vehicle);

    /* Send velocity command to vehicle */
    void send_velocity_command(const Vector3f &velocity);

    /* Helper function */
    float radians_to_degrees(float rad);

    // State variables
    bool _tracking_enabled;
    bool _is_tracking;
    bool _gimbal_initialized;
    Vector3f _target_pos;          // Target position in NED frame
    Vector3f _target_vel;          // Target velocity
    Vector3f _prev_target_pos;     // Previous target position for velocity calc
    Vector3f _gimbal_angles;       // Current gimbal target angles (radians)
    Vector3f _drone_pos_target;    // Drone target position
    uint64_t _last_update_us;      // Last update time
    uint32_t _frame_count;         // Frame counter

    // Parameters
    AP_Int8 _mode;                // Tracking mode
    AP_Float _follow_distance;    // Desired horizontal distance from target
    AP_Float _follow_angle;       // Desired bearing from target
    AP_Float _gimbal_p_gain;      // Gimbal proportional gain
    AP_Int8 _gimbal_enabled;      // Enable gimbal control
    AP_Float _follow_alt_offset;  // Altitude offset from target
    AP_Int8 _gimbal_yaw_lock;     // Yaw lock mode (0=body, 1=earth)
    AP_Float _predict_time;       // Prediction time for smoother tracking
    AP_Float _velocity_gain;      // Gain for velocity control
    AP_Float _max_speed;          // Maximum follow speed
    AP_Float _min_distance;       // Minimum distance before stopping follow
    AP_Int8 _enable;
    // References to ArduPilot libraries
    AP_AHRS *_ahrs;
    AP_Mount *_mount;
    AP_Vehicle *_vehicle;
  
    // Singleton
    static AP_AI_Track *_singleton;
    
};