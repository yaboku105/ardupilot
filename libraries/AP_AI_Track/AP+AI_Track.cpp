/*
 * AP_AI_Track.cpp - AI-assisted target tracking implementation
 * Using AP_Mount for gimbal control
 * Fixed all compilation issues
 */

#include "AP_AI_Track.h"
#include <AP_Mount/AP_Mount.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;

// Helper function to clamp values without using the macro
static inline float clamp_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// Singleton instance
AP_AI_Track *AP_AI_Track::_singleton = nullptr;

// Parameter table
const AP_Param::GroupInfo AP_AI_Track::var_info[] = {
    // @Param: MODE
    // @DisplayName: Tracking Mode
    // @Description: Controls how the tracking system operates
    // @Values: 0:Gimbal Only, 1:Drone Follow
    // @User: Standard
    AP_GROUPINFO("MODE", 0, AP_AI_Track, _mode, 1),

    // @Param: FOLLOW_DIST
    // @DisplayName: Follow Distance
    // @Description: Desired horizontal distance from target in meters
    // @Range: 1 100
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("FOLLOW_DIST", 1, AP_AI_Track, _follow_distance, 10.0f),

    // @Param: FOLLOW_ANGLE
    // @DisplayName: Follow Angle
    // @Description: Desired bearing from target in degrees (0=North, 90=East)
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("FOLLOW_ANGLE", 2, AP_AI_Track, _follow_angle, 0.0f),

    // @Param: GIMBAL_P
    // @DisplayName: Gimbal Proportional Gain
    // @Description: Proportional gain for gimbal pointing control
    // @Range: 0.1 5.0
    // @User: Standard
    AP_GROUPINFO("GIMBAL_P", 3, AP_AI_Track, _gimbal_p_gain, 1.0f),

    // @Param: GIMBAL_EN
    // @DisplayName: Gimbal Enable
    // @Description: Enable gimbal control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("GIMBAL_EN", 4, AP_AI_Track, _gimbal_enabled, 1),

    // @Param: FOLLOW_ALT
    // @DisplayName: Follow Altitude Offset
    // @Description: Altitude offset from target in meters (positive = above target)
    // @Range: -50 50
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("FOLLOW_ALT", 5, AP_AI_Track, _follow_alt_offset, 5.0f),

    // @Param: GIMBAL_YAW_LOCK
    // @DisplayName: Gimbal Yaw Lock
    // @Description: Lock gimbal yaw to vehicle heading or target
    // @Values: 0:Vehicle Heading (Body Frame), 1:Target Direction (Earth Frame)
    // @User: Standard
    AP_GROUPINFO("GIMBAL_YAW_LOCK", 6, AP_AI_Track, _gimbal_yaw_lock, 1),

    // @Param: PREDICT_TIME
    // @DisplayName: Prediction Time
    // @Description: Time in seconds to predict target position forward for smoother tracking
    // @Range: 0.0 1.0
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("PREDICT_TIME", 7, AP_AI_Track, _predict_time, 0.15f),

    // @Param: VEL_GAIN
    // @DisplayName: Velocity Gain
    // @Description: Gain for velocity control when following target
    // @Range: 0.1 2.0
    // @User: Standard
    AP_GROUPINFO("VEL_GAIN", 8, AP_AI_Track, _velocity_gain, 0.5f),

    // @Param: MAX_SPEED
    // @DisplayName: Maximum Follow Speed
    // @Description: Maximum speed for drone following in m/s
    // @Range: 1 20
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("MAX_SPEED", 9, AP_AI_Track, _max_speed, 5.0f),

    // @Param: MIN_DIST
    // @DisplayName: Minimum Distance
    // @Description: Minimum distance to target before stopping follow
    // @Range: 0.1 5.0
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MIN_DIST", 10, AP_AI_Track, _min_distance, 0.5f),

    AP_GROUPEND
};

// Constructor
AP_AI_Track::AP_AI_Track()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
    
    // Initialize references
    _ahrs = nullptr;
    _mount = nullptr;
    _vehicle = nullptr;
    
    // Initialize state variables
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

// Singleton accessor
AP_AI_Track *AP_AI_Track::get_singleton()
{
    return _singleton;
}

// Main update function
void AP_AI_Track::update()
{
    if (!_tracking_enabled || !_is_tracking) {
        return;
    }

    // Get AHRS reference if not already set
    if (_ahrs == nullptr) {
        _ahrs = AP_AHRS::get_singleton();
        if (_ahrs == nullptr) {
            return;
        }
    }

    // Get AP_Mount reference if not already set and gimbal is enabled
    if (_gimbal_enabled && _mount == nullptr) {
        _mount = AP_Mount::get_singleton();
        if (_mount != nullptr && !_gimbal_initialized) {
            // Initialize gimbal to neutral position
            _mount->set_angle_target(0.0f, 0.0f, 0.0f, true);
            _gimbal_initialized = true;
        }
    }

    // Update frame counter
    _frame_count++;

    // Update gimbal control
    if (_gimbal_enabled && _mount != nullptr) {
        update_gimbal_control();
    }

    // Update drone control based on mode
    if (_mode == TRACK_MODE_DRONE_FOLLOW) {
        update_drone_control();
    }
}

// Update gimbal control using AP_Mount
void AP_AI_Track::update_gimbal_control()
{
    if (_mount == nullptr) {
        return;
    }

    // Get current drone position
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return;
    }

    // Calculate vector from drone to target with prediction
    Vector3f target_pos = _target_pos;
    
    // Apply velocity prediction for smoother tracking
    float predict_time = _predict_time;
    if (predict_time > 0.001f && _target_vel.length() > 0.1f) {
        target_pos += _target_vel * predict_time;
    }

    Vector3f target_vector = target_pos - drone_pos;
    float target_distance = target_vector.length();

    if (target_distance < 0.1f) {
        return; // Target is too close
    }

    // Calculate required gimbal angles in NED frame
    float pitch_angle_deg = degrees(atan2f(-target_vector.z,
                               sqrtf(sq(target_vector.x) + sq(target_vector.y))));
    float yaw_angle_deg = degrees(atan2f(target_vector.y, target_vector.x));

    // Apply proportional gain
    float gimbal_p = _gimbal_p_gain;
    pitch_angle_deg *= gimbal_p;
    yaw_angle_deg *= gimbal_p;

    // Clamp angles using our custom clamp function
    const float max_pitch = 45.0f; // degrees
    const float max_yaw = 180.0f; // degrees
    pitch_angle_deg = clamp_float(pitch_angle_deg, -max_pitch, max_pitch);
    yaw_angle_deg = clamp_float(yaw_angle_deg, -max_yaw, max_yaw);

    // Determine if yaw should be earth-frame (locked) or body-frame (follow)
    bool yaw_is_earth_frame = (_gimbal_yaw_lock == 1);

    // Set the angle target using AP_Mount
    _mount->set_angle_target(0.0f,          // roll_deg
                             pitch_angle_deg, // pitch_deg
                             yaw_angle_deg,   // yaw_deg
                             yaw_is_earth_frame);

    // Store target angles for logging (in radians)
    _gimbal_angles = Vector3f(0, radians(pitch_angle_deg), radians(yaw_angle_deg));

    // Log gimbal data
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write("AITG", "TimeUS,TargetX,TargetY,TargetZ,Pitch,Yaw,Distance,PredX,PredY,PredZ",
                     "Qfffffffff",
                     AP_HAL::micros64(),
                     (double)_target_pos.x,
                     (double)_target_pos.y,
                     (double)_target_pos.z,
                     (double)pitch_angle_deg,
                     (double)yaw_angle_deg,
                     (double)target_distance,
                     (double)target_pos.x,
                     (double)target_pos.y,
                     (double)target_pos.z);
    }
}

// Update drone control
void AP_AI_Track::update_drone_control()
{
    // Get current drone position
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return;
    }

    // Calculate desired position relative to target
    Vector3f desired_offset;
    
    // Horizontal offset (follow distance and angle)
    float follow_angle_rad = radians(_follow_angle);
    float follow_distance = _follow_distance;
    desired_offset.x = follow_distance * cosf(follow_angle_rad);
    desired_offset.y = follow_distance * sinf(follow_angle_rad);
    
    // Vertical offset (altitude above target)
    // Extract float value first, then negate
    float follow_alt = _follow_alt_offset;
    desired_offset.z = -follow_alt;  // NED frame: negative Z is up

    // Calculate target position for drone
    _drone_pos_target = _target_pos + desired_offset;

    // Calculate position error
    Vector3f error = _drone_pos_target - drone_pos;
    float error_magnitude = error.length();

    // Only move if error is significant
    float min_dist = _min_distance;
    if (error_magnitude < min_dist) {
        return;
    }

    // Calculate velocity command (P controller)
    float vel_gain = _velocity_gain;
    Vector3f velocity_command = error * vel_gain;

    // Limit velocity
    float max_speed = _max_speed;
    if (velocity_command.length() > max_speed) {
        velocity_command = velocity_command.normalized() * max_speed;
    }

    // Add feed-forward from target velocity for better tracking
    if (_target_vel.length() > 0.1f) {
        float ff_gain = 0.3f;
        velocity_command += _target_vel * ff_gain;
    }

    // Send velocity command to vehicle
    send_velocity_command(velocity_command);

    // Log drone tracking data
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write("AITD", "TimeUS,TargetX,TargetY,TargetZ,ErrorX,ErrorY,ErrorZ,VelX,VelY,VelZ,Dist",
                     "Qfffffffffff",
                     AP_HAL::micros64(),
                     (double)_drone_pos_target.x,
                     (double)_drone_pos_target.y,
                     (double)_drone_pos_target.z,
                     (double)error.x,
                     (double)error.y,
                     (double)error.z,
                     (double)velocity_command.x,
                     (double)velocity_command.y,
                     (double)velocity_command.z,
                     (double)error_magnitude);
    }
}

// Send velocity command to vehicle
void AP_AI_Track::send_velocity_command(const Vector3f &velocity)
{
    // This function sends velocity commands to the vehicle
    // The implementation depends on the vehicle type
    
    #ifdef AP_COPTER_H
    // For ArduCopter
    #include <AC_AttitudeControl/AC_PosControl.h>
    AC_PosControl *pos_control = AP::pos_control();
    if (pos_control != nullptr) {
        pos_control->set_velocity_target(velocity, false);
    }
    #endif

    #ifdef AP_PLANE_H
    // For ArduPlane - would need different implementation
    // Plane uses different navigation methods
    #endif

    // If no specific vehicle implementation, log the command
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write("AITV", "TimeUS,VelX,VelY,VelZ",
                     "Qfff",
                     AP_HAL::micros64(),
                     (double)velocity.x,
                     (double)velocity.y,
                     (double)velocity.z);
    }
}

// Point gimbal at target using AP_Mount
bool AP_AI_Track::point_gimbal_at_target(const Vector3f &target_pos, float yaw_override)
{
    if (_mount == nullptr) {
        return false;
    }

    // Get current drone position
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return false;
    }

    Vector3f target_vector = target_pos - drone_pos;
    float target_distance = target_vector.length();

    if (target_distance < 0.1f) {
        return false;
    }

    float pitch_angle_deg = degrees(atan2f(-target_vector.z,
                               sqrtf(sq(target_vector.x) + sq(target_vector.y))));
    float yaw_angle_deg = degrees(atan2f(target_vector.y, target_vector.x));

    // Apply yaw override if provided
    if (yaw_override != 0) {
        yaw_angle_deg = yaw_override;
    }

    // Clamp angles using our custom clamp function
    pitch_angle_deg = clamp_float(pitch_angle_deg, -45.0f, 45.0f);
    yaw_angle_deg = clamp_float(yaw_angle_deg, -180.0f, 180.0f);

    // Set target using earth frame (locked yaw)
    _mount->set_angle_target(0.0f, pitch_angle_deg, yaw_angle_deg, true);
    return true;
}

// Set tracking enabled/disabled
void AP_AI_Track::set_tracking_enabled(bool enable)
{
    _tracking_enabled = enable;
    
    if (!enable) {
        _is_tracking = false;
        
        // Reset gimbal to neutral position if enabled
        if (_gimbal_enabled && _mount != nullptr) {
            _mount->set_angle_target(0.0f, 0.0f, 0.0f, true);
        }
        
        // Clear velocity command
        send_velocity_command(Vector3f(0, 0, 0));
    }
}

// Set target position with velocity prediction
void AP_AI_Track::set_target_position(const Vector3f &pos_target)
{
    uint64_t current_time = AP_HAL::micros64();
    float dt = (current_time - _last_update_us) / 1000000.0f;
    
    // Calculate velocity if significant time has passed
    if (_last_update_us > 0 && dt > 0.01f) {
        Vector3f new_vel = (pos_target - _prev_target_pos) / dt;
        
        // Apply low-pass filter to velocity
        const float filter_alpha = 0.3f;
        _target_vel = _target_vel * (1.0f - filter_alpha) + new_vel * filter_alpha;
        
        // Limit velocity to reasonable values
        float max_velocity = 30.0f; // m/s
        if (_target_vel.length() > max_velocity) {
            _target_vel = _target_vel.normalized() * max_velocity;
        }
    }
    
    // Update state
    _prev_target_pos = _target_pos;
    _target_pos = pos_target;
    _last_update_us = current_time;
    _is_tracking = true;
}

// Run tracking with prediction
void AP_AI_Track::run_tracking()
{
    update();
}

// Calculate errors and control vectors
void AP_AI_Track::calculate_errors(const Vector3f &pos_target, const Vector3f &pos_drone,
                                   Vector3f &gimbal_error, Vector3f &drone_error)
{
    Vector3f target_vector = pos_target - pos_drone;
    
    float target_pitch = atan2f(-target_vector.z, 
                                sqrtf(sq(target_vector.x) + sq(target_vector.y)));
    float target_yaw = atan2f(target_vector.y, target_vector.x);
    
    gimbal_error = Vector3f(0, target_pitch, target_yaw);
    
    Vector3f desired_offset;
    float follow_angle_rad = radians(_follow_angle);
    float follow_distance = _follow_distance;
    desired_offset.x = follow_distance * cosf(follow_angle_rad);
    desired_offset.y = follow_distance * sinf(follow_angle_rad);
    
    // Extract float value first, then negate
    float follow_alt = _follow_alt_offset;
    desired_offset.z = -follow_alt;
    
    drone_error = (pos_target + desired_offset) - pos_drone;
}

// Get target vector for gimbal
Vector3f AP_AI_Track::get_target_vector_for_gimbal(const Vector3f &pos_target, 
                                                   const Vector3f &pos_vehicle)
{
    return pos_target - pos_vehicle;
}

// Helper functions
float AP_AI_Track::radians_to_degrees(float rad)
{
    return rad * 57.295779513f;
}

Vector3f AP_AI_Track::get_gimbal_angles() const
{
    return _gimbal_angles;
}

Vector3f AP_AI_Track::get_drone_target() const
{
    return _drone_pos_target;
}

Vector3f AP_AI_Track::get_target_position() const
{
    return _target_pos;
}

Vector3f AP_AI_Track::get_target_velocity() const
{
    return _target_vel;
}

// Get current status for GCS
void AP_AI_Track::get_status(bool &tracking, Vector3f &target_pos, Vector3f &gimbal_angles) const
{
    tracking = _is_tracking;
    target_pos = _target_pos;
    gimbal_angles = _gimbal_angles;
}

// Reset tracking system
void AP_AI_Track::reset()
{
    _target_pos.zero();
    _target_vel.zero();
    _prev_target_pos.zero();
    _gimbal_angles.zero();
    _drone_pos_target.zero();
    _is_tracking = false;
    _frame_count = 0;
    
    if (_gimbal_enabled && _mount != nullptr) {
        _mount->set_angle_target(0.0f, 0.0f, 0.0f, true);
    }
    
    send_velocity_command(Vector3f(0, 0, 0));
}

// Check if gimbal is initialized
bool AP_AI_Track::is_gimbal_initialized() const
{
    return _gimbal_initialized;
}

// Get current distance to target
float AP_AI_Track::get_target_distance() const
{
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return 0.0f;
    }
    return (_target_pos - drone_pos).length();
}

// Get target relative position in body frame
Vector3f AP_AI_Track::get_target_body_frame() const
{
    // Check if AHRS is available
    if (_ahrs == nullptr) {
        return Vector3f(0, 0, 0);
    }
    
    // Get current drone position
    Vector3f drone_pos;
    if (!_ahrs->get_relative_position_NED_home(drone_pos)) {
        return Vector3f(0, 0, 0);
    }
    
    // Calculate target vector in NED frame
    Vector3f target_vector = _target_pos - drone_pos;
    
    // FIXED: get_rotation_body_to_ned() returns the matrix directly
    Matrix3f rotation_matrix = _ahrs->get_rotation_body_to_ned();
    
    // Transpose to get NED to body rotation and apply to target vector
    return rotation_matrix.transposed() * target_vector;
}