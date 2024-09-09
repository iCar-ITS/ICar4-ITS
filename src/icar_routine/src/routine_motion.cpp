#include "icar_routine/routine.h"

void manual_throttle_steering(float throttle, float steering, int8_t transmission)
{
    /* Publishing the throttle and steering values to the topic `/throttle_steering`. */
    std_msgs::Int16MultiArray msg_throttle_steering;
    msg_throttle_steering.data.push_back(throttle);
    msg_throttle_steering.data.push_back(steering / steering_to_rad);
    pub_throttle_steering.publish(msg_throttle_steering);

    /* Publishing the transmission value to the topic `/transmission`. */
    std_msgs::Int8 msg_transmission;
    msg_transmission.data = transmission;
    pub_transmission.publish(msg_transmission);

    /* Update UI. */
    action_throttle = fmaxf(0, throttle);
    action_brake = -fminf(0, throttle);
}

void jalan_maju_setir_otomatis(float velocity, float steer, float slow_acceleration, float slow_deceleration, float emergency_brake, bool *is_stopped, bool *is_emergency)
{
    /* Calculating the time difference between the last time
    the function was called and the current time. */
    static ros::Time time_last, time_now;
    time_last = time_now;
    time_now = ros::Time::now();
    ros::Duration dt = time_now - time_last;

    static float velocity_input_buffer;
    static float steering_input_buffer;

    /* Reset the velocity output buffer and velocity input buffer
    if the time difference between the last time the function was
    called and the current time is greater than 1 second. */
    if (time_now - time_last > ros::Duration(1.0))
    {
        velocity_input_buffer = 0.0;
        steering_input_buffer = 0.0;
    }

    /* Calculate the velocity input buffer based on the velocity
    input and max allowed acceleration and deceleration. */
    if (velocity_input_buffer < velocity)
    {
        velocity_input_buffer += (0.10 * slow_acceleration + 0.25 * (1 - slow_acceleration)) * dt.toSec();
        if (velocity_input_buffer > velocity)
            velocity_input_buffer = velocity;
    }
    else if (velocity_input_buffer > velocity)
    {
        velocity_input_buffer -= (0.10 * slow_deceleration + 1.00 * (1 - slow_deceleration)) * dt.toSec();
        if (velocity_input_buffer < velocity)
            velocity_input_buffer = velocity;
    }

    /* Calculate maximum steering rate based on the velocity.
    The more the velocity, the less the steering rate. */
    static const float min_velocity = 5.0 / 3.6;               // 5 km/h
    static const float max_velocity = 15.0 / 3.6;              // 15 km/h
    static const float min_steering_rate = 2.5 * M_PI / 180.0; // 2.5 deg/s
    static const float max_steering_rate = 7.5 * M_PI / 180.0; // 7.5 deg/s
    static const float gradient_steering_rate = (min_steering_rate - max_steering_rate) / (max_velocity - min_velocity);

    float steering_rate = fmaxf(min_steering_rate,
                                fminf(max_steering_rate,
                                      gradient_steering_rate * (metric_velocity - min_velocity) + max_steering_rate));

    if (steering_input_buffer < steer)
    {
        steering_input_buffer += steering_rate * dt.toSec();
        if (steering_input_buffer > steer)
            steering_input_buffer = steer;
    }
    else if (steering_input_buffer > steer)
    {
        steering_input_buffer -= steering_rate * dt.toSec();
        if (steering_input_buffer < steer)
            steering_input_buffer = steer;
    }

    /* If the velocity input buffer is less than 0.1 m/s and the
    vehicle is not moving, then the vehicle is considered to be
    stopped. It will not move until the velocity input buffer is
    greater than 0.1 m/s. */
    static ros::Time time_stop = ros::Time::now();
    static bool status_stop = false;
    if (status_stop == false && velocity_input_buffer < 0.1 && metric_velocity < 0.5)
    {
        time_stop = ros::Time::now();

        status_stop = true;
        if (is_stopped != NULL)
            *is_stopped = true;
        _log.debug("Enter Stop Mode");
    }
    else if (status_stop == true && velocity_input_buffer > 0.1)
    {
        time_stop = ros::Time::now();

        status_stop = false;
        if (is_stopped != NULL)
            *is_stopped = false;
        _log.debug("Exit Stop Mode");
    }

    /* If the vehicle is in stop mode for more than 2 seconds,
    then set the emergency brake to 0.5. */
    if (status_stop == true && ros::Time::now() - time_stop > ros::Duration(2.0))
        emergency_brake = 0.5;

    /* If the emergency brake is greater than 0.5, then set the
    throttle proportional to the emergency brake value. */
    if (emergency_brake >= 0.5)
    {
        manual_throttle_steering(emergency_brake * -100,
                                 steering_input_buffer, 1);
        if (is_emergency != NULL)
            *is_emergency = true;
    }
    /* If the emergency brake is less than 0.5, then set the
    throttle equal to pid_velocity_maju.update(). */
    else
    {
        manual_throttle_steering(pid_velocity_maju.update(velocity_input_buffer, metric_velocity),
                                 steering_input_buffer, 1);
        if (is_emergency != NULL)  
            *is_emergency = false;
    }

    /* Update UI. */
    action_velocity = velocity_input_buffer;
    action_steering = steering_input_buffer;
}
