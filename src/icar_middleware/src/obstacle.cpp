#include "boost/thread/mutex.hpp"
#include "icar_middleware/obstacle_1_data.h"
#include "icar_middleware/obstacle_1_parameter.h"
#include "icar_middleware/obstacle_2_data.h"
#include "icar_middleware/obstacle_2_parameter.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "ps_ros_lib/help_log.h"
#include "ps_ros_lib/help_marker.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

#define LIMIT_X_MIN 0.00
#define LIMIT_X_MAX 25.00
#define LIMIT_Y_MIN -2.25
#define LIMIT_Y_MAX 2.25

#define EMERGENCY_X_MIN 0.00
#define EMERGENCY_X_MAX 2.50
#define EMERGENCY_Y_MIN -1.00
#define EMERGENCY_Y_MAX 1.00

#define BLINDSPOT_X_MIN -2.50
#define BLINDSPOT_X_MAX 2.50
#define BLINDSPOT_Y_MIN -1.75
#define BLINDSPOT_Y_MAX -1.00

//=====Prototype
void cllbck_tim_10hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_lidar_base_floor_points(const sensor_msgs::PointCloud2::ConstPtr &msg);
void cllbck_sub_lidar_base_obstacle_points(const sensor_msgs::PointCloud2::ConstPtr &msg);
void cllbck_sub_obstacle_1_parameter(const icar_middleware::obstacle_1_parameter::ConstPtr &msg);
void cllbck_sub_obstacle_2_parameter(const icar_middleware::obstacle_2_parameter::ConstPtr &msg);

int obstacle_init();
int obstacle_routine();

void obstacle_1_update();
void obstacle_2_update();

//=====Timer
ros::Timer tim_10hz;
ros::Timer tim_100hz;
//=====Susbcriber
ros::Subscriber sub_lidar_base_floor_points;
ros::Subscriber sub_lidar_base_obstacle_points;
ros::Subscriber sub_obstacle_1_parameter;
ros::Subscriber sub_obstacle_2_parameter;
//=====Publisher
ros::Publisher pub_obstacle_1_data;
ros::Publisher pub_obstacle_2_data;
//=====TransformListener
tf::TransformListener *transform_listener;
//=====Help
help_log _log;
help_marker _marker;

//-----Point cloud
//================
pcl::PointCloud<pcl::PointXYZ> lidar_base_floor_points;
pcl::PointCloud<pcl::PointXYZ> lidar_base_obstacle_points;

//-----Obstacle data dan parameter
//================================
icar_middleware::obstacle_1_data obstacle_1_data;
icar_middleware::obstacle_2_data obstacle_2_data;
icar_middleware::obstacle_1_parameter obstacle_1_parameter;
icar_middleware::obstacle_2_parameter obstacle_2_parameter;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_10hz = NH.createTimer(ros::Duration(0.10), cllbck_tim_10hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_lidar_base_floor_points = NH.subscribe("/lidar/base/floor_points", 1, cllbck_sub_lidar_base_floor_points);
    sub_lidar_base_obstacle_points = NH.subscribe("/lidar/base/obstacle_points", 1, cllbck_sub_lidar_base_obstacle_points);
    sub_obstacle_1_parameter = NH.subscribe("/obstacle/1/parameter", 1, cllbck_sub_obstacle_1_parameter);
    sub_obstacle_2_parameter = NH.subscribe("/obstacle/2/parameter", 1, cllbck_sub_obstacle_2_parameter);
    //=====Publisher
    pub_obstacle_1_data = NH.advertise<icar_middleware::obstacle_1_data>("/obstacle/1/data", 1);
    pub_obstacle_2_data = NH.advertise<icar_middleware::obstacle_2_data>("/obstacle/2/data", 1);
    //=====TransformListener
    transform_listener = new tf::TransformListener(NH);
    //=====Help
    _log.init(NH);
    _marker.init(NH);

    if (obstacle_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();

    tim_10hz.stop();
    tim_100hz.stop();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_10hz(const ros::TimerEvent &event)
{
    if (obstacle_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_lidar_base_floor_points(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, lidar_base_floor_points);
}

void cllbck_sub_lidar_base_obstacle_points(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, lidar_base_obstacle_points);

    obstacle_1_update();
    obstacle_2_update();
}

void cllbck_sub_obstacle_1_parameter(const icar_middleware::obstacle_1_parameter::ConstPtr &msg)
{
    obstacle_1_parameter = *msg;
}

void cllbck_sub_obstacle_2_parameter(const icar_middleware::obstacle_2_parameter::ConstPtr &msg)
{
    obstacle_2_parameter = *msg;
}

//------------------------------------------------------------------------------
//==============================================================================

int obstacle_init()
{
    ros::Duration(2).sleep();

    return 0;
}

int obstacle_routine()
{
    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

void obstacle_1_update()
{
    static tf::StampedTransform transform;
    static bool transform_initialized = false;

    /* Check whether the transform is already initialized.
    If not, try to get the transform from the tf tree. */
    if (!transform_initialized)
    {
        try
        {
            transform_listener->lookupTransform("base_link", "lidar_front_link", ros::Time(0), transform);
            transform_initialized = true;
        }
        catch (...)
        {
        }
    }

    //==================================

    static const float angle_start = -M_PI_2;
    static const float angle_stop = M_PI_2;
    static const float angle_step = (angle_stop - angle_start) / 32;

    //==================================

    float distance_max[32];
    float distance_zero[32];
    float distance_one[32];
    for (int i = 0; i < 32; i++)
    {
        distance_max[i] = obstacle_1_parameter.laser_scan_distance;

        if (distance_max[i] * sinf(angle_start + angle_step * (i + 0.5)) > LIMIT_Y_MAX)
            distance_max[i] = LIMIT_Y_MAX / sinf(angle_start + angle_step * (i + 0.5));
        else if (distance_max[i] * sinf(angle_start + angle_step * (i + 0.5)) < LIMIT_Y_MIN)
            distance_max[i] = LIMIT_Y_MIN / sinf(angle_start + angle_step * (i + 0.5));

        if (distance_max[i] * cosf(angle_start + angle_step * (i + 0.5)) > LIMIT_X_MAX)
            distance_max[i] = LIMIT_X_MAX / cosf(angle_start + angle_step * (i + 0.5));
        else if (distance_max[i] * cosf(angle_start + angle_step * (i + 0.5)) < LIMIT_X_MIN)
            distance_max[i] = LIMIT_X_MIN / cosf(angle_start + angle_step * (i + 0.5));

        distance_zero[i] = distance_max[i] * 0.5;
        distance_one[i] = distance_max[i] * 1.0;
    }

    //==================================

    float distance_raw[32];
    float distance_normalized[32];
    for (int i = 0; i < 32; i++)
    {
        distance_raw[i] = distance_max[i];
        distance_normalized[i] = 1.0;
    }

    //==================================

    for (int i = 0; i < lidar_base_obstacle_points.points.size(); i++)
    {
        float dx = lidar_base_obstacle_points.points[i].x - transform.getOrigin().x();
        float dy = lidar_base_obstacle_points.points[i].y - transform.getOrigin().y();
        float a = atan2f(dy, dx);

        //------------------------------

        /* The code is checking if the value of variable `a` is within a certain range defined by the variables
        `angle_start` and `angle_stop`. If `a` is less than `angle_start` or greater than `angle_stop`, the
        code will skip to the next iteration of the loop using the `continue` statement. */
        if (a < angle_start || a > angle_stop)
            continue;

        //------------------------------

        /* The code is calculating the index of a value 'a' in an array of angles, where 'angle_start' is the
        starting angle and 'angle_step' is the step size between consecutive angles. The index is calculated
        by subtracting the starting angle from 'a' and dividing the result by the step size. */
        int index = (a - angle_start) / angle_step;

        /* The above code is calculating the distance between two points in a two-dimensional space using the
        Pythagorean theorem. The variables `dx` and `dy` represent the differences in the x and y
        coordinates of the two points, respectively. The `sqrtf` function calculates the square root of the
        sum of the squares of `dx` and `dy`, which gives the distance between the two points. The result is
        stored in the variable `r`. */
        float r = sqrtf(dx * dx + dy * dy);

        /* The code is checking if a new distance value (r) is less than the current distance value stored in
        the array at the given index. If it is, then the new distance value is stored in the array at that
        index. Additionally, the code calculates the normalized distance value by subtracting the distance
        zero value from the new distance value and dividing it by the difference between the distance zero
        and distance one values. This normalized distance value is then clamped between 0 and 1 using the
        fminf and fmaxf functions. */
        if (r < distance_raw[index])
        {
            distance_raw[index] = r;
            distance_normalized[index] = fminf(1.0, fmaxf(0.0, (r - distance_zero[index]) / (distance_one[index] - distance_zero[index])));
        }
    }

    //==================================

    std::vector<geometry_msgs::Point> ps;
    geometry_msgs::Point p;

    p.z = transform.getOrigin().z();

    ps.clear();
    for (int i = 0; i < 32; i++)
    {
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        ps.push_back(p);
        p.x += distance_max[i] * cosf(angle_start + angle_step * (i + 0.5));
        p.y += distance_max[i] * sinf(angle_start + angle_step * (i + 0.5));
        ps.push_back(p);
    }

    _marker.line_list("base_link", "obstacle_1", 1, ps, 0.87, 0.47, 0.34, 1, 0.05);

    ps.clear();
    for (int i = 0; i < 32; i++)
    {
        p.x = transform.getOrigin().x() + distance_raw[i] * cosf(angle_start + angle_step * (i + 0.5));
        p.y = transform.getOrigin().y() + distance_raw[i] * sinf(angle_start + angle_step * (i + 0.5));
        ps.push_back(p);
    }

    _marker.sphere_list("base_link", "obstacle_1", 2, ps, 0.91, 0.67, 0.60, 1, 0.25, 0.25, 0.25);

    //==================================

    /* The above code is declaring a boolean variable named "is_emergency" and initializing it with the
    value "false". */
    bool is_emergency = false;

    /* The above code is iterating through an array of distance values and calculating the x and y
    coordinates for each value based on a given angle. It then checks if the x and y coordinates fall
    within a certain range (EMERGENCY_X_MIN, EMERGENCY_X_MAX, EMERGENCY_Y_MIN, EMERGENCY_Y_MAX) and
    sets a boolean variable is_emergency to true if any of the coordinates fall within that range. */
    for (int i = 0; i < 32; i++)
    {
        float x = distance_raw[i] * cosf(angle_start + angle_step * (i + 0.5));
        float y = distance_raw[i] * sinf(angle_start + angle_step * (i + 0.5));

        if (x > EMERGENCY_X_MIN && x < EMERGENCY_X_MAX && y > EMERGENCY_Y_MIN && y < EMERGENCY_Y_MAX)
        {
            is_emergency = true;
            break;
        }
    }

    /* The code is updating the output value for emergency situation of obstacle 1 based on whether it is
    an emergency or not. If it is an emergency, the output value is updated as a weighted average of
    the previous output value and 1.0, with a weight of 0.15 given to the new value. If it is not an
    emergency, the output value is updated as a weighted average of the previous output value and 0.0,
    with a weight of 0.15 given to the new value. The purpose of this code is to adjust the output
    value of obstacle 1 */
    if (is_emergency)
        obstacle_1_data.output_emergency = 0.85 * obstacle_1_data.output_emergency + 0.15 * 1.0;
    else
        obstacle_1_data.output_emergency = 0.85 * obstacle_1_data.output_emergency + 0.15 * 0.0;

    //==================================

    /* To influence steering output. Note that index 0 i on the right side of the car and index 31 is
    on the left side of the car. */
    static const float k_steering_to_right[32] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                                  0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.00, 0.00};
    /* To influence steering output. Note that index 0 i on the right side of the car and index 31 is
    on the left side of the car. */
    static const float k_steering_to_left[32] = {0.00, 0.00, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50,
                                                 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
    /* To influence velocity output. Note that index 0 i on the right side of the car and index 31 is
    on the left side of the car. */
    static const float k_velocity_to_right[32] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.50, 0.50,
                                                  0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.00, 0.00};
    /* To influence velocity output. Note that index 0 i on the right side of the car and index 31 is
    on the left side of the car. */
    static const float k_velocity_to_left[32] = {0.00, 0.00, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50,
                                                 0.50, 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
    /* To influence following velocity output. Note that index 0 i on the right side of the car and index 31 is
    on the left side of the car. */
    static const float k_velocity_following[32] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.25, 0.25, 0.50, 0.50,
                                                   0.50, 0.50, 0.25, 0.25, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

    /* The above code is calculating weighted averages of different control parameters (steering and
    velocity) based on the distance of an obstacle from the vehicle. It is using pre-defined
    coefficients (k_steering_to_right, k_steering_to_left, k_velocity_to_right, k_velocity_to_left,
    k_velocity_following) to calculate the numerator and denominator of each weighted average. The final
    output values are then updated using a weighted average of the previous output values and the newly
    calculated values. */
    float num_steering_to_right = 0.0, den_steering_to_right = 0.0;
    float num_steering_to_left = 0.0, den_steering_to_left = 0.0;
    float num_velocity_to_right = 0.0, den_velocity_to_right = 0.0;
    float num_velocity_to_left = 0.0, den_velocity_to_left = 0.0;
    float num_velocity_following = 0.0, den_velocity_following = 0.0;
    for (int i = 0; i < 32; i++)
    {
        num_steering_to_right += k_steering_to_right[i] * (1 - distance_normalized[i]);
        den_steering_to_right += k_steering_to_right[i];
        num_steering_to_left += k_steering_to_left[i] * (1 - distance_normalized[i]);
        den_steering_to_left += k_steering_to_left[i];
        num_velocity_to_right += k_velocity_to_right[i] * (1 - distance_normalized[i]);
        den_velocity_to_right += k_velocity_to_right[i];
        num_velocity_to_left += k_velocity_to_left[i] * (1 - distance_normalized[i]);
        den_velocity_to_left += k_velocity_to_left[i];
        num_velocity_following += k_velocity_following[i] * (1 - distance_normalized[i]);
        den_velocity_following += k_velocity_following[i];
    }
    obstacle_1_data.output_steering_to_right = 0.85 * obstacle_1_data.output_steering_to_right + 0.15 * (num_steering_to_right / den_steering_to_right);
    obstacle_1_data.output_steering_to_left = 0.85 * obstacle_1_data.output_steering_to_left + 0.15 * (num_steering_to_left / den_steering_to_left);
    obstacle_1_data.output_velocity_to_right = 0.85 * obstacle_1_data.output_velocity_to_right + 0.15 * (num_velocity_to_right / den_velocity_to_right);
    obstacle_1_data.output_velocity_to_left = 0.85 * obstacle_1_data.output_velocity_to_left + 0.15 * (num_velocity_to_left / den_velocity_to_left);
    obstacle_1_data.output_velocity_following = 0.85 * obstacle_1_data.output_velocity_following + 0.15 * (num_velocity_following / den_velocity_following);

    //==================================

    if (obstacle_1_parameter.obstacle_status_steering == false)
    {
        obstacle_1_data.output_steering_to_right = 0.0;
        obstacle_1_data.output_steering_to_left = 0.0;
    }
    if (obstacle_1_parameter.obstacle_status_velocity == false)
    {
        obstacle_1_data.output_velocity_to_right = 0.0;
        obstacle_1_data.output_velocity_to_left = 0.0;
        obstacle_1_data.output_velocity_following = 0.0;
    }

    //==================================

    pub_obstacle_1_data.publish(obstacle_1_data);
}

void obstacle_2_update()
{
    static tf::StampedTransform transform_base_to_body;
    static tf::StampedTransform transform_body_to_lidar_rearright;
    static bool transform_initialized = false;

    if (!transform_initialized)
    {
        try
        {
            transform_listener->lookupTransform("base_link", "body_link", ros::Time(0), transform_base_to_body);
            transform_listener->lookupTransform("body_link", "lidar_rearright_link", ros::Time(0), transform_body_to_lidar_rearright);
            transform_initialized = true;
        }
        catch (...)
        {
        }
    }

    //==================================

    std::vector<geometry_msgs::Point> ps;
    geometry_msgs::Point p;

    p.z = transform_body_to_lidar_rearright.getOrigin().z();

    p.x = BLINDSPOT_X_MIN;
    p.y = BLINDSPOT_Y_MIN;
    ps.push_back(p);
    p.x = BLINDSPOT_X_MAX;
    p.y = BLINDSPOT_Y_MIN;
    ps.push_back(p);
    p.x = BLINDSPOT_X_MAX;
    p.y = BLINDSPOT_Y_MAX;
    ps.push_back(p);
    p.x = BLINDSPOT_X_MIN;
    p.y = BLINDSPOT_Y_MAX;
    ps.push_back(p);
    p.x = BLINDSPOT_X_MIN;
    p.y = BLINDSPOT_Y_MIN;
    ps.push_back(p);

    _marker.line_strip("body_link", "obstacle_2", 1, ps, 1.0, 0.0, 0.0, 0.5, 0.05);

    //==================================

    /* The above code is declaring a boolean variable named "is_blindspot" and initializing it with the
    value "false". */
    bool is_blindspot = false;

    /* The above code is iterating through a list of lidar points and checking if each point falls within a
    defined blindspot region. The blindspot region is defined by minimum and maximum values for the x
    and y coordinates relative to the position of the lidar sensor. If any lidar point falls within the
    blindspot region, the boolean variable "is_blindspot" is set to true and the loop is exited. */
    for (int i = 0; i < lidar_base_obstacle_points.points.size(); i++)
    {
        float dx = lidar_base_obstacle_points.points[i].x - transform_base_to_body.getOrigin().x();
        float dy = lidar_base_obstacle_points.points[i].y - transform_base_to_body.getOrigin().y();

        if (dx > BLINDSPOT_X_MIN && dx < BLINDSPOT_X_MAX && dy > BLINDSPOT_Y_MIN && dy < BLINDSPOT_Y_MAX)
        {
            is_blindspot = true;
            break;
        }
    }

    /* The code is updating the value of `obstacle_2_data.output_blindspot` based on whether `is_blindspot`
    is true or false. If `is_blindspot` is true, the new value of `obstacle_2_data.output_blindspot` is
    a weighted average of its current value and 1.0, with a weight of 0.85 and 0.15 respectively. If
    `is_blindspot` is false, the new value of `obstacle_2_data.output_blindspot` is a weighted average
    of its current value and */
    if (is_blindspot)
        obstacle_2_data.output_blindspot = 0.85 * obstacle_2_data.output_blindspot + 0.15 * 1.0;
    else
        obstacle_2_data.output_blindspot = 0.85 * obstacle_2_data.output_blindspot + 0.15 * 0.0;

    //==================================

    pub_obstacle_2_data.publish(obstacle_2_data);
}