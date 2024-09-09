#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "ps_ros_lib/help_log.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16MultiArray.h"

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg);
void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg);

int odometry_init();
int odometry_routine();

void encoder_2_d_encoder(uint16_t encoder_kiri, uint16_t encoder_kanan, int16_t *d_encoder_kiri, int16_t *d_encoder_kanan);
void gyroscope_2_d_gyroscope(float gyroscope, float *d_gyroscope);

//=====Parameter
float odometry_to_meter;
//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_encoder;
ros::Subscriber sub_gyroscope;
//=====Publisher
ros::Publisher pub_odometry_twist;
ros::Publisher pub_odometry_pose2d;
//=====Help
help_log _log;

//-----Encoder and gyroscope
//==========================
uint16_t encoder_kiri;
uint16_t encoder_kanan;
float gyroscope;

//-----Pose2d
//===========
float pose_x;
float pose_y;
float pose_theta;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("/odometry/to_meter", odometry_to_meter);
    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_encoder = NH.subscribe("stm32topc/encoder", 1, cllbck_sub_encoder);
    sub_gyroscope = NH.subscribe("stm32topc/gyroscope", 1, cllbck_sub_gyroscope);
    //=====Publisher
    pub_odometry_twist = NH.advertise<geometry_msgs::Twist>("odometry/twist", 1);
    pub_odometry_pose2d = NH.advertise<geometry_msgs::Pose2D>("odometry/pose2d", 1);
    //=====Help
    _log.init(NH);

    if (odometry_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();

    tim_50hz.stop();
    tim_100hz.stop();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    if (odometry_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    encoder_kiri = msg->data[0];
    encoder_kanan = msg->data[1];
}

void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg)
{
    gyroscope = msg->data;
}

//------------------------------------------------------------------------------
//==============================================================================

int odometry_init()
{
    /* Printing the parameters. */
    ros::Duration(2).sleep();
    _log.info("Odometry to Meter: %f", odometry_to_meter);

    return 0;
}

int odometry_routine()
{
    /* Calculate delta value of encoder per 0.02 second. */
    int16_t d_encoder_kiri;
    int16_t d_encoder_kanan;
    encoder_2_d_encoder(encoder_kiri, encoder_kanan, &d_encoder_kiri, &d_encoder_kanan);
    if (abs(d_encoder_kiri) > 1024 || abs(d_encoder_kanan) > 1024)
    {
        _log.warn("Odometry: Encoder value is too big. (d_encoder_kiri: %d, d_encoder_kanan: %d)", d_encoder_kiri, d_encoder_kanan);
        return 0;
    }

    /* Calculate delta value of gyroscope per 0.02 second. */
    float d_gyroscope;
    gyroscope_2_d_gyroscope(gyroscope, &d_gyroscope);
    if (fabsf(d_gyroscope) > 90)
    {
        _log.warn("Odometry: Gyroscope value is too big. (d_gyroscope: %f)", d_gyroscope);
        return 0;
    }

    //==================================

    /* Calculate the pose x and y. */
    pose_x += (d_encoder_kiri + d_encoder_kanan) / 2 * cos(pose_theta * M_PI / 180) * odometry_to_meter;
    pose_y += (d_encoder_kiri + d_encoder_kanan) / 2 * sin(pose_theta * M_PI / 180) * odometry_to_meter;

    /* Calculate the pose theta. */
    pose_theta += d_gyroscope;
    if (pose_theta > 180)
        pose_theta -= 360;
    else if (pose_theta < -180)
        pose_theta += 360;

    //==================================

    geometry_msgs::Twist msg_odometry_twist;
    msg_odometry_twist.linear.x = (d_encoder_kiri + d_encoder_kanan) / 2 * odometry_to_meter / 0.02;
    msg_odometry_twist.angular.z = d_gyroscope * M_PI / 180 / 0.02;
    pub_odometry_twist.publish(msg_odometry_twist);

    geometry_msgs::Pose2D msg_odometry_pose2d;
    msg_odometry_pose2d.x = pose_x;
    msg_odometry_pose2d.y = pose_y;
    msg_odometry_pose2d.theta = pose_theta * M_PI / 180;
    pub_odometry_pose2d.publish(msg_odometry_pose2d);

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief Convert encoder value to delta encoder value.
 *
 * @param encoder_kiri Value of left encoder.
 * @param encoder_kanan Value of right encoder.
 * @param d_encoder_kiri Delta value of left encoder.
 * @param d_encoder_kanan Delta value of right encoder.
 */
void encoder_2_d_encoder(uint16_t encoder_kiri, uint16_t encoder_kanan, int16_t *d_encoder_kiri, int16_t *d_encoder_kanan)
{
    static uint16_t last_encoder_kiri;
    static uint16_t last_encoder_kanan;

    /* To prevent the first value of encoder to be used as the last value. */
    if (last_encoder_kiri == 0 || last_encoder_kanan == 0)
    {
        last_encoder_kiri = encoder_kiri;
        last_encoder_kanan = encoder_kanan;
        *d_encoder_kiri = 0;
        *d_encoder_kanan = 0;
        return;
    }

    /* To prevent the encoder value to be overflowed. */
    if (encoder_kiri < 1024 && last_encoder_kiri > 3072)
        *d_encoder_kiri = encoder_kiri + 4096 - last_encoder_kiri;
    else if (encoder_kiri > 3072 && last_encoder_kiri < 1024)
        *d_encoder_kiri = encoder_kiri - 4096 - last_encoder_kiri;
    else
        *d_encoder_kiri = encoder_kiri - last_encoder_kiri;

    if (encoder_kanan < 1024 && last_encoder_kanan > 3072)
        *d_encoder_kanan = encoder_kanan + 4096 - last_encoder_kanan;
    else if (encoder_kanan > 3072 && last_encoder_kanan < 1024)
        *d_encoder_kanan = encoder_kanan - 4096 - last_encoder_kanan;
    else
        *d_encoder_kanan = encoder_kanan - last_encoder_kanan;

    last_encoder_kiri = encoder_kiri;
    last_encoder_kanan = encoder_kanan;
}

/**
 * @brief Convert gyroscope value to delta gyroscope value.
 *
 * @param gyroscope Value of gyroscope.
 * @param d_gyroscope Delta value of gyroscope.
 */
void gyroscope_2_d_gyroscope(float gyroscope, float *d_gyroscope)
{
    static float last_gyroscope;

    /* To prevent the first value of gyroscope to be used as the last value. */
    if (last_gyroscope == 0)
    {
        last_gyroscope = gyroscope;
        *d_gyroscope = 0;
        return;
    }

    /* To prevent the gyroscope value to be overflowed. */
    if (gyroscope < -90 && last_gyroscope > 90)
        *d_gyroscope = gyroscope + 360 - last_gyroscope;
    else if (gyroscope > 90 && last_gyroscope < -90)
        *d_gyroscope = gyroscope - 360 - last_gyroscope;
    else
        *d_gyroscope = gyroscope - last_gyroscope;

    last_gyroscope = gyroscope;
}