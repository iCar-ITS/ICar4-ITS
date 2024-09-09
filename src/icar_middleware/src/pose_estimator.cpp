#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "ps_ros_lib/help_log.h"
#include "ps_ros_lib/help_marker.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#define RADIUS(b, a) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2))
#define ANGLE(b, a) atan2(b.y - a.y, b.x - a.x)

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_gps_navsatfix(const sensor_msgs::NavSatFixConstPtr &msg);
void cllbck_sub_gps_pose2d(const geometry_msgs::Pose2DConstPtr &msg);
void cllbck_sub_odometry_twist(const geometry_msgs::TwistConstPtr &msg);
void cllbck_sub_odometry_pose2d(const geometry_msgs::Pose2DConstPtr &msg);

int pose_estimator_init();
int pose_estimator_routine();

//=====Parameter
float cf_ratio_xy;
float cf_ratio_theta;
//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_gps_navsatfix;
ros::Subscriber sub_gps_pose2d;
ros::Subscriber sub_odometry_twist;
ros::Subscriber sub_odometry_pose2d;
//=====Publisher
ros::Publisher pub_pose_twist;
ros::Publisher pub_pose_pose2d;
//=====TransformListener
tf::TransformListener *transform_listener;
//=====Help
help_log _log;
help_marker _marker;

//-----GPS
//========
sensor_msgs::NavSatFix gps_navsatfix;
geometry_msgs::Pose2D gps_pose2d;

//-----Odometry
//=============
geometry_msgs::Twist odometry_twist;
geometry_msgs::Pose2D odometry_pose2d;

//-----Pose
//=========
geometry_msgs::Twist pose_twist;
geometry_msgs::Pose2D pose_pose2d;

//-----Complementary filter gain
//==============================
float gain_xy;
float gain_theta;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimator");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("/pose_estimator/cf_ratio_xy", cf_ratio_xy);
    NH.getParam("/pose_estimator/cf_ratio_theta", cf_ratio_theta);
    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_gps_navsatfix = NH.subscribe("gps/navsatfix", 1, cllbck_sub_gps_navsatfix);
    sub_gps_pose2d = NH.subscribe("gps/pose2d", 1, cllbck_sub_gps_pose2d);
    sub_odometry_twist = NH.subscribe("odometry/twist", 1, cllbck_sub_odometry_twist);
    sub_odometry_pose2d = NH.subscribe("odometry/pose2d", 1, cllbck_sub_odometry_pose2d);
    //=====Publisher
    pub_pose_twist = NH.advertise<geometry_msgs::Twist>("pose/twist", 1);
    pub_pose_pose2d = NH.advertise<geometry_msgs::Pose2D>("pose/pose2d", 1);
    //=====TransformListener
    transform_listener = new tf::TransformListener(NH);
    //=====Help
    _log.init(NH);
    _marker.init(NH);

    if (pose_estimator_init() == -1)
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
    if (pose_estimator_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_gps_navsatfix(const sensor_msgs::NavSatFixConstPtr &msg)
{
    gps_navsatfix = *msg;
}

void cllbck_sub_gps_pose2d(const geometry_msgs::Pose2DConstPtr &msg)
{
    static tf::StampedTransform transform;
    static bool transform_initialized = false;

    /* Check whether the transform is already initialized.
    If not, try to get the transform from the tf tree. */
    if (!transform_initialized)
    {
        try
        {
            transform_listener->lookupTransform("gps_link", "base_link", ros::Time(0), transform);
            transform_initialized = true;
        }
        catch (...)
        {
        }
    }

    /* Calculating the position of the GPS sensor relative to the robot.
    The position of the GPS sensor is calculated in the base_link frame. */
    float r = sqrtf(powf(transform.getOrigin().x(), 2) + powf(transform.getOrigin().y(), 2));
    float a = atan2f(transform.getOrigin().y(), transform.getOrigin().x());
    gps_pose2d.x = msg->x + r * cosf(a + pose_pose2d.theta);
    gps_pose2d.y = msg->y + r * sinf(a + pose_pose2d.theta);
    gps_pose2d.theta = msg->theta;

    geometry_msgs::Point p;
    p.x = msg->x;
    p.y = msg->y;
    _marker.sphere("map", "gps", 1, p, tf::createQuaternionMsgFromYaw(0), 1, 0, 0, 0.5, 0.25, 0.25, 0.25);
    p.x = gps_pose2d.x;
    p.y = gps_pose2d.y;
    _marker.sphere("map", "gps", 2, p, tf::createQuaternionMsgFromYaw(0), 1, 1, 1, 0.5, 0.25, 0.25, 0.25);
}

void cllbck_sub_odometry_twist(const geometry_msgs::TwistConstPtr &msg)
{
    odometry_twist = *msg;

    /* Calculating the position of the robot based on the odometry. */
    pose_pose2d.x += odometry_twist.linear.x * cosf(pose_pose2d.theta) * 0.02;
    pose_pose2d.y += odometry_twist.linear.x * sinf(pose_pose2d.theta) * 0.02;
    /* Calculating the orientation of the robot based on the odometry. */
    pose_pose2d.theta += odometry_twist.angular.z * 0.02;
    if (pose_pose2d.theta > M_PI)
        pose_pose2d.theta -= 2 * M_PI;
    else if (pose_pose2d.theta < -M_PI)
        pose_pose2d.theta += 2 * M_PI;
}

void cllbck_sub_odometry_pose2d(const geometry_msgs::Pose2DConstPtr &msg)
{
    odometry_pose2d = *msg;
}

//------------------------------------------------------------------------------
//==============================================================================

int pose_estimator_init()
{
    /* Printing the parameters. */
    ros::Duration(2).sleep();
    _log.info("Odometry-to-GPS Ratio for XY: %.2f", cf_ratio_xy);
    _log.info("Odometry-to-GPS Ratio for Theta: %.2f", cf_ratio_theta);

    //==================================

    /* Calculating the complementary filter gain. */
    gain_xy = cf_ratio_xy / (cf_ratio_xy + 1);
    gain_theta = cf_ratio_theta / (cf_ratio_theta + 1);

    return 0;
}

int pose_estimator_routine()
{
    const static uint8_t init_counter = 5;

    static uint8_t init_xy = 0;
    static uint8_t init_theta = 0;

    //==================================

    static geometry_msgs::Pose2D last_gps_pose2d_xy;
    static geometry_msgs::Pose2D last_gps_pose2d_theta;
    static geometry_msgs::Pose2D last_odometry_pose2d_xy;
    static geometry_msgs::Pose2D last_odometry_pose2d_theta;

    /* Checking whether the GPS data are initialized.
    If not, initialize the data using current data. */
    if (last_gps_pose2d_xy.x == 0 || last_gps_pose2d_xy.y == 0 ||
        last_gps_pose2d_theta.x == 0 || last_gps_pose2d_theta.y == 0)
    {
        last_gps_pose2d_xy = gps_pose2d;
        last_gps_pose2d_theta = gps_pose2d;
    }

    /* Checking whether the odometry data are initialized.
    If not, initialize the data using current data. */
    if (last_odometry_pose2d_xy.x == 0 || last_odometry_pose2d_xy.y == 0 ||
        last_odometry_pose2d_theta.x == 0 || last_odometry_pose2d_theta.y == 0)
    {
        last_odometry_pose2d_xy = odometry_pose2d;
        last_odometry_pose2d_theta = odometry_pose2d;
    }

    //==================================

    if (RADIUS(odometry_pose2d, last_odometry_pose2d_xy) > 0.1)
    {
        if (gps_navsatfix.status.status == sensor_msgs::NavSatStatus::STATUS_FIX)
        {
            /* Calculate difference between the current position
            using GPS and the current fused position. */
            float delta_x = gps_pose2d.x - pose_pose2d.x;
            float delta_y = gps_pose2d.y - pose_pose2d.y;

            /* Calculate the new position using the complementary filter. */
            if (init_xy >= init_counter)
            {
                pose_pose2d.x = (gain_xy) * (pose_pose2d.x) + (1 - gain_xy) * (pose_pose2d.x + delta_x);
                pose_pose2d.y = (gain_xy) * (pose_pose2d.y) + (1 - gain_xy) * (pose_pose2d.y + delta_y);
            }
            else
            {
                /* Calculate the new position using the complementary filter. */
                pose_pose2d.x = (0.00) * (pose_pose2d.x) + (1.00) * (pose_pose2d.x + delta_x);
                pose_pose2d.y = (0.00) * (pose_pose2d.y) + (1.00) * (pose_pose2d.y + delta_y);

                if (++init_xy == init_counter)
                    _log.info("XY initialized.");
            }
        }

        //------------------------------

        last_odometry_pose2d_xy = odometry_pose2d;
        last_gps_pose2d_xy = gps_pose2d;
    }

    //==================================

    // if (RADIUS(odometry_pose2d, last_odometry_pose2d_theta) > 4.0)
    // {
    //     if (gps_navsatfix.status.status == sensor_msgs::NavSatStatus::STATUS_FIX)
    //     {
    //         /* Calculate difference between the current theta
    //         using GPS and the current fused theta. */
    //         float a = ANGLE(gps_pose2d, last_gps_pose2d_theta);
    //         float o = pose_pose2d.theta;
    //         float delta_angle = a - o;
    //         if (delta_angle > M_PI)
    //             delta_angle -= 2 * M_PI;
    //         else if (delta_angle < -M_PI)
    //             delta_angle += 2 * M_PI;

    //         bool is_forward = fabsf(delta_angle) < M_PI_2 ? true : false;

    //         float delta_angle_corrected = delta_angle;
    //         if (!is_forward)
    //             delta_angle_corrected += M_PI;

    //         if (delta_angle_corrected > M_PI)
    //             delta_angle_corrected -= 2 * M_PI;
    //         else if (delta_angle_corrected < -M_PI)
    //             delta_angle_corrected += 2 * M_PI;

    //         /* Calculate the new theta using the complementary filter. */
    //         if (init_theta >= init_counter)
    //         {
    //             pose_pose2d.theta = (gain_theta) * (pose_pose2d.theta) + (1 - gain_theta) * (pose_pose2d.theta + delta_angle_corrected);
    //             if (pose_pose2d.theta > M_PI)
    //                 pose_pose2d.theta -= 2 * M_PI;
    //             else if (pose_pose2d.theta < -M_PI)
    //                 pose_pose2d.theta += 2 * M_PI;
    //         }
    //         else
    //         {
    //             pose_pose2d.theta = (0.75) * (pose_pose2d.theta) + (0.25) * (pose_pose2d.theta + delta_angle);
    //             if (pose_pose2d.theta > M_PI)
    //                 pose_pose2d.theta -= 2 * M_PI;
    //             else if (pose_pose2d.theta < -M_PI)
    //                 pose_pose2d.theta += 2 * M_PI;

    //             if (++init_theta == init_counter)
    //                 _log.info("Theta initialized.");
    //         }
    //     }

    //     //------------------------------

    //     last_odometry_pose2d_theta = odometry_pose2d;
    //     last_gps_pose2d_theta = gps_pose2d;
    // }

    //==================================

    if (RADIUS(odometry_pose2d, last_odometry_pose2d_theta) > 1.0)
    {
        if (gps_navsatfix.status.status == sensor_msgs::NavSatStatus::STATUS_FIX)
        {
            /* Calculate difference between the current theta
            using GPS and the current fused theta. */
            float a = gps_pose2d.theta;
            float o = pose_pose2d.theta;
            float delta_angle = a - o;
            if (delta_angle > M_PI)
                delta_angle -= 2 * M_PI;
            else if (delta_angle < -M_PI)
                delta_angle += 2 * M_PI;

            bool is_forward = fabsf(delta_angle) < M_PI_2 ? true : false;

            float delta_angle_corrected = delta_angle;
            if (!is_forward)
                delta_angle_corrected += M_PI;

            if (delta_angle_corrected > M_PI)
                delta_angle_corrected -= 2 * M_PI;
            else if (delta_angle_corrected < -M_PI)
                delta_angle_corrected += 2 * M_PI;

            /* Calculate the new theta using the complementary filter. */
            if (init_theta >= init_counter)
            {
                pose_pose2d.theta = (gain_theta) * (pose_pose2d.theta) + (1 - gain_theta) * (pose_pose2d.theta + delta_angle_corrected);
                if (pose_pose2d.theta > M_PI)
                    pose_pose2d.theta -= 2 * M_PI;
                else if (pose_pose2d.theta < -M_PI)
                    pose_pose2d.theta += 2 * M_PI;
            }
            else
            {
                pose_pose2d.theta = (0.01) * (pose_pose2d.theta) + (0.99) * (pose_pose2d.theta + delta_angle);
                if (pose_pose2d.theta > M_PI)
                    pose_pose2d.theta -= 2 * M_PI;
                else if (pose_pose2d.theta < -M_PI)
                    pose_pose2d.theta += 2 * M_PI;

                if (++init_theta == init_counter)
                    _log.info("Theta initialized.");
            }
        }

        //------------------------------

        last_odometry_pose2d_theta = odometry_pose2d;
        last_gps_pose2d_theta = gps_pose2d;
    }

    //==================================

    /* Making sure that the robot has moved enough before it starts publishing the pose. */
    if (init_xy < init_counter || init_theta < init_counter)
        return 0;

    //==================================

    static tf::TransformBroadcaster transform_broadcaster;

    tf::Vector3 origin;
    tf::Quaternion rotation;
    origin.setValue(pose_pose2d.x, pose_pose2d.y, 0.0);
    rotation.setRPY(0.0, 0.0, pose_pose2d.theta);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(rotation);

    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    //==================================

    geometry_msgs::Twist msg_pose_twist;
    msg_pose_twist = odometry_twist;
    pub_pose_twist.publish(msg_pose_twist);

    geometry_msgs::Pose2D msg_pose_pose2d;
    msg_pose_pose2d = pose_pose2d;
    pub_pose_pose2d.publish(msg_pose_pose2d);

    return 0;
}