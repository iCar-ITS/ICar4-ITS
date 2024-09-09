#include "tf/transform_broadcaster.h"
#include "ps_ros_lib/help_log.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

int transform_broadcaster_init();
int transform_broadcaster_routine();

void send_transform(tfScalar x, tfScalar y, tfScalar z, tfScalar roll, tfScalar pitch, tfScalar yaw, std::string frame_id, std::string child_frame_id);

//=====Parameter
std::vector<float> icar_tf_rear_axle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> icar_tf_front_axle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> icar_tf_gps = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> icar_tf_body = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> icar_tf_lidar_front = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> icar_tf_lidar_rearright = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_100hz;
//=====Help
help_log _log;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_broadcaster");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("icar/tf/rear_axle", icar_tf_rear_axle);
    NH.getParam("icar/tf/front_axle", icar_tf_front_axle);
    NH.getParam("icar/tf/gps", icar_tf_gps);
    NH.getParam("icar/tf/body", icar_tf_body);
    NH.getParam("icar/tf/lidar_front", icar_tf_lidar_front);
    NH.getParam("icar/tf/lidar_rearright", icar_tf_lidar_rearright);
    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Help
    _log.init(NH);

    if (transform_broadcaster_init() == -1)
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
    if (transform_broadcaster_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

int transform_broadcaster_init()
{
    /* Printing the parameters. */
    ros::Duration(2).sleep();
    _log.info("Rear Axle Tf:       %6.2fm %6.2fm %6.2fm %6.2fdeg %6.2fdeg %6.2fdeg", icar_tf_rear_axle[0], icar_tf_rear_axle[1], icar_tf_rear_axle[2], icar_tf_rear_axle[3], icar_tf_rear_axle[4], icar_tf_rear_axle[5]);
    _log.info("Front Axle Tf:      %6.2fm %6.2fm %6.2fm %6.2fdeg %6.2fdeg %6.2fdeg", icar_tf_front_axle[0], icar_tf_front_axle[1], icar_tf_front_axle[2], icar_tf_front_axle[3], icar_tf_front_axle[4], icar_tf_front_axle[5]);
    _log.info("GPS Tf:             %6.2fm %6.2fm %6.2fm %6.2fdeg %6.2fdeg %6.2fdeg", icar_tf_gps[0], icar_tf_gps[1], icar_tf_gps[2], icar_tf_gps[3], icar_tf_gps[4], icar_tf_gps[5]);
    _log.info("Body Tf:            %6.2fm %6.2fm %6.2fm %6.2fdeg %6.2fdeg %6.2fdeg", icar_tf_body[0], icar_tf_body[1], icar_tf_body[2], icar_tf_body[3], icar_tf_body[4], icar_tf_body[5]);
    _log.info("Lidar Front Tf:     %6.2fm %6.2fm %6.2fm %6.2fdeg %6.2fdeg %6.2fdeg", icar_tf_lidar_front[0], icar_tf_lidar_front[1], icar_tf_lidar_front[2], icar_tf_lidar_front[3], icar_tf_lidar_front[4], icar_tf_lidar_front[5]);
    _log.info("Lidar RearRight Tf: %6.2fm %6.2fm %6.2fm %6.2fdeg %6.2fdeg %6.2fdeg", icar_tf_lidar_rearright[0], icar_tf_lidar_rearright[1], icar_tf_lidar_rearright[2], icar_tf_lidar_rearright[3], icar_tf_lidar_rearright[4], icar_tf_lidar_rearright[5]);

    return 0;
}

int transform_broadcaster_routine()
{
    send_transform(icar_tf_rear_axle[0], icar_tf_rear_axle[1], icar_tf_rear_axle[2],
                   icar_tf_rear_axle[3], icar_tf_rear_axle[4], icar_tf_rear_axle[5],
                   "base_link", "rear_axle_link");
    send_transform(icar_tf_front_axle[0], icar_tf_front_axle[1], icar_tf_front_axle[2],
                   icar_tf_front_axle[3], icar_tf_front_axle[4], icar_tf_front_axle[5],
                   "base_link", "front_axle_link");
    send_transform(icar_tf_gps[0], icar_tf_gps[1], icar_tf_gps[2],
                   icar_tf_gps[3], icar_tf_gps[4], icar_tf_gps[5],
                   "base_link", "gps_link");
    send_transform(icar_tf_body[0], icar_tf_body[1], icar_tf_body[2],
                   icar_tf_body[3], icar_tf_body[4], icar_tf_body[5],
                   "base_link", "body_link");
    send_transform(icar_tf_lidar_front[0], icar_tf_lidar_front[1], icar_tf_lidar_front[2],
                   icar_tf_lidar_front[3], icar_tf_lidar_front[4], icar_tf_lidar_front[5],
                   "base_link", "lidar_front_link");
    send_transform(icar_tf_lidar_rearright[0], icar_tf_lidar_rearright[1], icar_tf_lidar_rearright[2],
                   icar_tf_lidar_rearright[3], icar_tf_lidar_rearright[4], icar_tf_lidar_rearright[5],
                   "base_link", "lidar_rearright_link");

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief Send a transform to the tf tree.
 *
 * @param x Position in x axis (in meters)
 * @param y Position in y axis (in meters)
 * @param z Position in z axis (in meters)
 * @param roll Orientation in roll axis (in degrees)
 * @param pitch Orientation in pitch axis (in degrees)
 * @param yaw Orientation in yaw axis (in degrees)
 * @param frame_id Frame id
 * @param child_frame_id Child frame id
 */
void send_transform(tfScalar x, tfScalar y, tfScalar z, tfScalar roll, tfScalar pitch, tfScalar yaw, std::string frame_id, std::string child_frame_id)
{
    static tf::TransformBroadcaster transform_broadcaster;

    tf::Vector3 origin;
    tf::Quaternion rotation;

    origin.setValue(x, y, z);
    rotation.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(rotation);

    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}