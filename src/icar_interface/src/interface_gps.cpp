#include "GeographicLib/UTMUPS.hpp"
#include "geometry_msgs/Pose2D.h"
#include "libserialport.h"
#include "lwgps/lwgps.h"
#include "mavros_msgs/RTCM.h"
#include "nmea_msgs/Sentence.h"
#include "ps_ros_lib/help_log.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_ntrip_client_rtcm(const mavros_msgs::RTCM::ConstPtr &msg);

int interface_gps_init();
int interface_gps_routine();

void gps_parser(uint8_t data);

//=====Parameter
std::string gps_port;
int gps_baud;
double gps_origin_lat;
double gps_origin_lon;
//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_ntrip_client_rtcm;
//=====Publisher
ros::Publisher pub_gps_navsatfix;
ros::Publisher pub_gps_pose2d;
ros::Publisher pub_ntrip_client_nmea;
//=====Help
help_log _log;

//-----Serial port
//================
struct sp_port *serial_port;
ros::Time serial_time;
uint16_t serial_tx_len;
uint16_t serial_rx_len;
uint8_t serial_tx_buffer[1024];
uint8_t serial_rx_buffer[1024];

//-----GPS
//========
lwgps_t gps;

//-----GPS origin
//===============
int gps_origin_zone;
bool gps_origin_nothp;
double gps_origin_x;
double gps_origin_y;

//-----GPS pose
//=============
int gps_pose_zone;
bool gps_pose_nothp;
double gps_pose_x;
double gps_pose_y;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_gps");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("/gps/port", gps_port);
    NH.getParam("/gps/baud", gps_baud);
    NH.getParam("gps/origin/lat", gps_origin_lat);
    NH.getParam("gps/origin/lon", gps_origin_lon);
    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_ntrip_client_rtcm = NH.subscribe("ntrip_client/rtcm", 1, cllbck_sub_ntrip_client_rtcm);
    //=====Publisher
    pub_gps_navsatfix = NH.advertise<sensor_msgs::NavSatFix>("gps/navsatfix", 1);
    pub_gps_pose2d = NH.advertise<geometry_msgs::Pose2D>("gps/pose2d", 1);
    pub_ntrip_client_nmea = NH.advertise<nmea_msgs::Sentence>("ntrip_client/nmea", 1);
    //=====Help
    _log.init(NH);

    if (interface_gps_init() == -1)
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
    if (interface_gps_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

void cllbck_sub_ntrip_client_rtcm(const mavros_msgs::RTCM::ConstPtr &msg)
{
    /* Set lenght of the RTCM message. */
    serial_tx_len = msg->data.size();
    /* Set data of the RTCM message. */
    for (int i = 0; i < serial_tx_len; i++)
        serial_tx_buffer[i] = msg->data[i];

    /* Send the RTCM message. */
    sp_nonblocking_write(serial_port, serial_tx_buffer, serial_tx_len);
}

//------------------------------------------------------------------------------
//==============================================================================

int interface_gps_init()
{
    /* Printing the parameters. */
    ros::Duration(2).sleep();
    _log.info("Port: %s", gps_port.c_str());
    _log.info("Baud: %d", gps_baud);
    _log.info("Origin Latitude: %f", gps_origin_lat);
    _log.info("Origin Longitude: %f", gps_origin_lon);

    //==================================

    /* Initializing the `lwgps_t` structure. */
    lwgps_init(&gps);

    //==================================

    /* Converting the latitude and longitude of the origin to UTM coordinates.
    This is used as the origin of the local coordinate system. */
    GeographicLib::UTMUPS::Forward(gps_origin_lat, gps_origin_lon, gps_origin_zone, gps_origin_nothp, gps_origin_x, gps_origin_y);

    //==================================

    /* Initializing the serial port using below configuration:
    - Data bits: 8
    - Parity: None
    - Stop bits: 1 */
    if (sp_get_port_by_name(realpath(gps_port.c_str(), NULL), &serial_port) != SP_OK)
    {
        _log.error("Unable to find the serial port. (%s:%d)", __FILE__, __LINE__);
        return -1;
    }
    sp_close(serial_port);
    if (sp_open(serial_port, SP_MODE_READ_WRITE) != SP_OK)
    {
        _log.error("Unable to open the serial port. (%s:%d)", __FILE__, __LINE__);
        return -1;
    }
    if (sp_set_baudrate(serial_port, gps_baud) != SP_OK)
    {
        _log.error("Unable to set the baudrate. (%s:%d)", __FILE__, __LINE__);
        return -1;
    }
    if (sp_set_bits(serial_port, 8) != SP_OK)
    {
        _log.error("Unable to set the number of bits. (%s:%d)", __FILE__, __LINE__);
        return -1;
    }
    if (sp_set_parity(serial_port, SP_PARITY_NONE) != SP_OK)
    {
        _log.error("Unable to set the parity. (%s:%d)", __FILE__, __LINE__);
        return -1;
    }
    if (sp_set_stopbits(serial_port, 1) != SP_OK)
    {
        _log.error("Unable to set the number of stop bits. (%s:%d)", __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

int interface_gps_routine()
{
    /* Converting the latitude and longitude of the origin to UTM coordinates.
    This is used as the present location of the local coordinate system. */
    GeographicLib::UTMUPS::Forward(gps.latitude, gps.longitude, gps_pose_zone, gps_pose_nothp, gps_pose_x, gps_pose_y);

    //==================================

    sensor_msgs::NavSatFix msg_gps_navsatfix;
    //-----
    msg_gps_navsatfix.header.stamp = ros::Time::now();
    msg_gps_navsatfix.header.frame_id = "gps_link";
    //-----
    msg_gps_navsatfix.status.status = gps.fix == 0 || gps.fix_mode == 1 ? sensor_msgs::NavSatStatus::STATUS_NO_FIX : sensor_msgs::NavSatStatus::STATUS_FIX;
    msg_gps_navsatfix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    //-----
    msg_gps_navsatfix.latitude = gps.latitude;
    msg_gps_navsatfix.longitude = gps.longitude;
    msg_gps_navsatfix.altitude = gps.altitude;
    //-----
    msg_gps_navsatfix.position_covariance[0] = gps.dop_h * gps.dop_h;
    msg_gps_navsatfix.position_covariance[4] = gps.dop_h * gps.dop_h;
    msg_gps_navsatfix.position_covariance[8] = gps.dop_v * gps.dop_v;
    msg_gps_navsatfix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    //-----
    pub_gps_navsatfix.publish(msg_gps_navsatfix);

    //==================================

    static geometry_msgs::Pose2D msg_gps_pose2d;
    //-----
    msg_gps_pose2d.x = gps_pose_x - gps_origin_x;
    msg_gps_pose2d.y = gps_pose_y - gps_origin_y;
    /* This is used to set the heading of the robot. If the
    speed is less than 0.5 m/s, the heading is not updated. */
    if (gps.speed > 0.5)
        msg_gps_pose2d.theta = (90 - gps.course) * M_PI / 180;
    //-----
    pub_gps_pose2d.publish(msg_gps_pose2d);

    //==================================

    sp_return serial_return;
    uint8_t serial_data;

    /* This is a timeout to reset serial_rx_len if no data is received.
    This is used to check if the GPS module is sending data or not. */
    if (ros::Time::now() - serial_time > ros::Duration(1.0))
    {
        serial_time = ros::Time::now();
        serial_rx_len = 0;
    }

    /* Reading the data from the serial port and passing it to the gps_parser function.
    This function is called when the GPS module sends a character. */
    while ((serial_return = sp_nonblocking_read(serial_port, &serial_data, 1)) > 0)
    {
        serial_time = ros::Time::now();
        gps_parser(serial_data);
    }

    /* This is checking if the serial port is able to read the data.
    If it is not able to read the data, then it will return an error. */
    if (serial_return < 0)
    {
        _log.error("Unable to read from the serial port. (%s:%d)", __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief This function is called when the GPS module sends a character.
 *
 * @param data The character sent by the GPS module.
 */
void gps_parser(uint8_t data)
{
    static ros::Time timer_nmea;

    /* Storing the data in the serial_rx_buffer array. */
    serial_rx_buffer[serial_rx_len++] = data;

    /* This is checking if the last two characters in the buffer are `0x0D` and `0x0A`.
    If they are, then the buffer is sent to the `lwgps_process` function. */
    if (serial_rx_len >= 2)
        if (serial_rx_buffer[serial_rx_len - 2] == 0x0D &&
            serial_rx_buffer[serial_rx_len - 1] == 0x0A)
        {
            /* If serial_rx_buffer starts with `$` or `!`, then it is an NMEA sentence. */
            if (serial_rx_buffer[0] == '$' ||
                serial_rx_buffer[0] == '!')
            {
                std::string sentence = std::string((char *)serial_rx_buffer, serial_rx_len - 2);

                /* This is checking if the NMEA sentence contains `GGA` and if the time
                since the last NMEA sentence is greater than 0.2 seconds. */
                if (sentence.find("GGA") != std::string::npos && ros::Time::now() - timer_nmea > ros::Duration(0.2))
                {
                    timer_nmea = ros::Time::now();

                    nmea_msgs::Sentence msg_ntrip_client_nmea;
                    msg_ntrip_client_nmea.header.stamp = ros::Time::now();
                    msg_ntrip_client_nmea.header.frame_id = "gps_link";
                    msg_ntrip_client_nmea.sentence = sentence;
                    pub_ntrip_client_nmea.publish(msg_ntrip_client_nmea);
                }
            }

            lwgps_process(&gps, serial_rx_buffer, serial_rx_len);
            serial_rx_len = 0;
        }
}