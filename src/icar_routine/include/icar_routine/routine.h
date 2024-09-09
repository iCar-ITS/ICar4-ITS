#ifndef ROUTINE_H
#define ROUTINE_H

#include "boost/date_time.hpp"
#include "boost/filesystem.hpp"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "icar_middleware/obstacle_1_data.h"
#include "icar_middleware/obstacle_1_parameter.h"
#include "icar_middleware/obstacle_2_data.h"
#include "icar_middleware/obstacle_2_parameter.h"
#include "icar_ui/ui_input_data.h"
#include "icar_ui/ui_output_data.h"
#include "icar_ui/ui_output_sound.h"
#include "ps_ros_lib/help_log.h"
#include "ps_ros_lib/help_marker.h"
#include "ps_ros_lib/pid.h"
#include "ps_ros_lib/pure_pursuit.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt8.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#define RADIUS(b, a) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2))
#define ANGLE(b, a) atan2(b.y - a.y, b.x - a.x)

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_51hz(const ros::TimerEvent &event);
void cllbck_tim_52hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_ui_output_data(const icar_ui::ui_output_dataConstPtr &msg);
void cllbck_sub_pose_pose2d(const geometry_msgs::Pose2DConstPtr &msg);
void cllbck_sub_pose_twist(const geometry_msgs::TwistConstPtr &msg);
void cllbck_sub_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg);
void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg);
void cllbck_sub_throttle_steering_position(const std_msgs::Int16MultiArrayConstPtr &msg);
void cllbck_sub_obstacle_1_data(const icar_middleware::obstacle_1_dataConstPtr &msg);
void cllbck_sub_obstacle_2_data(const icar_middleware::obstacle_2_dataConstPtr &msg);

int routine_init();
int routine_routine();

void process_marker();
void process_metric();
void process_ui();

void process_record_and_load();
void process_mission();
void process_accessory();
void process_test();

int is_inside_action_main(float radius_offset = 0);
int is_inside_action_accessory(float radius_offset = 0);

int button_read();
void play_sound(uint8_t id, uint8_t action, std::string filename);
void encoder_2_d_encoder(uint16_t encoder_kiri, uint16_t encoder_kanan, int16_t *d_encoder_kiri, int16_t *d_encoder_kanan);
void gyroscope_2_d_gyroscope(float gyroscope, float *d_gyroscope);
void accessory_bitwise_op(uint8_t &accessory, uint8_t bit_index, int8_t bit_value);

void manual_throttle_steering(float throttle, float steering, int8_t transmission);
void jalan_maju_setir_otomatis(float velocity, float steer, float slow_acceleration = 0, float slow_deceleration = 0, float emergency_brake = 0, bool *is_stopped = NULL, bool *is_emergency = NULL);

//=====Parameter
extern float odometry_to_meter;
extern float steering_to_rad;
extern std::string log_path;
extern std::string route_path;
extern std::string bag_path;
extern std::string rviz_path;
extern std::string param_path;
extern float icar_wheelbase;
extern int icar_tyre_width;
extern int icar_tyre_aspect_ratio;
extern int icar_tyre_rim;
extern float icar_body_length;
extern float icar_body_width;
extern float icar_body_height;
//=====Timer
extern ros::Timer tim_50hz;
extern ros::Timer tim_51hz;
extern ros::Timer tim_52hz;
extern ros::Timer tim_100hz;
//=====Subscriber
extern ros::Subscriber sub_ui_output_data;
extern ros::Subscriber sub_pose_pose2d;
extern ros::Subscriber sub_pose_twist;
extern ros::Subscriber sub_encoder;
extern ros::Subscriber sub_gyroscope;
extern ros::Subscriber sub_throttle_steering_position;
extern ros::Subscriber sub_obstacle_1_data;
extern ros::Subscriber sub_obstacle_2_data;
//=====Publisher
extern ros::Publisher pub_ui_input_data;
extern ros::Publisher pub_ui_output_sound;
extern ros::Publisher pub_throttle_steering;
extern ros::Publisher pub_transmission;
extern ros::Publisher pub_accessory;
extern ros::Publisher pub_obstacle_1_parameter;
extern ros::Publisher pub_obstacle_2_parameter;
extern ros::Publisher pub_pp_active_goal;
//=====TransformListener
extern tf::TransformListener *transform_listener;
//=====Help
extern help_log _log;
extern help_marker _marker;

//-----Status algoritma
//=====================
extern unsigned int status_algoritma_record;
extern unsigned int status_algoritma_load;
extern unsigned int status_algoritma_mission;
extern unsigned int status_algoritma_accessory;
extern unsigned int status_algoritma_test;

//-----Pose2d dan twist
//=====================
extern geometry_msgs::Pose2D pose_pose2d;
extern geometry_msgs::Twist pose_twist;

//-----UI data
//============
extern icar_ui::ui_input_data ui_input_data;
extern icar_ui::ui_output_data ui_output_data;

//-----Encoder dan gyroscope
//==========================
extern uint16_t encoder_kiri;
extern uint16_t encoder_kanan;
extern float gyroscope;

//-----Throttle dan steering position
//===================================
extern float throttle_position;
extern float steering_position;

//-----Throttle dan steering
//==========================
extern float action_throttle;
extern float action_brake;
extern float action_velocity;
extern float action_steering;

//-----Metric
//===========
extern float metric_velocity;
extern float metric_acceleration;
extern float metric_jerk;

//-----Accessory
//==============
extern uint8_t accessory;

//-----Pure pursuit
//=================
extern pure_pursuit pp_active;
extern pure_pursuit pp_limit_kanan;
extern pure_pursuit pp_limit_kiri;

//-----PID
//========
extern pid pid_velocity_maju;

//-----Obstacle data dan parameter
//================================
extern icar_middleware::obstacle_1_data obstacle_1_data;
extern icar_middleware::obstacle_2_data obstacle_2_data;
extern icar_middleware::obstacle_1_parameter obstacle_1_parameter;

//-----Route
//==========
extern std::vector<geometry_msgs::Point> route_active;
extern std::vector<geometry_msgs::Point> route_limit_kanan;
extern std::vector<geometry_msgs::Point> route_limit_kiri;

typedef struct
{
    float x;
    float y;
    float radius;

    uint16_t action_type;
    uint16_t obstacle_status;

    float laser_scan_distance;
    float look_ahead_distance;
    float target_velocity;

    std::string comment;
} action_main;

typedef struct
{
    float x;
    float y;
    float radius;

    uint16_t action_type;

    int16_t leftsignal;
    int16_t rightsignal;
    int16_t headlight;
    int16_t r;
    int16_t g;
    int16_t b;

    std::string comment;
} action_accessory;

//-----Action
//===========
extern std::vector<action_main> actions_main;
extern std::vector<action_accessory> actions_accessory;

//-----Transform
//==============
extern tf::StampedTransform transform_map_to_base;
extern tf::StampedTransform transform_base_to_lidar_front;
extern tf::StampedTransform transform_base_to_lidar_rearright;
extern tf::StampedTransform transform_map_to_lidar_front;
extern tf::StampedTransform transform_map_to_lidar_rearright;
extern bool transform_initialized;

#endif