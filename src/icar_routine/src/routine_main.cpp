#include "icar_routine/routine.h"

//=====Parameter
float odometry_to_meter;
float steering_to_rad;
std::string log_path;
std::string route_path;
std::string bag_path;
std::string rviz_path;
std::string param_path;
float icar_wheelbase;
int icar_tyre_width;
int icar_tyre_aspect_ratio;
int icar_tyre_rim;
float icar_body_length;
float icar_body_width;
float icar_body_height;
//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_51hz;
ros::Timer tim_52hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_ui_output_data;
ros::Subscriber sub_pose_pose2d;
ros::Subscriber sub_pose_twist;
ros::Subscriber sub_encoder;
ros::Subscriber sub_gyroscope;
ros::Subscriber sub_throttle_steering_position;
ros::Subscriber sub_obstacle_1_data;
ros::Subscriber sub_obstacle_2_data;
//=====Publisher
ros::Publisher pub_ui_input_data;
ros::Publisher pub_ui_output_sound;
ros::Publisher pub_throttle_steering;
ros::Publisher pub_transmission;
ros::Publisher pub_accessory;
ros::Publisher pub_obstacle_1_parameter;
ros::Publisher pub_obstacle_2_parameter;
ros::Publisher pub_pp_active_goal;
//=====TransformListener
tf::TransformListener *transform_listener;
//=====Help
help_log _log;
help_marker _marker;

//-----Status algoritma
//=====================
unsigned int status_algoritma_record = 0;
unsigned int status_algoritma_load = 0;
unsigned int status_algoritma_mission = 0;
unsigned int status_algoritma_accessory = 0;
unsigned int status_algoritma_test = 0;

//-----Pose2d dan twist
//=====================
geometry_msgs::Pose2D pose_pose2d;
geometry_msgs::Twist pose_twist;

//-----UI data
//============
icar_ui::ui_input_data ui_input_data;
icar_ui::ui_output_data ui_output_data;

//-----Encoder dan gyroscope
//==========================
uint16_t encoder_kiri;
uint16_t encoder_kanan;
float gyroscope;

//-----Throttle dan steering position
//===================================
float throttle_position;
float steering_position;

//-----Throttle dan steering
//==========================
float action_throttle;
float action_brake;
float action_velocity;
float action_steering;

//-----Metric
//===========
float metric_velocity;
float metric_acceleration;
float metric_jerk;

//-----Accessory
//==============
uint8_t accessory = 0b001111;

//-----Pure pursuit
//=================
pure_pursuit pp_active;
pure_pursuit pp_limit_kanan;
pure_pursuit pp_limit_kiri;

//-----PID
//========
pid pid_velocity_maju;

//-----Obstacle data dan parameter
//================================
icar_middleware::obstacle_1_data obstacle_1_data;
icar_middleware::obstacle_2_data obstacle_2_data;
icar_middleware::obstacle_1_parameter obstacle_1_parameter;
icar_middleware::obstacle_2_parameter obstacle_2_parameter;

//-----Route
//==========
std::vector<geometry_msgs::Point> route_active;
std::vector<geometry_msgs::Point> route_limit_kanan;
std::vector<geometry_msgs::Point> route_limit_kiri;

//-----Action
//===========
std::vector<action_main> actions_main;
std::vector<action_accessory> actions_accessory;

//-----Transform
//==============
tf::StampedTransform transform_map_to_base;
tf::StampedTransform transform_base_to_lidar_front;
tf::StampedTransform transform_base_to_lidar_rearright;
tf::StampedTransform transform_map_to_lidar_front;
tf::StampedTransform transform_map_to_lidar_rearright;
bool transform_initialized = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routine");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("odometry/to_meter", odometry_to_meter);
    NH.getParam("steering/to_rad", steering_to_rad);
    NH.getParam("log_path", log_path);
    NH.getParam("route_path", route_path);
    NH.getParam("bag_path", bag_path);
    NH.getParam("rviz_path", rviz_path);
    NH.getParam("param_path", param_path);
    NH.getParam("icar/wheelbase", icar_wheelbase);
    NH.getParam("icar/tyre/width", icar_tyre_width);
    NH.getParam("icar/tyre/aspect_ratio", icar_tyre_aspect_ratio);
    NH.getParam("icar/tyre/rim", icar_tyre_rim);
    NH.getParam("icar/body/length", icar_body_length);
    NH.getParam("icar/body/width", icar_body_width);
    NH.getParam("icar/body/height", icar_body_height);
    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_51hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_51hz);
    tim_52hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_52hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_ui_output_data = NH.subscribe("/ui_output/data", 1, cllbck_sub_ui_output_data);
    sub_pose_pose2d = NH.subscribe("/pose/pose2d", 1, cllbck_sub_pose_pose2d);
    sub_pose_twist = NH.subscribe("/pose/twist", 1, cllbck_sub_pose_twist);
    sub_encoder = NH.subscribe("stm32topc/encoder", 1, cllbck_sub_encoder);
    sub_gyroscope = NH.subscribe("stm32topc/gyroscope", 1, cllbck_sub_gyroscope);
    sub_throttle_steering_position = NH.subscribe("stm32topc/throttle_steering_position", 1, cllbck_sub_throttle_steering_position);
    sub_obstacle_1_data = NH.subscribe("obstacle/1/data", 1, cllbck_sub_obstacle_1_data);
    sub_obstacle_2_data = NH.subscribe("obstacle/2/data", 1, cllbck_sub_obstacle_2_data);
    //=====Publisher
    pub_ui_input_data = NH.advertise<icar_ui::ui_input_data>("/ui_input/data", 1);
    pub_ui_output_sound = NH.advertise<icar_ui::ui_output_sound>("/ui_output/sound", 1);
    pub_throttle_steering = NH.advertise<std_msgs::Int16MultiArray>("stm32frompc/throttle_steering", 1);
    pub_transmission = NH.advertise<std_msgs::Int8>("stm32frompc/transmission", 1);
    pub_accessory = NH.advertise<std_msgs::UInt8>("stm32frompc/accessory", 1);
    pub_obstacle_1_parameter = NH.advertise<icar_middleware::obstacle_1_parameter>("obstacle/1/parameter", 1);
    pub_obstacle_2_parameter = NH.advertise<icar_middleware::obstacle_2_parameter>("obstacle/2/parameter", 1);
    pub_pp_active_goal = NH.advertise<geometry_msgs::Point>("/pp/active/goal", 1);
    //=====TransformListener
    transform_listener = new tf::TransformListener(NH);
    //=====Help
    _log.init(NH);
    _marker.init(NH);

    if (routine_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();

    tim_50hz.stop();
    tim_51hz.stop();
    tim_52hz.stop();
    tim_100hz.stop();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    if (routine_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_51hz(const ros::TimerEvent &event)
{
}

void cllbck_tim_52hz(const ros::TimerEvent &event)
{
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_ui_output_data(const icar_ui::ui_output_dataConstPtr &msg)
{
    ui_output_data = *msg;
}

void cllbck_sub_pose_pose2d(const geometry_msgs::Pose2DConstPtr &msg)
{
    pose_pose2d = *msg;
}

void cllbck_sub_pose_twist(const geometry_msgs::TwistConstPtr &msg)
{
    pose_twist = *msg;
}

void cllbck_sub_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    encoder_kiri = msg->data[0];
    encoder_kanan = msg->data[1];
}

void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg)
{
    gyroscope = msg->data;
}

void cllbck_sub_throttle_steering_position(const std_msgs::Int16MultiArrayConstPtr &msg)
{
    throttle_position = msg->data[0];
    steering_position = msg->data[1] * steering_to_rad;
}

void cllbck_sub_obstacle_1_data(const icar_middleware::obstacle_1_dataConstPtr &msg)
{
    obstacle_1_data = *msg;
    pub_obstacle_1_parameter.publish(obstacle_1_parameter);
}

void cllbck_sub_obstacle_2_data(const icar_middleware::obstacle_2_dataConstPtr &msg)
{
    obstacle_2_data = *msg;
    pub_obstacle_2_parameter.publish(obstacle_2_parameter);
}

//------------------------------------------------------------------------------
//==============================================================================

int routine_init()
{
    /* Printing the parameters. */
    ros::Duration(2).sleep();
    _log.info("Log Path: %s", log_path.c_str());
    _log.info("Route Path: %s", route_path.c_str());
    _log.info("Bag Path: %s", bag_path.c_str());
    _log.info("Rviz Path: %s", rviz_path.c_str());
    _log.info("Param Path: %s", param_path.c_str());
    _log.info("Steering to Rad: %f", steering_to_rad);
    _log.info("Wheelbase: %.2fm", icar_wheelbase);
    _log.info("Tyre Width: %dmm", icar_tyre_width);
    _log.info("Tyre Aspect Ratio: %d%%", icar_tyre_aspect_ratio);
    _log.info("Tyre Rim: %din", icar_tyre_rim);
    _log.info("Body Length: %.2fm", icar_body_length);
    _log.info("Body Width: %.2fm", icar_body_width);
    _log.info("Body Height: %.2fm", icar_body_height);

    /* Checking if the directory exists, if not, it will create the directory. */
    if (!boost::filesystem::exists(route_path))
        boost::filesystem::create_directories(route_path);

    /* Setting the default route to be used. */
    ui_input_data.route_option = "N/A";
    ui_output_data.route_option = "default";
    status_algoritma_load = 20;

    /* Initializing pure pursuit. */
    pp_active.init(&pose_pose2d, &route_active, icar_wheelbase, 3 * icar_wheelbase);
    pp_limit_kanan.init(&pose_pose2d, &route_limit_kanan, icar_wheelbase, 3 * icar_wheelbase);
    pp_limit_kiri.init(&pose_pose2d, &route_limit_kiri, icar_wheelbase, 3 * icar_wheelbase);

    /* Initializing the PID controller. */
    pid_velocity_maju.init(15, 1, 0, 0.02, -25.0, 50.0, -12.5, 25.0);

    /* Initializing obstacle 1 parameter. */
    obstacle_1_parameter.laser_scan_distance = 10.0;
    obstacle_1_parameter.obstacle_status_steering = true;
    obstacle_1_parameter.obstacle_status_velocity = true;

    return 0;
}

int routine_routine()
{
    switch (button_read())
    {
    case icar_ui::ui_output_data::BUTTON_RECORD_START:
        status_algoritma_record = 10;
        break;
    case icar_ui::ui_output_data::BUTTON_RECORD_STOP:
        status_algoritma_record = 20;
        break;
    case icar_ui::ui_output_data::BUTTON_LOAD_REFRESH:
        status_algoritma_load = 10;
        break;
    case icar_ui::ui_output_data::BUTTON_LOAD_OPEN:
        status_algoritma_load = 20;
        break;
    case icar_ui::ui_output_data::BUTTON_START:
        status_algoritma_mission = 1;
        break;
    case icar_ui::ui_output_data::BUTTON_STOP:
        status_algoritma_mission = 0;
        break;
    }

    if (transform_initialized == false)
    {
        try
        {
            transform_listener->lookupTransform("map", "base_link", ros::Time(0), transform_map_to_base);
            transform_listener->lookupTransform("base_link", "lidar_front_link", ros::Time(0), transform_base_to_lidar_front);
            transform_listener->lookupTransform("base_link", "lidar_rearright_link", ros::Time(0), transform_base_to_lidar_rearright);
            transform_listener->lookupTransform("map", "lidar_front_link", ros::Time(0), transform_map_to_lidar_front);
            transform_listener->lookupTransform("map", "lidar_rearright_link", ros::Time(0), transform_map_to_lidar_rearright);
            transform_initialized = true;
        }
        catch (...)
        {
        }
    }

    pp_active.updateAll();
    pp_limit_kanan.updateAll();
    pp_limit_kiri.updateAll();

    geometry_msgs::Point msg_pp_active_goal;
    msg_pp_active_goal.x = pp_active.goalPositionX;
    msg_pp_active_goal.y = pp_active.goalPositionY;
    pub_pp_active_goal.publish(msg_pp_active_goal);

    process_marker();
    process_metric();
    process_ui();

    process_record_and_load();
    process_mission();
    process_accessory();
    // process_test();

    std_msgs::UInt8 msg_accessory;
    msg_accessory.data = accessory;
    pub_accessory.publish(msg_accessory);

    return 0;
}