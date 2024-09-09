#include "arpa/inet.h"
#include "ps_ros_lib/help_log.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt8.h"

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_stm32frompc_throttle_steering(const std_msgs::Int16MultiArrayConstPtr &msg);
void cllbck_sub_stm32frompc_transmission(const std_msgs::Int8ConstPtr &msg);
void cllbck_sub_stm32frompc_accessory(const std_msgs::UInt8ConstPtr &msg);

int interface_stm32_init();
int interface_stm32_routine();

//=====Parameter
std::string stm32_ip;
int stm32_port;
//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_stm32frompc_throttle_steering;
ros::Subscriber sub_stm32frompc_transmission;
ros::Subscriber sub_stm32frompc_accessory;
//=====Publisher
ros::Publisher pub_stm32topc_remote;
ros::Publisher pub_stm32topc_encoder;
ros::Publisher pub_stm32topc_gyroscope;
ros::Publisher pub_stm32topc_throttle_steering_position;
ros::Publisher pub_stm32topc_mode;
//=====Help
help_log _log;

//-----Socket connection
//======================
int socket_fd = 0;
struct sockaddr_in socket_server_address;
struct sockaddr_in socket_client_address;
ros::Time socket_time;
uint16_t socket_tx_len;
uint16_t socket_rx_len;
uint8_t socket_tx_buffer[1024];
uint8_t socket_rx_buffer[1024];

//-----Data to PC
//===============
uint32_t epoch_to_pc;
uint16_t remote[16];
uint16_t encoder_kiri;
uint16_t encoder_kanan;
float gyroscope;
int16_t throttle_position;
int16_t steering_position;
uint8_t mode;

//-----Data from PC
//=================
uint32_t epoch_from_pc;
int16_t throttle;
int16_t steering;
int8_t transmission;
uint8_t accessory;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_stm32");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("/stm32/ip", stm32_ip);
    NH.getParam("/stm32/port", stm32_port);
    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_stm32frompc_throttle_steering = NH.subscribe("stm32frompc/throttle_steering", 1, cllbck_sub_stm32frompc_throttle_steering);
    sub_stm32frompc_transmission = NH.subscribe("stm32frompc/transmission", 1, cllbck_sub_stm32frompc_transmission);
    sub_stm32frompc_accessory = NH.subscribe("stm32frompc/accessory", 1, cllbck_sub_stm32frompc_accessory);
    //=====Publisher
    pub_stm32topc_remote = NH.advertise<std_msgs::UInt16MultiArray>("stm32topc/remote", 1);
    pub_stm32topc_encoder = NH.advertise<std_msgs::UInt16MultiArray>("stm32topc/encoder", 1);
    pub_stm32topc_gyroscope = NH.advertise<std_msgs::Float32>("stm32topc/gyroscope", 1);
    pub_stm32topc_throttle_steering_position = NH.advertise<std_msgs::Int16MultiArray>("stm32topc/throttle_steering_position", 1);
    pub_stm32topc_mode = NH.advertise<std_msgs::UInt8>("stm32topc/mode", 1);
    //=====Help
    _log.init(NH);

    if (interface_stm32_init() == -1)
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
    if (interface_stm32_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_stm32frompc_throttle_steering(const std_msgs::Int16MultiArrayConstPtr &msg)
{
    throttle = msg->data[0];
    steering = msg->data[1];
}

void cllbck_sub_stm32frompc_transmission(const std_msgs::Int8ConstPtr &msg)
{
    transmission = msg->data;
}

void cllbck_sub_stm32frompc_accessory(const std_msgs::UInt8ConstPtr &msg)
{
    accessory = msg->data;
}

//------------------------------------------------------------------------------
//==============================================================================

int interface_stm32_init()
{
    /* Printing the parameters. */
    ros::Duration(2).sleep();
    _log.info("IP: %s", stm32_ip.c_str());
    _log.info("Port: %d", stm32_port);

    //==================================

    /* Creating a socket. */
    if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        _log.error("Socket creation error. Returned: %d. (%s, %d)", socket_fd, __FILE__, __LINE__);
        return -1;
    }

    /* Setting the socket address. */
    socket_server_address.sin_family = AF_INET;
    socket_server_address.sin_addr.s_addr = inet_addr(stm32_ip.c_str());
    socket_server_address.sin_port = htons(stm32_port);

    return 0;
}

int interface_stm32_routine()
{
    // Communication Protocol (PC -> STM32)
    // ====================================
    // Offset   | Size  | Description
    // 0        | 4     | epoch_from_pc
    // 4        | 2     | throttle
    // 6        | 2     | steering
    // 8        | 1     | transmission
    // 10       | 1     | accessory
    memcpy(socket_tx_buffer + 0, &epoch_from_pc, 4);
    memcpy(socket_tx_buffer + 4, &throttle, 2);
    memcpy(socket_tx_buffer + 6, &steering, 2);
    memcpy(socket_tx_buffer + 8, &transmission, 1);
    memcpy(socket_tx_buffer + 10, &accessory, 1);

    /* Sending and receiving data from the STM32. */
    int len_data_from_pc = sendto(socket_fd, socket_tx_buffer, 64, MSG_DONTWAIT, (struct sockaddr *)&socket_server_address, sizeof(socket_server_address));
    int len_data_to_pc = recvfrom(socket_fd, socket_rx_buffer, 64, MSG_DONTWAIT, NULL, NULL);

    // Communication Protocol (STM32 -> PC)
    // ====================================
    // Offset   | Size  | Description
    // 0        | 4     | epoch_to_pc
    // 4        | 32    | remote
    // 36       | 2     | encoder_kiri
    // 38       | 2     | encoder_kanan
    // 40       | 4     | gyroscope
    // 44       | 2     | throttle_position
    // 46       | 2     | steering_position
    // 48       | 1     | mode
    memcpy(&epoch_to_pc, socket_rx_buffer + 0, 4);
    memcpy(&remote, socket_rx_buffer + 4, 32);
    memcpy(&encoder_kiri, socket_rx_buffer + 36, 2);
    memcpy(&encoder_kanan, socket_rx_buffer + 38, 2);
    memcpy(&gyroscope, socket_rx_buffer + 40, 4);
    memcpy(&throttle_position, socket_rx_buffer + 44, 2);
    memcpy(&steering_position, socket_rx_buffer + 46, 2);
    memcpy(&mode, socket_rx_buffer + 48, 1);

    //==================================

    epoch_from_pc++;

    //==================================

    static uint32_t last_epoch_to_pc = 0;

    /* This is to prevent the epoch from being checked when the epoch is 0.
    This is because the epoch is 0 when the program is started. */
    if (last_epoch_to_pc == 0)
    {
        // _log.warn("last epoch is 0. Initializing the last epoch to: %d. (%s, %d)", epoch_to_pc, __FILE__, __LINE__);
        last_epoch_to_pc = epoch_to_pc;
        return 0;
    }

    /* This is to check if the epoch is continuous.
    If the epoch is not continuous, it means that the STM32 is not receiving the data from the PC. */
    if (epoch_to_pc != last_epoch_to_pc + 1)
    {
        // _log.error("Epoch is not continuous. Expected: %d. Received: %d. (%s, %d)", last_epoch_to_pc + 1, epoch_to_pc, __FILE__, __LINE__);
        last_epoch_to_pc = 0;
        return 0;
    }

    /* This is to check if the data from the STM32 is received.
    If the data is not received, it means that the STM32 is not sending the data to the PC. */
    if (len_data_to_pc < 0)
    {
        // _log.error("Data from STM32 not received. Returned: %d. (%s, %d)", len_data_to_pc, __FILE__, __LINE__);
        return 0;
    }

    last_epoch_to_pc = epoch_to_pc;

    //==================================

    std_msgs::UInt16MultiArray msg_stm32topc_remote;
    for (int i = 0; i < 16; i++)
        msg_stm32topc_remote.data.push_back(remote[i]);
    pub_stm32topc_remote.publish(msg_stm32topc_remote);

    std_msgs::Int16MultiArray msg_stm32topc_encoder;
    msg_stm32topc_encoder.data.push_back(encoder_kiri);
    msg_stm32topc_encoder.data.push_back(encoder_kanan);
    pub_stm32topc_encoder.publish(msg_stm32topc_encoder);

    std_msgs::Float32 msg_stm32topc_gyroscope;
    msg_stm32topc_gyroscope.data = gyroscope;
    pub_stm32topc_gyroscope.publish(msg_stm32topc_gyroscope);

    std_msgs::Int16MultiArray msg_stm32topc_throttle_steering_position;
    msg_stm32topc_throttle_steering_position.data.push_back(throttle_position);
    msg_stm32topc_throttle_steering_position.data.push_back(steering_position);
    pub_stm32topc_throttle_steering_position.publish(msg_stm32topc_throttle_steering_position);

    std_msgs::UInt8 msg_stm32topc_mode;
    msg_stm32topc_mode.data = mode;
    pub_stm32topc_mode.publish(msg_stm32topc_mode);

    return 0;
}