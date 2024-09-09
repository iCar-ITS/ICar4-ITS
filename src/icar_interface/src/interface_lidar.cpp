#include "boost/thread/mutex.hpp"
#include "icar_interface/rings.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/passthrough.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "ps_ros_lib/help_log.h"
#include "ps_ros_lib/help_marker.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

//=====Prototype
void cllbck_tim_10hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_lidar_front_points(const sensor_msgs::PointCloud2ConstPtr &msg);
void cllbck_sub_lidar_rearright_points(const sensor_msgs::PointCloud2ConstPtr &msg);

int interface_lidar_init();
int interface_lidar_routine();

void lidar_rings_update();

//=====Parameter
float icar_body_length;
float icar_body_width;
float icar_body_height;
//=====Timer
ros::Timer tim_10hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_lidar_front_points;
ros::Subscriber sub_lidar_rearright_points;
//=====Publisher
ros::Publisher pub_lidar_base_points;
ros::Publisher pub_lidar_base_floor_points;
ros::Publisher pub_lidar_base_obstacle_points;
ros::Publisher pub_lidar_rings;
//=====TransformListener
tf::TransformListener *transform_listener;
//=====Mutex
boost::mutex mutex_lidar_front;
boost::mutex mutex_lidar_rearright;
//====Help
help_log _log;
help_marker _marker;

struct EIGEN_ALIGN16 PointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, time, time))

//-----Point cloud
//================
pcl::PointCloud<pcl::PointXYZ> lidar_front_points_all, lidar_front_points_all_on_base;
pcl::PointCloud<pcl::PointXYZ> lidar_front_points_ring[16], lidar_front_points_ring_on_base[16];
pcl::PointCloud<pcl::PointXYZ> lidar_rearright_points_all, lidar_rearright_points_all_on_base;
pcl::PointCloud<pcl::PointXYZ> lidar_base_points;
pcl::PointCloud<pcl::PointXYZ> lidar_base_floor_points;
pcl::PointCloud<pcl::PointXYZ> lidar_base_obstacle_points;

//-----Lidar ring
//===============
icar_interface::rings lidar_rings;
const float lidar_ring_angle_start = -M_PI_2;
const float lidar_ring_angle_stop = M_PI_2;
const float lidar_ring_angle_step = (lidar_ring_angle_stop - lidar_ring_angle_start) / 90;
icar_interface::ring default_lidar_ring;
icar_interface::rings default_lidar_rings;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_lidar");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("icar/body/length", icar_body_length);
    NH.getParam("icar/body/width", icar_body_width);
    NH.getParam("icar/body/height", icar_body_height);
    //=====Timer
    tim_10hz = NH.createTimer(ros::Duration(0.10), cllbck_tim_10hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_lidar_front_points = NH.subscribe("/lidar/front/points", 1, cllbck_sub_lidar_front_points);
    sub_lidar_rearright_points = NH.subscribe("/lidar/rearright/points", 1, cllbck_sub_lidar_rearright_points);
    //=====Publisher
    pub_lidar_base_points = NH.advertise<sensor_msgs::PointCloud2>("/lidar/base/points", 1);
    pub_lidar_base_floor_points = NH.advertise<sensor_msgs::PointCloud2>("/lidar/base/floor_points", 1);
    pub_lidar_base_obstacle_points = NH.advertise<sensor_msgs::PointCloud2>("/lidar/base/obstacle_points", 1);
    pub_lidar_rings = NH.advertise<icar_interface::rings>("/lidar/rings", 1);
    //=====TransformListener
    transform_listener = new tf::TransformListener(NH);
    //=====Help
    _log.init(NH);
    _marker.init(NH);

    if (interface_lidar_init() == -1)
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
    if (interface_lidar_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_lidar_front_points(const sensor_msgs::PointCloud2ConstPtr &msg)
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

    pcl::PointCloud<PointXYZIRT> points;
    pcl::fromROSMsg(*msg, points);

    /* Clearing the point cloud. */
    lidar_front_points_all.clear();
    for (int i = 0; i < 16; i++)
        lidar_front_points_ring[i].clear();
    /* Copying the point cloud data. */
    for (int i = 0; i < points.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = points.points[i].x;
        point.y = points.points[i].y;
        point.z = points.points[i].z;

        lidar_front_points_all.push_back(point);
        lidar_front_points_ring[points.points[i].ring].push_back(point);
    }

    //==================================

    mutex_lidar_front.lock();
    pcl_ros::transformPointCloud(lidar_front_points_all, lidar_front_points_all_on_base, transform);
    for (int i = 0; i < 16; i++)
        pcl_ros::transformPointCloud(lidar_front_points_ring[i], lidar_front_points_ring_on_base[i], transform);
    mutex_lidar_front.unlock();
}

void cllbck_sub_lidar_rearright_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    static tf::StampedTransform transform;
    static bool transform_initialized = false;

    /* Check whether the transform is already initialized.
     If not, try to get the transform from the tf tree. */
    if (!transform_initialized)
    {
        try
        {
            transform_listener->lookupTransform("base_link", "lidar_rearright_link", ros::Time(0), transform);
            transform_initialized = true;
        }
        catch (...)
        {
        }
    }

    //==================================

    pcl::fromROSMsg(*msg, lidar_rearright_points_all);

    //==================================

    mutex_lidar_rearright.lock();
    pcl_ros::transformPointCloud(lidar_rearright_points_all, lidar_rearright_points_all_on_base, transform);
    mutex_lidar_rearright.unlock();
}

//------------------------------------------------------------------------------
//==============================================================================

int interface_lidar_init()
{
    /* Printing the parameters. */
    ros::Duration(2).sleep();
    _log.info("Body Length: %.2fm", icar_body_length);
    _log.info("Body Width: %.2fm", icar_body_width);
    _log.info("Body Height: %.2fm", icar_body_height);

    default_lidar_ring.point.resize(90);
    for (int i = 0; i < 90; i++)
    {
        default_lidar_ring.point[i].x = 0;
        default_lidar_ring.point[i].y = 0;
        default_lidar_ring.point[i].z = 0;
    }

    default_lidar_rings.ring.resize(16);
    for (int i = 0; i < 16; i++)
    {
        default_lidar_rings.ring[i] = default_lidar_ring;
    }

    return 0;
}

int interface_lidar_routine()
{
    static tf::StampedTransform transform;
    static bool transform_initialized = false;

    /* Check whether the transform is already initialized.
    If not, try to get the transform from the tf tree. */
    if (!transform_initialized)
    {
        try
        {
            transform_listener->lookupTransform("base_link", "body_link", ros::Time(0), transform);
            transform_initialized = true;
        }
        catch (...)
        {
        }
    }

    //==================================

    lidar_base_points.clear();

    //==================================

    mutex_lidar_front.lock();
    lidar_base_points += lidar_front_points_all_on_base;
    mutex_lidar_front.unlock();

    mutex_lidar_rearright.lock();
    lidar_base_points += lidar_rearright_points_all_on_base;
    mutex_lidar_rearright.unlock();

    //==================================

    /* Cropping the point cloud to remove the points that are inside the car. */
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(lidar_base_points.makeShared());
    crop_box.setNegative(true);
    crop_box.setMin(Eigen::Vector4f(transform.getOrigin().x() - icar_body_length / 2 - 0.25,
                                    transform.getOrigin().y() - icar_body_width / 2 - 0.25,
                                    transform.getOrigin().z() - icar_body_height / 2 - 0.25,
                                    1.0));
    crop_box.setMax(Eigen::Vector4f(transform.getOrigin().x() + icar_body_length / 2 + 0.25,
                                    transform.getOrigin().y() + icar_body_width / 2 + 0.25,
                                    transform.getOrigin().z() + icar_body_height / 2 + 0.25,
                                    1.0));
    crop_box.filter(lidar_base_points);

    //==================================

    pcl::PassThrough<pcl::PointXYZ> pass_through;
    /* Filtering the point cloud to remove the points that are outside the range. */
    pass_through.setInputCloud(lidar_base_points.makeShared());
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(-12.5, 12.5);
    pass_through.filter(lidar_base_points);
    /* Filtering the point cloud to get the floor points and non-floor points. */
    pass_through.setInputCloud(lidar_base_points.makeShared());
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(-0.30, 0.30);
    pass_through.filter(lidar_base_floor_points);
    pass_through.setFilterLimits(0.60, 6.00);
    pass_through.filter(lidar_base_obstacle_points);

    //==================================

    lidar_rings_update();

    //==================================

    sensor_msgs::PointCloud2 msg_lidar_base_points;
    pcl::toROSMsg(lidar_base_points, msg_lidar_base_points);
    msg_lidar_base_points.header.frame_id = "base_link";
    msg_lidar_base_points.header.stamp = ros::Time::now();
    pub_lidar_base_points.publish(msg_lidar_base_points);

    sensor_msgs::PointCloud2 msg_lidar_base_floor_points;
    pcl::toROSMsg(lidar_base_floor_points, msg_lidar_base_floor_points);
    msg_lidar_base_floor_points.header.frame_id = "base_link";
    msg_lidar_base_floor_points.header.stamp = ros::Time::now();
    pub_lidar_base_floor_points.publish(msg_lidar_base_floor_points);

    sensor_msgs::PointCloud2 msg_lidar_base_obstacle_points;
    pcl::toROSMsg(lidar_base_obstacle_points, msg_lidar_base_obstacle_points);
    msg_lidar_base_obstacle_points.header.frame_id = "base_link";
    msg_lidar_base_obstacle_points.header.stamp = ros::Time::now();
    pub_lidar_base_obstacle_points.publish(msg_lidar_base_obstacle_points);

    //==================================

    pub_lidar_rings.publish(lidar_rings);

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

void lidar_rings_update()
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

    lidar_rings = default_lidar_rings;

    //==================================

    for (int i_ring = 0; i_ring < 16; i_ring++)
    {
        for (int i = 0; i < lidar_front_points_ring_on_base[i_ring].size(); i++)
        {
            float dx = lidar_front_points_ring_on_base[i_ring][i].x - transform.getOrigin().x();
            float dy = lidar_front_points_ring_on_base[i_ring][i].y - transform.getOrigin().y();
            float a = atan2f(dy, dx);

            //------------------------------

            /* The code is checking if the value of variable `a` is within a certain range defined by the variables
            `lidar_ring_angle_start` and `lidar_ring_angle_stop`. If `a` is not within that range, the
            `continue` statement skips the current iteration of the loop and moves on to the next one. */
            if (a < lidar_ring_angle_start || a > lidar_ring_angle_stop)
                continue;

            //------------------------------

            /* The code is calculating the index of an array or vector based on the value of `a` and
            the starting angle and step size of a LIDAR (Light Detection and Ranging) sensor's
            scanning range. The resulting index `i_point` can be used to access the corresponding
            element in the array or vector that represents the LIDAR data at that angle. */
            int i_point = (a - lidar_ring_angle_start) / lidar_ring_angle_step;

            /* The code is calculating the distance between two points in a 2D space. The coordinates of the first
            point are given by the variables "dx" and "dy", while the coordinates of the second point are stored
            in the "x" and "y" fields of a point object stored in a ring object in an array. The distances are
            calculated using the Pythagorean theorem, where the square root of the sum of the squares of the
            differences in the x and y coordinates is taken for each point. The results are stored in the
            variables "r_new" and "r_old". */
            float r_new = sqrtf(dx * dx + dy * dy);
            float r_old = sqrtf(lidar_rings.ring[i_ring].point[i_point].x * lidar_rings.ring[i_ring].point[i_point].x +
                                lidar_rings.ring[i_ring].point[i_point].y * lidar_rings.ring[i_ring].point[i_point].y);

            /* The code is checking if the value of `r_new` is less than `r_old`. If it is, then it updates the
            `x`, `y`, and `z` coordinates of a point in a Lidar ring based on the values of `dx`, `dy`, and
            `transform.getOrigin().x()`, `transform.getOrigin().y()`, and
            `lidar_front_points_ring_on_base[i_ring][i].z`. */
            if (r_new < r_old || r_old == 0.0)
            {
                lidar_rings.ring[i_ring].point[i_point].x = dx + transform.getOrigin().x();
                lidar_rings.ring[i_ring].point[i_point].y = dy + transform.getOrigin().y();
                lidar_rings.ring[i_ring].point[i_point].z = lidar_front_points_ring_on_base[i_ring][i].z;
            }
        }
    }
}