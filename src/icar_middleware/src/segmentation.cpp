#include "boost/thread/mutex.hpp"
#include "icar_interface/rings.h"
#include "pcl_ros/filters/passthrough.h"
#include "pcl_ros/point_cloud.h"
#include "ps_ros_lib/help_log.h"
#include "ps_ros_lib/help_marker.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

//=====Prototype
void cllbck_tim_10hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_lidar_base_points(const sensor_msgs::PointCloud2ConstPtr &msg);
void cllbck_sub_lidar_base_floor_points(const sensor_msgs::PointCloud2ConstPtr &msg);
void cllbck_sub_lidar_base_obstacle_points(const sensor_msgs::PointCloud2ConstPtr &msg);
void cllbck_sub_lidar_rings(const icar_interface::ringsConstPtr &msg);
void cllbck_sub_lidar_minimap_points(const sensor_msgs::PointCloud2ConstPtr &msg);
void cllbck_sub_pp_active_goal(const geometry_msgs::PointConstPtr &msg);

int segmentation_init();
int segmentation_routine();

void ring_to_cloud(const icar_interface::ring &ring, pcl::PointCloud<pcl::PointXYZ> &cloud);
void rotate_cloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, float yaw_offset = 0, float center_x = 0, float center_y = 0);
void rotate_point(const pcl::PointXYZ &point_in, pcl::PointXYZ &point_out, float yaw_offset = 0, float center_x = 0, float center_y = 0);

void find_delta_height(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointXYZ &point_out_left, pcl::PointXYZ &point_out_right);
void find_delta_length(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointXYZ &point_out_left, pcl::PointXYZ &point_out_right);
void find_tangent_angle(const icar_interface::ring &ring, pcl::PointXYZ &point_out_left, pcl::PointXYZ &point_out_right, float road_angle = 0, float origin_x = 0, float origin_y = 0);

//=====Timer
ros::Timer tim_10hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_lidar_base_points;
ros::Subscriber sub_lidar_base_floor_points;
ros::Subscriber sub_lidar_base_obstacle_points;
ros::Subscriber sub_lidar_rings;
ros::Subscriber sub_lidar_minimap_points;
ros::Subscriber sub_pp_active_goal;
//=====Publisher
ros::Publisher pub_lidar_ransac_floor_points;
ros::Publisher pub_lidar_ransac_obstacle_points;
//=====TransformListener
tf::TransformListener *transform_listener;
//=====Mutex
boost::mutex mutex_lidar_base_points;
boost::mutex mutex_lidar_base_floor_points;
boost::mutex mutex_lidar_base_obstacle_points;
boost::mutex mutex_lidar_minimap_points;
//=====Help
help_log _log;
help_marker _marker;

//-----Point cloud
//================
pcl::PointCloud<pcl::PointXYZ> lidar_base_points;
pcl::PointCloud<pcl::PointXYZ> lidar_base_floor_points;
pcl::PointCloud<pcl::PointXYZ> lidar_base_obstacle_points;
pcl::PointCloud<pcl::PointXYZ> lidar_minimap_points;

//-----Lidar ring
//===============
icar_interface::rings lidar_rings;

//-----Pure pursuit
//=================
float pp_goal_x;
float pp_goal_y;

//-----Transform
//==============
tf::StampedTransform transform_map_to_base;
tf::StampedTransform transform_map_to_lidar_front;
tf::StampedTransform transform_base_to_lidar_front;

//-----Misc
//=========
float road_angle;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segmentation");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_10hz = NH.createTimer(ros::Duration(0.10), cllbck_tim_10hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_lidar_base_points = NH.subscribe("/lidar/base/points", 1, cllbck_sub_lidar_base_points);
    sub_lidar_base_floor_points = NH.subscribe("/lidar/base/floor_points", 1, cllbck_sub_lidar_base_floor_points);
    sub_lidar_base_obstacle_points = NH.subscribe("/lidar/base/obstacle_points", 1, cllbck_sub_lidar_base_obstacle_points);
    sub_lidar_rings = NH.subscribe("/lidar/rings", 1, cllbck_sub_lidar_rings);
    sub_lidar_minimap_points = NH.subscribe("/lidar/minimap/points", 1, cllbck_sub_lidar_minimap_points);
    sub_pp_active_goal = NH.subscribe("/pp/active/goal", 1, cllbck_sub_pp_active_goal);
    //=====Publisher
    pub_lidar_ransac_floor_points = NH.advertise<sensor_msgs::PointCloud2>("/lidar/ransac/floor_points", 1);
    pub_lidar_ransac_obstacle_points = NH.advertise<sensor_msgs::PointCloud2>("/lidar/ransac/obstacle_points", 1);
    //=====TransformListener
    transform_listener = new tf::TransformListener(NH);
    //=====Help
    _log.init(NH);
    _marker.init(NH);

    if (segmentation_init() == -1)
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
    if (segmentation_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_lidar_base_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lidar_base_points.lock();
    pcl::fromROSMsg(*msg, lidar_base_points);
    mutex_lidar_base_points.unlock();
}

void cllbck_sub_lidar_base_floor_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lidar_base_floor_points.lock();
    pcl::fromROSMsg(*msg, lidar_base_floor_points);
    mutex_lidar_base_floor_points.unlock();
}

void cllbck_sub_lidar_base_obstacle_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lidar_base_obstacle_points.lock();
    pcl::fromROSMsg(*msg, lidar_base_obstacle_points);
    mutex_lidar_base_obstacle_points.unlock();
}

void cllbck_sub_lidar_rings(const icar_interface::ringsConstPtr &msg)
{
    lidar_rings = *msg;
}

void cllbck_sub_lidar_minimap_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lidar_minimap_points.lock();
    pcl::fromROSMsg(*msg, lidar_minimap_points);
    mutex_lidar_minimap_points.unlock();
}

void cllbck_sub_pp_active_goal(const geometry_msgs::PointConstPtr &msg)
{
    pp_goal_x = msg->x;
    pp_goal_y = msg->y;

    float dx = pp_goal_x - transform_map_to_lidar_front.getOrigin().x();
    float dy = pp_goal_y - transform_map_to_lidar_front.getOrigin().y();
    float a = atan2(dy, dx) - tf::getYaw(transform_map_to_base.getRotation());

    if (a > M_PI)
        road_angle = a - 2 * M_PI;
    else if (a < -M_PI)
        road_angle = a + 2 * M_PI;
    else
        road_angle = a;
}

//------------------------------------------------------------------------------
//==============================================================================

int segmentation_init()
{
    ros::Duration(2).sleep();

    return 0;
}

int segmentation_routine()
{
    try
    {
        transform_listener->lookupTransform("map", "base_link", ros::Time(0), transform_map_to_base);
        transform_listener->lookupTransform("map", "lidar_front_link", ros::Time(0), transform_map_to_lidar_front);
        transform_listener->lookupTransform("base_link", "lidar_front_link", ros::Time(0), transform_base_to_lidar_front);
    }
    catch (...)
    {
    }

    //----------------------------------

    if (lidar_rings.ring.size() != 16)
        return 0;

    //----------------------------------

    /**
     * Delta Height
     */
    pcl::PointCloud<pcl::PointXYZ> cloud_delta_height_left;
    pcl::PointCloud<pcl::PointXYZ> cloud_delta_height_right;
    for (int i = 0; i < 8; i++)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_0;
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        pcl::PointXYZ point_2_left;
        pcl::PointXYZ point_2_right;
        pcl::PointXYZ point_3_left;
        pcl::PointXYZ point_3_right;

        //------------------------------

        ring_to_cloud(lidar_rings.ring[i], cloud_0);
        rotate_cloud(cloud_0, cloud_1,
                     -road_angle,
                     transform_base_to_lidar_front.getOrigin().x(),
                     transform_base_to_lidar_front.getOrigin().y());
        find_delta_height(cloud_1, point_2_left, point_2_right);
        rotate_point(point_2_left, point_3_left,
                     road_angle,
                     transform_base_to_lidar_front.getOrigin().x(),
                     transform_base_to_lidar_front.getOrigin().y());
        rotate_point(point_2_right, point_3_right,
                     road_angle,
                     transform_base_to_lidar_front.getOrigin().x(),
                     transform_base_to_lidar_front.getOrigin().y());

        //------------------------------

        if (point_2_left.x != 0 && point_2_left.y != 0)
            cloud_delta_height_left.push_back(point_3_left);
        if (point_2_right.x != 0 && point_2_right.y != 0)
            cloud_delta_height_right.push_back(point_3_right);
    }

    /**
     * Delta Length
     */
    pcl::PointCloud<pcl::PointXYZ> cloud_delta_length_left;
    pcl::PointCloud<pcl::PointXYZ> cloud_delta_length_right;
    for (int i = 6; i < 10; i++)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_0;
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        pcl::PointXYZ point_2_left;
        pcl::PointXYZ point_2_right;
        pcl::PointXYZ point_3_left;
        pcl::PointXYZ point_3_right;

        //------------------------------

        ring_to_cloud(lidar_rings.ring[i], cloud_0);
        rotate_cloud(cloud_0, cloud_1,
                     -road_angle,
                     transform_base_to_lidar_front.getOrigin().x(),
                     transform_base_to_lidar_front.getOrigin().y());
        find_delta_length(cloud_1, point_2_left, point_2_right);
        rotate_point(point_2_left, point_3_left,
                     road_angle,
                     transform_base_to_lidar_front.getOrigin().x(),
                     transform_base_to_lidar_front.getOrigin().y());
        rotate_point(point_2_right, point_3_right,
                     road_angle,
                     transform_base_to_lidar_front.getOrigin().x(),
                     transform_base_to_lidar_front.getOrigin().y());

        //------------------------------

        if (point_2_left.x != 0 && point_2_left.y != 0)
            cloud_delta_length_left.push_back(point_3_left);
        if (point_2_right.x != 0 && point_2_right.y != 0)
            cloud_delta_length_right.push_back(point_3_right);
    }

    /**
     * Tangent Angle
     */
    pcl::PointCloud<pcl::PointXYZ> cloud_tangent_angle_left;
    pcl::PointCloud<pcl::PointXYZ> cloud_tangent_angle_right;
    for (int i = 8; i < 10; i++)
    {
        pcl::PointXYZ point_0_left;
        pcl::PointXYZ point_0_right;

        //------------------------------

        find_tangent_angle(lidar_rings.ring[i], point_0_left, point_0_right,
                           road_angle,
                           transform_base_to_lidar_front.getOrigin().x(),
                           transform_base_to_lidar_front.getOrigin().y());

        //------------------------------

        if (point_0_left.x != 0 && point_0_left.y != 0)
            cloud_tangent_angle_left.push_back(point_0_left);
        if (point_0_right.x != 0 && point_0_right.y != 0)
            cloud_tangent_angle_right.push_back(point_0_right);
    }

    //==============
    // Visualization
    //==============
    std::vector<geometry_msgs::Point> ps;
    geometry_msgs::Point p;

    ps.clear();
    for (int i = 0; i < cloud_delta_height_left.size(); i++)
    {
        p.x = cloud_delta_height_left[i].x;
        p.y = cloud_delta_height_left[i].y;
        p.z = cloud_delta_height_left[i].z;
        ps.push_back(p);
    }
    for (int i = 0; i < cloud_delta_height_right.size(); i++)
    {
        p.x = cloud_delta_height_right[i].x;
        p.y = cloud_delta_height_right[i].y;
        p.z = cloud_delta_height_right[i].z;
        ps.push_back(p);
    }
    _marker.sphere_list("base_link", "delta_height", 1, ps, 0.0, 1.0, 0.0, 1.0, 0.1, 0.1, 0.1);

    ps.clear();
    for (int i = 0; i < cloud_delta_length_left.size(); i++)
    {
        p.x = cloud_delta_length_left[i].x;
        p.y = cloud_delta_length_left[i].y;
        p.z = cloud_delta_length_left[i].z;
        ps.push_back(p);
    }
    for (int i = 0; i < cloud_delta_length_right.size(); i++)
    {
        p.x = cloud_delta_length_right[i].x;
        p.y = cloud_delta_length_right[i].y;
        p.z = cloud_delta_length_right[i].z;
        ps.push_back(p);
    }
    _marker.sphere_list("base_link", "delta_length", 1, ps, 1.0, 0.0, 0.0, 1.0, 0.1, 0.1, 0.1);

    ps.clear();
    for (int i = 0; i < cloud_tangent_angle_left.size(); i++)
    {
        p.x = cloud_tangent_angle_left[i].x;
        p.y = cloud_tangent_angle_left[i].y;
        p.z = cloud_tangent_angle_left[i].z;
        ps.push_back(p);
    }
    for (int i = 0; i < cloud_tangent_angle_right.size(); i++)
    {
        p.x = cloud_tangent_angle_right[i].x;
        p.y = cloud_tangent_angle_right[i].y;
        p.z = cloud_tangent_angle_right[i].z;
        ps.push_back(p);
    }
    _marker.sphere_list("base_link", "tangent_angle", 1, ps, 1.0, 1.0, 0.0, 1.0, 0.1, 0.1, 0.1);

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

void ring_to_cloud(const icar_interface::ring &ring, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();
    for (int i = 0; i < ring.point.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = ring.point[i].x;
        point.y = ring.point[i].y;
        point.z = ring.point[i].z;
        cloud.push_back(point);
    }
}

void rotate_cloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, float yaw_offset, float center_x, float center_y)
{
    float s = sinf(yaw_offset);
    float c = cosf(yaw_offset);

    cloud_out.clear();
    for (int i = 0; i < cloud_in.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = center_x + (cloud_in[i].x - center_x) * c - (cloud_in[i].y - center_y) * s;
        point.y = center_y + (cloud_in[i].x - center_x) * s + (cloud_in[i].y - center_y) * c;
        point.z = cloud_in[i].z;
        cloud_out.push_back(point);
    }
}

void rotate_point(const pcl::PointXYZ &point_in, pcl::PointXYZ &point_out, float yaw_offset, float center_x, float center_y)
{
    float s = sinf(yaw_offset);
    float c = cosf(yaw_offset);

    point_out.x = center_x + (point_in.x - center_x) * c - (point_in.y - center_y) * s;
    point_out.y = center_y + (point_in.x - center_x) * s + (point_in.y - center_y) * c;
    point_out.z = point_in.z;
}

//------------------------------------------------------------------------------
//==============================================================================

void find_delta_height(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointXYZ &point_out_left, pcl::PointXYZ &point_out_right)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in.makeShared());
    pass.setFilterFieldName("y");

    //==========
    // Left side
    //==========
    for (float y = 0; y < 5.0; y += 0.2)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_temp;

        pass.setFilterLimits(y, y + 0.2);
        pass.filter(cloud_temp);

        float max_x = -1000;
        float min_x = 1000;
        float max_z = -1000;
        float min_z = 1000;
        for (int i = 0; i < cloud_temp.size(); i++)
        {
            if (cloud_temp[i].x > max_x)
                max_x = cloud_temp[i].x;
            if (cloud_temp[i].x < min_x)
                min_x = cloud_temp[i].x;
            if (cloud_temp[i].z > max_z)
                max_z = cloud_temp[i].z;
            if (cloud_temp[i].z < min_z)
                min_z = cloud_temp[i].z;
        }

        if (max_z - min_z > 0.07)
        {
            point_out_left.x = (max_x + min_x) / 2;
            point_out_left.y = y + 0.1;
            point_out_left.z = (max_z + min_z) / 2;
            break;
        }
    }

    //===========
    // Right side
    //===========
    for (float y = 0; y > -5.0; y -= 0.2)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_temp;

        pass.setFilterLimits(y - 0.2, y);
        pass.filter(cloud_temp);

        float max_x = -1000;
        float min_x = 1000;
        float max_z = -1000;
        float min_z = 1000;
        for (int i = 0; i < cloud_temp.size(); i++)
        {
            if (cloud_temp[i].x > max_x)
                max_x = cloud_temp[i].x;
            if (cloud_temp[i].x < min_x)
                min_x = cloud_temp[i].x;
            if (cloud_temp[i].z > max_z)
                max_z = cloud_temp[i].z;
            if (cloud_temp[i].z < min_z)
                min_z = cloud_temp[i].z;
        }

        if (max_z - min_z > 0.07)
        {
            point_out_right.x = (max_x + min_x) / 2;
            point_out_right.y = y - 0.1;
            point_out_right.z = (max_z + min_z) / 2;
            break;
        }
    }
}

void find_delta_length(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointXYZ &point_out_left, pcl::PointXYZ &point_out_right)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in.makeShared());
    pass.setFilterFieldName("y");

    //==========
    // Left side
    //==========
    for (float y = 0; y < 5.0; y += 0.2)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_temp;

        pass.setFilterLimits(y, y + 0.2);
        pass.filter(cloud_temp);

        float max_x = -1000;
        float min_x = 1000;
        float max_z = -1000;
        float min_z = 1000;
        for (int i = 0; i < cloud_temp.size(); i++)
        {
            if (cloud_temp[i].x > max_x)
                max_x = cloud_temp[i].x;
            if (cloud_temp[i].x < min_x)
                min_x = cloud_temp[i].x;
            if (cloud_temp[i].z > max_z)
                max_z = cloud_temp[i].z;
            if (cloud_temp[i].z < min_z)
                min_z = cloud_temp[i].z;
        }
        if (max_x - min_x > 0.14)
        {
            point_out_left.x = (max_x + min_x) / 2;
            point_out_left.y = y + 0.1;
            point_out_left.z = (max_z + min_z) / 2;
            break;
        }
    }

    //===========
    // Right side
    //===========
    for (float y = 0; y > -5.0; y -= 0.2)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_temp;

        pass.setFilterLimits(y - 0.2, y);
        pass.filter(cloud_temp);

        float max_x = -1000;
        float min_x = 1000;
        float max_z = -1000;
        float min_z = 1000;
        for (int i = 0; i < cloud_temp.size(); i++)
        {
            if (cloud_temp[i].x > max_x)
                max_x = cloud_temp[i].x;
            if (cloud_temp[i].x < min_x)
                min_x = cloud_temp[i].x;
            if (cloud_temp[i].z > max_z)
                max_z = cloud_temp[i].z;
            if (cloud_temp[i].z < min_z)
                min_z = cloud_temp[i].z;
        }
        if (max_x - min_x > 0.14)
        {
            point_out_right.x = (max_x + min_x) / 2;
            point_out_right.y = y - 0.1;
            point_out_right.z = (max_z + min_z) / 2;
            break;
        }
    }
}

void find_tangent_angle(const icar_interface::ring &ring, pcl::PointXYZ &point_out_left, pcl::PointXYZ &point_out_right, float road_angle, float origin_x, float origin_y)
{
    const static float angle_start = -M_PI / 2;
    const static float angle_stop = M_PI / 2;
    const static float angle_step = (angle_stop - angle_start) / 90;

    int index_offset = road_angle / angle_step;
    if (index_offset < -43)
        index_offset = -43;
    else if (index_offset > 43)
        index_offset = 43;

    //==========
    // Left side
    //==========
    for (int i = index_offset + 45; i < 88; i++)
    {
        float tangent_0_x = ring.point[i - 2].x;
        float tangent_0_y = ring.point[i - 2].y;
        float tangent_1_x = ring.point[i + 2].x;
        float tangent_1_y = ring.point[i + 2].y;
        float radial_x = ring.point[i].x;
        float radial_y = ring.point[i].y;

        float angle_tangent = atan2f(tangent_1_y - tangent_0_y, tangent_1_x - tangent_0_x);
        float angle_radial = atan2f(radial_y - origin_y, radial_x - origin_x);

        if (sinf(angle_tangent - angle_radial) < 0.75)
        {
            point_out_left.x = radial_x;
            point_out_left.y = radial_y;
            point_out_left.z = ring.point[i].z;
            break;
        }
    }

    //===========
    // Right side
    //===========
    for (int i = index_offset + 44; i >= 2; i--)
    {
        float tangent_0_x = ring.point[i - 2].x;
        float tangent_0_y = ring.point[i - 2].y;
        float tangent_1_x = ring.point[i + 2].x;
        float tangent_1_y = ring.point[i + 2].y;
        float radial_x = ring.point[i].x;
        float radial_y = ring.point[i].y;

        float angle_tangent = atan2f(tangent_1_y - tangent_0_y, tangent_1_x - tangent_0_x);
        float angle_radial = atan2f(radial_y - origin_y, radial_x - origin_x);

        if (sinf(angle_tangent - angle_radial) < 0.75)
        {
            point_out_right.x = radial_x;
            point_out_right.y = radial_y;
            point_out_right.z = ring.point[i].z;
            break;
        }
    }
}
