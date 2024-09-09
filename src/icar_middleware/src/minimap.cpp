#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#define RADIUS(b, a) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2))
#define ANGLE(b, a) atan2(b.y - a.y, b.x - a.x)

//=====Prototype
void cllbck_tim_10hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_odometry_twist(const geometry_msgs::TwistConstPtr &msg);
void cllbck_sub_odometry_pose2d(const geometry_msgs::Pose2DConstPtr &msg);
void cllbck_sub_lidar_base_points(const sensor_msgs::PointCloud2ConstPtr &msg);

int minimap_init();
int minimap_routine();

void minimap_update();

//=====Timer
ros::Timer tim_10hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_odometry_twist;
ros::Subscriber sub_odometry_pose2d;
ros::Subscriber sub_lidar_base_points;
//=====Publisher
ros::Publisher pub_lidar_minimap_points;

//-----Odometry
//=============
geometry_msgs::Twist odometry_twist;
geometry_msgs::Pose2D odometry_pose2d;

//-----Point cloud
//================
pcl::PointCloud<pcl::PointXYZ> lidar_base_points;
pcl::PointCloud<pcl::PointXYZ> lidar_minimap_points;

//-----Point pool
//===============
pcl::PointCloud<pcl::PointXYZ> points_pool[5];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "minimap");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_10hz = NH.createTimer(ros::Duration(0.10), cllbck_tim_10hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_odometry_twist = NH.subscribe("/odometry/twist", 1, cllbck_sub_odometry_twist);
    sub_odometry_pose2d = NH.subscribe("/odometry/pose2d", 1, cllbck_sub_odometry_pose2d);
    sub_lidar_base_points = NH.subscribe("/lidar/base/points", 1, cllbck_sub_lidar_base_points);
    //=====Publisher
    pub_lidar_minimap_points = NH.advertise<sensor_msgs::PointCloud2>("/lidar/minimap/points", 1);

    if (minimap_init() == -1)
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
    if (minimap_routine() == -1)
        ros::shutdown();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_odometry_twist(const geometry_msgs::TwistConstPtr &msg)
{
    odometry_twist = *msg;
}

void cllbck_sub_odometry_pose2d(const geometry_msgs::Pose2DConstPtr &msg)
{
    odometry_pose2d = *msg;
}

void cllbck_sub_lidar_base_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, lidar_base_points);

    minimap_update();
}

//------------------------------------------------------------------------------
//==============================================================================

int minimap_init()
{
    ros::Duration(2).sleep();

    return 0;
}

int minimap_routine()
{
    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

void minimap_update()
{
    static geometry_msgs::Pose2D last_odometry_pose2d;

    if (RADIUS(odometry_pose2d, last_odometry_pose2d) > 0.5)
    {
        float a = ANGLE(odometry_pose2d, last_odometry_pose2d);
        float o = odometry_pose2d.theta;
        float delta_angle = a - o;
        if (delta_angle > M_PI)
            delta_angle -= 2 * M_PI;
        else if (delta_angle < -M_PI)
            delta_angle += 2 * M_PI;

        bool is_forward = fabsf(delta_angle) < M_PI_2 ? true : false;

        //------------------------------

        /* This code block is creating an affine transformation matrix using the Eigen library. The
        transformation matrix is used to transform the point cloud data from the previous position to the
        current position based on the change in odometry pose. */
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        float tf_position = RADIUS(odometry_pose2d, last_odometry_pose2d);
        float tf_orientation = odometry_pose2d.theta - last_odometry_pose2d.theta;
        transform.translate(Eigen::Vector3f(is_forward ? -tf_position : tf_position, 0.0, 0.0));
        transform.rotate(Eigen::AngleAxisf(-tf_orientation, Eigen::Vector3f::UnitZ()));

        //------------------------------

        /* This code block is transforming the point clouds in the `points_pool` vector based on the change in
        odometry pose. It starts by iterating through the `points_pool` vector from index 1 to 4 and
        transforms each point cloud using the affine transformation matrix `transform`. The transformed
        point cloud is then stored in the previous index of the vector. Finally, the current lidar base
        point cloud is stored in the last index of the vector. This process effectively shifts all the point
        clouds in the vector by one index and adds the current lidar base point cloud to the end of the
        vector. */
        for (int i = 1; i < 5; i++)
            pcl::transformPointCloud(points_pool[i], points_pool[i - 1], transform);
        points_pool[4] = lidar_base_points;

        //------------------------------

        /* This code block is updating the `lidar_minimap_points` point cloud by concatenating the 5 point
        clouds stored in the `points_pool` vector. It first clears the `lidar_minimap_points` point cloud
        and then iterates through the `points_pool` vector from index 0 to 4 and adds each point cloud to
        the `lidar_minimap_points` point cloud using the `+=` operator. This process effectively combines
        the 20 point clouds into a single point cloud, which represents the minimap of the lidar data. */
        lidar_minimap_points.clear();
        for (int i = 0; i < 5; i++)
            lidar_minimap_points += points_pool[i];

        //------------------------------

        /* A voxel grid filter. It is used to downsample the point cloud. */
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(lidar_minimap_points.makeShared());
        voxel_grid.setLeafSize(0.25, 0.25, 0.01);
        voxel_grid.filter(lidar_minimap_points);

        //------------------------------

        sensor_msgs::PointCloud2 msg_lidar_minimap_points;
        pcl::toROSMsg(lidar_minimap_points, msg_lidar_minimap_points);
        msg_lidar_minimap_points.header.frame_id = "base_link";
        msg_lidar_minimap_points.header.stamp = ros::Time::now();
        pub_lidar_minimap_points.publish(msg_lidar_minimap_points);

        //------------------------------

        /* Saving the current odometry pose2d to the last odometry pose2d. */
        last_odometry_pose2d = odometry_pose2d;
    }
}