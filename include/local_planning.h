#ifndef LOCAL_PLANNING_H
#define LOCAL_PLANNING_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <easondrone_msgs/ControlCommand.h>
#include "apf.h"

using namespace std;

#define MIN_DIS 0.2

namespace Local_Planning{
class Local_Planner{
private:
    ros::NodeHandle local_planner_nh;

    // odometry state
    ros::Subscriber odom_sub_;
    bool have_odom_;
    // TODO: change to odom lost check
    ros::Time last_odom_stamp_;
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    double odom_roll_, odom_pitch_, odom_yaw_;

    // 参数
    double max_planning_vel;
    double safe_distance;

    // 订阅目标点、传感器数据（生成地图）
    ros::Subscriber goal_sub;
    ros::Subscriber local_point_clound_sub;

    ros::Publisher rviz_vel_pub;
    ros::Timer mainloop_timer, control_timer;

    // 局部避障算法 算子
    local_planning_alg::Ptr local_alg_ptr;

    // 无人机当前执行命令
    easondrone_msgs::ControlCommand ctrl_cmd_out_;
    // 发布控制指令
    ros::Publisher easondrone_ctrl_pub; 

    double distance_to_goal;

    // 规划器状态
    bool goal_ready; 
    bool is_safety;
    bool path_ok;

    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;
    double goal_yaw;

    int planner_state;
    Eigen::Vector3d desired_vel;

    geometry_msgs::Point vel_rviz;

    // 五种状态机
    enum EXEC_STATE{
        WAIT_GOAL,
        PLANNING,
    };
    EXEC_STATE exec_state;

    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void mainloop_cb(const ros::TimerEvent& e);
    void control_cb(const ros::TimerEvent& e);

public:
    Local_Planner(void): local_planner_nh("~") {}
    ~Local_Planner(){}

    double obs_distance;
    double att_distance;

    Eigen::Matrix<double, 3, 1> total_force;

    void init(ros::NodeHandle& nh);
};
}
#endif 
