#include "local_planning.h"
#include <cmath>

namespace Local_Planning{
// 局部规划算法 初始化函数
void Local_Planner::init(ros::NodeHandle& nh){
    // 最大速度
    nh.param("local_planner/max_planning_vel", max_planning_vel, 0.4);

    // 定时函数,执行周期为1Hz
    mainloop_timer = nh.createTimer(ros::Duration(0.2), &Local_Planner::mainloop_cb, this);
    // 控制定时器
    control_timer = nh.createTimer(ros::Duration(0.05), &Local_Planner::control_cb, this);

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>
                ("/mavros/local_position/odom", 10, &Local_Planner::odometryCallback, this);
    // 订阅目标点
    goal_sub = nh.subscribe
    ("/planner/goal", 1, &Local_Planner::goal_cb, this);
    // 订阅传感器点云信息,该话题名字可在launch文件中任意指定
    local_point_clound_sub = nh.subscribe<sensor_msgs::PointCloud2>
    ("/prometheus/sensors/3Dlidar_scan", 1, &Local_Planner::localcloudCallback, this);

    // 发布 期望速度
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>
    ("/prometheus/control_command", 10);
    // 发布速度用于显示
    rviz_vel_pub = nh.advertise<geometry_msgs::Point>
    ("/prometheus/local_planner/desired_vel", 10); 

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(4);

    cout << "[planner] apf_local_planner initialized!" << endl;

    // 选择避障算法
    local_alg_ptr.reset(new APF);
    local_alg_ptr->init(nh);

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    goal_ready = false;
    path_ok = false;

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.source = NODE_NAME;
    desired_yaw = 0.0;

    // 地图初始化
    sensor_msgs::PointCloud2ConstPtr init_local_map(new sensor_msgs::PointCloud2());
    local_map_ptr_ = init_local_map;

    ros::spin();
}

// 保存无人机当前里程计信息，包括位置、速度和姿态
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // TODO: add odom lost check
    have_odom_ = true;
    last_odom_stamp_ = ros::Time::now();

    odom_pos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;
    start_pos = odom_pos_;

    odom_vel_ << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;
    start_vel = odom_vel_;

    //odom_acc_ = estimateAcc( msg );

    // 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    tf::Quaternion odom_q_(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
    );

    tf::Matrix3x3(odom_q_).getRPY(odom_roll_, odom_pitch_, odom_yaw_);

    if (local_alg_ptr) {
        local_alg_ptr->set_odom(*msg);
    } else {
        ROS_ERROR("local_alg_ptr is nullptr");
    }
}

void Local_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg){
    goal_pos << msg->pose.position.x, msg->pose.position.y, odom_pos_[2];
    goal_vel.setZero();
    goal_yaw = 2 * std::atan2(msg->pose.orientation.z, msg->pose.orientation.w);

    goal_ready = true;
}

void Local_Planner::localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    local_map_ptr_ = msg;
    local_alg_ptr->set_local_map(local_map_ptr_);

    pcl::fromROSMsg(*msg, latest_local_pcl_);
}

void Local_Planner::control_cb(const ros::TimerEvent& e){
    if(!path_ok){
        return;
    }

    distance_to_goal = (start_pos - goal_pos).norm();

    // 抵达终点
    if(distance_to_goal < MIN_DIS){
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = goal_pos[0];
        Command_Now.Reference_State.position_ref[1]     = goal_pos[1];
        Command_Now.Reference_State.position_ref[2]     = goal_pos[2];
        Command_Now.Reference_State.yaw_ref             = goal_yaw;

        command_pub.publish(Command_Now);

        cout << "[planner] Reach the goal!" << endl;
        
        // 停止执行
        path_ok = false;
        // 转换状态为等待目标
        exec_state = EXEC_STATE::WAIT_GOAL;
        return;
    }

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_VEL;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.velocity_ref[0]     = desired_vel[0];
    Command_Now.Reference_State.velocity_ref[1]     = desired_vel[1];
    Command_Now.Reference_State.velocity_ref[2]     = desired_vel[2];
    Command_Now.Reference_State.yaw_ref             = atan2(desired_vel(1), desired_vel(0));

    command_pub.publish(Command_Now);

    //　发布rviz显示
    vel_rviz.x = desired_vel(0);
    vel_rviz.y = desired_vel(1);
    vel_rviz.z = desired_vel(2);

    rviz_vel_pub.publish(vel_rviz);
}

void Local_Planner::mainloop_cb(const ros::TimerEvent& e){
    switch (exec_state){
        case WAIT_GOAL:{
            path_ok = false;
            if(goal_ready){
                // 获取到目标点后，生成新轨迹
                exec_state = EXEC_STATE::PLANNING;
                goal_ready = false;
            }
            break;
        }

        case PLANNING:{
            // desired_vel是返回的规划速度；返回值为2时,飞机不安全(距离障碍物太近)
            planner_state = local_alg_ptr->compute_force(goal_pos, desired_vel);

            path_ok = true;

            //　对规划的速度进行限幅处理
            if(desired_vel.norm() > max_planning_vel){
                desired_vel = desired_vel / desired_vel.norm() * max_planning_vel;
            }

            if(planner_state == 1){
                cout << "[planner] desired vel: " << desired_vel(0) << " " << desired_vel(1) << " " << desired_vel(2) << endl;
            }
            else if(planner_state == 2){
                cout << "[planner] Dangerous!" << endl;
            }
            break;
        }
    }
}
}
