//
// Created by dj on 2026/10/30.
//

#ifndef SRC_HERO_CHASSIS_CONTROLLER_H
#define SRC_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_buffer.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/global_modeConfig.h>

#include "hero_chassis_controller/power_msgs.h"
#include <hardware_interface/imu_sensor_interface.h>
#include <sensor_msgs/Imu.h>


namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
        //插件管理器循环调用的函数
        void update(const ros::Time &time, const ros::Duration &period) override;
        //掉线重新加载的开始函数
        void starting(const ros::Time& time) override;

    private:

        //关节接口
        hardware_interface::JointHandle front_left_joint_,
                                        front_right_joint_,
                                        back_left_joint_,
                                        back_right_joint_;
        std::vector<hardware_interface::JointHandle> joints_;

        std::vector<double> commands_{0.0, 0.0, 0.0, 0.0}; //最新速度命令

        //底盘参数
        double wheel_base_ = 0.4;//前后轴距
        double wheel_radius_ = 0.07625;//轮子半径
        double wheel_track_ = 0.04;//左右轮距

        //PID接口
        control_toolbox::Pid fl_pid_, fr_pid_, bl_pid_, br_pid_;
        std::array<control_toolbox::Pid*, 4> pids_ = {&fl_pid_, &fr_pid_, &bl_pid_, &br_pid_};
        //cmd_vel
        realtime_tools::RealtimeBuffer<geometry_msgs::Twist> raw_cmd_buffer_;//存获取的速度消息
        ros::Subscriber sub_command_;//订阅速度命令

        //odom里程计
        std::vector<double> w_;//四个轮子角速度
        double x_ = 0.0;//正运动学解算的位置
        double y_ = 0.0;
        double th_ = 0.0;
        double alpha_ = 0.2; // 低通滤波参数：(0,1) 越小越平滑,延迟越大
        double v_x_smoothed_ = 0.0;
        double v_y_smoothed_ = 0.0;
        double w_z_smoothed_ = 0.0;
        ros::Publisher odom_pub_ ;//发布odom里程计话题
        tf2_ros::TransformBroadcaster odom_broadcaster_;

        //底盘/全局模式变换
        bool global_frame_mode_ = false;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::string odom_frame_ = "odom";
        std::string base_frame_ = "base_link";

        //动态参数调节底盘/全局模式
        std::unique_ptr<dynamic_reconfigure::Server<
            hero_chassis_controller::global_modeConfig>> dyn_server_;

        //底盘加速度限制
        double max_lin_acc_x_ = 2.0,
               max_lin_acc_y_ = 2.0,
               max_lin_dec_x_ = 4.0,
               max_lin_dec_y_ = 4.0;//线加速度
        double max_ang_acc_z_ = 4.0, max_ang_dec_z_ = 8.0;//角加速度
        geometry_msgs::Twist last_smoothed_cmd_;//记录上一次平滑处理后的命令

        //底盘功率控制
        double max_Power_ = 0.0;//最大功率限制
        double scale_k_ = 1.0;//限制系数
        double in_Power_ = 0.0;//输入功率
        double out_Power_ = 0.0;//输出功率
        std::vector<double> commanded_efforts_;//pid的原始力矩
        hero_chassis_controller::power_msgs power_msg_;
        ros::Publisher power_pub_;

        // ---- Joint Limits ----
        std::vector<joint_limits_interface::EffortJointSaturationHandle> effort_limit_handles_;

        //IMU
        ros::Subscriber imu_sub_;
        realtime_tools::RealtimeBuffer<sensor_msgs::Imu> imu_rt_buffer_; // 实时缓冲区
        sensor_msgs::Imu current_imu_data_;

        //======================函数===================
        //速度订阅回调函数
        void cmdVelCallback(const geometry_msgs::Twist &msg);
        //base_link到odom里程计发布函数
        void odom_tf(const ros::Time &time, const ros::Duration &period);
        //动态参数回调函数
        void dynReconfCallback(hero_chassis_controller::global_modeConfig &config,uint32_t level);
        //加速度限制
        void applyVelocityLimits(geometry_msgs::Twist& cmd, const ros::Duration& period);
        //底盘功率控制
        void PowerLimit();
        //IMU
        void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    };
}// namespace hero_chassis_controller

#endif //SRC_HERO_CHASSIS_CONTROLLER_H