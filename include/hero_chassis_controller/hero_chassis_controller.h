//
// Created by dj on 2026/10/30.
//

#ifndef SRC_HERO_CHASSIS_CONTROLLER_H
#define SRC_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <vector>

namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void cmdVelCallback(const geometry_msgs::Twist &msg);

        void odom_tf(const ros::Time &time, const ros::Duration &period);

        hardware_interface::JointHandle front_left_joint_,
                                        front_right_joint_,
                                        back_left_joint_,
                                        back_right_joint_;
        std::vector<hardware_interface::JointHandle> joints_;
        std::vector<double> commands_{0.0, 0.0, 0.0, 0.0}; //最新速度命令


    private:
        //机器人基本参数
        double wheel_base_;//前后轴距
        double wheel_radius_;//轮子半径
        double wheel_track_;//左右轮距


        control_toolbox::Pid fl_pid_, fr_pid_, bl_pid_, br_pid_;
        std::vector<control_toolbox::Pid> pids_;

        //订阅速度命令
        ros::Subscriber sub_command_;

        //odom
        std::vector<double> w_;//四个轮子角速度
        //正运动学解算的位置
        double x_ = 0.0;//记录位置
        double y_ = 0.0;
        double th_ = 0.0;
        //低通滤波参数
        double alpha_ = 0.2; // (0,1) 越小越平滑,延迟越大
        double v_x_smoothed_ = 0.0;
        double v_y_smoothed_ = 0.0;
        double w_z_smoothed_ = 0.0;
        //发布odom话题
        ros::Publisher odom_pub_ ;
        tf2_ros::TransformBroadcaster odom_broadcaster_;

    };
}// namespace hero_chassis_controller

#endif //SRC_HERO_CHASSIS_CONTROLLER_H