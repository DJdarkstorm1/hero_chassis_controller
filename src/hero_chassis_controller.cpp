//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>


namespace hero_chassis_controller {
bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    setlocale(LC_ALL, "");
    //0:fl 1:fr 2:bl 3br
    front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ =
      effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");
    joints_.push_back(front_left_joint_);
    joints_.push_back(front_right_joint_);
    joints_.push_back(back_left_joint_);
    joints_.push_back(back_right_joint_);

    //初始化PID
    fl_pid_.init(ros::NodeHandle(controller_nh, "fl_pid"));
    fr_pid_.init(ros::NodeHandle(controller_nh, "fr_pid"));
    bl_pid_.init(ros::NodeHandle(controller_nh, "bl_pid"));
    br_pid_.init(ros::NodeHandle(controller_nh, "br_pid"));
    pids_.push_back(fl_pid_);
    pids_.push_back(fr_pid_);
    pids_.push_back(bl_pid_);
    pids_.push_back(br_pid_);

    //机器人基本参数
    //ros::param::get("wheel_base",wheel_base_);//前后轴距
//
    //ros::param::get("wheel_radius",wheel_radius_);;//轮子半径
    //ros::param::get("wheel_track",wheel_track_);;//左右轮距
    wheel_base_ = 0.4;
    wheel_track_ = 0.4;
    wheel_radius_ = 0.07625;
    //速度
    commands_.assign(4, 0.0);
    sub_command_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

    //odom消息
    odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber("/joint_states",1,)
    return true;
}

void odom_pub_callback(const nav_msgs::Odometry &odom)
{

}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist &msg)
{
    //逆运动学解算
    double vel_x = msg.linear.x;
    double vel_y = msg.linear.y;
    double vel_rad = msg.angular.z;

    double fl_vel = (vel_x - vel_y -(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;
    double fr_vel = (vel_x + vel_y +(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;
    double bl_vel = (vel_x + vel_y -(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;
    double br_vel = (vel_x - vel_y +(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;

    ROS_INFO("轮距：%f,轴距：%f,轮半径：%f",wheel_track_,wheel_base_,wheel_radius_);
    ROS_INFO("cmd_vel: linear:[%f, %f], angular:%f", vel_x, vel_y, vel_rad);

    commands_[0]= fl_vel;//转动关节为角速度，要换算为先速度
    commands_[1]= fr_vel;//转动关节为角速度，要换算为先速度
    commands_[2]= bl_vel;//转动关节为角速度，要换算为先速度
    commands_[3]= br_vel;//转动关节为角速度，要换算为先速度
    ROS_INFO("fl:%f, fr:%f, bl:%f, br:%f",fl_vel*wheel_radius_,fr_vel*wheel_radius_,bl_vel*wheel_radius_,br_vel*wheel_radius_);

}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
    if (joints_.empty()) return;

    std::vector<double> errors;
    for (int i = 0; i < joints_.size(); i++)
    {
      double error = commands_[i] - joints_[i].getVelocity();
      errors.push_back(error);
    }

    std::vector<double> commanded_efforts;
    for (int i = 0; i < joints_.size(); i++)
    {
        double commanded_effort = pids_[i].computeCommand(errors[i], period);\
        commanded_efforts.push_back(commanded_effort);
    }
    //发布命令
    for (int i = 0; i < joints_.size(); i++)
    {
        joints_[i].setCommand(commanded_efforts[i]);
    }

}


PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}
