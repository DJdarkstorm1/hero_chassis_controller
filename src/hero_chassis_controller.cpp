//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    controller_nh.getParam("wheel_base",wheel_base_);//前后轴距
    controller_nh.getParam("wheel_radius",wheel_radius_);;//轮子半径
    controller_nh.getParam("wheel_track",wheel_track_);;//左右轮距
    //wheel_base_ = 0.4;
    //wheel_track_ = 0.4;
    //wheel_radius_ = 0.07625;
    //速度
    commands_.assign(4, 0.0);
    sub_command_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

    //odom
    w_.assign(4, 0.0);//关节角速度
    odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 100);

    //底盘/全局
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    controller_nh.param<std::string>("odom_frame", odom_frame_, "odom");
    controller_nh.param<std::string>("base_frame", base_frame_, "base_link");
    //controller_nh.param("global_frame_mode", global_frame_mode_, false);
    dyn_server_ = std::make_unique<dynamic_reconfigure::Server<
      hero_chassis_controller::global_modeConfig>>(
      ros::NodeHandle(controller_nh, "global_frame_mode"));
    dyn_server_->setCallback(
      boost::bind(&HeroChassisController::dynReconfCallback, this, _1, _2));
    return true;
}

void HeroChassisController::dynReconfCallback(hero_chassis_controller::global_modeConfig &config,uint32_t level)
{
    global_frame_mode_ =config.global_frame_mode;
}

void HeroChassisController::odom_tf(const ros::Time &time, const ros::Duration &period)
{
    //正运动学解算

    for(int i = 0; i < 4; i++) {
        w_[i] = joints_[i].getVelocity();
    }

    double v_x = ( w_[0]+w_[1]+w_[2]+w_[3])*wheel_radius_/4;
    double v_y = (-w_[0]+w_[1]+w_[2]-w_[3])*wheel_radius_/4;
    double w_z = (-w_[0]+w_[1]-w_[2]+w_[3])*wheel_radius_/(4*(wheel_base_+wheel_track_));

    double dt = period.toSec();
    double delta_x = (v_x * cos(th_) - v_y * sin(th_)) * dt;
    double delta_y = (v_x * sin(th_) + v_y * cos(th_)) * dt;
    double delta_th = w_z * dt;
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
    //在2维平面只要Yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // 设置速度
    odom.child_frame_id = "base_link";


    v_x_smoothed_ = alpha_ * v_x + (1.0 - alpha_) * v_x_smoothed_;
    v_y_smoothed_ = alpha_ * v_y + (1.0 - alpha_) * v_y_smoothed_;
    w_z_smoothed_ = alpha_ * w_z + (1.0 - alpha_) * w_z_smoothed_;
    odom.twist.twist.linear.x  = v_x_smoothed_;
    odom.twist.twist.linear.y  = v_y_smoothed_;
    odom.twist.twist.angular.z = w_z_smoothed_;//w_z;

    // 发布消息
    odom_pub_.publish(odom);
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist &msg)
{
    //逆运动学解算
    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_rad  = 0.0;

    //底盘/全局
    if (global_frame_mode_) {
        try {
            // 1. 查询 odom → base_link 的变换
            //    参数顺序："target_frame", "source_frame", time
            //    返回的 transform 表示把 source 中的数据变换到 target
            geometry_msgs::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(
                    base_frame_,   // 子坐标系
                    odom_frame_,   // 父坐标系
                    ros::Time(0),
                    ros::Duration(0.1));  // 超时 100ms，避免阻塞太久

            // 2. 取出旋转部分，转成 tf2::Quaternion
            tf2::Quaternion q;
            tf2::fromMsg(transform_stamped.transform.rotation, q);

            // 3. 用四元数旋转线速度向量（速度是自由向量，只旋转不平移）
            tf2::Vector3 v_world(msg.linear.x, msg.linear.y, 0.0);
            tf2::Vector3 v_body = tf2::quatRotate(q, v_world);

            // 4. 写回 Twist（angular.z 在平面运动中不变）
            vel_x = v_body.x();
            vel_y = v_body.y();

        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0,
                "TF2 lookup %s -> %s failed: %s, using original command",
                odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        }
    }
    else
    {
        vel_x = msg.linear.x;
        vel_y = msg.linear.y;

    }
    vel_rad = msg.angular.z;

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
    //正运动学解算+odom发布
    odom_tf(time,period);
}


PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}
