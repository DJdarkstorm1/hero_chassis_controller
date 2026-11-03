//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits_urdf.h>

namespace hero_chassis_controller {
bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    setlocale(LC_ALL, "");
    //关节力控接口
    //0:fl 1:fr 2:bl 3br
    front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ =
      effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");
    joints_ = {front_left_joint_,front_right_joint_,back_left_joint_,back_right_joint_};

    //初始化PID
    fl_pid_.init(ros::NodeHandle(controller_nh, "fl_pid"));
    fr_pid_.init(ros::NodeHandle(controller_nh, "fr_pid"));
    bl_pid_.init(ros::NodeHandle(controller_nh, "bl_pid"));
    br_pid_.init(ros::NodeHandle(controller_nh, "br_pid"));

    //底盘参数
    controller_nh.param("wheel_base",wheel_base_,0.4);//前后轴距
    controller_nh.param("wheel_radius",wheel_radius_,0.07625);;//轮子半径
    controller_nh.param("wheel_track",wheel_track_,0.4);;//左右轮距

    //加速度限制参数
    controller_nh.param("max_lin_acc_x", max_lin_acc_x_, 2.0);
    controller_nh.param("max_lin_dec_x", max_lin_dec_x_, 4.0);
    controller_nh.param("max_lin_acc_y", max_lin_acc_y_, 2.0);
    controller_nh.param("max_lin_dec_y", max_lin_dec_y_, 4.0);
    controller_nh.param("max_ang_acc_z", max_ang_acc_z_, 4.0);
    controller_nh.param("max_ang_dec_z", max_ang_dec_z_, 8.0);
    last_smoothed_cmd_ = geometry_msgs::Twist();

    //cmd_vel速度话题订阅
    commands_.assign(4, 0.0);
    sub_command_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

    //odom里程计
    w_.assign(4, 0.0);//关节角速度
    odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 100);

    //底盘/全局模式
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    controller_nh.param<std::string>("odom_frame", odom_frame_, "odom");
    controller_nh.param<std::string>("base_frame", base_frame_, "base_link");

    //动态参数
    dyn_server_ = std::make_unique<dynamic_reconfigure::Server<
      hero_chassis_controller::global_modeConfig>>(
      ros::NodeHandle(controller_nh, "global_frame_mode"));
    dyn_server_->setCallback(
      boost::bind(&HeroChassisController::dynReconfCallback, this, _1, _2));

    return true;
}

void HeroChassisController::starting(const ros::Time& time)
{
    //重置速度平滑状态
    last_smoothed_cmd_ = geometry_msgs::Twist();
    //重置里程计参数
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    v_x_smoothed_ = 0.0;
    v_y_smoothed_ = 0.0;
    w_z_smoothed_ = 0.0;
    //重置PID
    for (auto& pid : pids_) pid->reset();
}

void HeroChassisController::dynReconfCallback(hero_chassis_controller::global_modeConfig &config,uint32_t level)
{
    global_frame_mode_ = config.global_frame_mode;
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
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // 设置速度(加了滤波)
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
    geometry_msgs::Twist cmd = msg;

    //底盘/全局
    if (global_frame_mode_) {
        try {
            geometry_msgs::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(
                    base_frame_,   // 子坐标系
                    odom_frame_,   // 父坐标系
                    ros::Time(0),
                    ros::Duration(0.1));

            // 取出旋转部分，转成 tf2::Quaternion
            tf2::Quaternion q;
            tf2::fromMsg(transform_stamped.transform.rotation, q);

            // 用四元数旋转线速度向量（速度是自由向量，只旋转不平移）
            tf2::Vector3 v_world(cmd.linear.x, cmd.linear.y, 0.0);
            tf2::Vector3 v_body = tf2::quatRotate(q, v_world);

            cmd.linear.x = v_body.x();
            cmd.linear.y= v_body.y();
            //Yaw旋转不必转换

        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0,
                "TF2 lookup %s -> %s failed: %s, using original command",
                odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    raw_cmd_buffer_.writeFromNonRT(cmd);

}

void HeroChassisController::applyVelocityLimits(geometry_msgs::Twist& cmd,const ros::Duration& period)
{
    //记录时间间隔
    const double dt = period.toSec();
    if (dt <= 0.0) return;

    //线速度:vx,vy合成为合成增量
    const double vx_cur = last_smoothed_cmd_.linear.x;
    const double vy_cur = last_smoothed_cmd_.linear.y;
    const double vx_inc  = cmd.linear.x - vx_cur;//计算增量
    const double vy_inc  = cmd.linear.y - vy_cur;

    const double linear_inc_mag = std::hypot(vx_inc, vy_inc);//计算目标直线增量的模
    const double linear_cur_mag = std::hypot(vx_cur, vy_cur);//计算当前速度向量的模
    const double linear_tgt_mag = std::hypot(cmd.linear.x, cmd.linear.y);//目标速度向量的模

    //判断加/减速：看合成速度幅值是变大还是变小
    const bool linear_acc = linear_tgt_mag >= linear_cur_mag;
    const double max_linear_inc_x = (linear_acc ? max_lin_acc_x_ : max_lin_dec_x_)*dt;//计算最大速度增量
    const double max_linear_inc_y = (linear_acc ? max_lin_acc_y_ : max_lin_dec_y_)*dt;
    const double max_linear_inc = std::hypot(max_linear_inc_x, max_linear_inc_y);//最大增量的模

    //角速度：(同理)
    const double wz_cur = last_smoothed_cmd_.angular.z;
    const double wz_inc  = cmd.angular.z - wz_cur;
    const bool wz_acc = (std::abs(cmd.angular.z) >= std::abs(wz_cur));
    const double max_wz_inc = (wz_acc ? max_ang_acc_z_ : max_ang_dec_z_)*dt;

    //联合约束，保证前进方向
    double allowed_linear_inc = max_linear_inc;
    double allowed_wz_inc = max_wz_inc;

    if (linear_inc_mag > 1e-9 && std::abs(wz_inc)>1e-9)
    {
        const double MA = std::hypot(linear_inc_mag,std::abs(wz_inc));
        const double MB = std::hypot(max_linear_inc,std::abs(wz_inc));
        const double Av = linear_inc_mag/MA;
        const double Aw = std::abs(wz_inc)/MA;
        const double Bv = max_linear_inc/MB;
        const double Bw = max_wz_inc/MB;
        const double theta = std::atan2(Bw,Bv) - std::atan2(Aw,Av);

        if (theta < 0.0)
        {
            //角速度方向更陡，压缩合成直线增量，保持 vx:vy 比例
            allowed_linear_inc = max_wz_inc*linear_inc_mag/std::abs(wz_inc);
        }
        else
        {
            //直线方向更陡，压缩角速度增量
            allowed_wz_inc = max_linear_inc*std::abs(wz_inc)/linear_inc_mag;
        }
    }

    double linear_scale = 1.0;
    if (linear_inc_mag > 1e-9 && linear_inc_mag > allowed_linear_inc)
    {
        linear_scale = allowed_linear_inc/linear_inc_mag;
    }
    cmd.linear.x = vx_cur + vx_inc * linear_scale;
    cmd.linear.y = vy_cur + vy_inc * linear_scale;

    if (std::abs(wz_inc) > allowed_wz_inc ) {
        cmd.angular.z = wz_cur + (wz_inc > 0 ? 1.0 : -1.0 ) * allowed_wz_inc;
    }
    else {
        cmd.angular.z = wz_cur + wz_inc ;
    }

    last_smoothed_cmd_ = cmd;//更新速度平滑后的状态

}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
    if (joints_.empty()) return;

    //加速度限制
    geometry_msgs::Twist cmd = *raw_cmd_buffer_.readFromRT();
    applyVelocityLimits(cmd, period);

    //逆运动学解算:车速->轮速
    double vel_x = cmd.linear.x;
    double vel_y = cmd.linear.y;
    double vel_rad  = cmd.angular.z;

    double fl_vel = (vel_x - vel_y -(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;
    double fr_vel = (vel_x + vel_y +(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;
    double bl_vel = (vel_x + vel_y -(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;
    double br_vel = (vel_x - vel_y +(wheel_base_+wheel_track_)*vel_rad)/wheel_radius_;

    //ROS_INFO("轮距：%f,轴距：%f,轮半径：%f",wheel_track_,wheel_base_,wheel_radius_);
    ROS_INFO("cmd_vel: linear:[%f, %f], angular:%f", vel_x, vel_y, vel_rad);
    ROS_INFO("fl:%f, fr:%f, bl:%f, br:%f",fl_vel*wheel_radius_,fr_vel*wheel_radius_,bl_vel*wheel_radius_,br_vel*wheel_radius_);


    commands_[0]= fl_vel;//转动关节为角速度，要换算为线速度
    commands_[1]= fr_vel;//转动关节为角速度，要换算为线速度
    commands_[2]= bl_vel;//转动关节为角速度，要换算为线速度
    commands_[3]= br_vel;//转动关节为角速度，要换算为线速度

    //PID
    std::vector<double> commanded_efforts;
    for (int i = 0; i < joints_.size(); i++)
    {
        const double error = commands_[i] - joints_[i].getVelocity();
        double commanded_effort = pids_[i]->computeCommand(error, period);
        if (std::abs(commands_[i]) < 1e-4 && std::abs(joints_[i].getVelocity()) < 0.5) {
            pids_[i]->reset();
            commanded_effort = 0.0;
        }
        commanded_efforts.push_back(commanded_effort);
    }

    //发布命令
    for (int i = 0; i < joints_.size(); i++)
    {
        joints_[i].setCommand(commanded_efforts[i]);
        // ROS_INFO("关节：%s cmd=%f effort=%f vel=%f ",joints_[i].getName().c_str(),
        //      joints_[i].getCommand(),
        //      joints_[i].getEffort(),
        //      joints_[i].getVelocity());
    }
    //正运动学解算+odom发布
    odom_tf(time,period);
}


PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}
