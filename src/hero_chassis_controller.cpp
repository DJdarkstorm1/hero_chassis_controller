//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>

namespace hero_chassis_controller {
bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ =
      effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

    //初始化PID
    fl_pid_.init(ros::NodeHandle(controller_nh, "fl_pid"));
    fr_pid_.init(ros::NodeHandle(controller_nh, "fr_pid"));
    bl_pid_.init(ros::NodeHandle(controller_nh, "bl_pid"));
    br_pid_.init(ros::NodeHandle(controller_nh, "br_pid"));

    return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
    double tau = 0.2;  // torque
    double fl_error = command_ - front_left_joint_.getVelocity();
    double fr_error = command_ - front_right_joint_.getVelocity();
    double bl_error = command_ - back_left_joint_.getVelocity();
    double br_error = command_ - back_right_joint_.getVelocity();

    double fl_commanded_effort = fl_pid_.computeCommand(fl_error, period);
    double fr_commanded_effort = fl_pid_.computeCommand(fr_error, period);
    double bl_commanded_effort = fl_pid_.computeCommand(bl_error, period);
    double br_commanded_effort = fl_pid_.computeCommand(br_error, period);

    //发布命令
    front_left_joint_.setCommand(fl_commanded_effort);
    front_right_joint_.setCommand(fr_commanded_effort);
    back_left_joint_.setCommand(bl_commanded_effort);
    back_right_joint_.setCommand(br_commanded_effort);

    //fl_controller_state_publisher_
    //fr_controller_state_publisher_
    //bl_controller_state_publisher_
    //br_controller_state_publisher_

  if(loop_count_ % 10 == 0)
  {
    if(fl_controller_state_publisher_ && fl_controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;

      double dummy;
      bool antiwindup;
      getGains(controller_state_publisher_->msg_.p,
        controller_state_publisher_->msg_.i,
        controller_state_publisher_->msg_.d,
        controller_state_publisher_->msg_.i_clamp,
        dummy,
        antiwindup);
      controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

}

void pid_update(const ros::Time &time, const ros::Duration &period)
{

}


PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}
