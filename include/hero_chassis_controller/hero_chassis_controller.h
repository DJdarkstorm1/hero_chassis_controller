//
// Created by dj on 2026/10/30.
//

#ifndef SRC_HERO_CHASSIS_CONTROLLER_H
#define SRC_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointControllerState.h>

namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        hardware_interface::JointHandle front_left_joint_,
                                        front_right_joint_,
                                        back_left_joint_,
                                        back_right_joint_;

        double command_; //最新速度命令
    private:
        //pid接口,参考joint_velocity_controller.h
        int loop_count_;

        control_toolbox::Pid fl_pid_, fr_pid_, bl_pid_, br_pid_;

        std::unique_ptr<
                        realtime_tools::RealtimePublisher<
                        control_msgs::JointControllerState> > fl_controller_state_publisher_,
                                                              fr_controller_state_publisher_,
                                                              bl_controller_state_publisher_,
                                                              br_controller_state_publisher_;
        ros::Subscriber sub_command_;


    };
}// namespace hero_chassis_controller

#endif //SRC_HERO_CHASSIS_CONTROLLER_H