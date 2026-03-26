// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_legged_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "unitree_legged_msgs/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorState_kd
{
public:
  explicit Init_MotorState_kd(::unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::unitree_legged_msgs::msg::MotorState kd(::unitree_legged_msgs::msg::MotorState::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_kp
{
public:
  explicit Init_MotorState_kp(::unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_kd kp(::unitree_legged_msgs::msg::MotorState::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MotorState_kd(msg_);
  }

private:
  ::unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_dq
{
public:
  explicit Init_MotorState_dq(::unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_kp dq(::unitree_legged_msgs::msg::MotorState::_dq_type arg)
  {
    msg_.dq = std::move(arg);
    return Init_MotorState_kp(msg_);
  }

private:
  ::unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_q
{
public:
  explicit Init_MotorState_q(::unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_dq q(::unitree_legged_msgs::msg::MotorState::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_MotorState_dq(msg_);
  }

private:
  ::unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_tau_est
{
public:
  explicit Init_MotorState_tau_est(::unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_q tau_est(::unitree_legged_msgs::msg::MotorState::_tau_est_type arg)
  {
    msg_.tau_est = std::move(arg);
    return Init_MotorState_q(msg_);
  }

private:
  ::unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_mode
{
public:
  Init_MotorState_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_tau_est mode(::unitree_legged_msgs::msg::MotorState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorState_tau_est(msg_);
  }

private:
  ::unitree_legged_msgs::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_legged_msgs::msg::MotorState>()
{
  return unitree_legged_msgs::msg::builder::Init_MotorState_mode();
}

}  // namespace unitree_legged_msgs

#endif  // UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
