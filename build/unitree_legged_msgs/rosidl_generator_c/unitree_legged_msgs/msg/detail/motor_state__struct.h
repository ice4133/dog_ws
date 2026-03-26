// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_legged_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorState in the package unitree_legged_msgs.
typedef struct unitree_legged_msgs__msg__MotorState
{
  /// 电机模式
  uint8_t mode;
  /// 估计力矩
  double tau_est;
  /// 位置
  double q;
  /// 速度（rad/s）
  double dq;
  double kp;
  double kd;
} unitree_legged_msgs__msg__MotorState;

// Struct for a sequence of unitree_legged_msgs__msg__MotorState.
typedef struct unitree_legged_msgs__msg__MotorState__Sequence
{
  unitree_legged_msgs__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_legged_msgs__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
