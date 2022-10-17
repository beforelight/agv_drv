#pragma once
#include "tim.h"
#include "upload_pc.h"
#include "upload_ros.h"
#include <math.h>
#include <stdbool.h>
/*
 * 索引地址——电机位置——M引脚编号——编码器定时器编号——TIM1的PWM通道
 * 0      —— 右前  —— 3      —— 2            —— 4
 * 1      —— 左前  —— 4      —— 3            —— 3
 * 2      —— 左后  —— 1      —— 4            —— 2
 * 3      —— 右后  —— 2      —— 5            —— 1
 */
#define CAR_SPEED_REVERSE_MASK (0b0000) // 交换编码线的话请设置相应索引地址的掩码
#define CAR_U_OUT_REVERSE_MASK (0b0000) // 交换电机线的话请设置相应索引地址的掩码

typedef struct __pid {
  float ref; // 目标
  float fb;  // 反馈
  float u;   // 输出
  float kp;
  float ki;
  float kd;

  // 运行中间变量
  float err0;
  float err1;
  float err2;

  // 以下变量不允许直接修改
  float _Ts;
  float _Ts_inver;
  float _u_max;
  float _u_min;
} pid_t; // pid类型，并行结构，增量式，微分项无滤波

//初始化PID控制器
void pid_init(pid_t *s, float T, float u_max, float u_min);
//更新一次PID控制
void pid_update(pid_t *s);

typedef struct __car {
  float speed[4]; // m/s
  float speed_dst[4];
  float u_out[4]; // 驱动器输出的电压值，标幺值，-1代表倒转满占空比，1代表正转满占空比
  pid_t spd_pid[4];
  bool spd_ctrl_identify_mode; // 为true时开启离速度环辨识

  Upload_Data Send_Data; // ros节点通讯数据
  Upload_Data Recive_Data;
  volatile uint32_t Recive_Timer; //	单位ms，计时，当ROS节点失联后放弃执行速度命令
  Frame_t upload_frame;
  float vbus;
  bool vbus_warning; // 低电压警告
  int upload_flag;   // 上传变量 0不上传 1上传当前的速度 和控制量
} car_t;

extern car_t car;

void car_init(int car_ctrl_period_ms);
void car_speed_get();
void car_speed_ctrl();

void car_upload_to_ros_node();

void Car_Kinematics_Positive(float vx, float vy, float vth);

void Car_Update_Pid_Param(float kp, float ki, float kd);
