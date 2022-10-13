#pragma once
#include "tim.h"
#include "upload_pc.h"
#include "upload_ros.h"
#include <math.h>
#include <stdbool.h>

//定义电机对应了原理图的Mx编号，注意c原因编号为Mx编号-1
#define MOTOR_LF_IDX (4 - 1)
#define MOTOR_RF_IDX (3 - 1)
#define MOTOR_LB_IDX (1 - 1)
#define MOTOR_RB_IDX (2 - 1)

//是否将电机速度反馈取反，以适应当编码器线和电机线交换其中一个的情况
#define MOTOR_LF_SPD_FB_INVERT (true)
#define MOTOR_RF_SPD_FB_INVERT (false)
#define MOTOR_LB_SPD_FB_INVERT (true)
#define MOTOR_RB_SPD_FB_INVERT (false)

//是否将驱动器速度目标取反，以适应编码器线和电机线同时交换的情况
#define MOTOR_LF_SPD_REF_INVERT (true)
#define MOTOR_RF_SPD_REF_INVERT (false)
#define MOTOR_LB_SPD_REF_INVERT (true)
#define MOTOR_RB_SPD_REF_INVERT (false)

typedef struct __pid {
  float ref; //目标
  float fb;  //反馈
  float u;   //输出
  float kp;
  float ki;
  float kd;

  //运行中间变量
  float err0;
  float err1;
  float err2;

  //以下变量不允许直接修改
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
  float u_out[4]; //驱动器输出的电压值，标幺值，-1代表倒转满占空比，1代表正转满占空比
  pid_t spd_pid[4];
  bool spd_ctrl_identify_mode; //为true时开启离速度环辨识

  Upload_Data Send_Data; // ros节点通讯数据
  Upload_Data Recive_Data;
  volatile uint32_t Recive_Timer; //	单位ms，计时，当ROS节点失联后放弃执行速度命令
  Frame_t upload_frame;
  float vbus;
  int upload_flag; //上传变量 0不上传 1上传当前的速度 和控制量

} car_t;

extern car_t car;

void car_init(int car_ctrl_period_ms);
void car_speed_get();
void car_speed_ctrl();

void car_upload_to_ros_node();

void Car_Kinematics_Positive(float vx, float vy, float vth);

void Car_Update_Pid_Param(float kp, float ki, float kd);
