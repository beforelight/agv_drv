#include "drv_car.h"
#include "drv_imu_instance.h"
#include "rng.h"
#include "usart.h"
#include <string.h>

#define Pi_v                    3.14159f
#define ENCODER_TTL_COUNT_VALUE 1300.0f //轮子每圈的脉冲数
#define ROBOT_WHEEL_DIAMETER    0.085f  //主动轮直径
#define ROBOT_WHEEL_WIDTH       0.185f  //机器人水平轮距,宽度
#define ROBOT_WHEEL_LENGTH      0.175f  //机器人垂直轮距,长度

#define CURRENT_MOTO_SPEED(X) ((ROBOT_WHEEL_DIAMETER * Pi_v * (X / ENCODER_TTL_COUNT_VALUE)) / CONTROL_TIMER_CYCLE) //轮子的运动速度m/s

car_t car;
float Ts = 0.005;
float Ts_inver = 200;

//这里的pid参数由Matlab计算出来
#define default_kp (1.5f)
#define default_ki (25.4f)
#define default_kd (0.00358f)

void update_speed(float *spd, TIM_HandleTypeDef *htim)
{
  int32_t cnt = *((int16_t *) &__HAL_TIM_GET_COUNTER(htim));
  __HAL_TIM_GET_COUNTER(htim) = 0;
  if (cnt > 0 && __HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
    // cnt -= 0x1 << 16;
    cnt = 0;
  } else if (cnt < 0 && (!__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))) {
    // cnt += 0x1 << 16;
    cnt = 0;
  }
  *spd = ROBOT_WHEEL_DIAMETER * Pi_v * (float) cnt * (1.0f / ENCODER_TTL_COUNT_VALUE) * Ts_inver;
}

void pid_init(pid_t *s, float T, float u_max, float u_min)
{
  s->_Ts = T;
  s->_Ts_inver = 1.0f / T;
  s->_u_max = u_max;
  s->_u_min = u_min;

  s->err0 = 0;
  s->err1 = 0;
  s->err2 = 0;
}

void pid_update(pid_t *s)
{
  //更新控制
  s->err0 = s->ref - s->fb;
  float err_diff01 = s->err0 - s->err1;
  float err_diff12 = s->err1 - s->err2;
  float delta_u = s->kp * err_diff01
                  + s->ki * s->_Ts * s->err0
                  + s->kd * s->_Ts_inver * (err_diff01 - err_diff12);
  s->u += delta_u;
  //差分
  s->err2 = s->err1;
  s->err1 = s->err0;
  //限幅
  if (s->u > s->_u_max) {
    s->u = s->_u_max;
  }
  if (s->u < s->_u_min) {
    s->u = s->_u_min;
  }
}

void car_init(int car_ctrl_period_ms)
{
  memset(&car, 0, sizeof(car_t)); //清0
  Ts = 0.001 * car_ctrl_period_ms;
  Ts_inver = 1 / Ts;
  for (int i = 0; i < 4; i++) {
    pid_init(&car.spd_pid[i], Ts, 1, -1);
    car.spd_pid[i].kp = default_kp;
    car.spd_pid[i].ki = default_ki;
    car.spd_pid[i].kd = default_kd;
  }
  FRAME_Init(&car.upload_frame);
}

void car_speed_get()
{
  update_speed(&car.speed[3 - 1], &htim2);
  update_speed(&car.speed[4 - 1], &htim3);
  update_speed(&car.speed[1 - 1], &htim4);
  update_speed(&car.speed[2 - 1], &htim5);
#define __FBINVERT(idx) car.speed[idx] = -car.speed[idx]
#if MOTOR_LF_SPD_FB_INVERT
  __FBINVERT(MOTOR_LF_IDX);
#endif
#if MOTOR_RF_SPD_FB_INVERT
  __FBINVERT(MOTOR_RF_IDX);
#endif
#if MOTOR_LB_SPD_FB_INVERT
  __FBINVERT(MOTOR_LB_IDX);
#endif
#if MOTOR_RB_SPD_FB_INVERT
  __FBINVERT(MOTOR_RB_IDX);
#endif
}

void car_speed_ctrl()
{
  //计算控制量
  if (!car.spd_ctrl_identify_mode) {
    //速度目标方向校正，并且缓和速度目标的变化
#define __GET_PID_REF(idx, invert) car.spd_pid[idx].ref = 0.9f * car.spd_pid[idx].ref + 0.1f * (invert ? (-car.speed_dst[idx]) : (car.speed_dst[idx]))
    __GET_PID_REF(MOTOR_LF_IDX, MOTOR_LF_SPD_REF_INVERT);
    __GET_PID_REF(MOTOR_RF_IDX, MOTOR_RF_SPD_REF_INVERT);
    __GET_PID_REF(MOTOR_LB_IDX, MOTOR_LB_SPD_REF_INVERT);
    __GET_PID_REF(MOTOR_RB_IDX, MOTOR_RB_SPD_REF_INVERT);
    for (int i = 0; i < 4; i++) {
      car.spd_pid[i].fb = car.speed[i];
      pid_update(&car.spd_pid[i]);
      car.u_out[i] = car.spd_pid[i].u;
    }
  } else {
    //使用随机信号作为辨识输入
    uint32_t urnd = HAL_RNG_GetRandomNumber(&hrng);
    int16_t *rnd = (int16_t *) &urnd;
    float frnd = (1.0 / 32768.0) * ((float) (*rnd));
    if (frnd > 0.9) frnd = 0.9;
    if (frnd < -0.9) frnd = -0.9;
    static float frnd_filter = 0;
    frnd_filter = 0.3 * frnd + 0.7 * frnd_filter;
    car.u_out[0] = frnd_filter;
    car.u_out[1] = frnd_filter;
    car.u_out[2] = frnd_filter;
    car.u_out[3] = frnd_filter;
  }

  //根据控制量输出
  car.u_out[0] < 0 ? (HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET))
                   : (HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_RESET));
  car.u_out[1] < 0 ? (HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET))
                   : (HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_RESET));
  car.u_out[2] < 0 ? (HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET))
                   : (HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_RESET));
  car.u_out[3] < 0 ? (HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET))
                   : (HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_SET), HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_RESET));

  htim1.Instance->CCR2 = fabsf(car.u_out[1 - 1]) * (float) htim1.Instance->ARR;
  htim1.Instance->CCR1 = fabsf(car.u_out[2 - 1]) * (float) htim1.Instance->ARR;
  htim1.Instance->CCR4 = fabsf(car.u_out[3 - 1]) * (float) htim1.Instance->ARR;
  htim1.Instance->CCR3 = fabsf(car.u_out[4 - 1]) * (float) htim1.Instance->ARR;
}

void car_upload_to_ros_node()
{
  unsigned char i = 0;
  car.Send_Data.Sensor_Str.Header = PROTOCOL_HEADER;
  car.Send_Data.Sensor_Str.End_flag = PROTOCOL_END;

  car.Send_Data.Sensor_Str.X_speed = (car.speed[0] + car.speed[1] + car.speed[2] + car.speed[3]) / 4;
  car.Send_Data.Sensor_Str.Y_speed = (car.speed[3] - car.speed[2] + car.speed[1] - car.speed[0]) / 4;
  car.Send_Data.Sensor_Str.Z_speed = (car.speed[3] - car.speed[2] - car.speed[1] + car.speed[0]) / (4 * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_LENGTH) * 0.5f);

  car.Send_Data.Sensor_Str.Source_Voltage = car.vbus;

  car.Send_Data.Sensor_Str.MotoStr[0].Moto_CurrentSpeed = car.speed[0];
  car.Send_Data.Sensor_Str.MotoStr[0].Moto_TargetSpeed = car.speed_dst[0];

  car.Send_Data.Sensor_Str.MotoStr[1].Moto_CurrentSpeed = car.speed[1];
  car.Send_Data.Sensor_Str.MotoStr[1].Moto_TargetSpeed = car.speed_dst[1];

  car.Send_Data.Sensor_Str.MotoStr[2].Moto_CurrentSpeed = car.speed[2];
  car.Send_Data.Sensor_Str.MotoStr[2].Moto_TargetSpeed = car.speed_dst[2];

  car.Send_Data.Sensor_Str.MotoStr[3].Moto_CurrentSpeed = car.speed[3];
  car.Send_Data.Sensor_Str.MotoStr[3].Moto_TargetSpeed = car.speed_dst[3];
  //返回的速度也需要反向
#define __CURINVERT(idx) car.Send_Data.Sensor_Str.MotoStr[idx].Moto_CurrentSpeed = -car.Send_Data.Sensor_Str.MotoStr[idx].Moto_CurrentSpeed
#if MOTOR_LF_SPD_REF_INVERT
  __CURINVERT(MOTOR_LF_IDX);
#endif
#if MOTOR_RF_SPD_REF_INVERT
  __CURINVERT(MOTOR_RF_IDX);
#endif
#if MOTOR_LB_SPD_REF_INVERT
  __CURINVERT(MOTOR_LB_IDX);
#endif
#if MOTOR_RB_SPD_REF_INVERT
  __CURINVERT(MOTOR_RB_IDX);
#endif

  if (imu0 != 0 && IMU_IsOpen(imu0)) {
    int16_t buf[9];
    IMU_ConvertRaw(imu0, buf);
    car.Send_Data.Sensor_Str.Link_Accelerometer.X_data = buf[0];
    car.Send_Data.Sensor_Str.Link_Accelerometer.Y_data = buf[1];
    car.Send_Data.Sensor_Str.Link_Accelerometer.Z_data = buf[2];
    car.Send_Data.Sensor_Str.Link_Gyroscope.X_data = buf[0 + 3];
    car.Send_Data.Sensor_Str.Link_Gyroscope.Y_data = buf[1 + 3];
    car.Send_Data.Sensor_Str.Link_Gyroscope.Z_data = buf[2 + 3];
  } else {
    car.Send_Data.Sensor_Str.Link_Accelerometer.X_data = 0;
    car.Send_Data.Sensor_Str.Link_Accelerometer.Y_data = 0;
    car.Send_Data.Sensor_Str.Link_Accelerometer.Z_data = 0;
    car.Send_Data.Sensor_Str.Link_Gyroscope.X_data = 0;
    car.Send_Data.Sensor_Str.Link_Gyroscope.Y_data = 0;
    car.Send_Data.Sensor_Str.Link_Gyroscope.Z_data = 0;
  }

  HAL_UART_Transmit_DMA(&huart2, car.Send_Data.buffer, sizeof(Upload_Data));
  // HAL_UART_Transmit(&huart2, car.Send_Data.buffer, sizeof(Upload_Data), ~0);
}

void Car_Kinematics_Positive(float vx, float vy, float vth)
{
  car.speed_dst[0] = vx - vy + vth * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_LENGTH) * 0.5f;
  car.speed_dst[1] = vx + vy - vth * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_LENGTH) * 0.5f;
  car.speed_dst[2] = vx - vy - vth * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_LENGTH) * 0.5f;
  car.speed_dst[3] = vx + vy + vth * (ROBOT_WHEEL_WIDTH + ROBOT_WHEEL_LENGTH) * 0.5f;
}

void Car_Update_Pid_Param(float kp, float ki, float kd)
{
  // ros节点传出的默认pid参数为190.0，147.0，54.0，50ms周期，并行式，不考虑控制周期的影响
  for (int i = 0; i < 4; i++) {
    car.spd_pid[i].kp = kp * (default_kp / 190.0f);
    car.spd_pid[i].ki = ki * (default_ki / 147.0f);
    car.spd_pid[i].kd = kd * (default_kd / 54.0f);
  }
}
