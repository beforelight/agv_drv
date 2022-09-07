#include "drv_imu_invensense.h"

struct _fdata {
  float acc_x;
  float acc_y;
  float acc_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float mag_x;
  float mag_y;
  float mag_z;
};

typedef union _imu_data {
  float farray[9];
  struct _fdata val;
} imu_data_t;

extern imu_data_t imu_data;
extern inv_imu_handle_t imu0;
extern float imu_temp;

void imu_init();

void imu_read_callback();
