#include "drv_imu_instance.h"
#include "__drv_imu_syslog.h"
#include "i2c.h"

int IMU_INV_I2C_TransferNonBlocking(const inv_i2c_transfer_t *h)
{
  if (h->direction == inv_i2c_direction_Read) {
    return HAL_I2C_Mem_Read_DMA(&hi2c1, h->slaveAddress << 1, h->subAddress, h->subAddressSize, h->data, h->dataSize);
  } else {
    return HAL_I2C_Mem_Write_DMA(&hi2c1, h->slaveAddress << 1, h->subAddress, h->subAddressSize, h->data, h->dataSize);
  }
  return -1;
}
int IMU_INV_I2C_TransferBlocking(const inv_i2c_transfer_t *h)
{
  if (h->direction == inv_i2c_direction_Read) {
    return HAL_I2C_Mem_Read(&hi2c1, h->slaveAddress << 1, h->subAddress, h->subAddressSize, h->data, h->dataSize, ~0);
  } else {
    return HAL_I2C_Mem_Write(&hi2c1, h->slaveAddress << 1, h->subAddress, h->subAddressSize, h->data, h->dataSize, ~0);
  }
  return -1;
}

inv_i2c_t i2c0 =
    {
        .masterTransferBlocking = IMU_INV_I2C_TransferBlocking,
        .masterTransferNonBlocking = IMU_INV_I2C_TransferNonBlocking,
};
imu_data_t imu_data;
inv_imu_handle_t imu0;
float imu_temp;
void imu_init()
{
  imu0 = IMU_AutoConstructI2C(i2c0, IMU_SlaveAddressAutoDetect);
  if (imu0 != NULL) {
    SYSLOG_I("当前使用的传感器为 %s", IMU_Report(imu0));

    inv_imu_config_t cfg;
    cfg.gyroFullScale = MPU_FS_2000dps;
    cfg.gyroBandwidth = MPU_GBW_10;
    cfg.gyroUnit = MPU_UNIT_DegPerSec;
    cfg.accelFullScale = MPU_FS_8G;
    cfg.accelBandwidth = MPU_ABW_10;
    cfg.accelUnit = MPU_UNIT_MetersPerSquareSecond;

    if (0 == IMU_Init(imu0, cfg)) {
      SYSLOG_I("ST = %d,0为自检通过,自检时保持静止", IMU_SelfTest(imu0));
      HAL_Delay(10); // delay 10ms
      IMU_ReadSensorBlocking(imu0);
      float temp;
      float data[9];
      float *buf = data;
      IMU_Convert(imu0, data);
      IMU_ConvertTemp(imu0, &temp);
      SYSLOG_I("先初次读取数据temp = %f", temp);
      SYSLOG_I("accel(xyz) = %.3f %.3f %.3f\t gyro(xyz) = %.3f %.3f %.3f\t mag(xyz) = %.3f %.3f %.3f \r\n",
               data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
    } else {
      SYSLOG_I("初始化失败");
    }
  } else {
    SYSLOG_I("imu0 == NULL，没接或者iic/spi读写出错");
  }
}

void imu_read_callback()
{
  if (imu0 != NULL) {
    IMU_Convert(imu0, imu_data.farray);
    IMU_ConvertTemp(imu0, &imu_temp);
  }
}
