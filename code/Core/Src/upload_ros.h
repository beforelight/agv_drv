#pragma once
#define PROTOCOL_HEADER 0XFEFEFEFE
#define PROTOCOL_END    0XEE

#define PROTOCL_DATA_SIZE 77
#pragma pack(push)
#pragma pack(1)

typedef struct __Mpu6050_Str_ {
  short X_data;
  short Y_data;
  short Z_data;
} Mpu6050_Str;

typedef struct __Moto_Str_ {
  float Moto_CurrentSpeed;
  float Moto_TargetSpeed;
} Moto_Str;

// 00 00 00 80 00 00 00 80 00 00 00 00 00 00 00 80

typedef union _Upload_Data_ {
  unsigned char buffer[PROTOCL_DATA_SIZE];
  struct _Sensor_Str_ {
    unsigned int Header;  // EE FE FE FE FE
    float X_speed;        // 00 00 00 00
    float Y_speed;        // 00 00 00 00
    float Z_speed;        // 00 00 00 00
    float Source_Voltage; // D7 E3 47 41

    Mpu6050_Str Link_Accelerometer; // C4 06 F0 FD 6C 38
    Mpu6050_Str Link_Gyroscope;     // D8 FF 03 00 EB FF

    Moto_Str MotoStr[4]; //
    float PID_Param[3];  // 00 00 00 00 00 00 00 00 00 00 00 00

    unsigned char End_flag; // EE
  } Sensor_Str;
} Upload_Data;

//#pragma pack(4)
#pragma pack(pop)