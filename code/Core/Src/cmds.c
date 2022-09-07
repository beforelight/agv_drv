#include "drv_beep.h"
#include "drv_car.h"
#include "drv_shell.h"
int cmd_beep(int tick)
{
  beep_on(tick);
  return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), beep, cmd_beep, 蜂鸣器响);

int cmd_led(int tick)
{
  led_on(tick);
  return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), led, led_on, LED响);

int cmd_speed(int cnt, char **argv)
{
  myPrintf("spd %f,%f,%f,%f\r\n", car.speed[0], car.speed[1], car.speed[2], car.speed[3]);
  return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), speed, cmd_speed, 速度显示);

int cmd_identify(int cnt, char **argv)
{
  car.spd_ctrl_identify_mode = !car.spd_ctrl_identify_mode;
  myPrintf("已切换辨识模式为%s", car.spd_ctrl_identify_mode ? "true" : "false");
  return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), identify, cmd_identify, 切换辨识模式);

int upload_switch(int mode)
{
  car.upload_flag = mode;
  return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), upload_switch, upload_switch, 修改上传的变量);
