#include "drv_digi.h"

char idx = 0; //
char num[3];  //那三位数字

void digi_set_num(int a)
{
  int b;
  for (int i = 0; i < 3; i++) {
    b = a % 10;
    a = a / 10;
    num[i] = b;
  }
  //前导零处理
  for (int i = 2; i > 0; i--) {
    if (num[i] == 0) {
      num[i] = 0xff;
    } else {
      break;
    }
  }
  idx = 0;
  _digi_decoder(); //立刻刷新
}

void digi_set_num_no_refresh(int a)
{
  int b;
  for (int i = 0; i < 3; i++) {
    b = a % 10;
    a = a / 10;
    num[i] = b;
  }
  //前导零处理
  for (int i = 2; i > 0; i--) {
    if (num[i] == 0) {
      num[i] = 0xff;
    } else {
      break;
    }
  }
}

void digi_tick_callback()
{
  idx++;
  idx %= 3;
  _digi_decoder();
}

void _digi_decoder()
{
  HAL_GPIO_WritePin(Dig1_GPIO_Port, Dig1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Dig2_GPIO_Port, Dig2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Dig3_GPIO_Port, Dig3_Pin, GPIO_PIN_RESET);
  switch (num[idx]) {
  case 0:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_SET);
    break;
  case 1:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_SET);
    break;
  case 2:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_RESET);
    break;
  case 3:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_RESET);
    break;
  case 4:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_RESET);
    break;
  case 5:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_RESET);
    break;
  case 6:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_RESET);
    break;
  case 7:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_SET);
    break;
  case 8:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_RESET);
    break;
  case 9:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_RESET);
    break;
  default:
    HAL_GPIO_WritePin(DigA_GPIO_Port, DigA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigB_GPIO_Port, DigB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigC_GPIO_Port, DigC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigD_GPIO_Port, DigD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigE_GPIO_Port, DigE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigF_GPIO_Port, DigF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DigG_GPIO_Port, DigG_Pin, GPIO_PIN_SET);
    break;
  }
  switch (idx) {
  case 2:
    HAL_GPIO_WritePin(Dig1_GPIO_Port, Dig1_Pin, GPIO_PIN_SET);
    break;
  case 1:
    HAL_GPIO_WritePin(Dig2_GPIO_Port, Dig2_Pin, GPIO_PIN_SET);
    break;
  case 0:
  default:
    HAL_GPIO_WritePin(Dig3_GPIO_Port, Dig3_Pin, GPIO_PIN_SET);
    break;
  }
}
