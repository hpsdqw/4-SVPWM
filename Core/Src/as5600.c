#include "as5600.h"

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f

static float angle_data_prev1; //上次位置1
static float full_rotation_offset1; //转过的整圈数1
static float angle_data_prev2; //上次位置2
static float full_rotation_offset2; //转过的整圈数2
static volatile int as5600_1_State = 0;
static volatile int as5600_2_State = 0;

static volatile uint16_t raw_angle_1;
static volatile uint16_t raw_angle_2;
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == &hi2c1)
	{
		as5600_1_State = 2;
	}
	if (hi2c == &hi2c2)
	{
		as5600_2_State = 2;
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == &hi2c1)
	{
		as5600_1_State = 4;
	}
	if (hi2c == &hi2c2)
	{
		as5600_2_State = 4;
	}
}

void bsp_as5600_1_Init(void) {
  /* init i2c interface */
  
  /* init var */
  full_rotation_offset1 = 0;
  angle_data_prev1 = bsp_as5600_1_GetRawAngle();
  HAL_I2C_Master_Transmit_DMA(&AS5600_I2C1_HANDLE, AS5600_ADDR, 0, 1) ;
  HAL_I2C_Master_Transmit_DMA(&AS5600_I2C1_HANDLE, AS5600_ADDR | 1, 0, 1) ;	
}

static int i2c1Write(uint8_t dev_addr,uint8_t *pData, uint32_t count) {
  int status;
  
  status = HAL_I2C_Master_Transmit_DMA(&AS5600_I2C1_HANDLE, dev_addr, pData, count);
  return status;
}

static int i2c1Read(uint8_t dev_addr,uint8_t *pData, uint32_t count) {
  int status;
  
  status = HAL_I2C_Master_Receive_DMA(&AS5600_I2C1_HANDLE, (dev_addr | 1), pData, count);
  return status;
}

uint16_t bsp_as5600_1_GetRawAngle(void) {
  static volatile uint8_t buffer[2];
  static volatile uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
  //简易状态机DMA读取as5600
  switch (as5600_1_State)
  {
	  case 0 :
	  {
		i2c1Write(AS5600_ADDR, (uint8_t *)&raw_angle_register, 1);
	    as5600_1_State = 1;
		break;
	  }
	  
	  case 1:
	  {
		  
		break;  
	  }
	  
	  case 2 :
	  {
		i2c1Read(AS5600_ADDR, (uint8_t *)buffer, 2);
		as5600_1_State = 3;
		break;
	  }
	  
	  case 3:
	  {
		  
		break;
	  }
	  
	  case 4 :
	  {
		raw_angle_1 = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
		as5600_1_State = 0;
		break;
	  }
	  default: // 添加默认行为，避免状态卡死
      {
        as5600_1_State = 0;
        break;
	  }
  }
//  i2c1Write(AS5600_ADDR, &raw_angle_register, 1);
//  i2c1Read(AS5600_ADDR, buffer, 2);
//  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle_1;
}

float bsp_as5600_1_GetAngle(void) {
  float angle_data = bsp_as5600_1_GetRawAngle();
  
  float d_angle = angle_data - angle_data_prev1;
  if(abs(d_angle) > (0.8 * AS5600_RESOLUTION)) {
    full_rotation_offset1 += (d_angle > 0 ? -_2PI : _2PI);
  }
  angle_data_prev1 = angle_data;
  
  return (full_rotation_offset1 + (angle_data / (float)AS5600_RESOLUTION)*_2PI);
}

void bsp_as5600_2_Init(void) {
  /* init i2c interface */
  
  /* init var */
  full_rotation_offset2 = 0;
  angle_data_prev2 = bsp_as5600_2_GetRawAngle();
  HAL_I2C_Master_Transmit_DMA(&AS5600_I2C2_HANDLE, AS5600_ADDR, 0, 1) ;
  HAL_I2C_Master_Transmit_DMA(&AS5600_I2C2_HANDLE, AS5600_ADDR | 1, 0, 1) ;	
}

static int i2c2Write(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  
  status = HAL_I2C_Master_Transmit_DMA(&AS5600_I2C2_HANDLE, dev_addr, pData, count);
  return status;
}

static int i2c2Read(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  
  status = HAL_I2C_Master_Receive_DMA(&AS5600_I2C2_HANDLE, (dev_addr | 1), pData, count);
  return status;
}

uint16_t bsp_as5600_2_GetRawAngle(void) {
  static volatile uint8_t buffer[2] = {0};
  static volatile uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
  //简易状态机DMA读取as5600
  switch (as5600_2_State)
  {
	  case 0 :
	  {
		i2c2Write(AS5600_ADDR, (uint8_t *)&raw_angle_register, 1);
	    as5600_2_State = 1;
		break;
	  }
	  
	  case 1:
	  {
		  
		break;  
	  }
	  
	  case 2 :
	  {
		i2c2Read(AS5600_ADDR, (uint8_t *)buffer, 2);
		as5600_2_State = 3;
		break;
	  }
	  
	  case 3:
	  {
		  
		break;  
	  }
	  
	  case 4 :
	  {
		
		raw_angle_2 = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
		as5600_2_State = 0;
		break;
	  }
	  default: // 添加默认行为，避免状态卡死
      {
        as5600_2_State = 0;
        break;
	  }
  }
  
//  i2c2Write(AS5600_ADDR, &raw_angle_register, 1);
//  i2c2Read(AS5600_ADDR, buffer, 2);
//  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle_2;
}

float bsp_as5600_2_GetAngle(void) {
  float angle_data = bsp_as5600_2_GetRawAngle();
  
  float d_angle = angle_data - angle_data_prev2;
  if(abs(d_angle) > (0.8 * AS5600_RESOLUTION)) {
    full_rotation_offset2 += (d_angle > 0 ? -_2PI : _2PI);
  }
  angle_data_prev2 = angle_data;
  
  return (full_rotation_offset2 + (angle_data / (float)AS5600_RESOLUTION)*_2PI);
}
