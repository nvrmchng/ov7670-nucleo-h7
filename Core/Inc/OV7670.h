#ifndef __OV7670_H
#define __OV7670_H

#include "main.h"
//#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;

enum IMG_SENSOR_STATE {
    IMG_SENSOR_STOP = 0, IMG_SENSOR_INIT = 1, IMG_SENSOR_WAIT_VSYNC = 2, IMG_SENSOR_VSYNC = 3, IMG_SENSOR_WAIT_HREF = 4, IMG_SENSOR_HREF = 5,
	IMG_SENSOR_PCLK = 6, IMG_SENSOR_COMPLETE = 7, IMG_SENSOR_RESTART = 8
};

extern enum IMG_SENSOR_STATE OV7670_State;
extern uint16_t vs_count, hs_count, pclk_count;

void OV7670_Init_Pixel_Array(void);
bool OV7670_Init_Setting(void);
void OV7670_Sync_Output_Detection(TIM_HandleTypeDef *htim);
void OV7670_Grab_Pixel_Byte_Output(void);
uint8_t* OV7670_Get_Pix_Array_By_Row(uint16_t row);

#endif	/* __OV7670_H */
