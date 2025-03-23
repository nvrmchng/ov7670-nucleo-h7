#include "OV7670.h"

enum IMG_SENSOR_STATE OV7670_State = IMG_SENSOR_STOP;
uint16_t vs_count = 0, hs_count = 0, pclk_count = 0;
__attribute__((section(".Img_Frame_Buffer"))) uint8_t pix_array[240][640];

void OV7670_Init_Pixel_Array(void)
{
	uint16_t i, j;
	for(i = 0; i < 240; i++){
		for(j = 0; j < 640; j++){
			pix_array[i][j] = 0x43;	// set all to black
		}
	}
}

bool OV7670_Init_Setting(void)
{
	uint16_t sccb_ip_address = 0x0042;	// 0x42
	uint8_t sccb_sub_address = 0x0A;	// PID register
	uint8_t sccb_read_data = 0;
	uint8_t sccb_write_data[2] = {0, 0};
	uint32_t error_stat = HAL_I2C_ERROR_NONE;

/** Register PID (0x0A) **/
	if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, &sccb_sub_address, 1, 5) != HAL_OK)		/* 2-Phase Write */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(sccb_read_data != 0x76){	return false;	}	// Verify the PID is correct = 0x76

/** Register CLKRC (0x11) **/
	sccb_sub_address = 0x11;
	sccb_write_data[0] = sccb_sub_address;
	sccb_write_data[1] = 0x9D;		// Set internal clock prescaler to 29 so we'd expect PCLK = XCLK / (29 + 1) = 15 MHz / 30 = 0.5 MHz

	if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(sccb_read_data != 0x9D){	return false;	}	// Verify that CLKRC register has been written to 0x9D

/** Register COM7 (0x12) **/
	sccb_sub_address = 0x12;
	sccb_write_data[0] = sccb_sub_address;
	sccb_write_data[1] = 0x14;		// Use pre-defined QVGA resolution scaler setting. Output format RGB selection.

	if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(sccb_read_data != 0x14){	return false;	}	// Verify that COM7 register has been written to 0x14

/** Register COM15 (0x40) **/
	sccb_sub_address = 0x40;
	sccb_write_data[0] = sccb_sub_address;
	sccb_write_data[1] = 0xD0;		// Output range: [00] to [FF]. RGB565 output format option.

	if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(sccb_read_data != 0xD0){	return false;	}	// Verify that COM15 register has been written to 0xD0

	//OV7670_Init_Pixel_Array();
	return true;
}

void OV7670_Sync_Output_Detection(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){// VSYNC
		if(OV7670_State == IMG_SENSOR_WAIT_VSYNC)
		{	OV7670_State = IMG_SENSOR_VSYNC;	}
		else if((OV7670_State > IMG_SENSOR_WAIT_VSYNC) && (OV7670_State < IMG_SENSOR_COMPLETE))
		{	OV7670_State = IMG_SENSOR_COMPLETE;	}
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){// HREF
		if(OV7670_State == IMG_SENSOR_WAIT_HREF)
		{	OV7670_State = IMG_SENSOR_HREF;	}
	}
}

void OV7670_Grab_Pixel_Byte_Output(void)
{
	uint8_t byte;

	byte = (LL_GPIO_IsInputPinSet(D0_GPIO_Port, D0_Pin) ? 0x01 : 0x00) |
			(LL_GPIO_IsInputPinSet(D1_GPIO_Port, D1_Pin) ? 0x02 : 0x00) |
			(LL_GPIO_IsInputPinSet(D2_GPIO_Port, D2_Pin) ? 0x04 : 0x00) |
			(LL_GPIO_IsInputPinSet(D3_GPIO_Port, D3_Pin) ? 0x08 : 0x00) |
			(LL_GPIO_IsInputPinSet(D4_GPIO_Port, D4_Pin) ? 0x10 : 0x00) |
			(LL_GPIO_IsInputPinSet(D5_GPIO_Port, D5_Pin) ? 0x20 : 0x00) |
			(LL_GPIO_IsInputPinSet(D6_GPIO_Port, D6_Pin) ? 0x40 : 0x00) |
			(LL_GPIO_IsInputPinSet(D7_GPIO_Port, D7_Pin) ? 0x80 : 0x00);

	pix_array[hs_count][pclk_count] = byte;
}

uint8_t* OV7670_Get_Pix_Array_By_Row(uint16_t row)
{
	return (&pix_array[row][0]);
}
