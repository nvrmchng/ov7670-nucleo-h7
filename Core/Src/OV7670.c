#include "OV7670.h"

enum IMG_SENSOR_STATE OV7670_State = IMG_SENSOR_STOP;
uint16_t vs_count = 0, hs_count = 0, pclk_count = 0;
__attribute__((section(".Img_Frame_Buffer"))) uint8_t pix_array[240][640];

void OV7670_Init_Pixel_Array(void)
{
	uint16_t i, j;
	for(i = 0; i < 240; i++){
		for(j = 0; j < 640; j++){
			pix_array[i][j] = 0x00;	// set all to black
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

/** Register DBLV (0x6B) **/
	//sccb_sub_address = 0x6B;
	//sccb_write_data[0] = sccb_sub_address;
	//sccb_write_data[1] = 0x4A;		// PLL will multiply input clock by x4
	
	//if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(sccb_read_data != 0x4A){	return false;	}	// Verify that DBLV register has been written to 0x4A

/** Register CLKRC (0x11) **/
	sccb_sub_address = 0x11;
	sccb_write_data[0] = sccb_sub_address;
	sccb_write_data[1] = 0x97;	// Set internal clock prescaler to 59 so we'd expect PCLK = XCLK x PLL / 2*(59 + 1) = 15 MHz x 4 / (2x60) = 0.5 MHz

	if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(sccb_read_data != 0x97){	return false;	}	// Verify that CLKRC register has been written to 0x9D

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
	
/** Register COM11 (0x3B) **/
	//sccb_sub_address = 0x3B;
	//sccb_write_data[0] = sccb_sub_address;
	//sccb_write_data[1] = 0x08;		// Select 50 Hz as Banding Filter value (because Indonesia AC signal is 50 Hz based)
	
	//if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(sccb_read_data != 0x08){	return false;	}	// Verify that COM11 register has been written to 0x08
	
/** Register COM8 (0x13) **/
	//sccb_sub_address = 0x13;
	//sccb_write_data[0] = sccb_sub_address;
	//sccb_write_data[1] = 0xAF;		// Turn On the Banding Filter function
	
	//if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(sccb_read_data != 0xAF){	return false;	}	// Verify that COM8 register has been written to 0xAF
	
/** Register COM14 (0x3E) **/
	//sccb_sub_address = 0x3E;
	//sccb_write_data[0] = sccb_sub_address;
	//sccb_write_data[1] = 0x1C;		// Scaling PCLK, manual scaling enable for predefined res, PCLK divider set to divide by 16
	
	//if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(sccb_read_data != 0x1C){	return false;	}	// Verify that COM14 register has been written to 0x1C
	
/** Register SCALING_PCLK_DIV (0x73) **/
	//sccb_sub_address = 0x73;
	//sccb_write_data[0] = sccb_sub_address;
	//sccb_write_data[1] = 0xF4;		// Enable clock divider, PCLK divider set to divide by 16
	
	//if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	//{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	//Delay_us_Rough(100);
	//if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	//if(sccb_read_data != 0xF4){	return false;	}	// Verify that SCALING_PCLK_DIV register has been written to 0xF4
	
/** Register COM16 (0x41) **/
	sccb_sub_address = 0x41;
	sccb_write_data[0] = sccb_sub_address;
	sccb_write_data[1] = 0x18;		// De-noise function to reduce noise level is set to automatic mode
	
	if(HAL_I2C_Master_Transmit(&hi2c1, sccb_ip_address, sccb_write_data, 2, 5) != HAL_OK)		/* 3-Phase Write */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(HAL_I2C_Master_Receive(&hi2c1, (sccb_ip_address | 0x0001), &sccb_read_data, 1, 5) != HAL_OK)		/* 2-Phase Read */
	{	error_stat = HAL_I2C_GetError(&hi2c1);	}
	Delay_us_Rough(100);
	if(error_stat != HAL_I2C_ERROR_NONE){	return false;	}

	if(sccb_read_data != 0x18){	return false;	}	// Verify that COM16 register has been written to 0x18

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
