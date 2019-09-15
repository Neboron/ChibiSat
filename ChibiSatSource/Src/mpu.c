
//================================================================//
//==                                                            ==//
//==                 mpu 6050(MPU6000 I2C MODE)                 ==//
//==                                                            ==//
//================================================================//

//INCLUDES:
#include "mpu.h"



//DEFINES:
/* Default I2C address */
#define MPU60XX_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU60XX_I_AM				0x68

/* MPU6050 registers */
#define MPU60XX_AUX_VDDIO			0x01
#define MPU60XX_SMPLRT_DIV			0x19
#define MPU60XX_CONFIG				0x1A
#define MPU60XX_GYRO_CONFIG			0x1B
#define MPU60XX_ACCEL_CONFIG		0x1C
#define MPU60XX_MOTION_THRESH		0x1F
#define MPU60XX_INT_PIN_CFG			0x37
#define MPU60XX_INT_ENABLE			0x38
#define MPU60XX_INT_STATUS			0x3A
#define MPU60XX_ACCEL_XOUT_H		0x3B
#define MPU60XX_ACCEL_XOUT_L		0x3C
#define MPU60XX_ACCEL_YOUT_H		0x3D
#define MPU60XX_ACCEL_YOUT_L		0x3E
#define MPU60XX_ACCEL_ZOUT_H		0x3F
#define MPU60XX_ACCEL_ZOUT_L		0x40
#define MPU60XX_TEMP_OUT_H			0x41
#define MPU60XX_TEMP_OUT_L			0x42
#define MPU60XX_GYRO_XOUT_H			0x43
#define MPU60XX_GYRO_XOUT_L			0x44
#define MPU60XX_GYRO_YOUT_H			0x45
#define MPU60XX_GYRO_YOUT_L			0x46
#define MPU60XX_GYRO_ZOUT_H			0x47
#define MPU60XX_GYRO_ZOUT_L			0x48
#define MPU60XX_MOT_DETECT_STATUS	0x61
#define MPU60XX_SIGNAL_PATH_RESET	0x68
#define MPU60XX_MOT_DETECT_CTRL		0x69
#define MPU60XX_USER_CTRL			0x6A
#define MPU60XX_PWR_MGMT_1			0x6B
#define MPU60XX_PWR_MGMT_2			0x6C
#define MPU60XX_FIFO_COUNTH			0x72
#define MPU60XX_FIFO_COUNTL			0x73
#define MPU60XX_FIFO_R_W			0x74
#define MPU60XX_WHO_AM_I			0x75

/* Gyro sensitivities in degrees/s */
#define MPU60XX_GYRO_SENS_250		((float) 131)
#define MPU60XX_GYRO_SENS_500		((float) 65.5)
#define MPU60XX_GYRO_SENS_1000		((float) 32.8)
#define MPU60XX_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU60XX_ACCE_SENS_2			((float) 16384)
#define MPU60XX_ACCE_SENS_4			((float) 8192)
#define MPU60XX_ACCE_SENS_8			((float) 4096)
#define MPU60XX_ACCE_SENS_16		((float) 2048)



//FUNCTIONS:

MPU60XX_Result MPU60XX_Init(I2C_HandleTypeDef* I2Cx,
                               MPU60XX* DataStruct, 
                               MPU60XX_Device DeviceNumber,
                               MPU60XX_Accelerometer AccelerometerSensitivity, 
                               MPU60XX_Gyroscope GyroscopeSensitivity)
{
	uint8_t WHO_AM_I = (uint8_t)MPU60XX_WHO_AM_I;
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t d[2];


	/* Format I2C address */
	DataStruct->Address = MPU60XX_I2C_ADDR | (uint8_t)DeviceNumber;
	uint8_t address = DataStruct->Address;

	/* Check if device is connected */
    
	if(HAL_I2C_IsDeviceReady(Handle,address,2,5)!=HAL_OK)
	{
	    return MPU60XX_Result_Error;
	}
	/* Check who am I */
	//------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(Handle, address, &WHO_AM_I, 1, 1000) != HAL_OK)
		{
			return MPU60XX_Result_Error;
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(Handle, address, &temp, 1, 1000) != HAL_OK)
		{
			return MPU60XX_Result_Error;
		}

		/* Checking */
		while(temp != MPU60XX_I_AM)
		{
			/* Return error */
			return MPU60XX_Result_DeviceInvalid;
		}
	//------------------

	/* Wakeup MPU60XX */
	//------------------
		/* Format array to send */
		d[0] = MPU60XX_PWR_MGMT_1;
		d[1] = 0x00;

		/* Try to transmit via I2C */
		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
			return MPU60XX_Result_Error;
		}
	//------------------

	/* Set sample rate to 1kHz */
	MPU60XX_SetDataRate(I2Cx,DataStruct, MPU60XX_DataRate_1KHz);

	/* Config accelerometer */
	MPU60XX_SetAccelerometer(I2Cx,DataStruct, AccelerometerSensitivity);

	/* Config Gyroscope */
	MPU60XX_SetGyroscope(I2Cx, DataStruct, GyroscopeSensitivity);

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_SetDataRate(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct, uint8_t rate)
{
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	/* Format array to send */
	d[0] = MPU60XX_SMPLRT_DIV;
	d[1] = rate;

	/* Set data sample rate */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,(uint8_t *)d,2,1000)!=HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_SetAccelerometer(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct, MPU60XX_Accelerometer AccelerometerSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU60XX_ACCEL_CONFIG;

	/* Config accelerometer */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case MPU60XX_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU60XX_ACCE_SENS_2;
			break;
		case MPU60XX_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU60XX_ACCE_SENS_4;
			break;
		case MPU60XX_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU60XX_ACCE_SENS_8;
			break;
		case MPU60XX_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU60XX_ACCE_SENS_16;
			break;
		default:
			break;
		}

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_SetGyroscope(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct, MPU60XX_Gyroscope GyroscopeSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU60XX_GYRO_CONFIG;

	/* Config gyroscope */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	switch (GyroscopeSensitivity) {
			case MPU60XX_Gyroscope_250s:
				DataStruct->Gyro_Mult = (float)1 / MPU60XX_GYRO_SENS_250;
				break;
			case MPU60XX_Gyroscope_500s:
				DataStruct->Gyro_Mult = (float)1 / MPU60XX_GYRO_SENS_500;
				break;
			case MPU60XX_Gyroscope_1000s:
				DataStruct->Gyro_Mult = (float)1 / MPU60XX_GYRO_SENS_1000;
				break;
			case MPU60XX_Gyroscope_2000s:
				DataStruct->Gyro_Mult = (float)1 / MPU60XX_GYRO_SENS_2000;
				break;
			default:
				break;
		}
	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_ReadAccelerometer(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU60XX_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read accelerometer data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_ReadGyroscope(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU60XX_GYRO_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read gyroscope data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_ReadTemperature(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct)
{
	uint8_t data[2];
	int16_t temp;
	uint8_t reg = MPU60XX_TEMP_OUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read temperature */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 2, 1000) != HAL_OK);

	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_ReadAll(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct)
{
	uint8_t data[14];
	int16_t temp;
	uint8_t reg = MPU60XX_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read full raw data, 14bytes */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 14, 1000) != HAL_OK);

	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_EnableInterrupts(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {MPU60XX_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= MPU60XX_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 14, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = MPU60XX_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_DisableInterrupts(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct)
{
	uint8_t reg[2] = {MPU60XX_INT_ENABLE,0x00};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Disable interrupts */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,reg,2,1000)!=HAL_OK);
	/* Return OK */
	return MPU60XX_Result_Ok;
}



MPU60XX_Result MPU60XX_ReadInterrupts(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct, MPU60XX_Interrupt* InterruptsStruct)
{
	uint8_t read;

	/* Reset structure */
	InterruptsStruct->Status = 0;
	uint8_t reg = MPU60XX_INT_STATUS;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &read, 14, 1000) != HAL_OK);

	/* Fill value */
	InterruptsStruct->Status = read;
	/* Return OK */
	return MPU60XX_Result_Ok;
}


