
#ifndef __MPU_H
#define __MPU_H

#ifdef __cplusplus
extern "C" {
#endif


//INCLUDES:
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"



//DEFINES:
/* Default I2C clock */
#ifndef MPU60XX_I2C_CLOCK
#define MPU60XX_I2C_CLOCK              400000    // Default I2C clock speed
#endif

#define MPU60XX_DataRate_8KHz       0         // Sample rate set to 8 kHz
#define MPU60XX_DataRate_4KHz       1         // Sample rate set to 4 kHz
#define MPU60XX_DataRate_2KHz       3         // Sample rate set to 2 kHz
#define MPU60XX_DataRate_1KHz       7         // Sample rate set to 1 kHz
#define MPU60XX_DataRate_500Hz      15        // Sample rate set to 500 Hz
#define MPU60XX_DataRate_250Hz      31        // Sample rate set to 250 Hz
#define MPU60XX_DataRate_125Hz      63        // Sample rate set to 125 Hz
#define MPU60XX_DataRate_100Hz      79        // Sample rate set to 100 Hz



//PRIVATE VARIABLES:
typedef enum
{
    MPU60XX_Device_0 = 0x00,                  // AD0 pin is set to low
    MPU60XX_Device_1 = 0x02                   // AD0 pin is set to high
}   MPU60XX_Device;



typedef enum
{
    MPU60XX_Result_Zero = 0x00,
	MPU60XX_Result_Ok,                        // Everything OK
	MPU60XX_Result_Error,                     // Unknown error
	MPU60XX_Result_DeviceNotConnected,        // There is no device with valid slave addres
	MPU60XX_Result_DeviceInvalid              // Connected device with address is not MPU6050
}   MPU60XX_Result;



typedef enum
{
    MPU60XX_Accelerometer_2G = 0x00,          // Range is +- 2G
	MPU60XX_Accelerometer_4G = 0x01,          // Range is +- 4G
	MPU60XX_Accelerometer_8G = 0x02,          // Range is +- 8G
	MPU60XX_Accelerometer_16G = 0x03          // Range is +- 16G
}   MPU60XX_Accelerometer;



typedef enum
{
	MPU60XX_Gyroscope_250s = 0x00,            // Range is +- 250 degrees/s
	MPU60XX_Gyroscope_500s = 0x01,            // Range is +- 500 degrees/s
	MPU60XX_Gyroscope_1000s = 0x02,           // Range is +- 1000 degrees/s
	MPU60XX_Gyroscope_2000s = 0x03            // Range is +- 2000 degrees/s
}   MPU60XX_Gyroscope;



typedef struct
{
	/* Private */
	uint8_t Address;          // I2C address of device
	float Gyro_Mult;          // Gyroscope corrector from raw data to "degrees/s". Only for private use
	float Acce_Mult;          // Accelerometer corrector from raw data to "g". Only for private use
	/* Public */
	int16_t Accelerometer_X;  // Accelerometer value X axis
	int16_t Accelerometer_Y;  // Accelerometer value Y axis
	int16_t Accelerometer_Z;  // Accelerometer value Z axis
	int16_t Gyroscope_X;      // Gyroscope value X axis
	int16_t Gyroscope_Y;      // Gyroscope value Y axis
	int16_t Gyroscope_Z;      // Gyroscope value Z axis
	float   Temperature;      // Temperature in degrees
	//I2C_HandleTypeDef* I2Cx;
}   MPU60XX;



typedef union
{
	struct
    {
		uint8_t DataReady:1;       // Data ready interrupt
		uint8_t reserved2:2;       // Reserved bits
		uint8_t Master:1;          // Master interrupt. Not enabled with library
		uint8_t FifoOverflow:1;    // FIFO overflow interrupt. Not enabled with library
		uint8_t reserved1:1;       // Reserved bit
		uint8_t MotionDetection:1; // Motion detected interrupt
		uint8_t reserved0:1;       // Reserved bit
	} F;
	uint8_t Status;
} MPU60XX_Interrupt;



//FUNCTIONS DECLARATION:
MPU60XX_Result MPU60XX_Init(I2C_HandleTypeDef* I2Cx,
                               MPU60XX* DataStruct, 
                               MPU60XX_Device DeviceNumber,
                               MPU60XX_Accelerometer AccelerometerSensitivity, 
                               MPU60XX_Gyroscope GyroscopeSensitivity);

MPU60XX_Result MPU60XX_SetDataRate(I2C_HandleTypeDef* I2Cx,
                                   MPU60XX* DataStruct,
                                   uint8_t rate);

MPU60XX_Result MPU60XX_SetAccelerometer(I2C_HandleTypeDef* I2Cx,
                                        MPU60XX* DataStruct,
                                        MPU60XX_Accelerometer AccelerometerSensitivity);

MPU60XX_Result MPU60XX_SetGyroscope(I2C_HandleTypeDef* I2Cx,
                                    MPU60XX* DataStruct,
                                    MPU60XX_Gyroscope GyroscopeSensitivity);
                                    
MPU60XX_Result MPU60XX_ReadTemperature(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct);
MPU60XX_Result MPU60XX_ReadAll(I2C_HandleTypeDef* I2Cx, MPU60XX* DataStruct);



#ifdef __cplusplus
}
#endif

#endif /* __MPU */

