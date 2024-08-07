/*
 * bno08x.hpp
 *
 *  Created on: Aug 7, 2024
 *      Author: BRL
 */

#ifndef INC_BNO08X_HPP_
#define INC_BNO08X_HPP_

#include "spi.h"
#include "main.h"
#include "math.h"

enum Registers
{
	CHANNEL_COMMAND = 0,
	CHANNEL_EXECUTABLE = 1,
	CHANNEL_CONTROL = 2,
	CHANNEL_REPORTS = 3,
	CHANNEL_WAKE_REPORTS = 4,
	CHANNEL_GYRO = 5
};

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many bu


#define BNO080_SPI_SCLK_PIN		GPIO_PIN_13
#define BNO080_SPI_SCLK_PORT	GPIOB
#define BNO080_SPI_MISO_PIN		GPIO_PIN_14
#define BNO080_SPI_MISO_PORT	GPIOB
#define BNO080_SPI_MOSI_PIN		GPIO_PIN_15
#define BNO080_SPI_MOSI_PORT	GPIOB
#define BNO080_SPI_CS_PIN		GPIO_PIN_12
#define BNO080_SPI_CS_PORT		GPIOB
#define BNO080_PS0_WAKE_PIN		GPIO_PIN_8
#define BNO080_PS0_WAKE_PORT	GPIOA
#define BNO080_RST_PIN			GPIO_PIN_9
#define BNO080_RST_PORT			GPIOC
#define BNO080_INT_PIN			GPIO_PIN_8
#define BNO080_INT_PORT			GPIOC

#define CHIP_SELECT(BNO080)		HAL_GPIO_WritePin(BNO080_SPI_CS_PORT, BNO080_SPI_CS_PIN, GPIO_PIN_RESET)
#define CHIP_DESELECT(BNO080)	HAL_GPIO_WritePin(BNO080_SPI_CS_PORT, BNO080_SPI_CS_PIN, GPIO_PIN_SET)
#define WAKE_HIGH()				HAL_GPIO_WritePin(BNO080_PS0_WAKE_PORT, BNO080_PS0_WAKE_PIN, GPIO_PIN_SET)
#define WAKE_LOW()				HAL_GPIO_WritePin(BNO080_PS0_WAKE_PORT, BNO080_PS0_WAKE_PIN, GPIO_PIN_RESET)
#define RESET_HIGH()			HAL_GPIO_WritePin(BNO080_RST_PORT, BNO080_RST_PIN, GPIO_PIN_SET)
#define RESET_LOW()				HAL_GPIO_WritePin(BNO080_RST_PORT, BNO080_RST_PIN, GPIO_PIN_RESET)

class BNO08x
{
public:
	BNO08x(SPI_HandleTypeDef* hspi_);
public:
	void BNO080_GPIO_SPI_Initialization(void);
	int BNO080_Initialization(void);
	uint8_t SPI2_SendByte(uint8_t data);
	int BNO080_dataAvailable(void);
	int BNO080_waitForSPI(void);
	int BNO080_receivePacket(void);
	int BNO080_sendPacket(uint8_t channelNumber, uint8_t dataLength);
	void BNO080_enableRotationVector(uint16_t timeBetweenReports);
	void BNO080_setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig);

	float BNO080_getQuatI();
	float BNO080_getQuatJ();
	float BNO080_getQuatK();
	float BNO080_getQuatReal();
	float BNO080_qToFloat(int16_t fixedPointValue, uint8_t qPoint);



	void BNO080_parseInputReport(void);
	void BNO080_parseCommandReport(void);
public:
	SPI_HandleTypeDef*  hspi;

	uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
	uint8_t shtpData[MAX_PACKET_SIZE];
	uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	uint32_t metaData[MAX_METADATA_SIZE];

	//These are the raw sensor values pulled from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t *_activityConfidences; //Array that store the confidences of the 9 possible activities
	uint8_t calibrationStatus;	 //Byte R0 of ME Calibration Response

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;

};


#endif /* INC_BNO08X_HPP_ */
