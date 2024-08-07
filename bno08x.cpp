/*
 * bno08x.cpp
 *
 *  Created on: Aug 7, 2024
 *      Author: BRL
 */




#include "bno08x.hpp"

BNO08x::BNO08x(SPI_HandleTypeDef hspi_)
{
  hspi = hspi_;

  hspi.Instance = SPI2;
  hspi.Init.Mode = SPI_MODE_MASTER;
  hspi.Init.Direction = SPI_DIRECTION_2LINES;
  hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi.Init.NSS = SPI_NSS_SOFT;
  hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi) != HAL_OK)
  {
	Error_Handler();
  }
}

void BNO08x::BNO080_GPIO_SPI_Initialization()
{
	CHIP_DESELECT(BNO080);
	WAKE_HIGH();
	RESET_HIGH();
}

int BNO08x::BNO080_Initialization()
{
	BNO080_GPIO_SPI_Initialization();

//	printf("Checking BNO080...");

	CHIP_DESELECT(BNO080);

	//Configure the BNO080 for SPI communication
	WAKE_HIGH();	//Before boot up the PS0/WAK pin must be high to enter SPI mode
	RESET_LOW();	//Reset BNO080
	HAL_Delay(400);	//Min length not specified in datasheet?
	RESET_HIGH();	//Bring out of reset

	BNO080_waitForSPI(); //Wait until INT pin goes low.

	//At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
	//host. It must not send any other data until this step is complete.
	//When BNO080 first boots it broadcasts big startup packet
	//Read it and dump it
	BNO080_waitForSPI(); //Wait for assertion of INT before reading advert message.
	BNO080_receivePacket();

	//The BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)
	//Read it and dump it
	BNO080_waitForSPI();  //Wait for assertion of INT before reading Init response
	BNO080_receivePacket();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;						 //Reserved

	//Transmit packet on channel 2, 2 bytes
	BNO080_sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	BNO080_waitForSPI();
	if (BNO080_receivePacket() == 1)
	{
//		printf("header: %d %d %d %d\n", shtpHeader[0], shtpHeader[1], shtpHeader[2], shtpHeader[3]);
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
//			printf("BNO080 who_am_i = 0x%02x...ok\n\n", shtpData[0]);
			return (0);
		}// Sensor OK
	}
//	printf("BNO080 Not OK: 0x%02x Should be 0x%02x\n", shtpData[0], SHTP_REPORT_PRODUCT_ID_RESPONSE);
	return (1); //Something went wrong
}

int BNO08x::BNO080_waitForSPI()
{
	for (uint32_t counter = 0; counter < 0xffffffff; counter++) //Don't got more than 255
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
		{
			//printf("\nData available\n");
			return (1);
		}
		//printf("SPI Wait %d\n", counter);
	}
//	printf("\nData not available\n");
	return (0);
}

int BNO08x::BNO080_receivePacket(void)
{
	uint8_t incoming;

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
		return (0); //Data is not available

	//Old way: if (BNO080_waitForSPI() == 0) return (0); //Something went wrong

	//Get first four bytes to find out how much data we need to read

	CHIP_SELECT(BNO080);
//	HAL_Delay(100);
	//Get the first four bytes, aka the packet header
	uint8_t packetLSB = SPI2_SendByte(0);
	uint8_t packetMSB = SPI2_SendByte(0);
	uint8_t channelNumber = SPI2_SendByte(0);
	uint8_t sequenceNumber = SPI2_SendByte(0); //Not sure if we need to store this or not
	packetLSB = SPI2_SendByte(0);
	packetLSB = SPI2_SendByte(0);
	packetLSB = SPI2_SendByte(0);
	packetLSB = SPI2_SendByte(0);
	packetLSB = SPI2_SendByte(0);
	//Store the header info
	shtpHeader[0] = packetLSB;
	shtpHeader[1] = packetMSB;
	shtpHeader[2] = channelNumber;
	shtpHeader[3] = sequenceNumber;

	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
	dataLength &= 0x7fff; //Clear the MSbit.
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
	//TODO catch this as an error and exit
	if (dataLength == 0)
	{
		//Packet is empty
		return (0); //All done
	}
	dataLength -= 4; //Remove the header bytes from the data count

	//printf("length: %d\n", dataLength);

	//Read incoming data into the shtpData array
	for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
	{
		incoming = SPI2_SendByte(0xFF);
		//printf("%d ", incoming);
		if (dataSpot < MAX_PACKET_SIZE)	//BNO080 can respond with upto 270 bytes, avoid overflow
			shtpData[dataSpot] = incoming; //Store data into the shtpData array
	}
	//printf("\n");

	CHIP_DESELECT(BNO080); //Release BNO080
	return (1); //We're done!
}

uint8_t BNO08x::SPI2_SendByte(unsigned char data)
{
//	while(LL_SPI_IsActiveFlag_TXE(BNO080_SPI_CHANNEL)==RESET);
//	LL_SPI_TransmitData8(BNO080_SPI_CHANNEL, data);
//
//	while(LL_SPI_IsActiveFlag_RXNE(BNO080_SPI_CHANNEL)==RESET);
//	return LL_SPI_ReceiveData8(BNO080_SPI_CHANNEL);
	//	while(LL_SPI_IsActiveFlag_TXE(BNO080_SPI_CHANNEL)==RESET);
	//	LL_SPI_TransmitData8(hdma_spi2_tx, data);

//		while(!__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_TXE));
//		data = 13;
		HAL_SPI_Transmit(&hspi, (uint8_t *) data, 1, HAL_MAX_DELAY);
//		while(!__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_RXNE));

		  uint8_t receivedData;
//		  if (__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_RXNE)) {
//			  // 接收数据
//
//			  unsigned char RX = receivedData;
//			  		return RX;
//			  // 处理接收到的数据
//		  }
//		HAL_SPI_Receive(&hspi, &RX_Buffer, 2, HAL_MAX_DELAY);
//		while(!__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_RXNE));
		  HAL_SPI_Receive(&hspi, &receivedData, 1, HAL_MAX_DELAY);
		return receivedData;
}

void BNO08x::BNO080_enableRotationVector(uint16_t timeBetweenReports)
{
	BNO080_setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}


int BNO08x::BNO080_sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	//Wait for BNO080 to indicate it is available for communication
	if (BNO080_waitForSPI() == 0)
		return (0); //Data is not available

	//BNO080 has max CLK of 3MHz, MSB first,
	//The BNO080 uses CPOL = 1 and CPHA = 1. This is mode3
	CHIP_SELECT(BNO080);

	//Send the 4 byte packet header
	SPI2_SendByte(packetLength & 0xFF);			//Packet length LSB
	SPI2_SendByte(packetLength >> 8);				//Packet length MSB
	SPI2_SendByte(channelNumber);					//Channel number
	SPI2_SendByte(sequenceNumber[channelNumber]++); 	//Send the sequence number, increments with each packet sent, different counter for each channel

	//Send the user's data packet
	for (uint8_t i = 0; i < dataLength; i++)
	{
		SPI2_SendByte(shtpData[i]);
	}

	CHIP_DESELECT(BNO080);

	return (1);
}



void BNO08x::BNO080_setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig)
{
	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;						 //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;							 //Feature flags
	shtpData[3] = 0;							 //Change sensitivity (LSB)
	shtpData[4] = 0;							 //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;							 //Batch Interval (LSB)
	shtpData[10] = 0;							 //Batch Interval
	shtpData[11] = 0;							 //Batch Interval
	shtpData[12] = 0;							 //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   	 //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   	 //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	 //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	 //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	BNO080_sendPacket(CHANNEL_CONTROL, 17);
}

float BNO08x::BNO080_getQuatI()
{
	return BNO080_qToFloat(rawQuatI, rotationVector_Q1);
}

float BNO08x::BNO080_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	return fixedPointValue * powf(2, qPoint * -1);
}
int BNO08x::BNO080_dataAvailable(void)
{
	//If we have an interrupt pin connection available, check if data is available.
	//If int pin is NULL, then we'll rely on BNO080_receivePacket() to timeout
	//See issue 13: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/13
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
		return (0); //Data is not available
	if (BNO080_receivePacket() == 1)
	{
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			BNO080_parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
			return (1);
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			BNO080_parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
			return (1);
		}
	}
	return (0);
}

void BNO08x::BNO080_parseInputReport(void)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | (shtpData[3] << (8 * 2)) | (shtpData[2] << (8 * 1)) | (shtpData[1] << (8 * 0));

	uint8_t status = shtpData[7] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[10] << 8 | shtpData[9];
	uint16_t data2 = (uint16_t)shtpData[12] << 8 | shtpData[11];
	uint16_t data3 = (uint16_t)shtpData[14] << 8 | shtpData[13];
	uint16_t data4 = 0;
	uint16_t data5 = 0;

	if (dataLength > 14)
	{
		data4 = (uint16_t)shtpData[16] << 8 | shtpData[15];
	}
	if (dataLength > 16)
	{
		data5 = (uint16_t)shtpData[18] << 8 | shtpData[17];
	}

	//Store these generic values to their proper global variable
	switch(shtpData[5])
	{
		case SENSOR_REPORTID_ACCELEROMETER:
		{
			accelAccuracy = status;
			rawAccelX = data1;
			rawAccelY = data2;
			rawAccelZ = data3;
			break;
		}
		case SENSOR_REPORTID_LINEAR_ACCELERATION:
		{
			accelLinAccuracy = status;
			rawLinAccelX = data1;
			rawLinAccelY = data2;
			rawLinAccelZ = data3;
			break;
		}
		case SENSOR_REPORTID_GYROSCOPE:
		{
			gyroAccuracy = status;
			rawGyroX = data1;
			rawGyroY = data2;
			rawGyroZ = data3;
			break;
		}
		case SENSOR_REPORTID_MAGNETIC_FIELD:
		{
			magAccuracy = status;
			rawMagX = data1;
			rawMagY = data2;
			rawMagZ = data3;
			break;
		}
		case SENSOR_REPORTID_ROTATION_VECTOR:
		case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
		{
			quatAccuracy = status;
			rawQuatI = data1;
			rawQuatJ = data2;
			rawQuatK = data3;
			rawQuatReal = data4;
			rawQuatRadianAccuracy = data5; //Only available on rotation vector, not game rot vector
			break;
		}
		case SENSOR_REPORTID_STEP_COUNTER:
		{
			stepCount = data3; //Bytes 8/9
			break;
		}
		case SENSOR_REPORTID_STABILITY_CLASSIFIER:
		{
			stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
			break;
		}
		case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
		{
			activityClassifier = shtpData[5 + 5]; //Most likely state

			//Load activity classification confidences into the array
			for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
				_activityConfidences[x] = shtpData[11 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
			break;
		}
		case SHTP_REPORT_COMMAND_RESPONSE:
		{
			//printf("!");
			//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
			uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

			if (command == COMMAND_ME_CALIBRATE)
			{
				//printf("ME Cal report found!");
				calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
			}
			break;
		}
		default:
		{
			//This sensor report ID is unhandled.
			//See reference manual to add additional feature reports as needed
		}
	}

	//TODO additional feature reports may be strung together. Parse them all.
}
void BNO08x::BNO080_parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
}
