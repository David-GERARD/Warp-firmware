#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devINA219.h"

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

// ----------------------------------------------------
void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	uint16_t configPackageINA219 = (kINA219ConfigRST << 15)
                            | (kINA219Config_ << 14)
                            | (kINA219ConfigBRNG << 13)
                            | (kINA219ConfigPG << 11)
                            | (kINA219ConfigBADC << 7)
                            | (kINA219ConfigSADC << 3)
                            | (kINA219ConfigMODE);

	warpPrint("\nConfig package: ");
	warpPrint(" 0b");
    // Print each bit of the result
    for (int i = 15; i >= 0; i--) {
        warpPrint("%u", (configPackageINA219 >> i) & 1);
    }
    warpPrint("\n");

	configureSensorINA219(configPackageINA219);

	return;
}

// ----------------------------------------------------

WarpStatus 
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
    uint8_t     payloadBytes[2], commandByte[1];
    i2c_status_t    status;

    switch (deviceRegister)
    {
        case 0x00: case 0x05:
        {
            /* OK */
            break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
    {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    commandByte[0] = deviceRegister;
    // Split the 16-bit payload into two 8-bit parts
    payloadBytes[0] = (payload >> 8) & 0xFF; // MSB
    payloadBytes[1] = payload & 0xFF; // LSB
    warpEnableI2Cpins();

    status = I2C_DRV_MasterSendDataBlocking(
        0 /* I2C instance */,
        &slave,
        commandByte,
        1,
        payloadBytes,
        2,
        gWarpI2cTimeoutMilliseconds);
    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}


// ----------------------------------------------------

WarpStatus
configureSensorINA219(uint16_t payloadCONF_REG)  
{
	WarpStatus	i2cWriteStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cWriteStatus = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219 /* register address for config */,
												  payloadCONF_REG 
	);


	return (i2cWriteStatus);
}

// ----------------------------------------------------

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: 
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

// ----------------------------------------------------	

void
printSensorDataINA219(bool hexModeFlag)
{
	uint8_t	readSensorRegisterValueLSB;
	int8_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);


	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Shunt_Voltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Bus_Voltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Power, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Current, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x, \n", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d, \n", readSensorRegisterValueCombined);
		}
	}


}

// ----------------------------------------------------


void
printRealValuesINA219(int32_t* ptr_time_start)
{
	uint8_t	readSensorRegisterValueLSB;
	int8_t	readSensorRegisterValueMSB;
	int16_t		readSensorValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	// i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Current, 2 /* numberOfBytes */);
	// readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	// readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	// readSensorValueCombined = computeCurrent(readSensorRegisterValueMSB,readSensorRegisterValueLSB);

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Shunt_Voltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorValueCombined = computeShuntVoltage(readSensorRegisterValueMSB,readSensorRegisterValueLSB);
	float current = readSensorValueCombined/shuntResitance;


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{

			warpPrint("%d, ", (uint32_t)current);

	}


	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Shunt_Voltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorValueCombined = computeShuntVoltage(readSensorRegisterValueMSB,readSensorRegisterValueLSB);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
	
			warpPrint("%d, ",readSensorValueCombined);

	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Bus_Voltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorValueCombined = computeBusVoltage(readSensorRegisterValueMSB,readSensorRegisterValueLSB);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
	
			warpPrint("%d, ",readSensorValueCombined);

	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Power, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorValueCombined = computePower(readSensorRegisterValueMSB,readSensorRegisterValueLSB);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
	
			warpPrint("%d, ",readSensorValueCombined);

	}
	// 
	uint16_t time = OSA_TimeGetMsec();
	int32_t time_diff = time - *ptr_time_start;
	if(time_diff<0){ // This appens when OSA_TimeGetMsec returns a number greater than 65535 and casts it as an int16
		*ptr_time_start = *ptr_time_start - 65534;
		time_diff = time - *ptr_time_start;
	}
	warpPrint("%d\n",time_diff);

}


int16_t 
computeShuntVoltage(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB)
{
	uint16_t combinedValue = ((uint16_t)readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	switch (kINA219ConfigPG)
	{
	case 0b00: // PGA = 1
		// Combine the MSB and LSB into a 16-bit unsigned value
		if (readSensorRegisterValueMSB & 0x80) combinedValue |= 0xF000;
		break;
	case 0b01: // PGA = %2
		// Combine the MSB and LSB into a 16-bit unsigned value
		if (readSensorRegisterValueMSB & 0x40) combinedValue |= 0xF800;
		break;
	case 0b10: // PGA = %4
		// Combine the MSB and LSB into a 16-bit unsigned value
		if (readSensorRegisterValueMSB & 0x20) combinedValue |= 0xFC00;
		break;
	case 0b11: // PGA = %8
		// Combine the MSB and LSB into a 16-bit unsigned value
		if (readSensorRegisterValueMSB & 0x10) combinedValue |= 0xFE00;
		break;
	default:
		warpPrint("Invalid PGA setting\n");
		break;


    }

	return combinedValue*10; // LSB is 10uV
}
		
	
	

int16_t 
computeBusVoltage(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB)
{
	// TODO : Calibration 

	uint16_t combinedValue = ((uint16_t)readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;
	return combinedValue;

}

int16_t 
computePower(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB)
{
	// TODO : Calibration

	uint16_t combinedValue = ((uint16_t)readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;
	return combinedValue;
}

int16_t 
computeCurrent(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB)
{
	// TODO : Calibration 

	uint16_t combinedValue = ((uint16_t)readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;
	return combinedValue;
}


// ----------------------------------------------------




uint8_t
appendSensorDataINA219(uint8_t* buf)
{
	uint8_t index = 0;
	uint8_t	readSensorRegisterValueLSB;
	int8_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);


	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Bus_Voltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Shunt_Voltage, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Power, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Current, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB );  // Current is given in Two's complement

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	return index;

}




