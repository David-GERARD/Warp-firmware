/*
	Authored 2024. David Gerard. 
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/


/*
This file contains functions that implement an Earthquake Early Dectection System (EEWS)
using a MMA8451Q 3 axis accelerometer.

It was tested using a FRDM KL03Z development board, using the Warp firmware developped by Phillip Stanley-Marbell.
*/

#include <stdlib.h>
#include "math.h"

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

#include "SEGGER_RTT.h"
#include "warp.h"
#include "devMMA8451Q.h"
#include "buffer.h"
#include "EEWS.h"




void
initEEWS(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts){
    // Initialise MMA8451Q accelerator
    initMMA8451Q(	i2cAddress	/* i2cAddress */,	operatingVoltageMillivolts	);

    // Configure MMA8451Q accelerator
    configureSensorMMA8451QforEEWS(	0x00, /* Payload: Disable FIFO */
									0x02 /*8g mode*/);

    return;
}


void
setAccelerationBiases(Ac_Biases * biases){
	int32_t AcX_bias = 0;
	int32_t AcY_bias = 0;
	int32_t AcZ_bias = 0;

	for (uint8_t i = 0; i < n_measurements_for_Ac_Biases; i++)
	{
		AcX_bias += getSensorDataMMA8451Q_X();
		AcY_bias += getSensorDataMMA8451Q_Y();
		AcZ_bias += getSensorDataMMA8451Q_Z();
	}
	biases->AcX = AcX_bias/n_measurements_for_Ac_Biases;
	biases->AcY = AcY_bias/n_measurements_for_Ac_Biases;
	biases->AcZ = AcZ_bias/n_measurements_for_Ac_Biases;

	return;

}

void
fillDataBuffer(CircularBuffer *cb,Ac_Biases *biases){
	int16_t AcX,AcY,AcZ;
	int status = 0;
	uint16_t seismicSignal;

	for (int i = 0; i < BUFFER_SIZE; i++)
	{	
		

		int16_t AcX = getSensorDataMMA8451Q_X() - biases->AcX;
		int16_t AcY = getSensorDataMMA8451Q_Y() - biases->AcY;
		int16_t AcZ = getSensorDataMMA8451Q_Z() - biases->AcZ;


		seismicSignal = computeSeismicSignal(AcX,AcY,AcZ);
		
		

		bufferAdd(cb,seismicSignal);
		// warpPrint("\nBuffer %d / %d\n",i,BUFFER_SIZE);
	}
	
	warpPrint("\nBuffer filled\n");

}


uint16_t 
computeSeismicSignal(int16_t AcX,int16_t AcY,int16_t AcZ){

  uint16_t result = sqrtf(AcX*AcX+AcY*AcY+AcZ*AcZ);

  return result;

}


void 
STAoverLTA(CircularBuffer *cb, STA_LTA_Result *result) {




    uint16_t n_samples_STA =  sampling_rate*STA_window_ms/1000;
    uint16_t n_samples_LTA =  sampling_rate*LTA_window_ms/1000;
	

    // Initialize count to 0 for STA calculation
    uint32_t sum = 0;
    for (int i = 0; i < n_samples_STA; i++) {
        sum += bufferGet(cb, i);
		//warpPrint("\n%d\n",bufferGet(cb, i));
		
    }
    uint32_t STA = sum / n_samples_STA;

	

    for (int i = n_samples_STA; i < n_samples_LTA; i++) {
        sum += bufferGet(cb, i);
		//warpPrint("\n%d - %d\n",(cb->head - 1 - i) % BUFFER_SIZE, bufferGet(cb, i));
    }

	
    uint32_t LTA = sum / n_samples_LTA;

	uint32_t ratio = 10 * STA / LTA;

    result->STA = STA;
    result->LTA = LTA;
    result->ratio = ratio;
}





void
printEEWSData(CircularBuffer *cb, Ac_Biases *biases){

	uint16_t start_time = OSA_TimeGetMsec();

	int16_t AcX = getSensorDataMMA8451Q_X() - biases->AcX;
	int16_t AcY = getSensorDataMMA8451Q_Y() - biases->AcY;
	int16_t AcZ = getSensorDataMMA8451Q_Z() - biases->AcZ;

	warpPrint("%d, ", AcX);
	warpPrint("%d, ", AcY);
	warpPrint("%d, ", AcZ);
	

	uint16_t seismicSignal = computeSeismicSignal(AcX,AcY,AcZ);
	warpPrint("%d, ", seismicSignal);

	bufferAdd(cb,seismicSignal);

	STA_LTA_Result results;
	STAoverLTA(cb,&results);

	warpPrint("%d, ", results.STA);
	warpPrint("%d, ", results.LTA);
	warpPrint("%d, ", results.ratio);


	uint16_t end_time = OSA_TimeGetMsec();

	if (end_time > start_time)
	{
		//warpPrint("%d, ", (1000/sampling_rate) - (end_time - start_time));
		OSA_TimeDelay((1000/sampling_rate) - (end_time - start_time));
	}
	else
	{
		//warpPrint("%d, ", (1000/sampling_rate) - 65535 - start_time + end_time);
		OSA_TimeDelay((1000/sampling_rate)-( 65535 - start_time + end_time));
	}
	warpPrint("\n");
	
}