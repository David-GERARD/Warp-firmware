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

/*WARNING - MAKE SURE THAT BUFFER_SIZE (declared in buffer.h) IN BIG ENOUGH TO store all the values in LTA window*/

const uint16_t STA_window_ms = 500; //ms
const uint16_t LTA_window_ms = 5000; //ms
const uint8_t sampling_rate = 15; //Hz

const uint8_t n_measurements_for_Ac_Biases = 100;

typedef struct {
    int16_t AcX;
    int16_t AcY;
    int16_t AcZ;
} Ac_Biases;

typedef struct {
    uint32_t STA;
    uint32_t LTA;
    uint32_t ratio;
} STA_LTA_Result;

void initEEWS(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
void setAccelerationBiases(Ac_Biases * biases);
void fillDataBuffer(CircularBuffer *cb,Ac_Biases *biases);

uint16_t computeSeismicSignal(int16_t AcX,int16_t AcY,int16_t AcZ);
void STAoverLTA(CircularBuffer *cb, STA_LTA_Result *result);

void printEEWSData(CircularBuffer *cb, Ac_Biases *biases);