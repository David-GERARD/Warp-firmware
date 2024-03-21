


void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payloadBtye);
WarpStatus configureSensorINA219(uint16_t payloadCONF_REG);
void printSensorDataINA219(bool hexModeFlag);
uint8_t appendSensorDataINA219(uint8_t* buf);
void printRealValuesINA219(int32_t* ptr_time_start);

int16_t computeShuntVoltage(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB);
int16_t computeBusVoltage(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB);
int16_t computePower(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB);
int16_t computeCurrent(uint8_t readSensorRegisterValueMSB, uint8_t readSensorRegisterValueLSB);


const uint8_t bytesPerMeasurementINA219            = 2;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 1;
const float shuntResitance = 0.1; // Ohms


uint16_t kINA219ConfigRST = 0b0; //Reset bit                - 1bit          - here: no reset
uint16_t kINA219Config_ = 0b0; //Empty buffer bit           - 1bit          - here: empty 
uint16_t kINA219ConfigBRNG = 0b0; //Bus volatage range      - 1bit          - here: 16V
uint16_t kINA219ConfigPG = 0b00; //Gain                      - 2bits         - here: 1
uint16_t kINA219ConfigBADC = 0b0011; //Bus ADC resolution      - 4bits         - here: 12-bit samples (default)
uint16_t kINA219ConfigSADC = 0b0011; //Shunt ADC resolution    - 4bits         - here: 12-bit samples (default)
uint16_t kINA219ConfigMODE = 0b111; //Operating mode          - 3bits         - here: Shunt and Bus, Continuous (default)


