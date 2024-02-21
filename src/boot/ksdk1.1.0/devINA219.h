


void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payloadBtye);
WarpStatus configureSensorINA219(uint8_t payloadCONF_REG);
void printSensorCurrentINA219(bool hexModeFlag);
uint8_t appendSensorDataINA219(uint8_t* buf);

const uint8_t bytesPerMeasurementINA219            = 2;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 1;

typedef enum
{
    kINA219ConfigRST = 0x00, //1bit
    kINA219Config_ = 0x00, //1bit
    kINA219ConfigBRNG = 0x00, //1bit
    kINA219ConfigPG = 0x03, //2bits
    kINA219ConfigBADC = 0x03, //4bits
    kINA219ConfigSADC = 0x03, //4bits
    kINA219ConfigMODE = 0x07, //3bits

}INA219Config;