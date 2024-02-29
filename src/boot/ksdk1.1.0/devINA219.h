


void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payloadBtye);
WarpStatus configureSensorINA219(uint16_t payloadCONF_REG);
void printSensorDataINA219(bool hexModeFlag);
uint8_t appendSensorDataINA219(uint8_t* buf);

const uint8_t bytesPerMeasurementINA219            = 2;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 1;

typedef enum
{
    kINA219ConfigRST = 0x00, //Reset bit                - 1bit          - here: no reset
    kINA219Config_ = 0x00, //Empty buffer bit           - 1bit          - here: empty 
    kINA219ConfigBRNG = 0x00, //Bus volatage range      - 1bit          - here: 16V
    kINA219ConfigPG = 0x00, //Gain                      - 2bits         - here: 1
    kINA219ConfigBADC = 0x03, //Bus ADC resolution      - 4bits         - here: 12-bit samples (default)
    kINA219ConfigSADC = 0x03, //Shunt ADC resolution    - 4bits         - here: 12-bit samples (default)
    kINA219ConfigMODE = 0x07, //Operating mode          - 3bits         - here: Shunt and Bus, Continuous (default)

}INA219Config;

uint16_t configPackageINA219 = 
    (kINA219ConfigRST << 15) | 
    (kINA219Config_ << 14) | 
    (kINA219ConfigBRNG << 13) | 
    (kINA219ConfigPG << 11) | // 2 bits shift left by 11 (because 13-2 = 11)
    (kINA219ConfigBADC << 7) | // 4 bits shift left by 7 (because 11-4 = 7)
    (kINA219ConfigSADC << 3) | // 4 bits shift left by 3 (because 7-4 = 3)
    (kINA219ConfigMODE); // 3 bits, no shift needed as it's the LSB
