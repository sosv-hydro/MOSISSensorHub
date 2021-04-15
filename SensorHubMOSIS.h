

/**
 * TSYS01 temperature sensor commands and address
 */
#define TSYS01_ADDR                        0x77
#define TSYS01_RESET                       0x1E
#define TSYS01_ADC_READ                    0x00
#define TSYS01_ADC_TEMP_CONV               0x48
#define TSYS01_PROM_READ                   0XA0

/**
 *  MS5837-30BA Pressure sensor commands and address
 */
#define MS5837_ADDR                         0x76
#define MS5837_RESET                        0x1E
#define MS5837_ADC_READ                     0x00
#define MS5837_PROM_READ                    0xA0
#define MS5837_CONVERT_D1_1024              0x44 //Convert D1 with OSR = 1024
#define MS5837_CONVERT_D2_1024              0x54 //Convert D2 with OSR = 1024

/**
 *  PH-EZO sensor
 */

#define PHEZO_ADDR                          0x63
#define PHEZO_READ                          0x72
#define PHEZO_LED                           0x4C
#define PHEZO_INFO                          0x69
#define PHEZO_STATUS                        "Status"

/**
 *  DO-EZO sensor
 */
#define DOEZO_ADDR                          0x67
#define DOEZO_DEVICETYPE                    0x00
#define DOEZO_FIRMWARENO                    0x01

#define DOEZO_ACTIVATE                      0x06
#define DOEZO_NEWREADINGAV                  0x07

#define DOEZO_DATA1BYTE_MGL                     0x22
#define DOEZO_DATA2BYTE_MGL                     0x23
#define DOEZO_DATA3BYTE_MGL                     0x24
#define DOEZO_DATA4BYTE_MGL                     0x25

#define DOEZO_DATA1BYTE_SAT                     0x26
#define DOEZO_DATA2BYTE_SAT                     0x27
#define DOEZO_DATA3BYTE_SAT                     0x28
#define DOEZO_DATA4BYTE_SAT                     0x29

#define DOEZO_CALIBRATION                       0x08
#define DOEZO_CALIBRATIONCONF                   0x09


#define SMALLDELAY                          416666
#define BIGDELAY                            25000000


extern void InitalizeSensorsAndSystem(uint32_t ui32SysClock);
extern float TSYS01ReadTemperature();
extern void MS5837readPressure(float arrayResult[]);
//extern float MS5837readPressure();
extern void PHEZOReadPH(uint8_t buffer[]);
