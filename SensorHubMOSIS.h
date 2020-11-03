

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


#define SMALLDELAY                          416666
#define BIGDELAY                            25000000
