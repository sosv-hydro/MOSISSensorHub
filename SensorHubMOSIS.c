/**
 *
 * Code for Sensor Hub module of the MOSIS project
 *
 */

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "SensorHubMOSIS.h"

//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - I2C0 peripheral
//! - GPIO Port B peripheral (for I2C0 pins)
//! - I2C0SCL - PB2
//! - I2C0SDA - PB3

const float Pa = 100.0f;
const float bar = 0.001f;
const float mbar = 1.0f;

const uint8_t MS5837_30BA = 0;


//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, 16000000);
}

/// receive n bytes from I2c
void I2CReceiveN(uint8_t slave_addr, uint8_t reg, uint8_t numElem, uint8_t buff[], int delay){

    uint32_t i;

    /// Sets Slave adress on master buffer, along with the WRITE bit
    HWREG(I2C0_BASE + I2C_O_MSA) = (slave_addr << 1) | false;
    //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    /// Inserts Data into Master buffer
    HWREG(I2C0_BASE + I2C_O_MDR) = reg;

    /// sends START (BIT1) and RUN (BIT0)
    /// from idle to transmit mode
    HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_SINGLE_SEND;
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // delay
    SysCtlDelay(delay);

    //specify that we are going to read from slave device
    HWREG(I2C0_BASE + I2C_O_MSA) = (slave_addr << 1) | true;
    //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    /// Repeated START condition followed by RECEIVE    (master remains in Master Receive state). pag. 1024
    /// repeated start
    HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_START;
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    SysCtlDelay(SMALLDELAY);
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    /// Get first byte from read command
    //buff[0] = (HWREG(I2C0_BASE + I2C_O_MDR));
    buff[0] = I2CMasterDataGet(I2C0_BASE);

    if(numElem > 1){
        for (i = 1; i < numElem - 1; i++){
                /// ReceivE operation (master remains in Master Receive state).
                HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
                //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);I2C_MASTER_CMD_BURST_RECEIVE_CONT
                //wait for MCU to finish transaction

                SysCtlDelay(SMALLDELAY);
                while(I2CMasterBusy(I2C0_BASE));

                buff[i] = I2CMasterDataGet(I2C0_BASE);
            //    UARTprintf("Received: '%c'\n", buff[i]);
            }

            /// ultimo elemento
            ///RECEIVE followed by STOP condition (master goes to Idle state).
            HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
            //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            //wait for MCU to finish transaction
            SysCtlDelay(SMALLDELAY);
            while(I2CMasterBusy(I2C0_BASE));
            buff[numElem - 1] = I2CMasterDataGet(I2C0_BASE);

           // UARTprintf("Received: '%c'\n",  buff[numElem - 1]);
    }

}


void I2CSendN(uint8_t slave_addr, uint8_t reg[], uint8_t numToSend){
    uint8_t i;

    /// Sets Slave adress on master buffer, along with the WRITE bit
    HWREG(I2C0_BASE + I2C_O_MSA) = (slave_addr << 1) | false;
    //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    /// Inserts Data into Master buffer
    HWREG(I2C0_BASE + I2C_O_MDR) = reg[0];

    /// sends START (BIT1) and RUN (BIT0)
    /// from idle to transmit mode
    HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_START;
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //Small delay
    SysCtlDelay(SMALLDELAY);
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    for (i = 1; i < numToSend - 1; i++){
        /// Inserts Data into Master buffer
           HWREG(I2C0_BASE + I2C_O_MDR) = reg[i];

           HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_CONT;
           //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

           //Small delay
           SysCtlDelay(SMALLDELAY);
           //wait for MCU to finish transaction
           while(I2CMasterBusy(I2C0_BASE));
    }
    /// Inserts Data into Master buffer
    HWREG(I2C0_BASE + I2C_O_MDR) = reg[i];

    HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_FINISH;
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //Small delay
    SysCtlDelay(SMALLDELAY);
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
}

/// receive n bytes from I2c
void I2CSendNReceiveN(uint8_t slave_addr, uint8_t reg[], uint8_t numToSend, uint8_t numElem, uint8_t buff[], int delay){

    uint32_t i;

    //Send command with N arguments to retrieve arguments
    I2CSendN(slave_addr, reg, numToSend);

    // Finished sending

    SysCtlDelay(delay);

    //specify that we are going to read from slave device
    HWREG(I2C0_BASE + I2C_O_MSA) = (slave_addr << 1) | true;
    //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    /// Repeated START condition followed by RECEIVE    (master remains in Master Receive state). pag. 1024
    /// repeated start
    HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_START;
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    SysCtlDelay(SMALLDELAY);
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    /// Get first byte from read command
    //buff[0] = (HWREG(I2C0_BASE + I2C_O_MDR));
    buff[0] = I2CMasterDataGet(I2C0_BASE);

    if(numElem > 1){
        for (i = 1; i < numElem - 1; i++){
            /// ReceivE operation (master remains in Master Receive state).
            HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
            //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);I2C_MASTER_CMD_BURST_RECEIVE_CONT
            //wait for MCU to finish transaction
            SysCtlDelay(SMALLDELAY);
            while(I2CMasterBusy(I2C0_BASE));

            buff[i] = I2CMasterDataGet(I2C0_BASE);
        //    UARTprintf("Received: '%c'\n", buff[i]);
        }

        /// ultimo elemento
        ///RECEIVE followed by STOP condition (master goes to Idle state).
        HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
        //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        //wait for MCU to finish transaction
        SysCtlDelay(SMALLDELAY);
        while(I2CMasterBusy(I2C0_BASE));
        buff[numElem - 1] = I2CMasterDataGet(I2C0_BASE);

       // UARTprintf("Received: '%c'\n",  buff[numElem - 1]);
    }
}


//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t args)
{
   /// Sets Slave adress on master buffer, along with the WRITE bit
   HWREG(I2C0_BASE + I2C_O_MSA) = (slave_addr << 1) | false;
   //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

   /// Inserts Data into Master buffer
   HWREG(I2C0_BASE + I2C_O_MDR) = args;

   /// sends START (BIT1) and RUN (BIT0)
   /// from idle to transmit mode
   HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_SINGLE_SEND;
   //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

   //wait for MCU to finish transaction
   while(I2CMasterBusy(I2C0_BASE));
}

void initI2C0(void)
{
   SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

   //reset I2C module
   SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

   //enable GPIO peripheral that contains I2C
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

   // Configure the pin muxing for I2C0 functions on port B2 and B3.
   GPIOPinConfigure(GPIO_PB2_I2C0SCL);
   GPIOPinConfigure(GPIO_PB3_I2C0SDA);

   // Select the I2C function for these pins.
   GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
   GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

   /***
    *   Setting up module clock
    * **/

   uint32_t ui32SysClock;
   ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_XTAL_25MHZ |
           SYSCTL_CFG_VCO_480), 100000000);

   //
   // Stop the Clock, Reset and Enable I2C Module
   // in Master Function
   //
   SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C0);
   SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

   //
   // Wait for the Peripheral to be ready for programming
   //
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));


   // Enable and initialize the I2C0 master module.  Use the system clock for
   // the I2C0 module.  The last parameter sets the I2C data transfer rate.
   // If false the data rate is set to 100kbps and if true the data rate will
   // be set to 400kbps.
   I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, false);

   //clear I2C FIFOs
   HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void readCalibrationValues(uint16_t PROM[], int n, uint8_t slave_address, uint8_t read_PROM_cmd){

    uint8_t fillbuff[2];
    uint8_t i;
    uint16_t temp;
    uint8_t buffindex;

    for(buffindex = 0; buffindex < 2; buffindex++)
        {
            fillbuff[buffindex] = 0;
        }

// Read calibration values
    for ( i = 0 ; i < n ; i++ ) {

        I2CReceiveN(slave_address, read_PROM_cmd+i*2, 2, fillbuff, SMALLDELAY);

        temp = fillbuff[0] << 8 | fillbuff[1];

        PROM[i] = fillbuff[0] << 8 | fillbuff[1];

        for(buffindex = 0; buffindex < 2; buffindex++){
                   fillbuff[buffindex] = 0;
          }
    }
}

uint32_t readADCValueTSYS01(){
    uint8_t fillbuff[3];

   uint32_t temp;
   uint8_t buffindex;

   for(buffindex = 0; buffindex < 3; buffindex++)
       {
           fillbuff[buffindex] = 0;
       }


   I2CReceiveN(TSYS01_ADDR, TSYS01_ADC_READ,3, fillbuff, SMALLDELAY);

   temp = fillbuff[0] << 8 | fillbuff[1];

   temp = temp << 8 | fillbuff[2];

   return temp;
}

/**
 * Calculate TSYS01 temperature according to datasheet
 */
float calculateTemperatureTSYS01(int sensorRead, uint16_t PROM[]){
    //int adc;
    uint16_t adc = sensorRead/256;

    float calculation;
    calculation = (-2) * PROM[1] / 1000000000000000000000.0 * pow(adc,4)+
            4 * PROM[2] / 10000000000000000.0 * pow(adc,3) +
          (-2) * PROM[3] / 100000000000.0 * pow(adc,2) +
            1 * PROM[4] / 1000000.0 * adc +
          (-1.5) * PROM[5] / 100 ;

    return calculation;
}

/**
 * Fletcher 16 algorithm to verify checksum
 */
uint16_t Fletcher16( uint16_t PROM[])
{
   int index;
   int i = 0;
   int count = 14;
   uint8_t chunks[14];
   uint16_t lower;
   uint16_t upper;

   for(index = 1; index < 8 ; index++){
      lower = (PROM[index] &0x00FF);
      upper = (PROM[index] &0xFF00) >> 8;

      chunks[i] = upper;
      chunks[i+1] = lower;
      i+=2;
   }


   uint16_t sum1 = 0;
   uint16_t sum2 = 0;

   for ( index = 0; index < count; ++index )
   {
      sum1 = (sum1 + chunks[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }

   return (sum2 << 8) | sum1;
}

void TSYS01ReadTemperatureProcess(){

     I2CSend(TSYS01_ADDR, TSYS01_RESET);

     SysCtlDelay(416666);

     // array to hold the PROM values for calibration
     uint16_t PROM[8];

     readCalibrationValues(PROM, 8, TSYS01_ADDR, TSYS01_PROM_READ);

     uint16_t checksumResult;
     checksumResult = Fletcher16(PROM);

     /** Send the command to initiate the conversion sequence.
      * Has to be done before reading the sensor ADC value, else
      * it will return 0
      */

     I2CSend(TSYS01_ADDR, TSYS01_ADC_TEMP_CONV);

     // delay for the conversion sequence
     SysCtlDelay(416666);

     uint32_t sensorRead;

     // ADC value from sensor
     sensorRead = readADCValueTSYS01();

     //SO FAR THIS IS WORKING, BUT NEED TO SEE IF VALUES ARE CORRECT

     float temperature;
     temperature = calculateTemperatureTSYS01((int)sensorRead, PROM);
}


uint8_t crc4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;

    uint8_t i;
    uint8_t n_bit;
    for (i = 0 ; i < 16; i++ ) {
        if ( i%2 == 1 ) {
            n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
        }



        for (n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
            if ( n_rem & 0x8000 ) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = ((n_rem >> 12) & 0x000F);

    return n_rem ^ 0x00;
}

bool sensorinitMS5837(uint16_t C[]) {

    // Wait for reset to complete
    SysCtlDelay(416666);

    // Read calibration values and CRC
 //   readCalibrationValues(C, 7, MS5837_ADDR, MS5837_PROM_READ);

    // Verify that data is correct with CRC
    uint8_t crcRead = C[0] >> 12;
    uint8_t crcCalculated = crc4(C);

    if ( crcCalculated == crcRead ) {
        return true; // Initialization success
    }

    return false; // CRC fail
}

uint32_t readADCValueMS5837(uint8_t slave_addr, uint8_t reg, uint8_t numElem){
   uint8_t fillbuff[3];

   uint32_t temp;
   uint8_t buffindex;

   for(buffindex = 0; buffindex < 3; buffindex++)
       {
           fillbuff[buffindex] = 0;
       }


   I2CReceiveN(slave_addr, reg,numElem, fillbuff,SMALLDELAY);

   temp = fillbuff[0] << 8 | fillbuff[1];

   temp = temp << 8 | fillbuff[2];

   return temp;
}

uint32_t readSensorDigitalPressureMS5837() {
    // Request D1 conversion
    I2CSend(MS5837_ADDR, MS5837_CONVERT_D1_1024);

    SysCtlDelay(4166666*2); // Max conversion time per datasheet

    // Read D1 value after conversion
    uint32_t D1;
    D1 = readADCValueMS5837(MS5837_ADDR,MS5837_ADC_READ,3);

    return D1;
}

uint32_t readSensorDigitalTemperatureMS5837() {
    // Request D2 conversion
    I2CSend(MS5837_ADDR, MS5837_CONVERT_D2_1024);

    SysCtlDelay(4166666*2); // Max conversion time per datasheet

    // Read D2 value after conversion
    uint32_t D2;
    D2 = readADCValueMS5837(MS5837_ADDR,MS5837_ADC_READ,3);

    return D2;
}

void calculateMS5837(uint32_t D1, uint32_t D2, uint16_t C[], int32_t result[]) {
    // Given C1-C6 and D1, D2, calculated TEMP and P
    // Do conversion first and then second order temp compensation

    int32_t dT = 0;
    int32_t TEMP = 0;
    int32_t P = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int32_t SENSi = 0;
    int32_t OFFi = 0;
    int32_t Ti = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    // Terms called
    dT = D2-((uint32_t)C[5])*256l;

    SENS = ((int64_t)C[1])*32768l+(((int64_t)C[3])*dT)/256l;
    OFF = ((int64_t)C[2])*65536l+(((int64_t)C[4])*dT)/128l;
    P = (D1*SENS/(2097152l)-OFF)/(8192l);

    // Temp conversion
    TEMP = 2000l+((int64_t)dT)*C[6]/8388608LL;

    //Second order compensation
    if((TEMP/100)<20){         //Low temp
        Ti = (3*((int64_t)dT)*((int64_t)dT))/(8589934592LL);
        OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
        SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
        if((TEMP/100)<-15){    //Very low temp
            OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
            SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
        }
    }
    else if((TEMP/100)>=20){    //High temp
        Ti = 2*(dT*dT)/(137438953472LL);
        OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
        SENSi = 0;
    }

    OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
    SENS2 = SENS-SENSi;

    TEMP = (TEMP-Ti);

    P = (((D1*SENS2)/2097152l-OFF2)/8192l);

    result[0] = TEMP;
    result[1] = P;
}

float pressureCalc(float conversion, int32_t P) {
   return P*conversion/10.0f;
}

float temperature(int32_t TEMP) {
    return TEMP/100.0f;
}

float depthCalc( float fluidDensity, int32_t P ) {
    return (pressureCalc(Pa,P)-101300)/(fluidDensity*9.80665);
}

float altitudeCalc( int32_t P) {
    return (1-pow((pressureCalc(1,P)/1013.25),.190284))*145366.45*.3048;
}

void MS5837readPressureProcess(uint16_t C[], float fluidDensity){
    // array to hold the PROM values for calibration
    // uint16_t C[7];
    // float fluidDensity = 1029; //1029 for sea water and 997 for freshwater (kg/m^3)

      uint32_t D1;
      D1 = readSensorDigitalPressureMS5837();

      uint32_t D2;
      D2 =readSensorDigitalTemperatureMS5837();

      int32_t result[2];
      calculateMS5837(D1, D2, C, result); // result of this will be an array with 0: TEMP, 1: Pressure


      float press = pressureCalc(1, result[1]);
      UARTprintf("Pressure: '%d'\n",  (int32_t)press);

      float temp = temperature(result[0]);
      UARTprintf("temperature: '%d'\n",  (int32_t)temp);

      float depth = depthCalc(fluidDensity, result[1]);
      UARTprintf("depth: 0.'%d'\n",  depth*100);

      float altitude = altitudeCalc(result[1]);
      UARTprintf("altitude: '%d'\n",  (int32_t)altitude);

}

void readMS5837PressureSensor(){
    uint16_t C[7];

   // reset the device as per the data sheet
   I2CSend(MS5837_ADDR, MS5837_RESET);

   readCalibrationValues(C, 7, MS5837_ADDR, MS5837_PROM_READ);
   float fluidDensity = 1029; //1029 for sea water and 997 for freshwater (kg/m^3)
   if(sensorinitMS5837(C) == true){
       MS5837readPressureProcess(C,fluidDensity);
   } else{
       UARTprintf("Sensor was not properly initialized. Try again");
   }
}

void PHEZOStatus(){
    uint8_t buffer[18];

    // The command status
    uint8_t status[6] = {0x73, 0x74, 0x61, 0x74, 0x75, 0x73};
    uint8_t statusCode;

    I2CSendNReceiveN(PHEZO_ADDR, status,6, 18, buffer, BIGDELAY);

    statusCode = buffer[0];
}

uint8_t PHEZOSendCalibrationCmd(uint8_t cal, uint8_t calMode){
    uint8_t statusCode;
    uint8_t buffer[1];

    if(calMode == 0){
        // Command: " cal,low,n "
        uint8_t LowCalibration[9] = {0x63, 0x61, 0x6C, 0x2C, 0x6C, 0x6F, 0x77, 0x2C, cal};
        I2CSendNReceiveN(PHEZO_ADDR, LowCalibration, 9, 1, buffer, BIGDELAY);
    }
    else if(calMode == 1){
        // Command: " cal,mid,n "
        uint8_t MidCalibration[9] = {0x63, 0x61, 0x6C, 0x2C, 0x6D, 0x69, 0x64, 0x2C, cal};
        I2CSendNReceiveN(PHEZO_ADDR, MidCalibration, 9, 1, buffer, BIGDELAY);
    }
    else if(calMode == 2){
        // Command: " cal,high,n "
        uint8_t calibration[10] = {0x63, 0x61, 0x6C, 0x2C, 0x68, 0x69, 0x67, 0x68, 0x2C, cal};
        I2CSendNReceiveN(PHEZO_ADDR, calibration, 10, 1, buffer, BIGDELAY);
    }
    else{
        // Command: " cal,clear "
        uint8_t calibration[9] = {0x63, 0x61, 0x6C, 0x2C, 0x63, 0x6C, 0x65, 0x61, 0x72};
        I2CSendNReceiveN(PHEZO_ADDR, calibration, 9, 1, buffer, BIGDELAY);
    }

    statusCode = buffer[0];
    return statusCode;
}

void PHEZOReadPH(){
    uint8_t readNum;
    readNum = 10;
    uint8_t buffer[10];
    uint8_t i;
    uint8_t j;
    uint8_t statusCode;

    for (i =0; i< readNum; i++){
        I2CReceiveN(PHEZO_ADDR, PHEZO_READ, readNum, buffer, BIGDELAY);

        statusCode = buffer[0];

        if(statusCode == 1){
            UARTprintf("read: '%s'\n",  buffer);
        }
        for(j = 0; j< readNum; j++){
            buffer[j] = 0;
        }
        SysCtlDelay(1000000);
    }
}


void main(){
    InitConsole();

    initI2C0();

  //  uint16_t PROM[8];
 //   // reset the device as per the data sheet
 //   I2CSend(TSYS01_ADDR, 1, TSYS01_RESET);
   // readCalibrationValues(PROM, 8, TSYS01_ADDR, TSYS01_PROM_READ);


    //TSYS01ReadTemperatureProcess();

/*
 * Begin process for Pressure sensor
 */
  //  readMS5837PressureSensor();


    /*
     * Read PH sensor
     */
    PHEZOSendCalibrationCmd(4,0);


    return(0);
}
