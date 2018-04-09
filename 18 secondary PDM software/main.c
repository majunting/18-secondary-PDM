/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.15.1
        Device            :  PIC18F45K80
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"
#include "BNO055.h"

#define BNO055_READ_ADDR 0x51
#define BNO055_WRITE_ADDR 0x50
#define ADC_delay   8
    
void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_RepeatedStart();
void I2C_Master_Stop();
void I2C_Master_Write(uint8_t d);
void I2C_Master_Read(unsigned short a, uint8_t *data);
void BNO055Initialize();

/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    BNO055Initialize();
    
    // edit 3 bits in CIOCON register (CAN IO control)
    CIOCONbits.CLKSEL = 1;  // CAN clk select
    CIOCONbits.ENDRHI = 1;  // enable drive high (CANTX drives VDD when recessive)
    CIOCONbits.CANCAP = 1;
//    CIOCONbits.TX2SRC = 0;  // CANTX2 pin data source (output CANTX)
//    CIOCONbits.TX2EN = 1;   // CANTX pin enable (output CANRX or CAN clk based on TX2SRC)
    
    //variables for CAN transmission
    uint8_t BTFL_H, BTFL_L, BTFR_H, BTFR_L, pitot_H, pitot_L,
            spare_H, spare_L, spare2_H, spare2_L;
    uint8_t linear_accel_x_LSB, linear_accel_x_MSB, linear_accel_y_LSB,
            linear_accel_y_MSB, linear_accel_z_LSB, linear_accel_z_MSB;
    
    // counter to increase interval for ADC as it is read @ 10Hz
    uint8_t count = 0;
    
    double VDD = 5000.0;    // input voltage 5V
    double x = VDD / 4096.0;
 
    adc_result_t ADCResult;
    uint8_t *data;

    
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

//    double VDD = 5000.0;    // input voltage 5V
//    double x = VDD / 4096.0;
// 
//    adc_result_t ADCResult;
//    uint8_t *data;
    
    while (1)
    {        
        if (TMR2_GetTransmit() == true){
            if(count == ADC_delay){
                /** ADC */
                ADCResult = ADC_GetConversion(BT_FL) * x;
                BTFL_H = ADCResult >> 8;
                BTFL_L = ADCResult;
                ADCResult = ADC_GetConversion(BT_FR) * x;
                BTFR_H = ADCResult >> 8;
                BTFR_L = ADCResult;
                ADCResult = ADC_GetConversion(pitot) * x;
                pitot_H = ADCResult >> 8;
                pitot_L = ADCResult;
                ADCResult = ADC_GetConversion(spare) * x;
                spare_H = ADCResult >> 8;
                spare_L = ADCResult;
                ADCResult = ADC_GetConversion (spare2) * x;
                spare2_H = ADCResult >> 8;
                spare2_L = ADCResult;

                uCAN_MSG ADC1;

                ADC1.frame.idType=dSTANDARD_CAN_MSG_ID_2_0B;
                ADC1.frame.id=0x474;
                ADC1.frame.dlc=8;
                ADC1.frame.data0= BTFR_H;
                ADC1.frame.data1= BTFR_L;
                ADC1.frame.data2= pitot_H;
                ADC1.frame.data3= pitot_L;
                ADC1.frame.data4= spare_H;
                ADC1.frame.data5= spare_L;
                ADC1.frame.data6= spare2_H;
                ADC1.frame.data7= spare2_L;

                //CAN_transmit(&ADC1);
                count = 0;
            }
            else    count++;

            /** G sensor */
            //read linear acceleration data for 3 axis
            I2C_Master_Start();
            I2C_Master_Write(BNO055_WRITE_ADDR);
            I2C_Master_Write(BNO055_OPR_MODE_ADDR);
            I2C_Master_Write(OPERATION_MODE_ACCONLY);
            I2C_Master_Stop();

            I2C_Master_Start();
            I2C_Master_Write(BNO055_WRITE_ADDR);
            I2C_Master_Write(BNO055_ACCEL_DATA_X_LSB_ADDR);
            I2C_Master_Stop();

            I2C_Master_Start();         //Start condition
            I2C_Master_Write(BNO055_READ_ADDR);     //7 bit address + Read
            I2C_Master_Read(6, data); //Read + Acknowledge
            I2C_Master_Stop();          //Stop condition

            linear_accel_x_LSB = data[0];
            linear_accel_x_MSB = data[1];
            linear_accel_y_LSB = data[2];
            linear_accel_y_MSB = data[3];
            linear_accel_z_LSB = data[4];
            linear_accel_z_MSB = data[5];

            uCAN_MSG CAN_MESSAGE2;

            CAN_MESSAGE2.frame.idType=dSTANDARD_CAN_MSG_ID_2_0B;
            CAN_MESSAGE2.frame.id=0x470;
            CAN_MESSAGE2.frame.dlc=8;
            CAN_MESSAGE2.frame.data0 = linear_accel_y_MSB;
            CAN_MESSAGE2.frame.data1 = linear_accel_y_LSB;
            CAN_MESSAGE2.frame.data2 = linear_accel_x_MSB;
            CAN_MESSAGE2.frame.data3 = linear_accel_x_LSB;
            CAN_MESSAGE2.frame.data4 = linear_accel_z_MSB;
            CAN_MESSAGE2.frame.data5 = linear_accel_z_LSB;
            CAN_MESSAGE2.frame.data6 = BTFL_H;
            CAN_MESSAGE2.frame.data7 = BTFL_L;

            CAN_transmit ( &CAN_MESSAGE2 );
            TMR2_ClearTransmit(); 
//            INTERRUPT_GlobalInterruptEnable();
            INTERRUPT_PeripheralInterruptEnable();
        }   
    }
}

void I2C_Master_Wait()
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
  I2C_Master_Wait();
  SEN = 1;
}

void I2C_Master_RepeatedStart()
{
  I2C_Master_Wait();
  RSEN = 1;
}

void I2C_Master_Stop()
{
  I2C_Master_Wait();
  PEN = 1;
}

void I2C_Master_Write(uint8_t d)
{
  I2C_Master_Wait();
  SSPBUF = d;
}

void I2C_Master_Read(unsigned short a, uint8_t *data)
{
  unsigned short temp;
  uint8_t i = 0;
  while (i < a){
    I2C_Master_Wait();
    RCEN = 1;
    I2C_Master_Wait();
    data[i] = SSPBUF;
    i++;
    I2C_Master_Wait();
    ACKDT = (i < a)?0:1;
    ACKEN = 1;
  }
}

void BNO055Initialize()
{
    uint8_t *data;
    data[0] = 0;
    
    // set to config mode
    I2C_Master_Start();
    I2C_Master_Write(BNO055_WRITE_ADDR);
    I2C_Master_Write(BNO055_OPR_MODE_ADDR);
    I2C_Master_Write(OPERATION_MODE_CONFIG);
    I2C_Master_Stop();
    
    // reset BNO055
    I2C_Master_Start();
    I2C_Master_Write(BNO055_WRITE_ADDR);
    I2C_Master_Write(BNO055_SYS_TRIGGER_ADDR);
    I2C_Master_Write(0x20);
    I2C_Master_Stop();
    
    __delay_ms(20);
    
    // set to normal power mode
    I2C_Master_Start();
    I2C_Master_Write(BNO055_WRITE_ADDR);
    I2C_Master_Write(BNO055_PWR_MODE_ADDR);
    I2C_Master_Write(POWER_MODE_NORMAL);
    I2C_Master_Stop();
    
    __delay_ms(20);
    
    // set to page 0
    I2C_Master_Start();
    I2C_Master_Write(BNO055_WRITE_ADDR);
    I2C_Master_Write(BNO055_PAGE_ID_ADDR);
    I2C_Master_Write(0x0);
    I2C_Master_Stop();
    
    // set unit for acceleration to G
    I2C_Master_Start();
    I2C_Master_Write(BNO055_WRITE_ADDR);
    I2C_Master_Write(BNO055_UNIT_SEL_ADDR);
    I2C_Master_Write(0x1);
    I2C_Master_Stop();
    
    // clear sys trigger register
    I2C_Master_Start();
    I2C_Master_Write(BNO055_WRITE_ADDR);
    I2C_Master_Write(BNO055_SYS_TRIGGER_ADDR);
    I2C_Master_Write(0x00);
    I2C_Master_Stop();
    
    __delay_ms(20);
}
/**
 End of File
*/
