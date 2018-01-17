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

#define BNO055_MAX_RETRY 50

/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
//    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
//    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    // edit 4 bits in CIOCON register (CAN IO control)
    CIOCONbits.CLKSEL = 1;  // CAN clk select
    CIOCONbits.ENDRHI = 1;  // enable drive high (CANTX drives VDD when recessive)
    CIOCONbits.TX2SRC = 0;  // CANTX2 pin data source (output CANTX)
    CIOCONbits.TX2EN = 1;   // CANTX pin enable (output CANRX or CAN clk based on TX2SRC)
    
    double VDD = 5000.0;    // input voltage 5V
    double x = VDD / 4096.0;
    
    uint8_t BTFL_H, BTFL_L, BTFR_H, BTFR_L, pitot_H, pitot_L,
            spare_H, spare_L, spare2_H, spare2_L;
    adc_result_t ADCResult;
    uint8_t linear_accel_x_LSB = 0xEE, linear_accel_x_MSB = 0xFF, linear_accel_y_LSB = 0x00,
            linear_accel_y_MSB = 0x00, linear_accel_z_LSB = 0x00, linear_accel_z_MSB = 0x00;
    uint8_t *writeBuffer;
    uint8_t *data;
    uint8_t BNO055_address = BNO055_Initialize();
    uint16_t timeOut = 0;
    bool complete = false;
    bool timeOUT = false;
    bool fail = false;
    I2C_MESSAGE_STATUS flag = I2C_MESSAGE_PENDING;
    uint8_t error;
    
    while (1)
    {        
        // Add your application code
        
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
        ADCResult = ADC_GetConversion ( spare2 ) * x;
        spare2_H = ADCResult >> 8;
        spare2_L = ADCResult;
        
        uCAN_MSG ADC1;
        ADC1.frame.idType=dSTANDARD_CAN_MSG_ID_2_0B;
        ADC1.frame.id=0x634;
        ADC1.frame.dlc=8;
        ADC1.frame.data0=BTFL_H;
        ADC1.frame.data1=BTFL_L;
        ADC1.frame.data2=BTFR_H;
        ADC1.frame.data3=BTFR_L;
        ADC1.frame.data4=pitot_H;
        ADC1.frame.data5=pitot_L;
        ADC1.frame.data6=spare_H;
        ADC1.frame.data7=spare_L;
        
        CAN_transmit(&ADC1);
        
//        /** G sensor */
//        //read linear acceleration data for 3 axis
//        while (flag != I2C_MESSAGE_FAIL && timeOut < BNO055_MAX_RETRY){
            while ( I2C_MasterQueueIsFull() == true );
            writeBuffer[0] = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
            I2C_MasterWrite (writeBuffer, 1, BNO055_address, &flag);
//            while (flag == I2C_MESSAGE_PENDING);
//            if (flag == I2C_MESSAGE_COMPLETE)   timeOut = BNO055_MAX_RETRY;
//            timeOut++;
//        }
//        timeOut = 0;
//        
        while (I2C_MasterQueueIsFull() == true);
        flag = I2C_MESSAGE_PENDING;
        I2C_MasterRead (data, 6, BNO055_address, &flag);
//        while (flag == I2C_MESSAGE_PENDING);
        
        linear_accel_x_LSB = data[0];
        linear_accel_x_MSB = data[1];
        linear_accel_y_LSB = data[2];
        linear_accel_y_MSB = data[3];
        linear_accel_z_LSB = data[4];
        linear_accel_z_MSB = data[5];
        
        uCAN_MSG CAN_MESSAGE2;
        
        CAN_MESSAGE2.frame.idType=dSTANDARD_CAN_MSG_ID_2_0B;
        CAN_MESSAGE2.frame.id=0x635;
        CAN_MESSAGE2.frame.dlc=8;
        CAN_MESSAGE2.frame.data0 = spare2_H;
        CAN_MESSAGE2.frame.data1 = spare2_L;
        CAN_MESSAGE2.frame.data2 = linear_accel_x_LSB;
        CAN_MESSAGE2.frame.data3 = linear_accel_x_MSB;
        CAN_MESSAGE2.frame.data4 = linear_accel_y_LSB;
        CAN_MESSAGE2.frame.data5 = linear_accel_y_MSB;
        CAN_MESSAGE2.frame.data6 = linear_accel_z_LSB;
        CAN_MESSAGE2.frame.data7 = linear_accel_z_MSB;
         
        CAN_transmit ( &CAN_MESSAGE2 );
        
        linear_accel_y_LSB += 0x1;
        linear_accel_y_MSB += 0x1;
        linear_accel_z_LSB += 0x1;
        linear_accel_z_MSB += 0x1;
    }
}
/**
 End of File
*/