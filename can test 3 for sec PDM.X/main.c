/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
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
    //INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
//    RC7 = 1;
    //int i;
    CIOCONbits.CLKSEL = 1;
    CIOCONbits.ENDRHI = 1;
    
    uint16_t BPr, BPf, SA, BTfr, BTfl, pitot;
    
    double Vref = 5000.0;
    double x = Vref/4096.0;
//    
    while (1)
    {
        // Add your application code
        BPr = ADC_GetConversion(BPr) * x;
        BPf = ADC_GetConversion(BPf) * x;
        SA = ADC_GetConversion(SA) * x;
        BTfr = ADC_GetConversion(BTfr) * x;
        BTfl = ADC_GetConversion(BTfl) * x;
        pitot = ADC_GetConversion(pitot_tube) * x;
        
        uCAN_MSG CAN_MSG_1;
        CAN_MSG_1.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
        CAN_MSG_1.frame.id = 0x635;
        CAN_MSG_1.frame.dlc = 8;
        CAN_MSG_1.frame.data0 = BPr >> 8;
        CAN_MSG_1.frame.data1 = BPr;
        CAN_MSG_1.frame.data2 = BPf >> 8;
        CAN_MSG_1.frame.data3 = BPf;
        CAN_MSG_1.frame.data4 = SA >> 8;
        CAN_MSG_1.frame.data5 = SA;
        CAN_MSG_1.frame.data6 = BTfr >> 8;
        CAN_MSG_1.frame.data7 = BTfr;
        
        CAN_transmit(&CAN_MSG_1);
        
        uCAN_MSG CAN_MSG_2;
        CAN_MSG_2.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
        CAN_MSG_2.frame.id = 0x636;
        CAN_MSG_2.frame.dlc = 4;
        CAN_MSG_2.frame.data0 = BTfl >> 8;
        CAN_MSG_2.frame.data1 = BTfl;
        CAN_MSG_2.frame.data2 = pitot >> 8;
        CAN_MSG_2.frame.data3 = pitot;
        
        CAN_transmit(&CAN_MSG_2);
        //if(!TXB0CONbits.TXREQ||!TXB1CONbits.TXREQ||!TXB2CONbits.TXREQ)  RC7 = 1;
        //if(TXB0CONbits.TXLARB||TXB1CONbits.TXLARB||TXB2CONbits.TXLARB) RC7 = 0;
//            if(num == 1)  RC7 = 1;
//            if(num == 2)  RC7 = 1;
//            if(num == 3)  RC7 = 1;
//        }
//        __delay_ms(100);
//        RC7 = !RC7;
    }
}
/**
 End of File
*/