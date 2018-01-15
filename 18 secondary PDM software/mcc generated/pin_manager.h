/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.15.3
        Device            :  PIC18F46K80
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set spare aliases
#define spare_TRIS               TRISAbits.TRISA0
#define spare_LAT                LATAbits.LATA0
#define spare_PORT               PORTAbits.RA0
#define spare_ANS                ANCON0bits.ANSEL0
#define spare_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define spare_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define spare_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define spare_GetValue()           PORTAbits.RA0
#define spare_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define spare_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define spare_SetAnalogMode()  do { ANCON0bits.ANSEL0 = 1; } while(0)
#define spare_SetDigitalMode() do { ANCON0bits.ANSEL0 = 0; } while(0)

// get/set BT_FL aliases
#define BT_FL_TRIS               TRISAbits.TRISA1
#define BT_FL_LAT                LATAbits.LATA1
#define BT_FL_PORT               PORTAbits.RA1
#define BT_FL_ANS                ANCON0bits.ANSEL1
#define BT_FL_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define BT_FL_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define BT_FL_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define BT_FL_GetValue()           PORTAbits.RA1
#define BT_FL_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define BT_FL_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define BT_FL_SetAnalogMode()  do { ANCON0bits.ANSEL1 = 1; } while(0)
#define BT_FL_SetDigitalMode() do { ANCON0bits.ANSEL1 = 0; } while(0)

// get/set RA2 procedures
#define RA2_SetHigh()    do { LATAbits.LATA2 = 1; } while(0)
#define RA2_SetLow()   do { LATAbits.LATA2 = 0; } while(0)
#define RA2_Toggle()   do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define RA2_GetValue()         PORTAbits.RA2
#define RA2_SetDigitalInput()   do { TRISAbits.TRISA2 = 1; } while(0)
#define RA2_SetDigitalOutput()  do { TRISAbits.TRISA2 = 0; } while(0)
#define RA2_SetAnalogMode() do { ANCON0bits.ANSEL2 = 1; } while(0)
#define RA2_SetDigitalMode()do { ANCON0bits.ANSEL2 = 0; } while(0)

// get/set RA3 procedures
#define RA3_SetHigh()    do { LATAbits.LATA3 = 1; } while(0)
#define RA3_SetLow()   do { LATAbits.LATA3 = 0; } while(0)
#define RA3_Toggle()   do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define RA3_GetValue()         PORTAbits.RA3
#define RA3_SetDigitalInput()   do { TRISAbits.TRISA3 = 1; } while(0)
#define RA3_SetDigitalOutput()  do { TRISAbits.TRISA3 = 0; } while(0)
#define RA3_SetAnalogMode() do { ANCON0bits.ANSEL3 = 1; } while(0)
#define RA3_SetDigitalMode()do { ANCON0bits.ANSEL3 = 0; } while(0)

// get/set spare2 aliases
#define spare2_TRIS               TRISAbits.TRISA5
#define spare2_LAT                LATAbits.LATA5
#define spare2_PORT               PORTAbits.RA5
#define spare2_ANS                ANCON0bits.ANSEL4
#define spare2_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define spare2_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define spare2_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define spare2_GetValue()           PORTAbits.RA5
#define spare2_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define spare2_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define spare2_SetAnalogMode()  do { ANCON0bits.ANSEL4 = 1; } while(0)
#define spare2_SetDigitalMode() do { ANCON0bits.ANSEL4 = 0; } while(0)

// get/set GPS_btn aliases
#define GPS_btn_TRIS               TRISBbits.TRISB0
#define GPS_btn_LAT                LATBbits.LATB0
#define GPS_btn_PORT               PORTBbits.RB0
#define GPS_btn_WPU                WPUBbits.WPUB0
#define GPS_btn_ANS                ANCON1bits.ANSEL10
#define GPS_btn_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define GPS_btn_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define GPS_btn_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define GPS_btn_GetValue()           PORTBbits.RB0
#define GPS_btn_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define GPS_btn_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define GPS_btn_SetPullup()      do { WPUBbits.WPUB0 = 1; } while(0)
#define GPS_btn_ResetPullup()    do { WPUBbits.WPUB0 = 0; } while(0)
#define GPS_btn_SetAnalogMode()  do { ANCON1bits.ANSEL10 = 1; } while(0)
#define GPS_btn_SetDigitalMode() do { ANCON1bits.ANSEL10 = 0; } while(0)

// get/set DRS_btn aliases
#define DRS_btn_TRIS               TRISBbits.TRISB1
#define DRS_btn_LAT                LATBbits.LATB1
#define DRS_btn_PORT               PORTBbits.RB1
#define DRS_btn_WPU                WPUBbits.WPUB1
#define DRS_btn_ANS                ANCON1bits.ANSEL8
#define DRS_btn_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define DRS_btn_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define DRS_btn_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define DRS_btn_GetValue()           PORTBbits.RB1
#define DRS_btn_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define DRS_btn_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define DRS_btn_SetPullup()      do { WPUBbits.WPUB1 = 1; } while(0)
#define DRS_btn_ResetPullup()    do { WPUBbits.WPUB1 = 0; } while(0)
#define DRS_btn_SetAnalogMode()  do { ANCON1bits.ANSEL8 = 1; } while(0)
#define DRS_btn_SetDigitalMode() do { ANCON1bits.ANSEL8 = 0; } while(0)

// get/set RB2 procedures
#define RB2_SetHigh()    do { LATBbits.LATB2 = 1; } while(0)
#define RB2_SetLow()   do { LATBbits.LATB2 = 0; } while(0)
#define RB2_Toggle()   do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define RB2_GetValue()         PORTBbits.RB2
#define RB2_SetDigitalInput()   do { TRISBbits.TRISB2 = 1; } while(0)
#define RB2_SetDigitalOutput()  do { TRISBbits.TRISB2 = 0; } while(0)
#define RB2_SetPullup()     do { WPUBbits.WPUB2 = 1; } while(0)
#define RB2_ResetPullup()   do { WPUBbits.WPUB2 = 0; } while(0)

// get/set RB3 procedures
#define RB3_SetHigh()    do { LATBbits.LATB3 = 1; } while(0)
#define RB3_SetLow()   do { LATBbits.LATB3 = 0; } while(0)
#define RB3_Toggle()   do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define RB3_GetValue()         PORTBbits.RB3
#define RB3_SetDigitalInput()   do { TRISBbits.TRISB3 = 1; } while(0)
#define RB3_SetDigitalOutput()  do { TRISBbits.TRISB3 = 0; } while(0)
#define RB3_SetPullup()     do { WPUBbits.WPUB3 = 1; } while(0)
#define RB3_ResetPullup()   do { WPUBbits.WPUB3 = 0; } while(0)

// get/set RC2 procedures
#define RC2_SetHigh()    do { LATCbits.LATC2 = 1; } while(0)
#define RC2_SetLow()   do { LATCbits.LATC2 = 0; } while(0)
#define RC2_Toggle()   do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define RC2_GetValue()         PORTCbits.RC2
#define RC2_SetDigitalInput()   do { TRISCbits.TRISC2 = 1; } while(0)
#define RC2_SetDigitalOutput()  do { TRISCbits.TRISC2 = 0; } while(0)

// get/set RC3 procedures
#define RC3_SetHigh()    do { LATCbits.LATC3 = 1; } while(0)
#define RC3_SetLow()   do { LATCbits.LATC3 = 0; } while(0)
#define RC3_Toggle()   do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define RC3_GetValue()         PORTCbits.RC3
#define RC3_SetDigitalInput()   do { TRISCbits.TRISC3 = 1; } while(0)
#define RC3_SetDigitalOutput()  do { TRISCbits.TRISC3 = 0; } while(0)

// get/set RC4 procedures
#define RC4_SetHigh()    do { LATCbits.LATC4 = 1; } while(0)
#define RC4_SetLow()   do { LATCbits.LATC4 = 0; } while(0)
#define RC4_Toggle()   do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define RC4_GetValue()         PORTCbits.RC4
#define RC4_SetDigitalInput()   do { TRISCbits.TRISC4 = 1; } while(0)
#define RC4_SetDigitalOutput()  do { TRISCbits.TRISC4 = 0; } while(0)

// get/set autoshift aliases
#define autoshift_TRIS               TRISCbits.TRISC5
#define autoshift_LAT                LATCbits.LATC5
#define autoshift_PORT               PORTCbits.RC5
#define autoshift_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define autoshift_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define autoshift_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define autoshift_GetValue()           PORTCbits.RC5
#define autoshift_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define autoshift_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)

// get/set pitot aliases
#define pitot_TRIS               TRISEbits.TRISE0
#define pitot_LAT                LATEbits.LATE0
#define pitot_PORT               PORTEbits.RE0
#define pitot_ANS                ANCON0bits.ANSEL5
#define pitot_SetHigh()            do { LATEbits.LATE0 = 1; } while(0)
#define pitot_SetLow()             do { LATEbits.LATE0 = 0; } while(0)
#define pitot_Toggle()             do { LATEbits.LATE0 = ~LATEbits.LATE0; } while(0)
#define pitot_GetValue()           PORTEbits.RE0
#define pitot_SetDigitalInput()    do { TRISEbits.TRISE0 = 1; } while(0)
#define pitot_SetDigitalOutput()   do { TRISEbits.TRISE0 = 0; } while(0)
#define pitot_SetAnalogMode()  do { ANCON0bits.ANSEL5 = 1; } while(0)
#define pitot_SetDigitalMode() do { ANCON0bits.ANSEL5 = 0; } while(0)

// get/set BT_FR aliases
#define BT_FR_TRIS               TRISEbits.TRISE1
#define BT_FR_LAT                LATEbits.LATE1
#define BT_FR_PORT               PORTEbits.RE1
#define BT_FR_ANS                ANCON0bits.ANSEL6
#define BT_FR_SetHigh()            do { LATEbits.LATE1 = 1; } while(0)
#define BT_FR_SetLow()             do { LATEbits.LATE1 = 0; } while(0)
#define BT_FR_Toggle()             do { LATEbits.LATE1 = ~LATEbits.LATE1; } while(0)
#define BT_FR_GetValue()           PORTEbits.RE1
#define BT_FR_SetDigitalInput()    do { TRISEbits.TRISE1 = 1; } while(0)
#define BT_FR_SetDigitalOutput()   do { TRISEbits.TRISE1 = 0; } while(0)
#define BT_FR_SetAnalogMode()  do { ANCON0bits.ANSEL6 = 1; } while(0)
#define BT_FR_SetDigitalMode() do { ANCON0bits.ANSEL6 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/