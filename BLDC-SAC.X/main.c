/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system intialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.75.1
        Device            :  dsPIC33EV256GM102
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.35
        MPLAB 	          :  MPLAB X v5.05
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

/**
  Section: Included Files
*/
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pwm.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/tmr2.h"
#include "mcc_generated_files/adc1.h"
#include "mcc_generated_files/ecan1.h"
#include "mcc_generated_files/clock.h"

/*
                         Main application
 */
//#define FCY _XTAL_FREQ / 2
//#define BAUDRATE 115200
//#define BRGVAL ((FCY / BAUDRATE) / 16) - 1

void initUART() {
    // Reset UART
    U1MODE = 0;
//    U1BRG = BRGVAL;
    
    // Pin connections
    RPINR18bits.U1RXR = 38;
//    RPOR0bits.RP20R = 1;
    RPOR1bits.RP37R = 1;
    
    // Enable receive interrupt
    IPC2bits.U1RXIP = 0x01;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    
    // Turn on UART
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
}

int main(void)
{
    CNPDBbits.CNPDB4 = 1;
    // initialize the device
    SYSTEM_Initialize();
//    initUART();
    printf("start\r\n");
    
    // Set the status LEDs
    Stat1_SetLow();
    Stat2_SetHigh();
    Stat3_SetHigh();
    
    // Start the PWM
    MDC = 0;
    PWM_FaultInterruptStatusClear(PWM_GENERATOR_1);
    PWM_FaultInterruptStatusClear(PWM_GENERATOR_2);
    PWM_FaultInterruptStatusClear(PWM_GENERATOR_3);
    
    // Track the rotation of the motor
    int graycode = 0;
    uint16_t h1, h2, h3;
    
    // Track the RPS of the motor
    bool first = false;
    double timerPeriod = 1000; // ns
    double desiredRPS = 100;
    
    // Initialize CAN
//    uCAN1_MSG msg;
//    ECAN1_ReceiveEnable();
    
    while (1)
    {
        // Read any available CAN message
//        bool received = ECAN1_receive(&msg);
//        if (received) {
//            desiredRPS = msg.frame.data0;
//        }
        
        // Read the hall effect sensor
        h1 = Hall1_GetValue();
        h2 = Hall2_GetValue();
        h3 = Hall3_GetValue();
        graycode = (h3 << 2) | (h2 << 1) | h1;
        
        // Enable the PWM lines based on the position of the motor
        switch (graycode) {
            case 1:
                // Recalculate PWM duty cycle to run at a desired speed
//                if (first) {
//                    first = false;
//                    
//                    // Set the speed based on the time taken to loop once
//                    double period = TMR2_SoftwareCounterGet();
//                    TMR2_SoftwareCounterClear();
//                    // Twice as fast due to two cycles of the hall effect sensor per revolution
//                    double calculatedRPS = 5000.0 / period;
//                    if (calculatedRPS < 1) {
//                        calculatedRPS = desiredRPS;
//                    }
//                    double differencePercent = ((desiredRPS * 1.0) / calculatedRPS) - 1.0;
//                    uint32_t newValue = MDC + ((MDC * 0.01) * differencePercent);
//                    if (newValue > 0x37F) {
//                        newValue = 0x2FF;
//                    } else if (newValue < 0x0FF) {
//                        newValue = 0x0FF;
//                    }
//                    MDC = newValue; 
//                }
                Stat2_SetLow();
                Stat3_SetHigh();
                PWM_OverrideHighEnable(PWM_GENERATOR_1); // Yellow float
                PWM_OverrideLowEnable(PWM_GENERATOR_1);
                PWM_OverrideHighEnable(PWM_GENERATOR_2); // Green low
                PWM_OverrideLowDisable(PWM_GENERATOR_2);
                PWM_OverrideHighDisable(PWM_GENERATOR_3); // Blue high
                PWM_OverrideLowEnable(PWM_GENERATOR_3);
                break;
            case 5:
                Stat2_SetHigh();
                Stat3_SetHigh();
                PWM_OverrideHighEnable(PWM_GENERATOR_1); // Yellow low
                PWM_OverrideLowDisable(PWM_GENERATOR_1);
                PWM_OverrideHighEnable(PWM_GENERATOR_2); // Green float
                PWM_OverrideLowEnable(PWM_GENERATOR_2);
                PWM_OverrideHighDisable(PWM_GENERATOR_3); // Blue high
                PWM_OverrideLowEnable(PWM_GENERATOR_3);
                break;
            case 4:
                Stat2_SetHigh();
                Stat3_SetHigh();
                PWM_OverrideHighEnable(PWM_GENERATOR_1); // Yellow low
                PWM_OverrideLowDisable(PWM_GENERATOR_1);
                PWM_OverrideHighDisable(PWM_GENERATOR_2); // Green high
                PWM_OverrideLowEnable(PWM_GENERATOR_2);
                PWM_OverrideHighEnable(PWM_GENERATOR_3); // Blue float
                PWM_OverrideLowEnable(PWM_GENERATOR_3);
                break;
            case 6:
                first = true;
//                ADC1_SamplingStop();
                Stat2_SetHigh();
                Stat3_SetHigh();
                PWM_OverrideHighEnable(PWM_GENERATOR_1); // Yellow float
                PWM_OverrideLowEnable(PWM_GENERATOR_1);
                PWM_OverrideHighDisable(PWM_GENERATOR_2); // Green high
                PWM_OverrideLowEnable(PWM_GENERATOR_2);
                PWM_OverrideHighEnable(PWM_GENERATOR_3); // Blue low
                PWM_OverrideLowDisable(PWM_GENERATOR_3);
                break;
            case 2:
                Stat2_SetHigh();
                Stat3_SetHigh();
                PWM_OverrideHighDisable(PWM_GENERATOR_1); // Yellow high
                PWM_OverrideLowEnable(PWM_GENERATOR_1);
                PWM_OverrideHighEnable(PWM_GENERATOR_2); // Green float
                PWM_OverrideLowEnable(PWM_GENERATOR_2);
                PWM_OverrideHighEnable(PWM_GENERATOR_3); // Blue low
                PWM_OverrideLowDisable(PWM_GENERATOR_3);
                break;
            case 3:
                Stat2_SetHigh();
                Stat3_SetHigh();
                PWM_OverrideHighDisable(PWM_GENERATOR_1); // Yellow high
                PWM_OverrideLowEnable(PWM_GENERATOR_1);
                PWM_OverrideHighEnable(PWM_GENERATOR_2); // Green low
                PWM_OverrideLowDisable(PWM_GENERATOR_2);
                PWM_OverrideHighEnable(PWM_GENERATOR_3); // Blue float
                PWM_OverrideLowEnable(PWM_GENERATOR_3);
                break;
            default:
                first = false;
                MDC = 0x2FF;
                Stat2_SetHigh();
                Stat3_SetLow();
                PWM_OverrideHighEnable(PWM_GENERATOR_1); // All float
                PWM_OverrideLowEnable(PWM_GENERATOR_1);
                PWM_OverrideHighEnable(PWM_GENERATOR_2);
                PWM_OverrideLowEnable(PWM_GENERATOR_2);
                PWM_OverrideHighEnable(PWM_GENERATOR_3);
                PWM_OverrideLowEnable(PWM_GENERATOR_3);
        }
    }
    return 1; 
}
/**
 End of File
*/

