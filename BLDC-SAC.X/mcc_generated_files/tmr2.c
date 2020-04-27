
/**
  TMR2 Generated Driver API Source File 

  @Company
    Microchip Technology Inc.

  @File Name
    tmr2.c

  @Summary
    This is the generated source file for the TMR2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for TMR2. 
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.75.1
        Device            :  dsPIC33EV256GM102
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.35
        MPLAB             :  MPLAB X v5.05
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

#include <xc.h>
#include "tmr2.h"
#include "pwm.h"
#include "adc1.h"
#include "clock.h"
#define FCY _XTAL_FREQ/2
#include <libpic30.h>
#include <stdio.h>

/**
  Section: Data Type Definitions
*/

/** TMR Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintainence of the hardware instance.

  @Description
    This defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct _TMR_OBJ_STRUCT
{
    /* Timer Elapsed */
    bool                                                    timerElapsed;
    /*Software Counter value*/
    uint8_t                                                 count;

} TMR_OBJ;

static TMR_OBJ tmr2_obj;

/**
  Section: Driver Interface
*/

void TMR2_Initialize (void)
{
    //TMR3 0; 
    TMR3 = 0x00;
    //PR3 1; 
    PR3 = 0x01;
    //TMR2 0; 
    TMR2 = 0x00;
    //Period = 0.0010000036 s; Frequency = 69093750 Hz; PR2 3558; 
    PR2 = 0xDE6;
    //TCKPS 1:1; T32 32 Bit; TON enabled; TSIDL disabled; TCS FOSC/2; TGATE disabled; 
    T2CON = 0x8008;

    
    IFS0bits.T3IF = false;
    IEC0bits.T3IE = true;
	
    tmr2_obj.timerElapsed = false;

}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _T3Interrupt (  )
{
    /* Check if the Timer Interrupt/Status is set */

    //***User Area Begin

    // ticker function call;
    // ticker is 1 -> Callback function gets called everytime this ISR executes
        TMR2_CallBack();

    //***User Area End

    tmr2_obj.count++;
    tmr2_obj.timerElapsed = true;
    IFS0bits.T3IF = false;
}




void TMR2_Period32BitSet( uint32_t value )
{
    /* Update the counter values */
    PR2 = (value & 0x0000FFFF);
    PR3 = ((value & 0xFFFF0000)>>16);
}

uint32_t TMR2_Period32BitGet( void )
{
    uint32_t periodVal = 0xFFFFFFFF;

    /* get the timer period value and return it */
    periodVal = (((uint32_t)PR3 <<16) | PR2);

    return( periodVal );

}

void TMR2_Counter32BitSet( uint32_t value )
{
    /* Update the counter values */
   TMR3HLD = ((value & 0xFFFF0000)>>16);
   TMR2 = (value & 0x0000FFFF);

}

uint32_t TMR2_Counter32BitGet( void )
{
    uint32_t countVal = 0xFFFFFFFF;
    uint16_t countValUpper;
    uint16_t countValLower;

    countValLower = TMR2;
    countValUpper = TMR3HLD;

    /* get the current counter value and return it */
    countVal = (((uint32_t)countValUpper<<16)| countValLower );

    return( countVal );

}

// Performs PID calculations
void __attribute__ ((weak)) TMR2_CallBack(void)
{
    // PID configuration
    double Kp = 1.3642407803804;
    double Ki = 3.38261365385595;
    double Kd = 0.0373242127602274;
    double tSampling = 0.001; // 1 kHz
    double integralMin = -1000;
    double integralMax = 1000;
    double switchOn = 0.671554252;
    double switchOff = 0.327468231;
    uint16_t pwmMax = 0x3FF;
    
    // Get the dial
    ADC1_ChannelSelectSet(ADC1_DIAL);
    ADC1_SamplingStart();
    __delay_us(100);
    uint16_t dial = ADC1_Channel0ConversionResultGet();
    
    // Get the speed
    ADC1_ChannelSelectSet(ADC1_SPEED);
    ADC1_SamplingStart();
    __delay_us(100);
    uint16_t speed = ADC1_Channel0ConversionResultGet();
    
    // Normalize the input signals
    double signalDial = (0.00011455 * dial) + 0.327468230694037;
//    MDC = signalDial * 0x3ff;
    double speedVoltage = 0.001220703125 * speed;
    double signalSpeed = (-0.015504900608 * (speedVoltage * speedVoltage * speedVoltage * speedVoltage)) +
            (-0.02650420896 * (speedVoltage * speedVoltage * speedVoltage)) +
            (0.170362883294 * (speedVoltage * speedVoltage)) +
            (0.1552907996399 * speedVoltage) +
            0.341281524152;
    
    // Calculate the error
    double errorNew = signalDial - signalSpeed;
    
    // PID calculations
    static double errorOld = 0;
    static double integralOld = 0;
    double integralNew = integralOld + ((Ki * tSampling * (errorNew - errorOld)) / 2);
    if ((integralNew > integralMax) || (integralNew < integralMin)) {
        integralNew = integralOld;
    }
    double pidOutput = (Kp * errorNew) + ((Kd * (errorNew - errorOld)) / tSampling) + integralNew;
    integralOld = integralNew;
    errorOld = errorNew;
    
    // Handle the switch on and switch off points of the motor
    if (pidOutput > switchOn) {
        // PID output doesn't change, motor will always spin
    } else if (pidOutput < switchOff) {
        // Turn off PWM, motor can't actually spin
        pidOutput = 0;
    } else if (MDC > 0) {
        // PID output doesn't change, motor already spinning
    } else {
        // Motor can't spin yet, don't turn on PWM
        pidOutput = 0;
    }
    
    // Limit the PID output and set the PWM output value
    if (pidOutput <= 0) {
        MDC = 0;
    } else if (pidOutput > 0.8) {
        MDC = 0x32F;
    } else {
        MDC = pidOutput * pwmMax;
    }
    
    // Logging
    static int iterCounter = 0;
    if (iterCounter > 100) {
        iterCounter = 0;
        printf("e %f pid %f mdc %d\r\n", errorNew, pidOutput, MDC);
    } else {
        iterCounter++;
    }
}

void TMR2_Start( void )
{
    /* Reset the status information */
    tmr2_obj.timerElapsed = false;

    /*Enable the interrupt*/
    IEC0bits.T3IE = true;

    /* Start the Timer */
    T2CONbits.TON = 1;
}

void TMR2_Stop( void )
{
    /* Stop the Timer */
    T2CONbits.TON = false;

    /*Disable the interrupt*/
    IEC0bits.T3IE = false;
}

bool TMR2_GetElapsedThenClear(void)
{
    bool status;
    
    status = tmr2_obj.timerElapsed;

    if(status == true)
    {
        tmr2_obj.timerElapsed = false;
    }
    return status;
}

int TMR2_SoftwareCounterGet(void)
{
    return tmr2_obj.count;
}

void TMR2_SoftwareCounterClear(void)
{
    tmr2_obj.count = 0; 
}

/**
 End of File
*/
