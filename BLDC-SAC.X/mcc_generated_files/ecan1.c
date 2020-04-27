/**
  ECAN1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    ecan1.c

  @Summary
    This is the generated driver implementation file for the ECAN1 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for ECAN1.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.75.1
        Device            :  dsPIC33EV256GM102
        Driver Version    :  1.00
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

#include <xc.h>
#include "ecan1.h"
#include "dma.h"

#define ECAN1_TX_DMA_CHANNEL DMA_CHANNEL_0
#define ECAN1_RX_DMA_CHANNEL DMA_CHANNEL_1

/* Valid options are 4, 6, 8, 12, 16, 24, or 32. */
#define CAN1_MESSAGE_BUFFERS         32

#define CAN1_TX_BUFFER_COUNT 1

/******************************************************************************/

/******************************************************************************/
/* Private type definitions                                               */
/******************************************************************************/
typedef struct __attribute__((packed))
{
    unsigned priority                   :2;
    unsigned remote_transmit_enable     :1;
    unsigned send_request               :1;
    unsigned error                      :1;
    unsigned lost_arbitration           :1;
    unsigned message_aborted            :1;
    unsigned transmit_enabled           :1;
} CAN1_TX_CONTROLS;

/******************************************************************************/
/* Private variable definitions                                               */
/******************************************************************************/
/* This alignment is required because of the DMA's peripheral indirect 
 * addressing mode. */
static unsigned int ecan1msgBuf [CAN1_MESSAGE_BUFFERS][8] __attribute__((aligned(32 * 8 * 2)));

/******************************************************************************/
/* Private function prototypes                                                */
/******************************************************************************/
static void ECAN1_DMACopy(uint8_t buffer_number, uCAN1_MSG *message);
static void ECAN1_MessageToBuffer(uint16_t* buffer, uCAN1_MSG* message);

/* Null weak implementations of callback functions. */
void __attribute__((weak)) ECAN1_CallbackBusOff(void){}
void __attribute__((weak)) ECAN1_CallbackTxErrorPassive(void){}
void __attribute__((weak)) ECAN1_CallbackRxErrorPassive(void){}
void __attribute__((weak)) ECAN1_CallbackMessageReceived(void){}

/**
  Section: ECAN1 APIs
*****************************************************************************************/

void ECAN1_Initialize(void)
{
    // Disable interrupts before the Initialization
    IEC2bits.C1IE = 0;
    C1INTE = 0;

    // set the CAN_initialize module to the options selected in the User Interface

    /* put the module in configuration mode */
    C1CTRL1bits.REQOP = CAN1_CONFIGURATION_MODE;
    while(C1CTRL1bits.OPMODE != CAN1_CONFIGURATION_MODE);

    /* Set up the baud rate*/	
    C1CFG1 = 0x03;	//BRP TQ = (2 x 4)/FCAN; SJW 1 x TQ; 
    C1CFG2 = 0x1D8;	//WAKFIL disabled; SEG2PHTS Freely programmable; SEG2PH 2 x TQ; SEG1PH 4 x TQ; PRSEG 1 x TQ; SAM Three times at the sample point; 
    C1FCTRL = 0xC001;	//FSA Transmit/Receive Buffer TRB1; DMABS 32; 
    C1FEN1 = 0x01;	//FLTEN8 disabled; FLTEN7 disabled; FLTEN9 disabled; FLTEN0 enabled; FLTEN2 disabled; FLTEN10 disabled; FLTEN1 disabled; FLTEN11 disabled; FLTEN4 disabled; FLTEN3 disabled; FLTEN6 disabled; FLTEN5 disabled; FLTEN12 disabled; FLTEN13 disabled; FLTEN14 disabled; FLTEN15 disabled; 
    C1CTRL1 = 0x00;	//CANCKS FOSC/2; CSIDL disabled; ABAT disabled; REQOP Sets Normal Operation Mode; WIN Uses buffer window; CANCAP disabled; 

    /* Filter configuration */
    /* enable window to access the filter configuration registers */
    /* use filter window*/
    C1CTRL1bits.WIN=1;
	   
    /* select acceptance masks for filters */
    C1FMSKSEL1bits.F0MSK = 0x0; //Select Mask 0 for Filter 0

    /* Configure the masks */
    C1RXM0SIDbits.SID = 0x7ff; 
    C1RXM1SIDbits.SID = 0x0; 
    C1RXM2SIDbits.SID = 0x0; 

    C1RXM0SIDbits.EID = 0x0; 
    C1RXM1SIDbits.EID = 0x0; 
    C1RXM2SIDbits.EID = 0x0; 
     
    C1RXM0EID = 0x00;     	
    C1RXM1EID = 0x00;     	
    C1RXM2EID = 0x00;     	

    C1RXM0SIDbits.MIDE = 0x0; 
    C1RXM1SIDbits.MIDE = 0x0; 
    C1RXM2SIDbits.MIDE = 0x0; 
    
    /* Configure the filters */
    C1RXF0SIDbits.SID = 0x8; 

    C1RXF0SIDbits.EID = 0x0; 
    
    C1RXF0EID = 0x00; 

    C1RXF0SIDbits.EXIDE = 0x0; 

    /* Non FIFO Mode */
    C1BUFPNT1bits.F0BP = 0x1; //Filter 0 uses Buffer1

    /* clear window bit to access ECAN control registers */
    C1CTRL1bits.WIN=0;    

    /* ECAN1, Buffer 0 is a Transmit Buffer */
    C1TR01CONbits.TXEN0 = 0x1; // Buffer 0 is a Transmit Buffer 
    C1TR01CONbits.TXEN1 = 0x0; // Buffer 1 is a Receive Buffer 
    C1TR23CONbits.TXEN2 = 0x0; // Buffer 2 is a Receive Buffer 
    C1TR23CONbits.TXEN3 = 0x0; // Buffer 3 is a Receive Buffer 
    C1TR45CONbits.TXEN4 = 0x0; // Buffer 4 is a Receive Buffer 
    C1TR45CONbits.TXEN5 = 0x0; // Buffer 5 is a Receive Buffer 
    C1TR67CONbits.TXEN6 = 0x0; // Buffer 6 is a Receive Buffer 
    C1TR67CONbits.TXEN7 = 0x0; // Buffer 7 is a Receive Buffer 

    C1TR01CONbits.TX0PRI = 0x0; // Message Buffer 0 Priority Level
    C1TR01CONbits.TX1PRI = 0x0; // Message Buffer 1 Priority Level
    C1TR23CONbits.TX2PRI = 0x0; // Message Buffer 2 Priority Level
    C1TR23CONbits.TX3PRI = 0x0; // Message Buffer 3 Priority Level
    C1TR45CONbits.TX4PRI = 0x0; // Message Buffer 4 Priority Level
    C1TR45CONbits.TX5PRI = 0x0; // Message Buffer 5 Priority Level
    C1TR67CONbits.TX6PRI = 0x0; // Message Buffer 6 Priority Level
    C1TR67CONbits.TX7PRI = 0x0; // Message Buffer 7 Priority Level

    /* clear the buffer and overflow flags */   
    C1RXFUL1 = 0x0000;
    C1RXFUL2 = 0x0000;
    C1RXOVF1 = 0x0000;
    C1RXOVF2 = 0x0000;	

    /* configure the device to interrupt on the receive buffer full flag */
    /* clear the buffer full flags */ 	
    C1INTFbits.RBIF = 0;  

    /* put the module in normal mode */
    C1CTRL1bits.REQOP = CAN1_NORMAL_OPERATION_MODE;
    while(C1CTRL1bits.OPMODE != CAN1_NORMAL_OPERATION_MODE);	
	
    /* Enable ECAN1 Interrupt */
    IEC2bits.C1IE = 1;

    /* Enable Receive interrupt */
    C1INTEbits.RBIE = 1;
	
    /* Enable Error interrupt*/
    C1INTEbits.ERRIE = 1;

    
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_TransmitEnable
*    Description:       Setup the DMA for Transmit from the ECAN module.  The 
*                       relevant DMA module APIs are grouped in this function 
*                       and this API needs to be called after DMA_Initialize 
*                       and CAN_Initialize
*                                                                                                                                                       
******************************************************************************/
void ECAN1_TransmitEnable()
{
    /* setup channel 0 for peripheral indirect addressing mode 
    normal operation, word operation and select as Tx to peripheral */

    /* DMA_PeripheralIrqNumberSet and DMA_TransferCountSet would be done in the 
    DMA */
    
    /* setup the address of the peripheral ECAN1 (C1TXD) */ 
    DMA_PeripheralAddressSet(ECAN1_TX_DMA_CHANNEL, &C1TXD);

    /* DPSRAM start address offset value */ 
    /* DPSRAM start address offset value */ 
    DMA_StartAddressASet(ECAN1_TX_DMA_CHANNEL, (uint16_t)(&ecan1msgBuf));

    /* enable the channel */
    DMA_ChannelEnable(ECAN1_TX_DMA_CHANNEL);
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_ReceiveEnable
*    Description:       Setup the DMA for Receive on the ECAN module.  The 
*                       relevant DMA module APIs are grouped in this function 
*                       and this API needs to be called after DMA_Initialize 
*                       and CAN_Initialize
*                                                                                                                                                       
******************************************************************************/
void ECAN1_ReceiveEnable()
{
    /* setup DMA channel for peripheral indirect addressing mode 
    normal operation, word operation and select as Rx to peripheral */
    
    /* setup the address of the peripheral ECAN1 (C1RXD) */     
    /* DMA_TransferCountSet and DMA_PeripheralIrqNumberSet would be set in 
    the DMA_Initialize function */

    DMA_PeripheralAddressSet(ECAN1_RX_DMA_CHANNEL, &C1RXD);

    /* DPSRAM start address offset value */ 
    DMA_StartAddressASet(ECAN1_RX_DMA_CHANNEL, (uint16_t)(&ecan1msgBuf) );	 

    /* enable the channel */
    DMA_ChannelEnable(ECAN1_RX_DMA_CHANNEL);
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_transmit
*    Description:       Transmits the message from user buffer to CAN buffer
*                       as per the buffer number allocated.
*                       Allocation of the buffer number is done by user 
*                                                                             
*    Arguments:		priority : priority of the message to be transmitted
*                       sendCanMsg: pointer to the message object
*                                             
*    Return Value:      true - Transmit successful
*                       false - Transmit failure                                                                              
******************************************************************************/
bool ECAN1_transmit(ECAN1_TX_PRIOIRTY priority, uCAN1_MSG *sendCanMsg) 
{
    CAN1_TX_CONTROLS * pTxControls = (CAN1_TX_CONTROLS*)&C1TR01CON;
    uint_fast8_t i;
    bool messageSent = false;

    for(i=0; i<CAN1_TX_BUFFER_COUNT; i++)
    {
        if(pTxControls->transmit_enabled == 1)
        {
            if (pTxControls->send_request == 0)
            {
                ECAN1_MessageToBuffer( &ecan1msgBuf[i][0], sendCanMsg );

                pTxControls->priority = priority;

                /* set the message for transmission */
                pTxControls->send_request = 1; 

                messageSent = true;
                break;
            }
        }

        pTxControls++;
    }

    return messageSent;
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_receive
*    Description:       Receives the message from CAN buffer to user buffer 
*                                                                             
*    Arguments:		recCanMsg: pointer to the message object
*                                             
*    Return Value:      true - Receive successful
*                       false - Receive failure                                                                              
******************************************************************************/
bool ECAN1_receive(uCAN1_MSG *recCanMsg) 
{   
    /* We use a static buffer counter so we don't always check buffer 0 first
     * resulting in potential starvation of later buffers.
     */
    static uint_fast8_t currentDedicatedBuffer = 0;
    uint_fast8_t i;
    bool messageReceived = false;
    uint16_t receptionFlags;

    receptionFlags = C1RXFUL1;
	
    if (receptionFlags != 0)  
    {
        /* check which message buffer is free */  
        for (i=0 ; i < 16; i++)
        {
            if (((receptionFlags >> currentDedicatedBuffer ) & 0x1) == 0x1)
            {           
               ECAN1_DMACopy(currentDedicatedBuffer, recCanMsg);
           
               C1RXFUL1 &= ~(1 << currentDedicatedBuffer);
  
               messageReceived = true;
            }
            
            currentDedicatedBuffer++;
            
            if(currentDedicatedBuffer >= 16)
            {
                currentDedicatedBuffer = 0;
            }
            
            if(messageReceived == true)
            {
                break;
            }
        }
    }
        
    return (messageReceived);
}


/******************************************************************************
*                                                                             
*    Function:		ECAN1_isBusOff
*    Description:       Checks whether the transmitter in Bus off state
*                                                                             
                                             
*    Return Value:      true - Transmitter in Bus Off state
*                       false - Transmitter not in Bus Off state                                                                              
******************************************************************************/
bool ECAN1_isBusOff() 
{
    return C1INTFbits.TXBO;	
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_isRXErrorPassive
*    Description:       Checks whether the receive in error passive state
*                                             
*    Return Value:      true - Receiver in Error Passive state
*                       false - Receiver not in Error Passive state                                                                              
******************************************************************************/
bool ECAN1_isRXErrorPassive()
{
    return C1INTFbits.RXBP;   
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_isTXErrorPassive
*    Description:       Checks whether the transmitter in error passive state                                                                          
*                                             
*    Return Value:      true - Transmitter in Error Passive state
*                       false - Transmitter not in Error Passive state                                                                              
******************************************************************************/
bool ECAN1_isTXErrorPassive()
{
    return (C1INTFbits.TXBP);
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_messagesInBuffer
*    Description:       returns the number of messages that are received                                                                           
*                                             
*    Return Value:      Number of message received
******************************************************************************/
uint8_t ECAN1_messagesInBuffer() 
{
    uint_fast8_t messageCount;
    uint_fast8_t currentBuffer;
    uint16_t receptionFlags;
   
    messageCount = 0;

    /* Check any message in buffer 0 to buffer 15*/
    receptionFlags = C1RXFUL1;
    if (receptionFlags != 0) 
    {
        /* check whether a message is received */  
        for (currentBuffer=0 ; currentBuffer < 16; currentBuffer++)
        {
            if (((receptionFlags >> currentBuffer ) & 0x1) == 0x1)
            {
                messageCount++;
            }
        }
    }
            
    return (messageCount);
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_sleep
*    Description:       Puts ECAN1 module in disable mode.
*                                                                       
******************************************************************************/
void ECAN1_sleep(void) {
    C1INTFbits.WAKIF = 0;
    C1INTEbits.WAKIE = 1;

    /* put the module in disable mode */
    C1CTRL1bits.REQOP=CAN1_DISABLE_MODE;
    while(C1CTRL1bits.OPMODE != CAN1_DISABLE_MODE);
    
    //Wake up from sleep should set the CAN module straight into Normal mode
}

/*******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

/******************************************************************************
*                                                                             
*    Function:		ECAN1_DMACopy
*    Description:       moves the message from the DMA memory to RAM
*                                                                             
*    Arguments:		*message: a pointer to the message structure in RAM 
*			that will store the message. 
*	                                                                 
*                                                                              
******************************************************************************/
static void ECAN1_DMACopy(uint8_t buffer_number, uCAN1_MSG *message)
{
    uint16_t ide=0;
    uint16_t rtr=0;
    uint32_t id=0;

    /* read word 0 to see the message type */
    ide=ecan1msgBuf[buffer_number][0] & 0x0001U;			

    /* check to see what type of message it is */
    /* message is standard identifier */
    if(ide==0U)
    {
        message->frame.id=(ecan1msgBuf[buffer_number][0] & 0x1FFCU) >> 2U;		
        message->frame.idType = CAN1_FRAME_STD;
        rtr=ecan1msgBuf[buffer_number][0] & 0x0002U;
    }
    /* message is extended identifier */
    else
    {
        id=ecan1msgBuf[buffer_number][0] & 0x1FFCU;		
        message->frame.id = id << 16U;
        message->frame.id += ( ((uint32_t)ecan1msgBuf[buffer_number][1] & (uint32_t)0x0FFF) << 6U );
        message->frame.id += ( ((uint32_t)ecan1msgBuf[buffer_number][2] & (uint32_t)0xFC00U) >> 10U );		
        message->frame.idType = CAN1_FRAME_EXT;
        rtr=ecan1msgBuf[buffer_number][2] & 0x0200;
    }
    /* check to see what type of message it is */
    /* RTR message */
    if(rtr != 0U)
    {
        /* to be defined ?*/
        message->frame.msgtype = CAN1_MSG_RTR;	
    }
    /* normal message */
    else
    {
        message->frame.msgtype = CAN1_MSG_DATA;
        message->frame.data0 =(unsigned char)ecan1msgBuf[buffer_number][3];
        message->frame.data1 =(unsigned char)((ecan1msgBuf[buffer_number][3] & 0xFF00U) >> 8U);
        message->frame.data2 =(unsigned char)ecan1msgBuf[buffer_number][4];
        message->frame.data3 =(unsigned char)((ecan1msgBuf[buffer_number][4] & 0xFF00U) >> 8U);
        message->frame.data4 =(unsigned char)ecan1msgBuf[buffer_number][5];
        message->frame.data5 =(unsigned char)((ecan1msgBuf[buffer_number][5] & 0xFF00U) >> 8U);
        message->frame.data6 =(unsigned char)ecan1msgBuf[buffer_number][6];
        message->frame.data7 =(unsigned char)((ecan1msgBuf[buffer_number][6] & 0xFF00U) >> 8U);
        message->frame.dlc =(unsigned char)(ecan1msgBuf[buffer_number][2] & 0x000FU);
    }
}

/******************************************************************************
*                                                                             
*    Function:		ECAN1_MessageToBuffer
*    Description:       This function takes the input message, reformats it, 
*                       and copies it to the specified CAN module buffer
*                                                                             
*    Arguments:		*buffer: a pointer to the buffer where the message 
*                       would be stored 
*                       *message: pointer to the input message that is received
*                       by the CAN module 	                                                                 
*                                                                              
******************************************************************************/
static void ECAN1_MessageToBuffer(uint16_t* buffer, uCAN1_MSG* message)
{   
    if(message->frame.idType == CAN1_FRAME_STD)
    {
        buffer[0]= (message->frame.id & 0x000007FF) << 2;
        buffer[1]= 0;
        buffer[2]= message->frame.dlc & 0x0F;
    }
    else
    {
        buffer[0]= ( ( (uint16_t)(message->frame.id >> 16 ) & 0x1FFC ) ) | 0b1;
        buffer[1]= (uint16_t)(message->frame.id >> 6) & 0x0FFF;
        buffer[2]= (message->frame.dlc & 0x0F) + ( (uint16_t)(message->frame.id << 10) & 0xFC00);
    }

    buffer[3]= ((message->frame.data1)<<8) + message->frame.data0;
    buffer[4]= ((message->frame.data3)<<8) + message->frame.data2;
    buffer[5]= ((message->frame.data5)<<8) + message->frame.data4;
    buffer[6]= ((message->frame.data7)<<8) + message->frame.data6;
}


void __attribute__((__interrupt__, no_auto_psv)) _C1Interrupt(void)  
{   

    if (C1INTFbits.ERRIF)
    {
        
        if (C1INTFbits.TXBO == 1)
        {
            ECAN1_CallbackBusOff();
            C1INTFbits.TXBO = 0;
        }
        
        if (C1INTFbits.TXBP == 1)
        {
            ECAN1_CallbackTxErrorPassive();
            C1INTFbits.TXBP = 0;
        }

        if (C1INTFbits.RXBP == 1)
        {
            ECAN1_CallbackRxErrorPassive();
            C1INTFbits.RXBP = 0;
        }

        /* Call error notification function */
        C1INTFbits.ERRIF = 0;  
        
    }
    
    if(C1INTFbits.RBIF)
    {
        C1INTFbits.RBIF = 0;  
        
        /* Notification function */
        ECAN1_CallbackMessageReceived();  
    } 
    
   
    IFS2bits.C1IF = 0;
}



/**
 End of File
*/