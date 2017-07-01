/* EYEBALL 
 * For RC Servo Board using PIC 18F1330
 * Compiler: XC8 V1.42
 * Adapted from RC Servo - See ARCHIVE main.c for additional code
 * 
 * 6-27-17  Recompiled with XC8 compiler.
 * 6-30-17: NEW EYEBALL CODE
 * 7-1-17:  First version used in Gallery X installation.
 */

#include <xc.h>
#include "DELAY16.H"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include <math.h>
#include <pic18f1330.h>
#include "DELAY16.H"

#define MAX_RAND (32767)
#define RAND_DIVISOR (MAX_RAND / 16)
#define TIME_DIVISOR (MAX_RAND / 24)
#define STATE_DIVISOR (MAX_RAND / 3)

#define ENTER 13
#define BACKSPACE 8
#define SPACE ' '

#define TESTout PORTBbits.RB2

#define BOARD_ID 1
#define ANY_BOARD 0
#define STX '>'
#define ETX '\r'
#define DLE '/'
#define PLUS  '+' 

#define FIRST_SERVO 0
#define LAST_SERVO 5

// #define THIS_BOARD_NUMBER 0x81
#define SETBOARD 0x80

#define LED LATAbits.LA4

#define false 0
#define true !false
#define TRUE true
#define FALSE false

#define TEST_OUT LATBbits.LATB2

unsigned int convertDutyCycle(unsigned char servoPosition);
void initializePorts(void);
void putch(unsigned char byte);
unsigned short readAD(void);
void ADsetChannel(unsigned char channel);
unsigned char processInBuffer(unsigned short inLength);

unsigned short dutyCycle[6];

union {
    unsigned char byte[2];
    unsigned int val;
} integer;


#define MAXDEVICES 6
unsigned char DUTYbuffer[MAXDEVICES];

#define MAXDUTY 288 * 2
#define MINDUTY 288


// Configuration: High speed crystal (HS) oscillator, 
#pragma config OSC=HS, FCMEN=OFF, IESO=OFF,\
PWRT=OFF, BOR=OFF, BORV=3,\
WDT=OFF, WDTPS=32768, PWMPIN=ON, LPOL=HIGH, HPOL=HIGH,\
FLTAMX=RA5, T1OSCMX=HIGH, MCLRE=ON,\
STVREN=ON, BBSIZ=BB256,\
CP0=OFF, CP1=OFF, CPB=OFF, CPD=OFF,\
WRT0=OFF, WRT1=OFF, WRTC=OFF, WRTB=OFF, WRTD=OFF,\
EBTR0=OFF, EBTR1=OFF, EBTRB=OFF

union tag {
    unsigned char byte[2];
    unsigned short integer;
} convert;

enum {
    STANDBY = 0,    
    RUN,
    HIDE,
} state = STANDBY;


#define FORWARD 0
#define REVERSE 1
#define ONE_HUNDRED_MS 7

unsigned char LEDtimeout = 0;


#define GALLERY_EMPTY 24
#define OFFSET 150

#define MAXBUFFER 128
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;
unsigned char controlCommand = 0;

#define eyeUpDown dutyCycle[0]
#define eyeRightLeft dutyCycle[1]
#define eyeLid dutyCycle[2]

#define EYE_UPDOWN_CENTER 255
#define EYE_RIGHTLEFT_CENTER 255
#define EYELID_CENTER 255
#define CENTER 255

unsigned char getCommand(unsigned char *ptrString);
short getInteger(unsigned char *ptrString);

#define UP 240
#define DOWN 100
#define CENTER_VERT 170

#define LEFT 340
#define RIGHT 200
#define CENTER_HORIZ 260

#define CLOSED 60
#define OPEN 150

#define DOWNWARD 1
#define UPWARD 2
#define RIGHTWARD 1
#define LEFTWARD 2
#define OPENING 1
#define CLOSING 2
#define PAUSED 0

#define HORIZ_SPEED 1

void main(void) {
    unsigned short sensorDistance = 0;
    short sensorCounter = 0;
    unsigned short StateTimer = 0;    
    unsigned char command = 0;
    unsigned short intValue = 0;
    unsigned short seconds = 0;

    unsigned char upDownState = UPWARD;
    unsigned char rightLeftState = LEFTWARD;
    unsigned char openCloseState = OPENING;

    unsigned short rightLeftTimer = 0;
    unsigned short rightLeftTimeout = 0;

    unsigned short upDownTimer = 0;
    unsigned short upDownTimeout = 0;

    unsigned short openCloseTimer = 0;
    unsigned short openCloseTimeout = 0;

    eyeUpDown = CENTER_VERT;
    eyeRightLeft = CENTER_HORIZ;
    eyeLid = CLOSED;

    dutyCycle[3] = dutyCycle[4] = dutyCycle[5] = 256;

    initializePorts();
    DelayMs(100);
    printf("\rTesting state commands");
    ADsetChannel(0);
    LED = 0;

    while (1) {

        if (TMR0IF) {
            TMR0IF = 0;

            if (HOSTRxBufferFull) {
                HOSTRxBufferFull = false;
                if (command = getCommand(HOSTRxBuffer)) {
                    intValue = getInteger(HOSTRxBuffer);
                    printf("\rCommand: %c, val: %d\r", command, intValue);

                    if (intValue < 512 || command == 'X' || command == 'Z') {
                        switch (command) {
                            case 'U':
                                eyeUpDown = intValue;
                                break;
                            case 'R':
                                eyeRightLeft = intValue;
                                break;
                            case 'L':
                                eyeLid = intValue;
                                break;
                            case 'X':
                                printf("\rSTOP");
                                break;
                            case 'Z':
                                printf("\rSTART");
                                break;
                            default:
                                break;
                        }
                    } else printf("\rValue out of range");
                    if (command == ' ') {
                        if (state) state = STANDBY;
                        else state = RUN;
                    }
                    command = 0;
                }
            } // end if (HOSTRxBufferFull)

            sensorDistance = readAD();
            sensorCounter++;
            if (sensorCounter > 5){
                sensorCounter = 0;
                printf("\rSense: %d", sensorDistance);
            }

            if (sensorDistance > 55) state = RUN;
            else state = STANDBY;

            
            if (state) {
                if (StateTimer) StateTimer--;
                if (!StateTimer) {
                    StateTimer = ONE_HUNDRED_MS;
                    if (TEST_OUT) TEST_OUT = 0;
                    else TEST_OUT = 1;

                    rightLeftTimer++;
                    if (rightLeftTimer >= rightLeftTimeout) {
                        rightLeftTimeout = (unsigned short) ((rand() / TIME_DIVISOR) + 8);
                        rightLeftState = (unsigned short) (rand() / STATE_DIVISOR);
                        //printf("\rRIGHTLEFT Timeout: %d, state: %d", rightLeftTimeout, rightLeftState);
                        rightLeftTimer = 0;
                    }

                    upDownTimer++;
                    if (upDownTimer >= upDownTimeout) {
                        upDownTimeout = (unsigned short) ((rand() / TIME_DIVISOR) + 8);
                        upDownState = (unsigned short) (rand() / STATE_DIVISOR);
                        //printf("\rUPDOWN Timeout: %d, state: %d", upDownTimeout, upDownState);
                        upDownTimer = 0;
                    }

                    openCloseTimer++;
                    if (openCloseTimer >= openCloseTimeout) {
                        openCloseTimeout = (unsigned short) ((rand() / TIME_DIVISOR) + 8);
                        openCloseState = (unsigned short) (rand() / STATE_DIVISOR);
                        //printf("\rOPENCLOSE Timeout: %d, state: %d", openCloseTimeout, openCloseState);
                        openCloseTimer = 0;
                    }
                } // end (!StateTimer)             

                if (rightLeftState == LEFTWARD) {
                    eyeRightLeft++;
                    if (eyeRightLeft > LEFT) {
                        rightLeftState = RIGHTWARD;
                        //printf("\rRIGHT");
                    }
                } else if (rightLeftState == RIGHTWARD) {
                    eyeRightLeft--;
                    if (eyeRightLeft < RIGHT) {
                        rightLeftState = LEFTWARD;
                        //printf("\rRIGHT");
                    }
                }


                if (openCloseState == OPENING) {
                    eyeLid++;
                    if (eyeLid > OPEN) openCloseState = CLOSING;
                } else if (openCloseState == CLOSING) {
                    eyeLid--;
                    if (eyeLid < CLOSED) openCloseState = OPENING;
                }

                if (upDownState == UPWARD) {
                    eyeUpDown++;
                    if (eyeUpDown > UP) upDownState = DOWNWARD;
                } else if (upDownState == DOWNWARD) {
                    eyeUpDown--;
                    if (eyeUpDown < DOWN) upDownState = UPWARD;
                }


                if (OVDCOND == 0b00010101) {
                    OVDCOND = 0b00101010;
                    integer.val = dutyCycle[1] + OFFSET;
                    PDC0L = integer.byte[0];
                    PDC0H = integer.byte[1];

                    integer.val = dutyCycle[3] + OFFSET;
                    PDC1L = integer.byte[0];
                    PDC1H = integer.byte[1];

                    integer.val = dutyCycle[5] + OFFSET;
                    PDC2L = integer.byte[0];
                    PDC2H = integer.byte[1];
                } else {
                    OVDCOND = 0b00010101;
                    integer.val = dutyCycle[0] + OFFSET;
                    PDC0L = integer.byte[0];
                    PDC0H = integer.byte[1];

                    integer.val = dutyCycle[2] + OFFSET;
                    PDC1L = integer.byte[0];
                    PDC1H = integer.byte[1];

                    integer.val = dutyCycle[4] + OFFSET;
                    PDC2L = integer.byte[0];
                    PDC2H = integer.byte[1];
                }

                PTEN = 1; // Re-enable single shot PWM for next pulse
            }// end if (state)
            else OVDCOND = 0b00000000;
        } // end if (TMR0IF))
    } // end while(1))
} // end main()        


unsigned int convertDutyCycle(unsigned char servoPosition) {
    unsigned int temp;

    temp = servoPosition;
    temp = temp * 2;
    temp = temp; // + 150;  178;
    return (temp);
}

void initializePorts(void) {
    INTCON = 0x00; // First, clear all interrupts
    PIE1 = 0; // Clear all peripheral interrupts

    // Initialize ports
    ADCON0 = 0b00000000; // Turn off A/D for now.
    ADCON1 = 0b000001110; // Set up 18F1330 for one analog input, use VCC and VSS for references.    
    ADCON2 = 0; // Clear A/D control register 2
    ADCON2bits.ADFM = 0; // Left justified A/D result
    ADCON2bits.ACQT2 = 1; // Acquisition time max
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 1;
    ADCON2bits.ADCS2 = 1; // Conversion time = Fosc/16
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;

    TRISA = 0b00001111; // Port A
    TRISB = 0b00000000; // Port B
    RBPU = 0; // Enable Port B pullups

    // TIMER 0: set up for 14 ms rollover
    T0CON = 0x00; // Clear everything
    T016BIT = 1; // 8 bit mode
    PSA = 0; // Use prescaler    
    T0PS2 = 1; // 1:256 prescaler
    T0PS1 = 1;
    T0PS0 = 1;
    T0CS = 0; // Use clock input
    T0SE = 0; // Not used
    TMR0ON = 1; // Enable timer
    TMR0IF = 0;

    // TIMER 1: disabled for now
    T1CON = 0; // Clear
    T1RD16 = 1; // Enable 16 bit operation.
    TMR1CS = 0; // Use internal clock
    T1CKPS0 = 1; // 1:16 prescale for 0.888 ms interrupts (was 1:1))
    T1CKPS1 = 1;
    TMR1ON = 0; // Disabled

    // SET UP PEM
    PIE3 = 0; // Disable time bas interrupts
    FLTCONFIG = 0; // Disable faults
    PTCON0 = 0; // Clear PWM Timer Control Register 0
    PTCKPS0 = 1; // 1:64 Prescale
    PTCKPS1 = 1;
    PTMOD1 = 0; // Single shot mode
    PTMOD0 = 1;

    PTCON1 = 0; // Clear PWM Timer Control Register 1
    PTEN = 0; // PWM Time Base is disabled for now
    // Default time base counts UP

    PWMCON0 = 0; // Clear PWM COntrol Register 0
    PWMEN0 = 0; // Enable all PWM outputs
    PWMEN1 = 0;
    PWMEN2 = 1;
    PMOD0 = 1; // All PWM's in independent mode
    PMOD1 = 1;
    PMOD2 = 1;

    PWMCON1 = 0; // PWM Control Register 1 not used

    PTPERH = 0; // Set PWM period to about 2.4 milliseconds for 18.432 Mhz clock
    PTPERL = 170;

    // PDC0L = PDC0H = PDC1L = PDC1H = PDC2L = PDC2H = 0;  Clear duty cycles 

    OVDCOND = 0b00101010; // To start with, set outputs on even servos PWM0, PWM2, PWM4
    OVDCONS = 0b00000000; // When PWM's are off, keep outputs low.

    BRGH = 1; // high speed baud rate    
    SPBRG = 19; // Set the baud rate to 57600 for 18.432 Mhz clock    
    // SPBRG = 59;     // Set the baud rate to 19200 for 18.432 Mhz clock

    SYNC = 0; // asynchronous
    SPEN = 1; // enable serial port pins
    CREN = 1; // enable reception
    SREN = 0; // no effect
    TXIE = 0; // disable tx interrupts
    RCIE = 1; // Enable rx interrupts
    TX9 = 0; // 8 bit transmission
    RX9 = 0; // 8 bit reception
    TXEN = 1; // enable the transmitter
    TXIE = 0; // Disable UART Tx interrupts
    RCIE = 1; // Enabled UART Rx interrupts
    PEIE = 1; // Enable peripheral interrupts.
    TMR1IE = 0; // Disable timer 1 interrupts.
    TMR0IE = 0; // Disable Timer 0 interrupts
    GIE = 1; // Enable global interrupts
}


// This transmits one character over the RS485 bus.
// It enables RS485 transmission for the first character sent,
// and disables if a carriage return '\r' is tranmitted.

void putch(unsigned char ch) {
    while (!TXIF) // Wait for transmit buffer to be empty
        continue;
    TXREG = ch;
}

void ADsetChannel(unsigned char channel) {
    ADCON0 = (unsigned) ((channel << 2) + 0x01); // enable ADC, RC osc.
}

unsigned short readAD(void) {
    unsigned short ADresult;
    GODONE = 1;
    while (GODONE)
        continue; // wait for conversion complete
    ADresult = ADRESH;

    return (ADresult);
}

static void interrupt isr(void) {
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;

    if (RCIF == 1) { // If RX interrupt occurs:
        RCIF = 0;

        if (RCSTAbits.OERR) { // If overrun occurs, reset receive enable.
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }

        if (RCSTAbits.FERR) { // If frame error occurs, flush buffer
            ch = RCREG;
            ch = RCREG;
        } else {
            ch = RCREG;
            if (ch == '\n' || ch == 0);
            else if (ch == SPACE) {
                if (state) {
                    state = STANDBY;
                    HOSTRxBuffer[0] = 'X';
                    HOSTRxBuffer[1] = '\0';
                    HOSTRxBufferFull = true;
                } else {
                    state = RUN;
                    HOSTRxBuffer[0] = 'Z';
                    HOSTRxBuffer[1] = '\0';
                    HOSTRxBufferFull = true;
                }
            } else if (ch == BACKSPACE) {
                while (!TXIF); // Wait for transmit buffer to be empty
                TXREG = ' ';
                while (!TXIF); // Wait for transmit buffer to be empty
                TXREG = BACKSPACE;
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == ENTER) {
                HOSTRxBuffer[HOSTRxIndex] = '\0';
                HOSTRxBufferFull = true;
                HOSTRxIndex = 0;
            } else if (ch < 27) controlCommand = ch;
            else if (HOSTRxIndex < MAXBUFFER) {
                HOSTRxBuffer[HOSTRxIndex++] = ch;
            }
        }
    }
}

#define MAXNUMLENGTH 8

short getInteger(unsigned char *ptrString) {
    char ch = 0;
    unsigned char strNumber[MAXNUMLENGTH + 1];
    unsigned char negativeFlag = false;
    short i = 0, j = 0, value = 32767;

    while (ptrString[i] != NULL && j < MAXNUMLENGTH) {
        ch = ptrString[i];
        if (ch == '\0') {
            strNumber[j] = '\0';
            break;
        }
        if (ch == '-') negativeFlag = true;
        else if (isdigit(ch)) strNumber[j++] = ch;
        i++;
    }
    if (j) value = (short) atoi(strNumber);
    if (negativeFlag) value = 0 - value;
    return (value);
}

unsigned char getCommand(unsigned char *ptrString) {
    unsigned char ch = 0;
    short i = 0;

    while (ptrString[i] != NULL && i < MAXBUFFER) {
        ch = ptrString[i];
        if (isalpha(ch)) return (ch);
        if (ch == SPACE) return (SPACE);
    }
    return (0);
}
