/* EYEBALL ARCHIVE
 * 7-1-17:  Additional routines from RC Servo
 */


DUTYbuffer[0] = DUTYbuffer[1] = DUTYbuffer[2] = DUTYbuffer[3] = DUTYbuffer[4] = DUTYbuffer[5] = angle;
dutyCycle[0] = convertDutyCycle(DUTYbuffer[0]);
dutyCycle[1] = convertDutyCycle(DUTYbuffer[1]);
dutyCycle[2] = convertDutyCycle(DUTYbuffer[2]);
dutyCycle[3] = convertDutyCycle(DUTYbuffer[3]);
dutyCycle[4] = convertDutyCycle(DUTYbuffer[4]);
dutyCycle[5] = convertDutyCycle(DUTYbuffer[5]); 

                    
                    switch (state) {
                        case STANDBY:
                            if (sensorDistance <= GALLERY_EMPTY) state = OPEN;
                            break;
                        case OPEN:
                            break;
                        case RUN:
                            if (sensorDistance > GALLERY_EMPTY) state = CLOSE;
                            break;
                        case CLOSE:
                            break;
                        default:
                            state = STANDBY;
                            break;
                    }
                               

               
               
unsigned char processInBuffer(unsigned short inLength) {
    unsigned char i, j, k, numServosUpdated = 0;

    numServosUpdated = XBEERxBuffer[2];
    firstServo = XBEERxBuffer[1];
    for (i = 0; i < numServosUpdated; i++) {
        j = i + 3;
        k = i + firstServo;
        if (j < MAXBUFFER && k < MAXBUFFER)
            servoBuffer[k] = XBEERxBuffer[j];
        else {
            numServosUpdated = 0;
            break;
        }
    }
    if (numServosUpdated) {
        printf("\rUpdated: %d servos", numServosUpdated);
        for (i = 0; i < MAXDEVICES; i++) {
            DUTYbuffer[i] = servoBuffer[i];
        }
    }

    unsigned short result;
    unsigned char numBytes;

    convert.byte[0] = INbuffer[inLength - 2];
    convert.byte[1] = INbuffer[inLength - 1];

    numBytes = inLength - 2;
    result = CRCcalculate(INbuffer, numBytes);

    if (result != convert.integer) return (FALSE);  // ERROR: CRC doesn't match
    
    return (numServosUpdated);
}
 
#define STANDBY_STATE 0
#define START_STATE 1
#define ENABLED_STATE 2
#define START_CHAR 0xFF

static void interrupt isr(void) {
    static unsigned char state = 0;
    static unsigned char ch, servoID;

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
            if (ch == START_CHAR) state = START_STATE;
            else if (state == START_STATE) {
                if (ch <= LAST_SERVO) {
                    state = ENABLED_STATE;
                    servoID = (unsigned) (ch - FIRST_SERVO);
                    LEDtimeout = 100;
                } else state = STANDBY_STATE;
            } else if (state == ENABLED_STATE) {
                if (servoID < MAXDEVICES) {
                    DUTYbuffer[servoID] = ch;
                }
                state = STANDBY_STATE;
            }
        }
    }
}

static void interrupt isr(void) {
    unsigned char BoardID, ch;
    static unsigned char buffIndex = 0;
    static unsigned char escapeFlag = false;
    unsigned char i, j, k, firstServo;

    if (RCIF == 1) { // If RX interrupt occurs:
        RCIF = 0;

        if (RCSTAbits.OERR) { // If overrun occurs, reset receive enable.
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }

        if (RCSTAbits.FERR) // If frame error occurs, flush buffer
            ch = RCREG;
        ch = RCREG;

        if (ch == DLE && !escapeFlag)
            escapeFlag = true;
        else if (ch == STX && !escapeFlag)
            buffIndex = 0;
        else if (ch == ETX && !escapeFlag) {
            BoardID = XBEERxBuffer[0];
            if (BoardID == BOARD_ID)
                inLength = buffIndex;
            buffIndex = 0;
        } else {
            escapeFlag = false;
            if (buffIndex < MAXBUFFER) XBEERxBuffer[buffIndex++] = ch;
        }
    }
}

