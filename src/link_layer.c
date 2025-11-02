// MISC
#define _POSIX_SOURCE 1

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#define STUFF_BYTE 0x20

#define FLAG 0x7E
#define ESC 0x7D

#define A 0x03    // cmds/replies for Tx/Rx 
#define A_Rt 0x01 // cmds/replies for Rx/Tx 

#define C_UA 0x07
#define C_SET 0x03
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define C_I0 0x00
#define C_I1 0x80

// BCC1 calculation for command frames
#define BCC1_SET (A ^ C_SET)
#define BCC1_DISC (A ^ C_DISC)

// BCC1 calculation for reply frames from Receiver (A ^ C)
#define BCC1_UA_r (A ^ C_UA)
#define BCC1_RR0_r (A ^ C_RR0)
#define BCC1_RR1_r (A ^ C_RR1)
#define BCC1_REJ0_r (A ^ C_REJ0)
#define BCC1_REJ1_r (A ^ C_REJ1)

// BCC1 calculation for reply frames from Transmitter (A_Rt ^ C)
#define BCC1_UA_t (A_Rt ^ C_UA)
#define BCC1_DISC_t (A_Rt ^ C_DISC)

// state machine steps
#define START_STEP 0
#define FLAG_STEP 1
#define A_STEP 2
#define C_STEP 3
#define BCC1_STEP 4
#define DATA_STEP 5
#define ESCAPE_STEP 6
#define STOP_STEP 7

unsigned char SET[5] = {FLAG, A, C_SET, BCC1_SET, FLAG};
unsigned char UA_reply[5] = {FLAG, A_Rt, C_UA, BCC1_UA_t, FLAG};
unsigned char UA_cmd[5] = {FLAG, A, C_UA, BCC1_UA_r, FLAG};

unsigned char DISC_cmd[5] = {FLAG, A, C_DISC, BCC1_DISC, FLAG};
unsigned char DISC_reply[5] = {FLAG, A_Rt, C_DISC, BCC1_DISC_t, FLAG};

// supervision frames 
unsigned char RR0_t[5] = {FLAG, A, C_RR0, BCC1_RR0_r, FLAG};
unsigned char RR1_t[5] = {FLAG, A, C_RR1, BCC1_RR1_r, FLAG};
unsigned char REJ0_t[5] = {FLAG, A, C_REJ0, BCC1_REJ0_r, FLAG};
unsigned char REJ1_t[5] = {FLAG, A, C_REJ1, BCC1_REJ1_r, FLAG};

volatile int curr_seq = 0;
volatile int connection_active = FALSE;
static int serial_fd = -1;

LinkLayer connection_params;

volatile int timeoutFlag = 0;
volatile int timeoutCount = 0;

// --- HELPER FUNCTIONS ---

void alarmHandler(int signal)
{
    timeoutFlag = 1;
    timeoutCount++;
    printf(">>> TIMEOUT #%d - No response received\n", timeoutCount);
}

void setupAlarmHandler()
{
    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // ensures SIGALRM interrupts blocking read (EINTR)

    if (sigaction(SIGALRM, &sa, NULL) == -1)
    {
        perror("Error setting up SIGALRM handler");
    }
}

void stuffAndAddByte(unsigned char byte, unsigned char *frame, int *idx)
{
    if (byte == FLAG || byte == ESC)
    {
        frame[(*idx)++] = ESC;
        frame[(*idx)++] = byte ^ STUFF_BYTE;
    }
    else
    {
        frame[(*idx)++] = byte;
    }
}

unsigned char BCC2(const unsigned char *data, int size)
{
    unsigned char bcc = 0;
    for (int i = 0; i < size; i++)
    {
        bcc ^= data[i];
    }
    return bcc;
}

unsigned char read_supervision_frame(unsigned char expectedA, unsigned char expectedC, unsigned char expectedBCC1, int useTimeout)
{
    int step = START_STEP;

    if (useTimeout)
    {
        timeoutFlag = 0;
        alarm(connection_params.timeout);
    }

    // ensure the BCC1 is correct for the UA frame when checking Tx's reply
    if (expectedC == C_UA && expectedA == A)
    {
        expectedBCC1 = BCC1_UA_r;
    }

    while (!useTimeout || !timeoutFlag)
    {
        unsigned char byte;
        int bytesRead = readByteSerialPort(&byte);

        if (bytesRead < 0)
        {
            if (errno == EINTR)
            {
                // interrupted by SIGALRM
                continue;
            }
            break;
        }

        if (bytesRead > 0)
        {
            switch (step)
            {
            case START_STEP:
                if (byte == FLAG)
                    step = FLAG_STEP;
                break;
            case FLAG_STEP:
                if (byte == expectedA)
                    step = A_STEP;
                else if (byte != FLAG)
                    step = START_STEP;
                break;
            case A_STEP:
                if (byte == expectedC)
                    step = C_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case C_STEP:
                if (byte == expectedBCC1)
                    step = BCC1_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case BCC1_STEP:
                if (byte == FLAG)
                {
                    if (useTimeout)
                        alarm(0);
                    return expectedC; // success: return the C field
                }
                else
                    step = START_STEP;
                break;
            default:
                step = START_STEP;
                break;
            }
        }
    }

    if (useTimeout)
        alarm(0);
    return 0; // timeout/error
}

// --- LL FUNCTIONS ---

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    connection_params = connectionParameters;

    setupAlarmHandler();

    if (connectionParameters.role == LlTx)
    {
        serial_fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (serial_fd < 0)
            return -1;

        timeoutCount = 0;
        while (timeoutCount <= connection_params.nRetransmissions)
        {
            // send SET
            writeBytesSerialPort(SET, 5);
            printf("SET sent (attempt %d/%d)\n", timeoutCount + 1, connection_params.nRetransmissions);

            // wait for UA (expected: A=A, C=C_UA)
            if (read_supervision_frame(A, C_UA, BCC1_UA_r, TRUE))
            {
                printf("Received UA - Connection established!\n");
                connection_active = TRUE;
                return serial_fd;
            }

            if (timeoutCount >= connection_params.nRetransmissions)
                break;
            printf("TIMEOUT on attempt %d/%d - Retransmitting SET\n", timeoutCount, connection_params.nRetransmissions);
        }

        printf("Connection failed after %d attempts\n", connection_params.nRetransmissions);
        return -1;
    }
    else // LlRx
    {
        serial_fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (serial_fd < 0)
            return -1;

        // wait for SET frame (no timeout)
        printf("Waiting for SET frame...\n");
        if (read_supervision_frame(A, C_SET, BCC1_SET, FALSE))
        {
            printf("Received SET frame, sending UA...\n");

            // send UA response (A field = A, C field = C_UA)
            writeBytesSerialPort(UA_cmd, 5);

            connection_active = TRUE;
            return serial_fd;
        }
        else
        {
            printf("Error waiting for SET frame\n");
            return -1;
        }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    if (!connection_active || serial_fd < 0)
        return -1;

    // 1. build the frame
    int maxStuffedSize = 5 + (bufSize * 2) + 2;
    unsigned char *stuffedFrame = (unsigned char *)malloc(maxStuffedSize);
    if (!stuffedFrame)
        return -1;
    int pos = 0;

    // header
    stuffedFrame[pos++] = FLAG;
    stuffedFrame[pos++] = A;
    unsigned char C = (curr_seq == 0) ? C_I0 : C_I1;
    stuffedFrame[pos++] = C;
    unsigned char bcc1 = A ^ C;
    stuffedFrame[pos++] = bcc1;

    // data field (stuffing + BCC2)
    for (int i = 0; i < bufSize; i++)
    {
        stuffAndAddByte(buf[i], stuffedFrame, &pos);
    }
    unsigned char bcc2 = BCC2(buf, bufSize);
    stuffAndAddByte(bcc2, stuffedFrame, &pos);

    // trailer
    stuffedFrame[pos++] = FLAG;
    int frameLength = pos;

    // 2. transmit and wait for ACK/NACK
    int bytesWritten = -1;
    timeoutCount = 0;
    int success = FALSE;

    // determine the expected reply C fields
    unsigned char expected_RR_C = (curr_seq == 0) ? C_RR1 : C_RR0;
    unsigned char expected_REJ_C = (curr_seq == 0) ? C_REJ0 : C_REJ1;

    // determine the expected BCC1 for the reply frame (A=0x03)
    unsigned char expected_BCC1_RR = (curr_seq == 0) ? BCC1_RR1_r : BCC1_RR0_r;

    printf("=== LLWRITE STARTING for I-%d, %d bytes ===\n", curr_seq, bufSize);

    while (timeoutCount < connection_params.nRetransmissions)
    {
        // send the frame
        bytesWritten = writeBytesSerialPort(stuffedFrame, frameLength);
        if (bytesWritten < 0)
        {
            free(stuffedFrame);
            return -1;
        }
        printf(">>> Sent I-%d (attempt %d/%d)\n", curr_seq, timeoutCount + 1, connection_params.nRetransmissions);

        // wait for RR or REJ 
        unsigned char result = read_supervision_frame(A, expected_RR_C, expected_BCC1_RR, TRUE);

        if (result == 0)
        {
            printf(">>> Timeout waiting for RR-%d (attempt %d/%d)\n", (curr_seq == 0) ? 1 : 0, timeoutCount, connection_params.nRetransmissions);
            continue;
        }

        // if we received something, check what it was
        else if (result == expected_RR_C)
        {
            // ACK received
            curr_seq = 1 - curr_seq;
            success = TRUE;
            printf(">>> SUCCESS: I-%d acknowledged by RR-%d\n", 1 - curr_seq, curr_seq);
            break;
        }
        else if (result == expected_REJ_C)
        {
            // REJ received -> retransmit
            timeoutCount++;
            printf(">>> Received REJ-%d - Retransmitting I-%d (retries so far: %d)\n", curr_seq, curr_seq, timeoutCount);
            continue;
        }
        else
        {
            // unexpected supervision frame -> retransmit
            timeoutCount++;
            printf(">>> Received unexpected supervision frame C=0x%02X - Retransmitting I-%d (retries so far: %d)\n", result, curr_seq, timeoutCount);
            continue;
        }
    }

    free(stuffedFrame);

    if (success)
    {
        return frameLength;
    }
    else
    {
        printf(">>> FAILED to transmit I-%d after %d retransmissions. Closing connection.\n", curr_seq, connection_params.nRetransmissions);
        if (serial_fd >= 0)
        {
            closeSerialPort();
            serial_fd = -1;
        }
        connection_active = FALSE;
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    if (!connection_active || serial_fd < 0)
        return -1;

    static int expected_Ns = 0;

    while (TRUE)
    {
        int step = START_STEP;
        int data_index = 0;
        unsigned char control = 0;
        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int received_Ns = 0;

        while (step != STOP_STEP)
        {
            unsigned char byte;
            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead < 0)
            {
                if (errno == EINTR)
                    continue;
                return -1;
            }

            if (bytesRead > 0)
            {
                switch (step)
                {
                case START_STEP:
                    if (byte == FLAG)
                        step = FLAG_STEP;
                    break;
                case FLAG_STEP:
                    if (byte == A)
                        step = A_STEP;
                    else if (byte != FLAG)
                        step = START_STEP;
                    break;
                case A_STEP:
                    // check for I-frame (A=0x03) or DISC frame
                    if (byte == C_I0 || byte == C_I1)
                    {
                        control = byte;
                        step = C_STEP;
                    }
                    else if (byte == C_DISC)
                    {
                        // termination request detected, reply DISC 
                        writeBytesSerialPort(DISC_reply, 5);
                        printf("Rx received DISC, sent DISC reply.\n");
                        return 0;
                    }
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
                        step = START_STEP;
                    break;
                case C_STEP:
                    if (byte == (A ^ control))
                    {
                        received_Ns = (control == C_I1) ? 1 : 0;
                        step = DATA_STEP;
                        data_index = 0;
                    }
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
                        step = START_STEP;
                    break;

                case DATA_STEP:
                    if (byte == ESC)
                    {
                        step = ESCAPE_STEP;
                    }
                    else if (byte == FLAG)
                    {
                        step = STOP_STEP;

                        if (data_index < 1)
                        {
                            step = START_STEP;
                            continue;
                        }

                        unsigned char received_bcc2 = buffer[data_index - 1];
                        int data_size = data_index - 1;

                        unsigned char calc_bcc2 = BCC2(buffer, data_size);

                        if (received_bcc2 == calc_bcc2)
                        {
                            // BCC2 CORRECT
                            if (received_Ns == expected_Ns)
                            {
                                // NEW frame - accept data and send RR for next seq
                                printf(">>> Received new frame. Sending RR-%d.\n", received_Ns);
                                memcpy(packet, buffer, data_size);
                                unsigned char *rr_frame = (expected_Ns == 0) ? RR1_t : RR0_t;
                                writeBytesSerialPort(rr_frame, 5);
                                expected_Ns = 1 - expected_Ns;
                                return data_size;
                            }
                            else
                            {
                                // DUPLICATE frame - Discard data and send RR for current expected seq
                                printf(">>> Received duplicate frame. Sending RR-%d.\n", received_Ns);
                                unsigned char *rr_frame = (expected_Ns == 0) ? RR0_t : RR1_t;
                                writeBytesSerialPort(rr_frame, 5);
                                step = START_STEP;
                            }
                        }
                        else
                        {
                            // BCC2 INCORRECT
                            if (received_Ns == expected_Ns)
                            {
                                // NEW frame with error - send REJ
                                printf(">>> BCC2 Error on I-%d for new frame. Sending REJ-%d.\n", received_Ns, received_Ns);
                                received_Ns = (control == C_I1) ? 1 : 0;
                                unsigned char *rej_frame = (received_Ns == 0) ? REJ0_t : REJ1_t;
                                writeBytesSerialPort(rej_frame, 5);
                            }
                            else
                            {
                                // DUPLICATE frame with error - send RR
                                printf(">>> BCC2 Error on I-%d for duplicate frame. Sending RR-%d.\n", received_Ns, received_Ns);
                                unsigned char *rr_frame = (expected_Ns == 0) ? RR0_t : RR1_t;
                                writeBytesSerialPort(rr_frame, 5);
                            }
                            step = START_STEP;
                        }
                    }
                    else
                    {
                        if (data_index < MAX_PAYLOAD_SIZE)
                        {
                            buffer[data_index++] = byte;
                        }
                        else
                        {
                            step = START_STEP;
                        }
                    }
                    break;

                case ESCAPE_STEP:
                    byte = byte ^ STUFF_BYTE;
                    if (data_index < MAX_PAYLOAD_SIZE)
                    {
                        buffer[data_index++] = byte;
                        step = DATA_STEP;
                    }
                    else
                    {
                        step = START_STEP;
                    }
                    break;

                default:
                    step = START_STEP;
                    break;
                }
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    if (!connection_active || serial_fd < 0)
        return -1;

    int tx_success = FALSE;
    timeoutCount = 0;

    if (connection_params.role == LlTx)
    {
        while (timeoutCount <= connection_params.nRetransmissions)
        {
            // 1. send DISC frame
            writeBytesSerialPort(DISC_cmd, 5);
            printf("Tx sent DISC (attempt %d/%d)\n", timeoutCount + 1, connection_params.nRetransmissions);

            // 2. read DISC reply
            if (read_supervision_frame(A_Rt, C_DISC, BCC1_DISC_t, TRUE))
            {
                printf("Tx received DISC reply.\n");
                
                // 3. send final UA
                writeBytesSerialPort(UA_reply, 5);
                printf("Tx sent final UA.\n");
                tx_success = TRUE;
                break;
            }
            if (timeoutCount >= connection_params.nRetransmissions)
                break;
            printf("TIMEOUT - Retransmitting DISC\n");
        }

        if (!tx_success)
        {
            printf("Tx failed to close connection correctly.\n");
        }
    }
    else // LlRx
    {
        // 1. read DISC
        if (read_supervision_frame(A, C_DISC, BCC1_DISC, FALSE))
        {
            // 2. echo DISC
            writeBytesSerialPort(DISC_reply, 5);

            // 3. wait for UA
            if (read_supervision_frame(A_Rt, C_UA, BCC1_UA_t, FALSE))
            {
                printf("Rx received final UA.\n");

                // close port
                if (closeSerialPort() == -1)
                    return -1;
                serial_fd = -1;
                connection_active = FALSE;
                tx_success = TRUE;
            }
        }
    }

    return tx_success ? 0 : -1;
}