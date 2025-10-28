// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define STUFF_BYTE 0x20

#define FLAG 0x7E
#define ESC 0x7D

#define A 0x03    // cmds sent by the Transmitter or replies sent by the receiver
#define A_Rt 0x01 // cmds sent by the Receiver or replies sent by the transmitter

#define C_UA 0x07
#define C_SET 0x03
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define C_I0 0x00
#define C_I1 0x80 // in binary: 1000000

#define BCC1_SET (A ^ C_SET)
#define BCC1_UA (A ^ C_UA)
#define BCC1_UA_r (A_Rt ^ C_UA)
#define BCC1_DISC (A ^ C_DISC)
#define BCC1_DISC_r (A_Rt ^ C_DISC)
#define BCC1_RR0 (A ^ C_RR0)
#define BCC1_RR1 (A ^ C_RR1)
#define BCC1_REJ0 (A ^ C_REJ0)
#define BCC1_REJ1 (A ^ C_REJ1)

#define START_STEP 0
#define FLAG_STEP 1
#define A_STEP 2
#define C_STEP 3
#define BCC1_STEP 4
#define DATA_STEP 5
#define FOUND_ESC_STEP 6
#define STOP_STEP 7

unsigned char UA[5] = {FLAG, A, C_UA, BCC1_UA, FLAG};
unsigned char UA_reply[5] = {FLAG, A_Rt, C_UA, BCC1_UA_r, FLAG};
unsigned char SET[5] = {FLAG, A, C_SET, BCC1_SET, FLAG};

// supervision frames
unsigned char RR0[5] = {FLAG, A, C_RR0, BCC1_RR0, FLAG};    // ACK for frame 0
unsigned char RR1[5] = {FLAG, A, C_RR1, BCC1_RR1, FLAG};    // ACK for frame 1
unsigned char REJ0[5] = {FLAG, A, C_REJ0, BCC1_REJ0, FLAG}; // NACK for frame 0
unsigned char REJ1[5] = {FLAG, A, C_REJ1, BCC1_REJ1, FLAG}; // NACK for frame 1

unsigned char DISC[5] = {FLAG, A, C_DISC, BCC1_DISC_r, FLAG}; // DISConnect

volatile int timeoutFlag = 0;

volatile int curr_seq = 0; // 0 or 1
volatile int connection_active = FALSE;
static int serial_fd = -1;

LinkLayer connection_params;

void alarmHandler(int signal)
{
    timeoutFlag = 1;
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

// ======= STATE MACHINES =======

int read_SET()
{
    int step = START_STEP;
    timeoutFlag = FALSE;
    alarm(connection_params.timeout);

    while (!timeoutFlag)
    {
        unsigned char byte;
        int bytesRead = readByteSerialPort(&byte);

        if (bytesRead > 0)
        {
            printf("SET SM - Byte: 0x%02X, State: %d\n", byte, step);

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
                if (byte == C_SET)
                    step = C_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case C_STEP:
                if (byte == BCC1_SET)
                    step = BCC1_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case BCC1_STEP:
                if (byte == FLAG)
                {
                    alarm(0);
                    return 1; // success
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
    return 0; // timeout
}

int read_UA(int isFromTransmitter)
{
    int step = START_STEP;
    timeoutFlag = 0;
    alarm(connection_params.timeout);

    while (!timeoutFlag)
    {
        unsigned char byte;
        int bytesRead = readByteSerialPort(&byte);

        if (bytesRead > 0)
        {
            printf("Byte received: 0x%02X, State: %d\n", byte, step);

            switch (step)
            {
            case START_STEP:
                if (byte == FLAG)
                    step = FLAG_STEP;
                break;
            case FLAG_STEP:
                if ((!isFromTransmitter && byte == A) || (isFromTransmitter && byte == A_Rt))
                    step = A_STEP;
                else if (byte != FLAG)
                    step = START_STEP;
                break;
            case A_STEP:
                if (byte == C_UA)
                    step = C_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case C_STEP:
                if ((!isFromTransmitter && byte == BCC1_UA) || (isFromTransmitter && byte == BCC1_UA_r))
                    step = BCC1_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case BCC1_STEP:
                if (byte == FLAG)
                {
                    alarm(0);
                    return 1; // success
                }
                else
                    step = START_STEP;
                break;
            }
        }
    }
    return 0; // timeout
}

int read_DISC(int isEcho)
{
    int step = START_STEP;
    timeoutFlag = 0;
    alarm(connection_params.timeout);

    while (!timeoutFlag)
    {
        unsigned char byte;
        int bytesRead = readByteSerialPort(&byte);

        if (bytesRead > 0)
        {
            printf("Byte received: 0x%02X, State: %d\n", byte, step);

            switch (step)
            {
            case START_STEP:
                if (byte == FLAG)
                    step = FLAG_STEP;
                break;
            case FLAG_STEP:
                if ((!isEcho && byte == A) || (isEcho && byte == A_Rt))
                    step = A_STEP;
                else if (byte != FLAG)
                    step = START_STEP;
                break;
            case A_STEP:
                if (byte == C_DISC)
                    step = C_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case C_STEP:
                if ((!isEcho && byte == BCC1_DISC) || (isEcho && byte == BCC1_DISC_r))
                    step = BCC1_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case BCC1_STEP:
                if (byte == FLAG)
                {
                    alarm(0);
                    return 1; // success
                }
                else
                    step = START_STEP;
                break;
            }
        }
    }
    return 0; // timeout
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    connection_params = connectionParameters;
    connection_params.timeout = connectionParameters.timeout;
    connection_params.nRetransmissions = connectionParameters.nRetransmissions;

    (void)signal(SIGALRM, alarmHandler);

    if (connectionParameters.role == LlTx)
    {
        serial_fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (serial_fd < 0)
        {
            perror("error opening serial port");
            return -1;
        }

        for (int attempt = 0; attempt < connection_params.nRetransmissions; attempt++)
        {
            // send SET
            int bytes = writeBytesSerialPort(SET, 5);
            printf("SET sent (%d bytes) - attempt %d/%d\n", bytes, attempt + 1, connection_params.nRetransmissions);

            // wait for UA
            if (read_UA(FALSE)) // isFromTransmitter = FALSE
            {
                printf("Received UA - Connection established!\n");
                connection_active = TRUE;
                return 0;
            }
            else
            {
                printf("TIMEOUT on attempt %d/%d - Retransmitting SET\n", attempt + 1, connection_params.nRetransmissions);
            }
        }

        printf("Connection failed after %d attempts\n", connection_params.nRetransmissions);
        return -1;
    }
    else
    {
        serial_fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (serial_fd < 0)
        {
            perror("error opening serial port");
            return -1;
        }

        // Wait for SET frame
        if (read_SET())
        {
            printf("Received SET frame, sending UA...\n");

            // Send UA response
            int bytes = writeBytesSerialPort(UA, 5);
            printf("UA sent (%d bytes)\n", bytes);

            connection_active = TRUE;
            return 0;
        }
        else
        {
            printf("Timeout waiting for SET frame\n");
            return -1;
        }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    if (!connection_active)
    {
        printf("ERROR: Connection is not active!");
        return -1;
    }

    int maxStuffedSize = 5 + (bufSize * 2) + 2;
    unsigned char stuffedFrame[maxStuffedSize];
    int pos = 0;

    stuffedFrame[pos++] = FLAG;
    stuffedFrame[pos++] = A;

    unsigned char C = (curr_seq == 0) ? C_I0 : C_I1;
    stuffedFrame[pos++] = C;

    unsigned char bcc1 = A ^ C;
    stuffedFrame[pos++] = bcc1;

    for (int i = 0; i < bufSize; i++)
    {
        stuffAndAddByte(buf[i], stuffedFrame, &pos);
    }

    unsigned char bcc2 = BCC2(buf, bufSize);
    stuffAndAddByte(bcc2, stuffedFrame, &pos);

    stuffedFrame[pos++] = FLAG;

    int bytesWritten = 0;
    int retransmissions = 0;
    int success = FALSE;

    while (retransmissions < connection_params.nRetransmissions && !success)
    {

        bytesWritten = writeBytesSerialPort(stuffedFrame, pos);
        printf("Sent I-%d, %d bytes (attempt %d/%d)\n", curr_seq, bytesWritten,
               retransmissions + 1, connection_params.nRetransmissions);

        // wait for supervision frame
        int step = START_STEP;
        int result_type = -1;
        int frame_complete = FALSE;
        int should_retransmit = FALSE;

        timeoutFlag = FALSE;
        alarm(connection_params.timeout);

        while (!frame_complete && !timeoutFlag && !should_retransmit)
        {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);

            if (bytes > 0)
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
                    if (byte == C_RR0)
                    {
                        step = C_STEP;
                        result_type = 0;
                    }
                    else if (byte == C_RR1)
                    {
                        step = C_STEP;
                        result_type = 1;
                    }
                    else if (byte == C_REJ0)
                    {
                        step = C_STEP;
                        result_type = 2;
                    }
                    else if (byte == C_REJ1)
                    {
                        step = C_STEP;
                        result_type = 3;
                    }
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
                        step = START_STEP;
                    break;
                case C_STEP:
                    if ((result_type == 0 && byte == BCC1_RR0) ||
                        (result_type == 1 && byte == BCC1_RR1) ||
                        (result_type == 2 && byte == BCC1_REJ0) ||
                        (result_type == 3 && byte == BCC1_REJ1))
                    {
                        step = BCC1_STEP;
                    }
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
                        step = START_STEP;
                    break;
                case BCC1_STEP:
                    if (byte == FLAG)
                    {
                        frame_complete = TRUE;
                        alarm(0);

                        if (result_type == 0) // RR0 - acknowledges I0
                        {
                            printf("Received RR0 - I0 acknowledged\n");
                            if (curr_seq == 0)
                            {
                                curr_seq = 1;
                                success = TRUE;
                            }
                            else
                            {
                                printf("ERROR: Sequence mismatch - sent I%d but got RR0\n", curr_seq);
                                should_retransmit = TRUE;
                            }
                        }
                        else if (result_type == 1) // RR1 - acknowledges I1
                        {
                            printf("Received RR1 - I1 acknowledged\n");
                            if (curr_seq == 1)
                            {
                                curr_seq = 0;
                                success = TRUE;
                            }
                            else
                            {
                                printf("ERROR: Sequence mismatch - sent I%d but got RR1\n", curr_seq);
                                should_retransmit = TRUE;
                            }
                        }
                        else if (result_type == 2) // REJ0 - reject I0
                        {
                            printf("REJ0 received - will retransmit I0\n");
                            if (curr_seq == 0)
                            {
                                should_retransmit = TRUE;
                            }
                            else
                            {
                                printf("ERROR: REJ0 received but we sent I%d\n", curr_seq);
                            }
                        }
                        else if (result_type == 3) // REJ1 - reject I1
                        {
                            printf("REJ1 received - will retransmit I1\n");
                            if (curr_seq == 1)
                            {
                                should_retransmit = TRUE;
                            }
                            else
                            {
                                printf("ERROR: REJ1 received but we sent I%d\n", curr_seq);
                            }
                        }
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

        if (timeoutFlag)
        {
            printf("TIMEOUT on attempt %d/%d - will retransmit I%d\n",
                   retransmissions + 1, connection_params.nRetransmissions, curr_seq);
            should_retransmit = TRUE;
        }

        // count retransmission if we need to resend
        if (should_retransmit && !success)
        {
            retransmissions++;
            printf("Retransmitting I-%d (attempt %d/%d)\n", curr_seq,
                   retransmissions + 1, connection_params.nRetransmissions);
        }
    }

    alarm(0);

    if (success)
    {
        printf("I-%d successfully transmitted after %d attempts\n",
               1 - curr_seq, retransmissions + 1);
        return bytesWritten;
    }
    else
    {
        printf("Failed to transmit I-%d after %d retransmissions\n",
               curr_seq, retransmissions);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    if (!connection_active)
    {
        printf("ERROR: Connection is not active!");
        return -1;
    }

    unsigned char buffer[1024];
    int step = START_STEP;
    int data_index = 0;
    unsigned char control;
    int destuff_next = FALSE; // next byte needs destuffing

    while (step != STOP_STEP)
    {
        unsigned char byte;
        int bytesRead = readByteSerialPort(&byte);

        if (bytesRead > 0)
        {
            printf("Byte received: 0x%02X, Step: %d, Destuff?: %d \n", byte, step, destuff_next);

            if (destuff_next)
            {
                byte = byte ^ STUFF_BYTE;
                destuff_next = FALSE;
                printf("Destuffed to: 0x%02X\n", byte);
            }
            else if (byte == ESC)
            {
                destuff_next = TRUE;
                continue; // skip to next byte
            }

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
                // if byte == FLAG => stay
                break;

            case A_STEP:
                if (byte == C_I0 || byte == C_I1)
                {
                    control = byte;
                    step = C_STEP;
                }
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;

                break;

            case C_STEP:
                if (byte == (A ^ control))
                {
                    step = BCC1_STEP;
                    data_index = 0; // reset data buffer
                }
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;

                break;

            case BCC1_STEP:
                // store data bytes (including potential BCC2)
                buffer[data_index++] = byte;

                if (byte == FLAG)
                {
                    // last stored byte before FLAG should be BCC2
                    if (data_index >= 2)
                    {
                        unsigned char rcv_bcc2 = buffer[data_index - 2]; // second to last is BCC2
                        int actual_data_size = data_index - 2;           // - BCC2 - FLAG

                        unsigned char calc_bcc2 = BCC2(buffer, actual_data_size);

                        if (rcv_bcc2 == calc_bcc2) // BCC OK
                        {
                            step = STOP_STEP;

                            // send ACK
                            if (control == C_I0)
                            {
                                writeBytesSerialPort(RR1, 5);
                                printf("Answered with RR1. Ready to Receive I1!\n");
                            }
                            else
                            {
                                writeBytesSerialPort(RR0, 5);
                                printf("Answered with RR0. Ready to Receive I0!\n");
                            }

                            // Copy data to output packet
                            for (int i = 0; i < actual_data_size; i++)
                            {
                                packet[i] = buffer[i];
                            }

                            return actual_data_size;
                        }
                        else
                        { // BCC ERROR
                            printf("FLAG in data detected, continuing...\n");
                        }
                    }
                    else
                    {
                        printf("ERROR: Frame too short\n");
                        return -1;
                    }
                }
                break;

            default:
                step = START_STEP;
                break;
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
    if (!connection_active)
    {
        printf("ERROR: no active connection to close\n");
        return -1;
    }
    if (serial_fd < 0)
    {
        printf("ERROR: serial port not properly opened\n");
        return -1;
    }

    if (connection_params.role == LlTx)
    {
        for (int attempt = 0; attempt < connection_params.nRetransmissions; attempt++)
        {
            writeBytesSerialPort(DISC, 5);

            if (read_DISC(TRUE)) // isEcho = TRUE -> echoed DISC
            {
                // send UA
                writeBytesSerialPort(UA_reply, 5);

                // close port
                closeSerialPort();
                connection_active = FALSE;
                serial_fd = -1;
                return 0;
            }
        }
        return -1;
    }
    else
    {
        if (read_DISC(FALSE)) // isEcho = FALSE -> og DISC
        {
            // echo DISC
            writeBytesSerialPort(DISC, 5);

            if (read_UA(TRUE)) // isFromTransmitter = TRUE -> UA_reply
            {
                // close port
                closeSerialPort();
                connection_active = FALSE;
                return 0;
            }
        }
    }
}