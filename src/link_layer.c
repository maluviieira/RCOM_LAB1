// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define DESTUFF_BYTE 0x20

#define FLAG 0x7E
#define ESC 0x7D

#define A 0x03

#define C_UA 0x07
#define C_SET 0x03
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define C_I0 0x00
#define C_I1 0x80 // in binary: 1000000

#define BCC_SET (A ^ C_SET)
#define BCC_UA (A ^ C_UA)
#define BCC_DISC (A ^ C_DISC)
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

unsigned char UA[5] = {FLAG, A, C_UA, BCC_UA, FLAG};
unsigned char SET[5] = {FLAG, A, C_SET, BCC_SET, FLAG};

// supervision frames
unsigned char RR0[5] = {FLAG, A, C_RR0, BCC1_RR0, FLAG};    // ACK for frame 0
unsigned char RR1[5] = {FLAG, A, C_RR1, BCC1_RR1, FLAG};    // ACK for frame 1
unsigned char REJ0[5] = {FLAG, A, C_REJ0, BCC1_REJ0, FLAG}; // NACK for frame 0
unsigned char REJ1[5] = {FLAG, A, C_REJ1, BCC1_REJ1, FLAG}; // NACK for frame 1

unsigned char DISC[5] = {FLAG, A, C_DISC, BCC_DISC, FLAG}; // DISConnect

volatile int timeoutFlag = 0;

void alarmHandler(int signal)
{
    timeoutFlag = 1;
}

volatile int curr_seq = 0; // 0 or 1
volatile int connection_active = 0;

int byteStuffing(const unsigned char *input, int size, char *output)
{
    int j = 0;

    for (int i = 0; i < size; i++)
    {
        if (input[i] == FLAG)
        {
            output[j++] = ESC;
            output[j++] = FLAG ^ 0x20;
        }
        else if (input[i] == ESC)
        {
            output[j++] = ESC;
            output[j++] = ESC ^ 0x20;
        }
        else
        {
            output[j++] = input[i];
        }
    }
    return j;
}

int byteDestuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int j = 0;
    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == ESC)
        {
            i++; // skip escape byte
            if (i < inputSize)
            {
                output[j++] = input[i] ^ DESTUFF_BYTE; // recover original
            }
        }
        else
        {
            output[j++] = input[i];
        }
    }
    return j;
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

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{

    volatile int STOP = FALSE;
    volatile int SUCCESS = FALSE;

    if (connectionParameters.role == LlRx)
    {
        int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (fd < 0)
        {
            perror("error opening serial port");
            return -1;
        }

        int nBytesBuf = 0;

        int step = 0;
        while (STOP == FALSE)
        {
            unsigned char byte;

            int bytes = readByteSerialPort(&byte);
            nBytesBuf += bytes;
            printf("Byte received: %d\n", byte);

            switch (step)
            {
            case 0: // start
                if (byte == FLAG)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 1: // flag step
                if (byte == A)
                {
                    step = 2;
                }
                else if (byte == FLAG)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 2: // A step
                if (byte == C_SET)
                {
                    step = 3;
                }
                else if (byte == FLAG)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 3: // C step
                if (byte == BCC_SET)
                {
                    step = 4;
                }
                else if (byte == FLAG)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 4: // BCC step
                if (byte == FLAG)
                {
                    printf("Received the whole SET. Stop reading from serial port.\n");
                    STOP = TRUE;
                    SUCCESS = TRUE;
                }
                else
                {
                    step = 0;
                }
                break;
            default:
                break;
            }
        }

        printf("Total bytes received: %d\n", nBytesBuf);

        if (SUCCESS == TRUE)
        {
            int bytes = writeBytesSerialPort(UA, 5);
            printf("%d bytes written to serial port\n", bytes);
            printf("UA sent.\n");
            sleep(1);
            connection_active = 1;
            return 0;
        }
    }

    else
    { // LlTx
        int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (fd < 0)
        {
            perror("error opening serial port");
            return -1;
        }

        (void)signal(SIGALRM, alarmHandler);

        int nBytesBuf = 0;
        int step = 0;

        for (int attempt = 0; attempt < connectionParameters.nRetransmissions; attempt++)
        {

            int bytes = writeBytesSerialPort(SET, 5);
            printf("%d bytes written to serial port\n", bytes);

            timeoutFlag = 0;
            alarm(connectionParameters.timeout);

            while (!timeoutFlag)
            {
                unsigned char byte;

                int bytes = readByteSerialPort(&byte);
                nBytesBuf += bytes;
                printf("Byte received: %d\n", byte);

                switch (step)
                {
                case 0: // start
                    if (byte == FLAG)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 1: // flag step
                    if (byte == A)
                    {
                        step = 2;
                    }
                    else if (byte == FLAG)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 2: // A step
                    if (byte == C_UA)
                    {
                        step = 3;
                    }
                    else if (byte == FLAG)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 3: // C step
                    if (byte == BCC_UA)
                    {
                        step = 4;
                    }
                    else if (byte == FLAG)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 4: // BCC step
                    if (byte == FLAG)
                    {
                        printf("Received the whole UA. Stop reading from serial port.\n");
                        alarm(0);
                        connection_active = 1;
                        return 0;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                default:
                    break;
                }
            }
        }

        printf("Timeout! Retrying...\n");
    }
    printf("Connection failed after %d attempts.\n", connectionParameters.nRetransmissions);
    return -1;
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

    unsigned char C = (curr_seq == 0) ? C_I0 : C_I1;
    unsigned char bcc1 = A ^ C;
    unsigned char bcc2 = BCC2(buf, bufSize);

    // build frame
    int frameSize = 4 + bufSize + 2; // FLAG+A+C+BCC1+ data +BCC2+FLAG
    unsigned char frame[frameSize];
    int pos = 0;

    frame[pos++] = FLAG;
    frame[pos++] = A;
    frame[pos++] = C;
    frame[pos++] = bcc1;

    for (int i = 0; i < bufSize; i++)
    {
        frame[pos++] = buf[i];
    }

    frame[pos++] = bcc2;
    frame[pos++] = FLAG;

    // byte stuffing
    unsigned char stuffedFrame[frameSize * 2]; // worst case = every byte stuffed
    int stuffedSize = byteStuffing(frame, frameSize, stuffedFrame);

    // send frame
    int bytesWritten = writeBytesSerialPort(stuffedFrame, stuffedSize);
    printf("Sent I-frame with seq=%d, %d bytes\n", curr_seq, bytesWritten);

    return bufSize;
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
    int bufferPos = 0;
    int step = START_STEP;
    int dataSize = 0;
    unsigned char address, control, bcc1, bcc2;
    int data_index = 0;

    while (step != STOP_STEP)
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
                {
                    step = FLAG_STEP;
                }
                break;

            case FLAG_STEP:
                if (byte == A)
                {
                    step = A_STEP;
                }
                else if (byte != FLAG)
                {
                    step = START_STEP;
                }
                // if byte == FLAG stay
                break;

            case A_STEP:
                if (byte == C_I0 || byte == C_I1)
                {
                    control = byte;
                    step = C_STEP;
                }
                else if (byte == FLAG)
                {
                    step = FLAG_STEP;
                }
                else
                {
                    step = START_STEP;
                }
                break;

            case C_STEP:
                if (byte == (A ^ control))
                { // bcc1
                    step = BCC1_STEP;
                }
                else if (byte == FLAG)
                {
                    step = FLAG_STEP;
                }
                else
                {
                    step = START_STEP;
                }
                break;

            case BCC1_STEP:
                if (byte == FLAG)
                {
                    // empty frame

                    step = STOP_STEP;

                    // send ACK
                    if (control == C_I0)
                    {
                        writeBytesSerialPort(RR0, 5);
                    }
                    else
                    {
                        writeBytesSerialPort(RR1, 5);
                    }

                    return 0;
                }
                else if (byte == ESC)
                {
                    step = FOUND_ESC_STEP;
                }
                else
                {
                    // normal data byte
                    buffer[data_index++] = byte;
                    step = DATA_STEP;
                }
                break;

            case DATA_STEP:
                if (byte == FLAG)
                {
                    step = STOP_STEP;

                    unsigned char rcv_bcc2 = buffer[data_index - 1];
                    int actual_data_size = data_index - 1;

                    unsigned char calc_bcc2 = BCC2(buffer, actual_data_size);

                    if (rcv_bcc2 == calc_bcc2)
                    { // success!
                        // send ACK
                        if (control == C_I0)
                        {
                            writeBytesSerialPort(RR0, 5);
                        }
                        else
                        {
                            writeBytesSerialPort(RR1, 5);
                        }

                        // copy data from temporary buffer to output packet
                        for (int i = 0; i < actual_data_size; i++)
                        {
                            packet[i] = buffer[i];
                        }

                        return actual_data_size;
                    }
                    else
                    { // bcc2 error
                        // send NACK
                        if (control == C_I0)
                        {
                            writeBytesSerialPort(REJ0, 5);
                            printf("Sent REJ0 - error in frame 0\n");
                        }
                        else
                        {
                            writeBytesSerialPort(REJ1, 5);
                            printf("Sent REJ1 - error in frame 1\n");
                        }
                        return -1;
                    }
                }
                else if (byte == ESC)
                {
                    step = FOUND_ESC_STEP;
                }
                else
                { // normal data byte
                    buffer[data_index++] = byte;
                }

                break;

            case FOUND_ESC_STEP:
                // destuffing
                unsigned char og_byte = byte ^ DESTUFF_BYTE;
                buffer[data_index++] = og_byte;
                
                step = DATA_STEP;
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
    // TODO: Implement this function

    return 0;
}