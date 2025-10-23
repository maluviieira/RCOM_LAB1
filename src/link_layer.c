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

#define A 0x03 // cmds sent by the transmitter or replies sent by the Receiver

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
volatile int connection_active = FALSE;

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
            case START_STEP:
                if (byte == FLAG)
                    step = FLAG_STEP;
                break;
            case FLAG_STEP:
                if (byte == A)
                    step = A_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
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
                if (byte == BCC_SET)
                    step = BCC1_STEP;
                else if (byte == FLAG)
                    step = FLAG_STEP;
                else
                    step = START_STEP;
                break;
            case BCC1_STEP:
                if (byte == FLAG)
                {
                    printf("Received the whole SET. Connection active here!\n");
                    STOP = TRUE;
                    SUCCESS = TRUE;
                }
                else
                    step = START_STEP;
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
            connection_active = TRUE;
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
                case START_STEP:
                    if (byte == FLAG)
                        step = FLAG_STEP;
                    break;
                case FLAG_STEP:
                    if (byte == A)
                        step = A_STEP;
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
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
                    if (byte == BCC_UA)
                        step = BCC1_STEP;
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
                        step = START_STEP;
                    break;
                case BCC1_STEP:
                    if (byte == FLAG)
                    {
                        printf("Received the whole UA. Connection active here!\n");
                        alarm(0);
                        connection_active = TRUE;
                        return 0;
                    }
                    else
                        step = START_STEP;
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

    int maxStuffedSize = 5 + (bufSize * 2) + 2;
    unsigned char stuffedFrame[maxStuffedSize];
    int pos = 0;

    stuffAndAddByte(FLAG, stuffedFrame, &pos);
    stuffAndAddByte(A, stuffedFrame, &pos);

    unsigned char C = (curr_seq == 0) ? C_I0 : C_I1;
    stuffAndAddByte(C, stuffedFrame, &pos);

    unsigned char bcc1 = A ^ C;
    stuffAndAddByte(bcc1, stuffedFrame, &pos);

    for (int i = 0; i < bufSize; i++)
    {
        stuffAndAddByte(buf[i], stuffedFrame, &pos);
    }

    unsigned char bcc2 = BCC2(buf, bufSize);
    stuffAndAddByte(bcc2, stuffedFrame, &pos);

    stuffAndAddByte(FLAG, stuffedFrame, &pos);

    int bytesWritten = 0;
    int retransmissions = 0;
    int maxRetransmissions = 3;

    while (retransmissions < maxRetransmissions)
    {
        // send the I frame
        bytesWritten = writeBytesSerialPort(stuffedFrame, pos);
        printf("Sent I-%d, %d bytes\n", curr_seq, bytesWritten);

        // wait for supervision frame
        int step = START_STEP;
        int result_type = -1; // 0=RR0, 1=RR1, 2=REJ0, 3=REJ1
        int frame_complete = FALSE;
        int timeout_counter = 0;

        while (!frame_complete && timeout_counter < 1000) // temp timeout
        {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);

            if (bytes > 0)
            {
                printf("Byte received: 0x%02X\n", byte);

                switch (step)
                {
                case START_STEP:
                    if (byte == FLAG)
                        step = FLAG_STEP;
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
                    {
                        step = FLAG_STEP;
                    }
                    else
                    {
                        step = START_STEP;
                    }
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
                        frame_complete = TRUE;

                        // response
                        if (result_type == 0) // RR0
                        {
                            printf("Received RR0 - ACK for frame 0\n");
                            if (curr_seq == 0) // success
                            {
                                curr_seq = 1;
                                return bytesWritten;
                            }
                            else
                            {
                                printf("ERROR: Sequence mismatch - sent I1 but got RR0\n");
                            }
                        }
                        else if (result_type == 1) // RR1
                        {
                            printf("Received RR1 - ACK for frame 1\n");
                            if (curr_seq == 1) // success
                            {
                                curr_seq = 0;
                                return bytesWritten;
                            }
                            else
                            {
                                printf("ERROR: Sequence mismatch - sent I0 but got RR1\n");
                            }
                        }
                        else if (result_type == 2) // REJ0
                        {
                            printf("REJ0 received - retransmitting\n");
                            break; // break inner loop to retransmit
                        }
                        else if (result_type == 3) // REJ1
                        {
                            printf("REJ1 received - retransmitting\n");
                            break;
                        }
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
            timeout_counter++;
        }

        retransmissions++;
        printf("Retransmission attempt %d/%d\n", retransmissions, maxRetransmissions);
    }

    printf("Failed after %d retransmissions\n", retransmissions);
    return -1;
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
                                writeBytesSerialPort(RR0, 5);
                                printf("Answered with RR0\n");
                            }
                            else
                            {
                                writeBytesSerialPort(RR1, 5);
                                printf("Answered with RR1\n");
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
    // TODO: Implement this function

    return 0;
}