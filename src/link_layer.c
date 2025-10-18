// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define ESC 0x7D

#define A 0x03

#define C_UA 0x07
#define C_SET 0x03
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55

#define BCC_SET (A ^ C_SET)
#define BCC_UA (A ^ C_UA)
#define BCC1_RR0 (A ^ C_RR0)
#define BCC1_RR1 (A ^ C_RR1)
#define BCC1_REJ0 (A ^ C_REJ0)
#define BCC1_REJ1 (A ^ C_REJ1)


unsigned char UA[5] = {
    FLAG,
    A,
    C_UA,
    BCC_UA,
    FLAG
};

unsigned char SET[5] = {
    FLAG,
    A,
    C_SET,
    BCC_SET,
    FLAG
};

unsigned char RR0[5] = {FLAG, A, C_RR0, BCC1_RR0, FLAG};  // ACK for frame 0
unsigned char RR1[5] = {FLAG, A, C_RR1, BCC1_RR1, FLAG};  // ACK for frame 1
unsigned char REJ0[5] = {FLAG, A, C_REJ0, BCC1_REJ0, FLAG}; // NACK for frame 0
unsigned char REJ1[5] = {FLAG, A, C_REJ1, BCC1_REJ1, FLAG}; // NACK for frame 1

volatile int timeoutFlag = 0;
void alarmHandler(int signal)
{
    timeoutFlag = 1; 
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
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}