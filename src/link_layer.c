// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

unsigned char UA[5] = {
    0x7E, // FLAG
    0x03, // Access
    0x07, // Control
    0x04, // BCC -- A XOR C = 0x03 XOR 0x07
    0x7E  // FLAG
};

unsigned char SET[5] = {
    0x7E,
    0x03,
    0x03,
    0x00,
    0x7E};

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
                if (byte == 0X7E)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 1: // flag step
                if (byte == 0x03)
                {
                    step = 2;
                }
                else if (byte == 0x7E)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 2: // A step
                if (byte == 0x03)
                {
                    step = 3;
                }
                else if (byte == 0x7E)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 3: // C step
                if (byte == 0x00)
                {
                    step = 4;
                }
                else if (byte == 0x7E)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 4: // BCC step
                if (byte == 0x7E)
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
                    if (byte == 0X7E)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 1: // flag step
                    if (byte == 0x03)
                    {
                        step = 2;
                    }
                    else if (byte == 0x7E)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 2: // A step
                    if (byte == 0x07)
                    {
                        step = 3;
                    }
                    else if (byte == 0x7E)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 3: // C step
                    if (byte == 0x04)
                    {
                        step = 4;
                    }
                    else if (byte == 0x7E)
                    {
                        step = 1;
                    }
                    else
                    {
                        step = 0;
                    }
                    break;
                case 4: // BCC step
                    if (byte == 0x7E)
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