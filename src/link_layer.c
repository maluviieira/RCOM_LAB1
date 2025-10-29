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

// :P
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
#define ESCAPE_STEP 6
#define STOP_STEP 7

unsigned char SET[5] = {FLAG, A, C_SET, BCC1_SET, FLAG};

unsigned char UA[5] = {FLAG, A, C_UA, BCC1_UA, FLAG};
unsigned char UA_reply[5] = {FLAG, A_Rt, C_UA, BCC1_UA_r, FLAG};

unsigned char DISC[5] = {FLAG, A, C_DISC, BCC1_DISC, FLAG};           // DISConnect
unsigned char DISC_echo[5] = {FLAG, A_Rt, C_DISC, BCC1_DISC_r, FLAG}; // DISC to be echoed by R

// supervision frames
unsigned char RR0[5] = {FLAG, A, C_RR0, BCC1_RR0, FLAG};    // ACK for frame 1 -> Ready to Receive I1
unsigned char RR1[5] = {FLAG, A, C_RR1, BCC1_RR1, FLAG};    // ACK for frame 0 -> Ready to Receive I0
unsigned char REJ0[5] = {FLAG, A, C_REJ0, BCC1_REJ0, FLAG}; // NACK for frame 0
unsigned char REJ1[5] = {FLAG, A, C_REJ1, BCC1_REJ1, FLAG}; // NACK for frame 1

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

                        if (result_type == 0) // RR0 - acknowledges I1 (N(r)=0)
                        {
                            printf("Received RR0 - I1 acknowledged\n");
                            if (curr_seq == 1) // We sent I1, got RR0 - correct!
                            {
                                curr_seq = 0;
                                success = TRUE;
                            }
                            else
                            {
                                printf("ERROR: Sequence mismatch - sent I%d but got RR0\n", curr_seq);
                                should_retransmit = TRUE;
                            }
                        }
                        else if (result_type == 1) // RR1 - acknowledges I0 (N(r)=1)
                        {
                            printf("Received RR1 - I0 acknowledged\n");
                            if (curr_seq == 0) // We sent I0, got RR1 - correct!
                            {
                                curr_seq = 1;
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
    static int expected_seq = 0;
    int is_duplicate = FALSE;

    printf("=== LLREAD STARTING - expected_seq=%d ===\n", expected_seq);

    while (1) // Loop until we get a correct I frame
    {
        step = START_STEP;
        data_index = 0;
        is_duplicate = FALSE;

        // Read one complete frame
        while (step != STOP_STEP)
        {
            unsigned char byte;
            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead > 0)
            {
                // printf("Byte received: 0x%02X, Step: %d\n", byte, step);

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
                    if (byte == C_I0 || byte == C_I1)
                    {
                        control = byte;
                        step = C_STEP;
                        printf(">>> CONTROL FIELD: %s (0x%02X)\n", 
                               (control == C_I0) ? "C_I0" : "C_I1", control);
                    }
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
                        step = START_STEP;
                    break;

                case C_STEP:
                    // Check BCC1
                    if (byte == (A ^ control))
                    {
                        // BCC1 is correct - check sequence number
                        int received_seq = (control == C_I1) ? 1 : 0;
                        
                        printf(">>> SEQUENCE CHECK: received_seq=%d, expected_seq=%d\n", 
                               received_seq, expected_seq);
                        
                        if (received_seq == expected_seq)
                        {
                            // New frame - expected sequence
                            is_duplicate = FALSE;
                            printf(">>> NEW FRAME (seq %d matches expected %d)\n", 
                                   received_seq, expected_seq);
                            step = DATA_STEP;
                            data_index = 0;
                        }
                        else
                        {
                            // Duplicate frame - wrong sequence
                            is_duplicate = TRUE;
                            printf(">>> DUPLICATE FRAME (seq %d != expected %d) - DISCARDING\n", 
                                   received_seq, expected_seq);
                            step = DATA_STEP;
                            data_index = 0;
                        }
                    }
                    else if (byte == FLAG)
                        step = FLAG_STEP;
                    else
                        step = START_STEP; // BCC1 incorrect - ignore frame
                    break;

                case DATA_STEP:
                    if (data_index >= 1024)
                    {
                        printf("Buffer overflow!\n");
                        step = START_STEP;
                        break;
                    }

                    if (byte == ESC)
                    {
                        step = ESCAPE_STEP;
                    }
                    else if (byte == FLAG)
                    {
                        // End of frame reached - process the complete frame
                        if (data_index >= 1) // Need at least BCC2
                        {
                            unsigned char received_bcc2 = buffer[data_index - 1];
                            int data_size = data_index - 1; // Data size excluding BCC2

                            // Calculate BCC2 from received data
                            unsigned char calc_bcc2 = BCC2(buffer, data_size);

                            printf(">>> BCC2 CHECK: received=0x%02X, calculated=0x%02X, data_size=%d\n", 
                                   received_bcc2, calc_bcc2, data_size);
                            printf(">>> FRAME STATUS: seq=%d, duplicate=%d, expected_seq=%d\n", 
                                   (control == C_I1) ? 1 : 0, is_duplicate, expected_seq);

                            // Handle based on BCC2 result and sequence
                            if (received_bcc2 == calc_bcc2)
                            {
                                // BCC2 correct
                                if (!is_duplicate)
                                {
                                    // New frame with correct BCC2 - send RR for next frame
                                    if (control == C_I0)
                                    {
                                        writeBytesSerialPort(RR1, 5);
                                        printf(">>> SUCCESS: I0 received - Sent RR1\n");
                                    }
                                    else
                                    {
                                        writeBytesSerialPort(RR0, 5);
                                        printf(">>> SUCCESS: I1 received - Sent RR0\n");
                                    }

                                    // TOGGLE expected sequence
                                    int old_expected = expected_seq;
                                    expected_seq = 1 - expected_seq;
                                    printf(">>> UPDATED: expected_seq %d -> %d\n", 
                                           old_expected, expected_seq);

                                    // Copy data to packet (excluding BCC2)
                                    for (int i = 0; i < data_size; i++)
                                    {
                                        packet[i] = buffer[i];
                                    }
                                    
                                    printf(">>> RETURNING data_size=%d\n", data_size);
                                    return data_size;
                                }
                                else
                                {
                                    // Duplicate frame with correct BCC2 - send RR for current expected frame
                                    if (expected_seq == 0)
                                    {
                                        writeBytesSerialPort(RR0, 5);
                                        printf(">>> DUPLICATE: Sent RR0 (still expecting I0)\n");
                                    }
                                    else
                                    {
                                        writeBytesSerialPort(RR1, 5);
                                        printf(">>> DUPLICATE: Sent RR1 (still expecting I1)\n");
                                    }
                                    step = START_STEP;
                                }
                            }
                            else
                            {
                                // BCC2 incorrect
                                if (!is_duplicate)
                                {
                                    // New frame with BCC2 error - send REJ
                                    if (control == C_I0)
                                    {
                                        writeBytesSerialPort(REJ0, 5);
                                        printf(">>> BCC2 ERROR: I0 - Sent REJ0\n");
                                    }
                                    else
                                    {
                                        writeBytesSerialPort(REJ1, 5);
                                        printf(">>> BCC2 ERROR: I1 - Sent REJ1\n");
                                    }
                                }
                                else
                                {
                                    // Duplicate frame with BCC2 error
                                    if (expected_seq == 0)
                                    {
                                        writeBytesSerialPort(RR0, 5);
                                        printf(">>> DUPLICATE+BCC2 ERROR: Sent RR0\n");
                                    }
                                    else
                                    {
                                        writeBytesSerialPort(RR1, 5);
                                        printf(">>> DUPLICATE+BCC2 ERROR: Sent RR1\n");
                                    }
                                }
                                step = START_STEP;
                            }
                        }
                        else
                        {
                            printf("Frame too short\n");
                            step = START_STEP;
                        }
                    }
                    else
                    {
                        // Normal data byte
                        buffer[data_index++] = byte;
                    }
                    break;

                case ESCAPE_STEP:
                    // Destuff the byte
                    byte = byte ^ STUFF_BYTE;
                    // printf("Destuffed to: 0x%02X\n", byte);
                    
                    if (data_index >= 1024)
                    {
                        printf("Buffer overflow!\n");
                        step = START_STEP;
                    }
                    else
                    {
                        buffer[data_index++] = byte;
                        step = DATA_STEP;
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
            writeBytesSerialPort(DISC_echo, 5);

            if (read_UA(TRUE)) // isFromTransmitter = TRUE -> UA_reply
            {
                alarm(0);

                // close port
                closeSerialPort();
                connection_active = FALSE;
                return 0;
            }
        }
    }
}