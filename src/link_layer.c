// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h> // For malloc/free

// --- CONSTANTS AND MACROS (Assuming these are in link_layer.h) ---
#define TRUE 1
#define FALSE 0

#define STUFF_BYTE 0x20

#define FLAG 0x7E
#define ESC 0x7D

#define A 0x03    // Cmds/Replies for Tx/Rx (A-Tx / A-Rx reply)
#define A_Rt 0x01 // Cmds/Replies for Rx/Tx (A-Rx / A-Tx reply)

#define C_UA 0x07
#define C_SET 0x03
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define C_I0 0x00
#define C_I1 0x80

// BCC1 calculation for Command Frames (A ^ C)
#define BCC1_SET (A ^ C_SET)
#define BCC1_DISC (A ^ C_DISC)

// BCC1 calculation for Reply Frames from Receiver (A ^ C)
#define BCC1_UA_r (A ^ C_UA)
#define BCC1_RR0_r (A ^ C_RR0)
#define BCC1_RR1_r (A ^ C_RR1)
#define BCC1_REJ0_r (A ^ C_REJ0)
#define BCC1_REJ1_r (A ^ C_REJ1)

// BCC1 calculation for Reply Frames from Transmitter (A_Rt ^ C)
#define BCC1_UA_t (A_Rt ^ C_UA)
#define BCC1_DISC_t (A_Rt ^ C_DISC)

// State machine steps
#define START_STEP 0
#define FLAG_STEP 1
#define A_STEP 2
#define C_STEP 3
#define BCC1_STEP 4
#define DATA_STEP 5
#define ESCAPE_STEP 6
#define STOP_STEP 7

// --- GLOBAL VARIABLES (As used in your original file) ---
unsigned char SET[5] = {FLAG, A, C_SET, BCC1_SET, FLAG};
unsigned char UA_reply[5] = {FLAG, A_Rt, C_UA, BCC1_UA_t, FLAG}; // UA sent by Tx to finish close
unsigned char UA_cmd[5] = {FLAG, A, C_UA, BCC1_UA_r, FLAG};      // UA sent by Rx to ack SET

unsigned char DISC_cmd[5] = {FLAG, A, C_DISC, BCC1_DISC, FLAG};           // DISC sent by Tx
unsigned char DISC_reply[5] = {FLAG, A_Rt, C_DISC, BCC1_DISC_t, FLAG}; // DISC sent by Rx

// Supervision frames (Replies from Receiver A_Rt, but checking the Reply A field from Tx)
unsigned char RR0_t[5] = {FLAG, A, C_RR0, BCC1_RR0_r, FLAG};    // Expected by Tx for next seq 1 (Nr=0 for I0 ack)
unsigned char RR1_t[5] = {FLAG, A, C_RR1, BCC1_RR1_r, FLAG};    // Expected by Tx for next seq 0 (Nr=1 for I1 ack)
unsigned char REJ0_t[5] = {FLAG, A, C_REJ0, BCC1_REJ0_r, FLAG}; // Expected by Tx for I0 retransmission
unsigned char REJ1_t[5] = {FLAG, A, C_REJ1, BCC1_REJ1_r, FLAG}; // Expected by Tx for I1 retransmission


volatile int curr_seq = 0; // Current Tx sequence number (0 or 1)
volatile int connection_active = FALSE;
static int serial_fd = -1;

LinkLayer connection_params;

volatile int timeoutFlag = 0;
volatile int timeoutCount = 0;


// --- HELPER FUNCTIONS ---

// FIX 1: The alarm handler now uses a simpler implementation
void alarmHandler(int signal)
{
    // The sigaction setup ensures this interrupts the blocking read.
    timeoutFlag = 1;
    timeoutCount++;
    printf(">>> TIMEOUT #%d - No response received\n", timeoutCount);
}

// FIX 2: Function to set up the alarm handler with no SA_RESTART
void setupAlarmHandler() {
    struct sigaction sa;
    sa.sa_handler = alarmHandler; 
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // Explicitly do NOT set SA_RESTART

    if (sigaction(SIGALRM, &sa, NULL) == -1) {
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


// --- STATE MACHINE READERS (Simplified & Fixed) ---

// General state machine for reading a fixed 5-byte supervision frame
// Expected A is the Address field (A or A_Rt)
// Expected C is the Control field (C_UA, C_DISC, etc.)
// Returns 1 on success, 0 on timeout/error
int read_supervision_frame(unsigned char expectedA, unsigned char expectedC, unsigned char expectedBCC1, int useTimeout)
{
    int step = START_STEP;

    if (useTimeout)
    {
        timeoutFlag = 0;
        alarm(connection_params.timeout);
    }
    
    // Check if expected frame is UA (different BCC1 depending on role)
    if (expectedC == C_UA && expectedA == A) {
        expectedBCC1 = (connection_params.role == LlTx) ? BCC1_UA_r : BCC1_UA_t;
    }

    while (!useTimeout || !timeoutFlag)
    {
        unsigned char byte;
        int bytesRead = readByteSerialPort(&byte);

        // FIX 3: Check for interrupt/error immediately
        if (bytesRead < 0) {
            if (errno == EINTR) {
                // Interrupted by SIGALRM, loop condition will now check timeoutFlag
                continue; 
            }
            // Other read error
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
    
    // FIX 2: Set up the alarm handler once
    setupAlarmHandler();

    if (connectionParameters.role == LlTx)
    {
        serial_fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (serial_fd < 0) return -1;

        timeoutCount = 0; // Reset global attempt counter
        while (timeoutCount <= connection_params.nRetransmissions) 
        {
            // Send SET
            writeBytesSerialPort(SET, 5);
            printf("SET sent (attempt %d/%d)\n", timeoutCount + 1, connection_params.nRetransmissions + 1);

            // Wait for UA (A field = A, C field = C_UA)
            if (read_supervision_frame(A, C_UA, BCC1_UA_r, TRUE)) 
            {
                printf("Received UA - Connection established!\n");
                connection_active = TRUE;
                return serial_fd;
            }
            
            // If timeout occurred, the loop continues to retransmit
            if (timeoutCount > connection_params.nRetransmissions) break;
            printf("TIMEOUT on attempt %d/%d - Retransmitting SET\n", timeoutCount, connection_params.nRetransmissions + 1);
        }

        printf("Connection failed after %d attempts\n", connection_params.nRetransmissions);
        return -1;
    }
    else // LlRx
    {
        serial_fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
        if (serial_fd < 0) return -1;

        // Wait for SET frame (No timeout)
        printf("Waiting for SET frame...\n");
        if (read_supervision_frame(A, C_SET, BCC1_SET, FALSE))
        {
            printf("Received SET frame, sending UA...\n");

            // Send UA response (A field = A, C field = C_UA)
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
    if (!connection_active || serial_fd < 0) return -1;

    // 1. Build the frame (Same as before)
    int maxStuffedSize = 5 + (bufSize * 2) + 2;
    unsigned char *stuffedFrame = (unsigned char *)malloc(maxStuffedSize);
    if (!stuffedFrame) return -1;
    int pos = 0;

    // Header
    stuffedFrame[pos++] = FLAG;
    stuffedFrame[pos++] = A;
    unsigned char C = (curr_seq == 0) ? C_I0 : C_I1;
    stuffedFrame[pos++] = C;
    unsigned char bcc1 = A ^ C;
    stuffedFrame[pos++] = bcc1;

    // Data Field (Stuffing + BCC2)
    for (int i = 0; i < bufSize; i++) {
        stuffAndAddByte(buf[i], stuffedFrame, &pos);
    }
    unsigned char bcc2 = BCC2(buf, bufSize);
    stuffAndAddByte(bcc2, stuffedFrame, &pos);

    // Trailer
    stuffedFrame[pos++] = FLAG;
    int frameLength = pos;

    // 2. Transmit and Wait for ACK/NACK
    int bytesWritten = -1;
    timeoutCount = 0;
    int success = FALSE;

    // Determine the expected reply C fields
    unsigned char expected_RR = (curr_seq == 0) ? C_RR1 : C_RR0;
    unsigned char expected_REJ = (curr_seq == 0) ? C_REJ0 : C_REJ1;
    // Determine the expected BCC1 for the reply frame
    unsigned char expected_BCC1_RR = (curr_seq == 0) ? BCC1_RR1_r : BCC1_RR0_r;
    unsigned char expected_BCC1_REJ = (curr_seq == 0) ? BCC1_REJ0_r : BCC1_REJ1_r;


    printf("=== LLWRITE STARTING for I-%d, %d bytes ===\n", curr_seq, bufSize);

    while (timeoutCount <= connection_params.nRetransmissions)
    {
        // Send the frame
        bytesWritten = writeBytesSerialPort(stuffedFrame, frameLength);
        if (bytesWritten < 0) {
            free(stuffedFrame);
            return -1;
        }
        printf(">>> Sent I-%d (attempt %d/%d)\n", curr_seq, timeoutCount + 1, connection_params.nRetransmissions + 1);

        // --- Start Waiting for ACK/NACK (A=A, C=RR/REJ) ---
        timeoutFlag = 0;
        alarm(connection_params.timeout);
        
        int result = 0; // 1 for success, 0 for failure

        // We use a small loop relying on the fact that if any of the expected frames
        // are detected by read_supervision_frame, the alarm will be cancelled internally.
        while (!timeoutFlag) 
        {
            unsigned char byte;
            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead < 0) {
                if (errno == EINTR) continue; // Alarm interrupted read
                break; // Other fatal error
            }

            if (bytesRead > 0)
            {
                // We run specialized SMs that only look for the expected frames.
                // Since read_supervision_frame returns 1 on success, we check the C field.
                
                // Check for expected RR
                if (read_supervision_frame(A, expected_RR, expected_BCC1_RR, FALSE)) {
                    result = expected_RR; // RR received
                    break;
                }
                // Check for expected REJ
                if (read_supervision_frame(A, expected_REJ, expected_BCC1_REJ, FALSE)) {
                    result = expected_REJ; // REJ received
                    break;
                }
            }
        }
        
        alarm(0); // Cancel alarm before processing result

        // Process the supervision frame result
        if (result != 0)
        {
            if (result == expected_RR) {
                curr_seq = 1 - curr_seq; // Toggle sequence number for next frame
                success = TRUE;
                printf(">>> SUCCESS: I-%d acknowledged by RR-%d\n", 1 - curr_seq, curr_seq);
                break;
            } else if (result == expected_REJ) {
                printf(">>> Received REJ-%d - Retransmitting I-%d\n", curr_seq, curr_seq);
                // Retransmission count increments below
            } else {
                // If it was another frame (e.g., RR for the old sequence), treat as loss/error
                printf(">>> Received unexpected frame (C=0x%02X) - Retransmitting I-%d\n", result, curr_seq);
            }
        }
        
        timeoutCount++;
    }

    free(stuffedFrame);

    if (success) {
        return frameLength;
    } else {
        printf(">>> FAILED to transmit I-%d after %d retransmissions. Closing connection.\n", curr_seq, connection_params.nRetransmissions);
        // Clean up connection on failure
        if (serial_fd >= 0) {
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
    if (!connection_active || serial_fd < 0) return -1;

    static int expected_Ns = 0; 
    
    // Loop until a correct, new I-frame is received
    while (TRUE) 
    {
        int step = START_STEP;
        int data_index = 0;
        unsigned char control = 0;
        unsigned char buffer[MAX_PAYLOAD_SIZE]; 
        int is_duplicate = FALSE;
        
        // Loop to read one complete frame (I or DISC)
        while (step != STOP_STEP)
        {
            unsigned char byte;
            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead < 0) {
                 if (errno == EINTR) continue;
                 return -1;
            }

            if (bytesRead > 0)
            {
                switch (step)
                {
                    case START_STEP:
                        if (byte == FLAG) step = FLAG_STEP;
                        break;
                    case FLAG_STEP:
                        if (byte == A) step = A_STEP;
                        else if (byte != FLAG) step = START_STEP;
                        break;
                    case A_STEP:
                        // Check for I-frame (C_I0/C_I1) or DISC frame (A=0x03)
                        if (byte == C_I0 || byte == C_I1) {
                            control = byte;
                            step = C_STEP;
                        } else if (byte == C_DISC) {
                            // Termination request detected, reply DISC and let app handle llclose
                            writeBytesSerialPort(DISC_reply, 5); 
                            printf("Rx received DISC, sent DISC reply.\n");
                            return 0; // Signal application layer to handle termination
                        } else if (byte == FLAG) step = FLAG_STEP;
                        else step = START_STEP;
                        break;
                    case C_STEP:
                        if (byte == (A ^ control)) 
                        {
                            int received_Ns = (control == C_I1) ? 1 : 0;
                            
                            if (received_Ns != expected_Ns) {
                                is_duplicate = TRUE;
                            }
                            step = DATA_STEP;
                            data_index = 0; // Start storing data and BCC2
                        }
                        else if (byte == FLAG) step = FLAG_STEP;
                        else step = START_STEP; // BCC1 incorrect - discard frame
                        break;

                    case DATA_STEP:
                        if (byte == ESC) {
                            step = ESCAPE_STEP;
                        } else if (byte == FLAG) {
                            // End of frame reached - process and check BCC2
                            step = STOP_STEP; // Temporarily exit inner loop for processing
                            
                            if (data_index < 1) { // Frame too short (must at least contain BCC2)
                                step = START_STEP; 
                                continue;
                            }
                            
                            unsigned char received_bcc2 = buffer[data_index - 1];
                            int data_size = data_index - 1; 
                            
                            unsigned char calc_bcc2 = BCC2(buffer, data_size);
                            
                            // Check BCC2 (Data Error Check)
                            if (received_bcc2 == calc_bcc2) 
                            {
                                // BCC2 CORRECT
                                if (!is_duplicate) 
                                {
                                    // NEW frame - Accept data and send RR for next seq
                                    memcpy(packet, buffer, data_size);
                                    unsigned char *rr_frame = (expected_Ns == 0) ? RR1_t : RR0_t;
                                    writeBytesSerialPort(rr_frame, 5);
                                    expected_Ns = 1 - expected_Ns; // Toggle expected Ns
                                    return data_size; // Return accepted data
                                } else {
                                    // DUPLICATE frame - Discard data and send RR for current expected seq
                                    unsigned char *rr_frame = (expected_Ns == 0) ? RR0_t : RR1_t;
                                    writeBytesSerialPort(rr_frame, 5);
                                    step = START_STEP; // Continue to next frame
                                }
                            } else {
                                // BCC2 INCORRECT (Data Error)
                                if (!is_duplicate) {
                                    // NEW frame with error - Send REJ to request retransmission
                                    unsigned char received_Ns = (control == C_I1) ? 1 : 0;
                                    unsigned char *rej_frame = (received_Ns == 0) ? REJ0_t : REJ1_t;
                                    writeBytesSerialPort(rej_frame, 5);
                                } else {
                                    // DUPLICATE frame with error - Send RR for current expected seq (to confirm reception of earlier frame)
                                    unsigned char *rr_frame = (expected_Ns == 0) ? RR0_t : RR1_t;
                                    writeBytesSerialPort(rr_frame, 5);
                                }
                                step = START_STEP; // Continue to next frame
                            }
                        } else {
                            // Normal data byte - store it
                            if (data_index < MAX_PAYLOAD_SIZE) {
                                buffer[data_index++] = byte;
                            } else {
                                step = START_STEP; 
                            }
                        }
                        break;

                    case ESCAPE_STEP:
                        // De-stuffing logic
                        byte = byte ^ STUFF_BYTE;
                        if (data_index < MAX_PAYLOAD_SIZE) {
                            buffer[data_index++] = byte;
                            step = DATA_STEP;
                        } else {
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
    if (!connection_active || serial_fd < 0) return -1;

    // Ensure initial state is clean for Tx
    int tx_success = FALSE;
    timeoutCount = 0;

    if (connection_params.role == LlTx)
    {
        while (timeoutCount <= connection_params.nRetransmissions)
        {
            // 1. Send DISC frame
            writeBytesSerialPort(DISC_cmd, 5);
            printf("Tx sent DISC (attempt %d/%d)\n", timeoutCount + 1, connection_params.nRetransmissions + 1);

            // 2. Wait for Rx's DISC reply
            if (read_supervision_frame(A_Rt, C_DISC, BCC1_DISC_t, TRUE)) 
            {
                printf("Tx received DISC reply.\n");
                // 3. Send final UA
                writeBytesSerialPort(UA_reply, 5);
                printf("Tx sent final UA.\n");
                tx_success = TRUE;
                break;
            }
            if (timeoutCount > connection_params.nRetransmissions) break;
            printf("TIMEOUT - Retransmitting DISC\n");
        }
        
        if (!tx_success) {
            printf("Tx failed to close connection gracefully.\n");
        }
    }
    else // LlRx
    {
        // Rx receives DISC from llread (returning 0) and is handled there.
        // It then waits for the final UA from Tx (no retransmission/timeout needed here).
        printf("Rx waiting for final UA...\n");
        if (read_supervision_frame(A_Rt, C_UA, BCC1_UA_t, FALSE))
        {
            printf("Rx received final UA.\n");
            tx_success = TRUE;
        } else {
            printf("Rx failed to receive final UA.\n");
        }
    }

    // Common cleanup
    if (closeSerialPort() == -1) {
        return -1;
    }
    serial_fd = -1;
    connection_active = FALSE;
    return tx_success ? 0 : -1;
}