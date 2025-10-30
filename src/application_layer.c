// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>

#define PACKET_SIZE 5

#define CONTROL_START 1
#define CONTROL_END 3
#define CONTROL_DATA 2

#define FILE_SIZE 0
#define FILE_NAME 1

#define PACKET_DATA_SIZE 512

int createControlPacket(unsigned char *packet, const char *filename, long fileSize, int control)
{
    int pos = 0;

    packet[pos++] = control;

    packet[pos++] = FILE_SIZE; // type
    packet[pos++] = 4;         // length (4 bytes for long)
    packet[pos++] = (fileSize >> 24) & 0xFF;
    packet[pos++] = (fileSize >> 16) & 0xFF;
    packet[pos++] = (fileSize >> 8) & 0xFF;
    packet[pos++] = fileSize & 0xFF;

    uint8_t nameLen = strlen(filename);
    packet[pos++] = FILE_NAME; // T
    packet[pos++] = nameLen;   // L
    memcpy(&packet[pos], filename, nameLen);
    pos += nameLen;

    return pos; // return packet size
}

int createDataPacket(unsigned char *packet, const unsigned char *data, int dataSize, int seq)
{
    int pos = 0;

    packet[pos++] = 2;
    packet[pos++] = seq; // sequence number

    // length
    packet[pos++] = (dataSize >> 8) & 0xFF; // L2
    packet[pos++] = dataSize & 0xFF;        // L1

    // data
    memcpy(&packet[pos], data, dataSize);
    pos += dataSize;

    return pos;
}

void parseControlPacket(unsigned char *packet, int size, char *filename, long *fileSize)
{

    int pos = 1; // skip control field

    while (pos < size)
    {
        uint8_t type = packet[pos++];
        uint8_t length = packet[pos++];

        if (type == FILE_SIZE)
        {
            *fileSize = ((long)packet[pos] << 24) |
                        ((long)packet[pos + 1] << 16) |
                        ((long)packet[pos + 2] << 8) |
                        ((long)packet[pos + 3]);
        }
        else if (type == FILE_NAME)
        {
            memcpy(filename, &packet[pos], length);
            filename[length] = '\0';
        }

        pos += length; // move to next tlv field
    }
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;

    ll.role = strcmp(role, "rx") ? LlTx : LlRx;
    strcpy(ll.serialPort, serialPort);
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    if (llopen(ll) < 0)
    {
        printf("Connection failed!\n");
        return;
    }

    if (ll.role == LlTx)
    {
        FILE *file = fopen(filename, "rb");
        if (!file)
        {
            perror("Error opening file\n");
            return;
        }
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        fseek(file, 0, SEEK_SET); // rewind to beginning

        printf("Sending file '%s' (size: %ld bytes)\n", filename, fileSize);

        // 1. send start packet
        int startSize = 1 + 2 + 4 + 2 + strlen(filename); // C + TLV + TLV
        unsigned char *startPacket = malloc(startSize);
        startSize = createControlPacket(startPacket, filename, fileSize, CONTROL_START);

        if (llwrite(startPacket, startSize) < 0)
        {
            printf("START packet transmission failed after max retransmissions. Closing.\n");
            free(startPacket);
            fclose(file);
            llclose();
            return;
        }
        printf("START packet sent (%d bytes)\n", startSize);
        free(startPacket);

        // 2. send data packets
        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int seq = 0, bytesRead;

        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0)
        {
            int packetSize = 4 + bytesRead; // C + N + L2 + L1 + data
            unsigned char *dataPacket = malloc(packetSize);
            packetSize = createDataPacket(dataPacket, buffer, bytesRead, seq);

            printf("Sending DATA #%d (%d bytes)\n", seq, bytesRead);

            if (llwrite(dataPacket, packetSize) < 0)
            {
                printf("DATA packet #%d transmission failed after max retransmissions. Closing.\n", seq);
                free(dataPacket);
                fclose(file);
                llclose();
                return;
            }

            free(dataPacket);

            seq = (seq + 1) % 256;
        }

        // 3. send end packet
        int endSize = 1 + 2 + 4 + 2 + strlen(filename);
        unsigned char *endPacket = malloc(endSize);
        endSize = createControlPacket(endPacket, filename, fileSize, CONTROL_END);

        if (llwrite(endPacket, endSize) < 0)
        {
            printf("END packet transmission failed after max retransmissions. Closing.\n");
            free(endPacket);
            fclose(file);
            llclose();
            return;
        }
        printf("END packet sent (%d bytes)\n", endSize);
        free(endPacket);

        fclose(file);
        printf("File transmission complete!\n");
    }
    else
    {
        // Receiver code remains unchanged...
        unsigned char packet[MAX_PAYLOAD_SIZE];
        char recvFilename[256];
        long recvFileSize = 0;
        FILE *output = NULL;
        int seq = 0;

        printf("Receiver waiting for incoming packets...\n");

        while (TRUE)
        {
            int size = llread(packet);
            if (size <= 0)
                continue;

            if (packet[0] == CONTROL_START)
            {
                parseControlPacket(packet, size, recvFilename, &recvFileSize);

                char outputFilename[256];
                snprintf(outputFilename, sizeof(outputFilename), "penguin-received.gif");

                output = fopen(outputFilename, "wb");
                if (!output)
                {
                    perror("Error opening output file");
                    break;
                }
                printf("START packet received: '%s' (%ld bytes)\n", recvFilename, recvFileSize);
                printf("Saving to: '%s'\n", outputFilename);
            }
            else if (packet[0] == CONTROL_DATA)
            {
                int dataLength = (packet[2] << 8) + packet[3];
                fwrite(&packet[4], 1, dataLength, output);
                printf("DATA #%d received (%d bytes)\n", packet[1], dataLength);
                seq = (seq + 1) % 256;
            }
            else if (packet[0] == CONTROL_END)
            {
                printf("END packet received. Transfer complete.\n");
                if (output)
                    fclose(output);
                break;
            }
        }
    }

    llclose();
}