// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

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
        printf("Connection failed!");
        return;
    }

    if (ll.role == LlTx)
    {

        FILE *file = fopen(filename, "rb");
        if (!file)
        {
            perror("Error opening file");
            return;
        }
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);

        printf("Sending file '%s' (size: %ld bytes)\n", filename, fileSize);

        // 1. send start packet
        int startSize = 1 + 2 + 4 + 2 + strlen(filename); // C + TLV + TLV
        unsigned char *startPacket = malloc(startSize);
        startSize = buildControlPacket(startPacket, CONTROL_START, filename, fileSize);
        llwrite(startPacket, startSize);
        printf("START packet sent (%d bytes)\n", startSize);
        free(startPacket);

        // 2. send data packets
        unsigned char buffer[PACKET_DATA_SIZE];
        int seq = 0, bytesRead;

        while ((bytesRead = fread(buffer, 1, PACKET_DATA_SIZE, file)) > 0)
        {
            int packetSize = 4 + bytesRead; // C + N + L2 + L1 + data
            unsigned char *dataPacket = malloc(packetSize);
            packetSize = createDataPacket(dataPacket, seq, buffer, bytesRead);

            printf("Sending DATA #%d (%d bytes)\n", seq, bytesRead);
            llwrite(dataPacket, packetSize);
            free(dataPacket);

            seq = (seq + 1) % 256;
        }

        // 3. send end packet
        int endSize = 1 + 2 + 4 + 2 + strlen(filename);
        unsigned char *endPacket = malloc(endSize);
        endSize = buildControlPacket(endPacket, CONTROL_END, filename, fileSize);
        llwrite(endPacket, endSize);
        printf("END packet sent (%d bytes)\n", endSize);
        free(endPacket);

        fclose(file);
        printf("File transmission complete!\n");
    }
    else
    {

        unsigned char packet[1024];
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
                output = fopen(recvFilename, "wb");
                if (!output)
                {
                    perror("Error opening output file");
                    break;
                }
                printf("START packet received: '%s' (%ld bytes)\n", recvFilename, recvFileSize);
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