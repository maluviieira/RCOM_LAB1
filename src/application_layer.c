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

    if (llopen(ll) == 0)
    {
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




            const char *og_data = "Hello World! This is a test message.";
            int totalLength = strlen(og_data);

            printf("Transmitting: '%s'\n", og_data);
            printf("Total: %d bytes, Packet size: %d bytes\n\n", totalLength, PACKET_SIZE);

            int packetCount = 0;
            int totalSent = 0;

            while (totalSent < totalLength)
            {
                packetCount++;

                // calculate how many bytes to send in this packet
                int bytesToSend = totalLength - totalSent;
                if (bytesToSend > PACKET_SIZE)
                {
                    bytesToSend = PACKET_SIZE;
                }

                printf("Packet %d: %d bytes -> ", packetCount, bytesToSend);

                int result = llwrite((unsigned char *)(og_data + totalSent), bytesToSend);

                if (result > 0)
                {
                    printf("✓ ('");

                    for (int i = 0; i < bytesToSend; i++)
                    {
                        printf("%c", og_data[totalSent + i]);
                    }

                    printf("')\n");

                    totalSent += bytesToSend;
                }
                else
                {
                    printf("✗\n");
                    break;
                }

                // small delay except after the last packet
                if (totalSent < totalLength)
                {
                    sleep(1);
                }
            }

            printf("\nTransmission complete: %d packets, %d/%d bytes sent\n",
                   packetCount, totalSent, totalLength);
        }
        else if (ll.role == LlRx)
        {
            unsigned char reconstructedData[1024];
            int totalReceived = 0;
            int packetCount = 0;

            printf("Waiting for packets (size: %d bytes)...\n", PACKET_SIZE);

            while (TRUE)
            {
                packetCount++;
                unsigned char packet[PACKET_SIZE + 1]; // +1 for null terminator for printing
                int size = llread(packet);

                if (size > 0)
                {
                    memcpy(reconstructedData + totalReceived, packet, size);
                    totalReceived += size;

                    // null terminate for printing
                    packet[size] = '\0';

                    printf("Packet %d: %d bytes ✓ ('%s')\n", packetCount, size, packet);

                    // If we received less than a full packet -> done
                    if (size < PACKET_SIZE)
                    {
                        printf("→ Received partial packet, assuming transmission complete\n");
                        break;
                    }
                }
                else
                {
                    printf("Packet %d: failed or no data ✗\n", packetCount);
                    // if no data after successful packets -> transmission complete
                    if (totalReceived > 0)
                    {
                        break;
                    }
                }
            }

            if (totalReceived > 0)
            {
                // null terminate for safe printing
                reconstructedData[totalReceived] = '\0';
                printf("\nReconstruction complete: %d packets, %d bytes total\n",
                       packetCount - 1, totalReceived);
                printf("Full message: '%s'\n", reconstructedData);
            }
            else
            {
                printf("\nNo data received\n");
            }
        }

        // llclose();
    }
    else
    {
        printf("Connection failed!\n");
    }
}