// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define PACKET_SIZE 5 

#define FILE_SIZE 0
#define FILE_NAME 1

int createControlPacket(unsigned char *packet, const char *filename, long file_size, unsigned char control) {
    int pos = 0;
   
    packet[pos++] = control;
    
    packet[pos++] = FILE_SIZE;  // type
    packet[pos++] = 4;              // length (4 bytes for long)
    packet[pos++] = (file_size >> 24) & 0xFF;
    packet[pos++] = (file_size >> 16) & 0xFF;
    packet[pos++] = (file_size >> 8) & 0xFF;
    packet[pos++] = file_size & 0xFF;
    
    return pos; // return packet size
}

int createDataPacket(unsigned char *packet, const unsigned char *data, int data_size) {
    int pos = 0;
    
    packet[pos++] = 2;
    
    // length 
    packet[pos++] = (data_size >> 8) & 0xFF;  // L2
    packet[pos++] = data_size & 0xFF;         // L1
    
    // data 
    memcpy(&packet[pos], data, data_size);
    pos += data_size;
    
    return pos; 
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