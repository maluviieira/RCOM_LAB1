// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

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
            unsigned char testData[] = "Hello World!";
            llwrite(testData, sizeof(testData));
        }
        else if (ll.role == LlRx)
        {
            unsigned char packet[256];
            int size = llread(packet);
            if (size > 0)
            {
                printf("Received: %s\n", packet);
            }
        }
    }
}
