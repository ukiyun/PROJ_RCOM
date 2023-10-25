// Link layer protocol implementation

#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


int TransmitterFrames = 0;
int ReceiverFrames = 0;


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    LinkLayerRole role = connectionParameters.role;
    if (role == LlTx) {
        return llOpenTransmitter(connectionParameters);
    }
    else {
        return llOpenReceiver(connectionParameters);
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////


int llwrite(int fd, const unsigned char *buf, int bufSize)
{
    newAlarm();

    unsigned char* new_frame = (unsigned char*)malloc(2 * bufSize + 6);
    
    
    // Header
    
    

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
