// Link layer protocol implementation

#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


frameTransmitterControl = 0;    // Control that the transmitter is sending
frameReceiverControl = 0;       // Control that the receiver is expecting

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
    // buf = array of characters to transmit

    buildInfoFrame(A_TX, buf, bufSize);

    unsigned char* new_frame = (unsigned char*)malloc(bufSize * 2);

    int new_frame_size = stuffing(mainFrame.frame, new_frame, mainFrame.size);

    STOP = FALSE;

    // implement new state machine to verify the control field in the frame and how many characters are written

    newAlarm(); // resets alarmCounter but nRetransmissions maintains also changes alarmEnable to FALSE

    int chars_written = 0;


    while (alarmConfig.Counter < alarmConfig.nreTransmissions) {
        alarmConfig.Counter++;

        //initiating state machine
        StateMachine* stM = (StateMachine*)malloc(sizeof(StateMachine));
        stateChange(stM, START);

        sendFrame(fd, new_frame, new_frame_size + 1);       // Writes newly created frame (main frame with stuffing) into the file descriptor
        
        
        /* STILL WORKING ON IMPLEMENTING
        unsigned char ua_frame[5];

        //state machine
        while ((stM->currentState != STOP_MACHINE) && (alarmConfig.alarmEnabled == FALSE)) {
            if (read(fd, ua_frame, 5) > 0) {            // if it reads a value apply switch function
                switch (stM->currentState) {
                    case START:
                        if ()
                }
            }
        }

        */

    }


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
