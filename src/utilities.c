#include "utilities.h"

STOP = FALSE;

struct mainFrame_struct mainFrame;
struct alarmConfig_struct alarmConfig;

int frameTransmitterControl;    // Control that the transmitter is sending
int frameReceiverControl;       // Control that the receiver is expecting


void newAlarm(){
    alarmConfig.alarmEnabled = FALSE;
    alarmConfig.Counter = 0;
}

void alarmHandler() {
    alarmConfig.alarmEnabled = TRUE;
    alarmConfig.Counter++;
    print("Alarm %d triggered.\n", alarmConfig.Counter);
}


int SerialPortHandling(char serialPortName[50]) {

    //open serial port device for reading and writing
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    //clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo, ...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5; // Blocking read until 5 chars received

    tcflush(fd, TCIOFLUSH); // Flushes data received but not read

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    return 1;

}

void buildSupFrame(unsigned char Address, unsigned char Control) {
    mainFrame.frame[0] = FLAG;
    mainFrame.frame[1] = Address;
    mainFrame.frame[2] = Control;
    mainFrame.frame[3] = Address ^ Control;
    mainFrame.frame[4] = FLAG;

    mainFrame.size = 5;

}


void buildInfoFrame(unsigned char Address, const unsigned char *packet, int packetSize) {
    // Header
    mainFrame.frame[0] = FLAG;
    mainFrame.frame[1] = Address;
    mainFrame.frame[2] = (frameTransmitterControl ? C_I1 : C_I0);   // if frameTransmitterControl = 0, the value assigned will be C_I0, otherwise it will be C_I1
    mainFrame.frame[3] = mainFrame.frame[1] ^ mainFrame.frame[2];
    int next_index = 4;

    frameTransmitterControl = !frameTransmitterControl;    // changes boolean value of frameTransmitterControl

    for (int i = 0; i < packetSize; i++) {
        mainFrame.frame[next_index++] = packet[i];        // adds characters to transmit to the end of frame header
    }

    // build bcc2 with the packet provided

    mainFrame.frame[next_index++] = BCC2(packet, packetSize);  // adds bcc2 at the end of the main_frame and increments frame index
    mainFrame.frame[next_index] = FLAG;

    mainFrame.size = next_index;    // Current size of frame

}

void sendFrame(int fd, unsigned char* frame, int n) {  //sends a frame of data over communication channel (fd = File Descriptor)
    write(fd, frame, n); // writes n bytes from the frame contents in file descriptor
    sleep(1);  // wait until bytes have been written;
    alarm(alarmConfig.timeout);     
}

int sendSup(int fd, unsigned char Address, unsigned char Control) {
    buildSupFrame(Address, Control);
    return write(fd, mainFrame.frame, mainFrame.size);
}


unsigned char BCC2(unsigned char* data, int size) {
    unsigned char bcc = data[0];
    for (int i = 1; i < size; i++) { // iterate through the data, D1 XOR D2 XOR D3 ... XOR Dn
        bcc = bcc ^ data[0];        // XOR operation
    }
    
    return bcc;
}

int stuffing(unsigned char* frame, unsigned char* stuffedFrame,int frameSize) {
    unsigned char* stuffedFrame = (unsigned char*)malloc(frameSize * 2 + 6);    //since stuffedFrame size is unknown we use malloc

    if (stuffedFrame == NULL) {  // if memory allocation doesn't work
        fprintf(stderr, "Memory allocation failed.\n"); // stderr = stream used to output error messages or diagnostics
        exit(1);
    }

    int stuffedIndex = 0;

    for (int i = 0; i < frameSize; i++) {
        if (frame[i] == FLAG) {   // detects flag so it inserts ESCAPE + 0x5E instead of the real value
            stuffedFrame[stuffedIndex++] = ESCAPE;   // having [stuffedIndex++] makes it so each time we add a new value the index after will be incremented by 1
            stuffedFrame[stuffedIndex++] = 0x5E;
        }
        else if (frame[i] == ESCAPE) {  // detects escape so it replaces ESCAPE with ESCAPE + 0x5E
            stuffedFrame[stuffedIndex++] = ESCAPE;
            stuffedFrame[stuffedIndex++] = 0x5D;
        }
        else { // no flag or escape detected, so normal frame byte is inserted
            stuffedFrame[stuffedIndex++] = frame[i];
        }
    }

    stuffedFrame = realloc(stuffedFrame, stuffedIndex * sizeof(unsigned char));     // attempting to reallocate the given area of memory, since now we know the certain size of stuffedFrame

    if (stuffedFrame == NULL) { // if memory reallocation doesn't work
        fprintf(stderr, "Memory reallocation failed\n");
        exit(1);
    }

    return stuffedIndex;

}

int destuffing(const unsigned char* stuffedFrame, unsigned char* destuffedFrame, int destuffedSize) {
    unsigned char* destuffedFrame = (unsigned char*)malloc(destuffedSize * 2 + 6);    //since stuffedFrame size is unknown we use malloc

    if (destuffedFrame == NULL) {  // if memory allocation doesn't work
        fprintf(stderr, "Memory allocation failed.\n"); // stderr = stream used to output error messages or diagnostics
        exit(1);
    }
    
    int destuffedIndex = 0;

    for (int i = 0; i < destuffedSize; i++) {
        if (stuffedFrame[i] == ESCAPE && stuffedFrame[i + 1] == 0x5E) {
            destuffedFrame[destuffedIndex++] = FLAG;
        }
        else if (stuffedFrame[i] == ESCAPE && stuffedFrame == 0x5D) {
            destuffedFrame[destuffedIndex++] = ESCAPE;
        }
        else {
            destuffedFrame[destuffedIndex++] = stuffedFrame[i];
        }
    }

    destuffedFrame = realloc(destuffedFrame, destuffedIndex * sizeof(unsigned char));     // attempting to reallocate the given area of memory, since now we know the certain size of stuffedFrame

    if (destuffedFrame == NULL) { // if memory reallocation doesn't work
        fprintf(stderr, "Memory reallocation failed\n");
        exit(1);
    }


    return destuffedIndex;

}