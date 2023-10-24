#include "utilities.h"

volatile int STOP = FALSE;

// alarm flag and counter for timeout
int alarmCounter = 0;
int alarmEnabled = FALSE;

void alarmHandler() {
    alarmEnabled = TRUE;
    alarmCounter++;
    print("Alarm %d triggered.\n", alarmCounter);
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

unsigned char* stuffing(const unsigned char* frame, int frameSize, int* newSize) {
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

    *newSize = stuffedIndex;

    return stuffedFrame;

}

unsigned char* destuffing(const unsigned char* stuffedFrame, int frameSize, int* newSize) {
    unsigned char* destuffedFrame = (unsigned char*)malloc(frameSize * 2 + 6);    //since stuffedFrame size is unknown we use malloc

    if (destuffedFrame == NULL) {  // if memory allocation doesn't work
        fprintf(stderr, "Memory allocation failed.\n"); // stderr = stream used to output error messages or diagnostics
        exit(1);
    }
    
    int destuffedIndex = 0;

    for (int i = 0; i < frameSize; i++) {
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

    *newSize = destuffedIndex;

    return destuffedFrame;

}