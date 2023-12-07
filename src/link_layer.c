// Link layer protocol implementation

#include "link_layer.h"
#include "macros.h"
// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// C Libraries

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>


volatile int STOP = FALSE;

int frameTransmitterControl = 0;
int frameReceiverControl = 1;


int alarmEnabled = FALSE;
int alarmCounter = 0;
int alarmTimeout = 0;
int nRetransmissions = 0;

int fd;     // file descriptor

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCounter++;
    printf("Alarm %d triggered.\n", alarmCounter);
}

////////////////////////////////////////////////
// State Machine 
////////////////////////////////////////////////

enum State{
    START,		// Initial State
    FLAG_RCV,		// Received Flag
    A_RCV,			// Received Address Field
    C_RCV,			// Received Control Field
    // CI_RCV,			// Received Control Field is C_I0 || C_I1
    BCC_OK,			// Received Independent Protection Field and is valid
    STOP_MACHINE,	// End State
    READ_DATA,      // State in which Payload is read
    RECEIVED_ESCAPE,   // Received data
};


int readTxFrame(enum State *currentState) {
    unsigned char byte;

    int errorControl;

    while (*currentState != STOP_MACHINE) {
        if ((errorControl = read(fd, &byte, 1)) > 0) {    // reads the data from fd and stores it in byte. Returns the number of bytes read
            switch (*currentState) {
                case START:
                    if (byte == FLAG) {
                        *currentState = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_RX) {
                        *currentState = A_RCV;
                    }
                    else if (byte != FLAG) {
                        *currentState = START;
                    }
                    break;
                case A_RCV:
                    if (byte == C_UA) {
                        *currentState = C_RCV;
                    }
                    else if (byte == FLAG) {
                        *currentState = FLAG_RCV;
                    }
                    else {
                        *currentState = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_RX ^ C_UA)) {
                        *currentState = BCC_OK;
                    }
                    else if (byte == FLAG) {
                        *currentState = FLAG_RCV;
                    }
                    else {
                        *currentState = START;
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        *currentState = STOP_MACHINE;
                    }
                    else{
                        *currentState = START;
                    }
                    break;
                default:
                    break;
            }
        }

        if (errorControl == 0) {
            fprintf(stderr, "Error in StateMachine TX");
            return 0;
        }
    }

    return 1;           // Successfully Read
}


int readRxFrame(enum State* currentState) {
    unsigned char byte;

    while (*currentState != STOP_MACHINE) {
        if (read(fd, &byte, 1) > 0) {
            switch (*currentState) {
                case START:
                    if (byte == FLAG) {
                        *currentState = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_TX) {
                        *currentState = A_RCV;
                    }
                    else if (byte != FLAG) {
                        *currentState = START;
                    }
                    break;
                case A_RCV:
                    if (byte == C_SET) {
                        *currentState = C_RCV;
                    }
                    else if (byte == FLAG) {
                        *currentState = FLAG_RCV;
                    }
                    else {
                        *currentState = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_TX ^ C_SET)) {
                        *currentState = BCC_OK;
                    }
                    else if (byte == FLAG) {
                        *currentState = FLAG_RCV;
                    }
                    else {
                        *currentState = START;
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        *currentState = STOP_MACHINE;
                    }
                    else {
                        *currentState = START;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    return 0;
}

int getControlField(int* errorControl) {
    printf("Entered getControlField function\n");
    unsigned char byte;                       // frame going to be read
    unsigned char control;                    // control field holder

    int successControl;                   // control read Errors
    
    enum State currentState = START;    // start state machine

    while (currentState != STOP_MACHINE) {
        printf("Entered while\n");
        if ((successControl = read(fd, &byte, 1)) > 0) {
            printf("Entered if\n");
            switch (currentState) {
                case START:
                    printf("Entered START\n");  
                    if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    printf("Entered FLAG_RCV\n");
                    if (byte == A_RX) {
                        currentState = A_RCV;
                    }
                    else if (byte != FLAG) {
                        currentState = START;
                    }
                    break;
                case A_RCV:
                    printf("Entered A_RCV\n");
                    if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == C_DISC) {
                        control = byte;
                        currentState = C_RCV;
                    }
                    else if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    else {
                        currentState = START;
                    }
                    break;
                case C_RCV:
                    printf("Entered C_RCV\n");
                    if (byte == (A_RX ^ control)) {
                        currentState = BCC_OK;
                    }
                    else if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    else {
                        currentState = START;
                    }
                    break;
                case BCC_OK:
                    printf("Entered BCC_OK\n");
                    if (byte == FLAG) {
                        currentState = STOP_MACHINE;
                    }
                    else {
                        currentState = START;
                    }
                    break;
                default:
                    break;
            }
            printf("Exited switch\n");
        }
        else if (successControl == 0) {      // Failed to read the Data from FD
            *errorControl = 0;
        }
        else {
            *errorControl = -1;                          // error
        }
    }
    printf("Exited while\n");
    *errorControl = 1;
    printf("Error Control: %d\n", *errorControl);
    printf("Control: %c\n", control);
    return control;                             // Will return  the Control Field of the Frame
}


////////////////////////////////////////////////
// Connection Control 
////////////////////////////////////////////////

int SerialPortHandling(const char* serialPortName) {    // arguments that will be obtained from LinkLayer struct
    // Function based on provided SerialPort function in LAB1 and LAB2

    //open serial port device for reading and writing
    fd = open(serialPortName, O_RDWR, O_NOCTTY);

    if (fd < 0) {
        perror(serialPortName);
        return -1;
    }



    struct termios oldtio;
    struct termios newtio;

    // save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

    //clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo, ...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 5; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0; // Blocking read until 5 chars received

    tcflush(fd, TCIOFLUSH); // Flushes data received but not read

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    printf("New termios structure set\n");

    return 1;       // returns true, meaning opening of Serial Port was correctly done

}


////////////////////////////////////////////////
// LLOPEN 
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters) {
    //Opens file descriptor with Serial Port
    if (SerialPortHandling(connectionParameters.serialPort) < 0) {
        fprintf(stderr, "Serial Port Problem");
        return -1;
    }

    // dealing with errors derived from Serial Port connection
    if (fd < 0) {
        perror("Connection Error");
        return -1;
    }

    printf("Attempting to open...\n");

    //saving connectionParameters to created struct
    alarmTimeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;

    // Sets the Machine State to the Beginning
    enum State currentState = START;

    //llopen will do different tasks depending on the role
    switch (connectionParameters.role) {
        case LlTx: {
            (void)signal(SIGALRM, alarmHandler);

            alarmEnabled = FALSE;

            //building set buf
            unsigned char set_frame[5];
            set_frame[0] = FLAG;
            set_frame[1] = A_TX;
            set_frame[2] = C_SET;
            set_frame[3] = set_frame[1] ^ set_frame[2];
            set_frame[4] = FLAG;

            while (connectionParameters.nRetransmissions > 0 && currentState != STOP_MACHINE) {
                if (alarmEnabled == FALSE) {
                    write(fd, set_frame, 5);            // Writes Specified number of bytes(5) from the set_frame into the fd
                    alarmEnabled = TRUE;
                    alarm(alarmTimeout);
                }
                connectionParameters.nRetransmissions--;
                readTxFrame(&currentState);
            }

            if (currentState != STOP_MACHINE) {
                return -1;                          // Means the end of state machine was not reached, therefore, there are errors
            }

        printf("Connection Established!\n");
            break;


        }  // close the case LlTx

        case LlRx: {
            
            readRxFrame(&currentState);

            unsigned char received_frame[5];
            received_frame[0] = FLAG;
            received_frame[1] = A_RX;
            received_frame[2] = C_UA;
            received_frame[3] = A_RX ^ C_UA;
            received_frame[4] = FLAG;

            write(fd, received_frame, 5);           // Writes received frame into the file descriptor

            printf("Connection Established!\n");
            break;

        }   // close the case LlRx

        default:
            return -1;
            break;


    }   // close the switch function


    return fd;

}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////


int llwrite(const unsigned char* buf, int bufSize)
{
    printf("Attempting to write... \n");


    // buf = array of characters to transmit / i.e. payload

    //Building Information Frame   / FLAG + ADDRESS + CONTROL + BCC + DATA + BCC2 + FLAG, DATA = buf
    int infoFrameSize = 2* bufSize + 6;                    // creating frame bigger than it needs to be, to avoid not having enough space, space reallocated at the end

    //HEADER
    unsigned char* infoFrame = (unsigned char*)malloc(infoFrameSize);                // packet size + control fields (Flags, Address, Control and Bcc's)
    infoFrame[0] = FLAG;
    infoFrame[1] = A_TX;
    infoFrame[2] = (frameTransmitterControl ? C_I1 : C_I0); // if frameTransmitterControl = 0, the value assigned will be C_I0, otherwise it will be C_I1
    infoFrame[3] = infoFrame[1] ^ infoFrame[2];

    int nextIndex = 4;

    //BCC2, Field that detects the occurrence of errors in the Data Field (buf) / done before stuffing
    unsigned char bcc2 = 0;
    for (int index = 0; index < bufSize; index++) {
        bcc2 = bcc2 ^ buf[index];                   // D1 XOR D2 XOR D3 ... XOR Dn
    }

    //Byte Stuffing
    for(int index = 0; index<bufSize; index++){
        if(buf[index]==FLAG || buf[index]== ESCAPE){
            infoFrame[nextIndex++] = ESCAPE;
            infoFrame[nextIndex++] = buf[index] ^ 0x20;         // escape octet followed by the result of the exclusive or of the octet replaced with octet 0x20          
        }
        else{
           infoFrame[nextIndex++] = buf[index]; 
        }
        
    }
    printf("Byte Stuffing Done\n");
    // TRAILER
    infoFrame[nextIndex++] = bcc2;
    infoFrame[nextIndex++] = FLAG;

    infoFrame = realloc(infoFrame, nextIndex);          // Memory Reallocation to the actual frame size

    printf("Info Frame built\n");
    printf("Info Frame: ");
    for (int i = 0; i < nextIndex; i++) {
        printf("%02X ", infoFrame[i]);
    }
    printf("\n");
    // Dealing with the different Control Field Values
    int checkControl = 0;
    alarmCounter = 0;

    while (alarmCounter<nRetransmissions) {
        alarmEnabled = FALSE;
        alarm(alarmTimeout);
        checkControl = 0;

        while(alarmEnabled == FALSE  && checkControl!= 1){
            write(fd, infoFrame, nextIndex);
            printf("Sent Frame: ");
            for (int i = 0; i < nextIndex; i++) {
                printf("%02X ", infoFrame[i]);
            }
            printf("\n");
            int* errorControl=(int*)malloc(sizeof(int));
            *errorControl = 1;
            printf("Error Control Value Init: %d\n",*errorControl);
            unsigned char controlField = getControlField(errorControl);          // gets the Value inside the Frame ControlField
            if(errorControl <= 0){
                printf("Error Reading Control Field\n");
                return -1;            
            }
                // ControlField Value makes the action change
            if (controlField == C_RR0 || controlField == C_RR1) {                   // If ControlField is the indication that Receiver is ready to receive InfoFrame
                printf("Received RR%d, Frame accepted\n", controlField);                
                checkControl = 1;                                                   // ControlField Accepted
                frameTransmitterControl = (frameTransmitterControl + 1) % 2;        // Makes Sure frameTransmitterControl is either 1 or 0
            }
            else if (controlField == C_REJ0 || controlField == C_REJ1) {    // If ControlField is the indication that Receiver rejects an infoFrame
                checkControl = 0;                                  // Going to Retry Writing
                printf("Control Field Rejected. Trying again...\n");
            }
            else{
                printf("Unexpected Control Field: %02X\n", controlField);
            } 

        }

    if(checkControl == 1) {
    printf("Frame Successfully acknowledged. \n");
    break;
    }else{
        printf("Frame successfully acknowledge. \n");
        alarmCounter++;
    }
    }
    
    free(infoFrame);        // Memory Allocation
    
    if (checkControl == 1) {
        return nextIndex;               // Returns number of bytes written
    }
    else {
        fprintf(stderr, "Program exceeded the number of possible Retransmissions while writing\n");
        llclose(0);                                                     // Closes Connection
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    printf("Attempting to read... \n");

    STOP = FALSE;

    unsigned char byte;
    unsigned char controlField;
    int position = 0;

    enum State currentState = START;
    
    unsigned char receivedFrame[5];
    int receivedFrameSize = 0;

while (currentState != STOP_MACHINE) {
    int bytesRead = read(fd, &byte, 1);
    if(bytesRead == -1){
        fprintf(stderr, "Error reading from file descriptor");
        return -1;
    }
    if (bytesRead > 0) {                       // If read is successful
        receivedFrame[receivedFrameSize++] = byte;
        printf("Received byte: %02X\n", byte);  // Log the received byte
        //State Machine
        switch (currentState) {
            case START:
                if (byte == FLAG) {
                    currentState = FLAG_RCV;
                }
                break;
            case FLAG_RCV:
                if (byte == A_TX) {
                    currentState = A_RCV;
                }
                else if (byte != FLAG) {
                    currentState = START;
                }
                break;
            case A_RCV:
                if (byte == C_I0 || byte == C_I1) {             // If Byte is equivalent to the Information Frame 0 or 1
                    controlField = byte;                        // Saving the Value stored in the Control Field
                    printf("Control field: %02X\n", controlField);  // Log the control field
                    currentState = C_RCV;

    
/*
    while (currentState != STOP_MACHINE) {
        int bytesRead = read(fd, &byte, 1);
        if(bytesRead == -1){
            fprintf(stderr, "Error reading from file descriptor");
            return -1;
}
        if (bytesRead > 0) {                       // If read is successful
            receivedFrame[receivedFrameSize++] = byte;
            //State Machine
            switch (currentState) {
                case START:
                    if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_TX) {
                        currentState = A_RCV;
                    }
                    else if (byte != FLAG) {
                        currentState = START;
                    }
                    break;
                case A_RCV:
                    if (byte == C_I0 || byte == C_I1) {             // If Byte is equivalent to the Information Frame 0 or 1
                        controlField = byte;                        // Saving the Value stored in the Control Field
                        currentState = C_RCV;
                    }*/
                    }
                    else if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    else {
                        currentState = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_TX ^ controlField)) {
                        currentState = READ_DATA;
                    }
                    else if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    else {
                        currentState = START;
                    }
                    break;
                case READ_DATA:
                    if (byte == ESCAPE) {
                        currentState = RECEIVED_ESCAPE;
                        printf("ESCAPE\n");
                    }
                    else if (byte == FLAG) {
                        //De-stuffing
                        printf("FLAG\n");
                        unsigned char bcc2 = packet[position-1];          // bcc2 + flag
                        position--;
                        packet[position] = '\0';                    // removed bcc2 and ending flag,  position will be equal to the size of whats left
                        
                        printf("Received Frame: ");
                        for (int i = 0; i < position; i++) {
                            printf("%02X ", packet[i]);
                        }
                        printf("\n");

                        // checking BCC2
                        unsigned char bcc2Check = 0;
                        for (int index = 0; index < position; index++) {
                            if (packet[index] == ESCAPE && index + 1 < position) {
                                bcc2Check = bcc2Check ^ (packet[++index] ^ 0x20);
                            } else {
                                bcc2Check = bcc2Check ^ packet[index];
                            }
                        }


                        if (bcc2 == bcc2Check) {
                            currentState = STOP_MACHINE;
                            printf("CURRENT STATE = STOP_MACHINE\n");
                            printf("Received Frame: ");
for (int i = 0; i < position; i++) {
    printf("%02X ", packet[i]);
}
printf("\n");
    
                            unsigned char reading_frame[5];
                            reading_frame[0] = FLAG;
                            reading_frame[1] = A_TX;
                            reading_frame[2] = C_RR(frameReceiverControl);
                            reading_frame[3] = A_TX ^ C_RR(frameReceiverControl);
                            reading_frame[4] = FLAG;

                            write(fd, reading_frame, 5);            // Writes the frame to the File Descriptor
                            frameReceiverControl = (frameReceiverControl +1)%2;
                            printf("Sent RR%d\n", frameReceiverControl);
                            printf("Position value: %d\n", position);                      
                            printf("Frame successfully received.\n");      
                            return position;                        // Return number of characters read
                        }
                        else {                                      // bcc2 and Check don't match
                            printf("BCC2 CHECK != 2\n");                            
                            printf("Position value: %d\n", position);
  
                            unsigned char reading_frame[5];                
                            reading_frame[0] = FLAG;
                            reading_frame[1] = A_TX;
                            reading_frame[2] = C_REJ(frameReceiverControl);
                            reading_frame[3] = A_TX ^ C_REJ(frameReceiverControl);
                            reading_frame[4] = FLAG;

                            write(fd, reading_frame, 5);                // Write frame rejected into the File Descriptor
                            return -1;                                  // Negative value because of error
                        }
                    }
                    else {
                        packet[position++] = byte;
                    }
                    break;
                case RECEIVED_ESCAPE:
                    printf("RECEIVED_ESCAPE\n");
                    currentState = READ_DATA;
                    packet[position++] = byte ^ 0x20;
                    break;
                case STOP_MACHINE:
                    break;
                default:
                    printf("Byte: %02X\n", byte);
                    break;
            }
        }
    }
    printf("Received Frame: ");
    for(int i = 0; i<receivedFrameSize; i++){
        printf("%02X ", receivedFrame[i]);
    }
    printf("\n");
    
    return -1;                  // read
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    printf("Closing...");

    enum State currentState = START;
 
    (void)signal(SIGALRM, alarmHandler);

    unsigned char byte;

    while (currentState != STOP_MACHINE && nRetransmissions != 0) {

        unsigned char disconnect_frame[5];
        disconnect_frame[0] = FLAG;
        disconnect_frame[1] = A_TX;
        disconnect_frame[2] = C_DISC;
        disconnect_frame[3] = A_TX ^ C_DISC;
        disconnect_frame[4] = FLAG;

        write(fd, disconnect_frame, 5);                 // Writes the full disconnect frame into the file descriptor

        alarm(alarmTimeout);
        alarmEnabled = FALSE;

        while (alarmEnabled == FALSE && currentState != STOP_MACHINE) {
            if (read(fd, &byte, 1) > 0) {           // reads byte by byte and avoids reading errors
                switch (currentState) {
                    case START:
                        if (byte == FLAG) {
                            currentState = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_RX) {
                            currentState = A_RCV;
                        }
                        else if (byte != FLAG) {
                            currentState = START;
                        }
                        break;
                    case A_RCV:
                        if (byte == C_DISC) {                   // Disconnect order
                            currentState = C_RCV;
                        }
                        else if (byte == FLAG) {
                            currentState = FLAG_RCV;
                        }
                        else {
                            currentState = START;
                        }
                        break;
                    case C_RCV:
                        if (byte == (A_RX ^ C_DISC)) {
                            currentState = BCC_OK;
                        }
                        else if (byte == FLAG) {
                            currentState = FLAG_RCV;
                        }
                        else {
                            currentState = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            currentState = STOP_MACHINE;
                        }
                        else {
                            currentState = START;
                        }
                        break;
                    default:
                        break;
                }
            }
            
            nRetransmissions--;
        }

    }
        
    if (currentState != STOP_MACHINE) {                             // error during state machine implementation
        return -1;
    }

    unsigned char ua_frame[5];
    ua_frame[0] = FLAG;
    ua_frame[1] = A_TX;
    ua_frame[2] = C_UA;
    ua_frame[3] = A_TX ^ C_UA;
    ua_frame[4] = FLAG;


    write(fd, ua_frame, 5);

    return close(fd);

}
