// Link layer protocol implementation

#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


volatile int STOP = FALSE;

int frameTransmitterControl = 0;
int frameReceiverControl = 1;

////////////////////////////////////////////////
// Frame Structure
////////////////////////////////////////////////

struct mainFrame_struct {
    unsigned char frame[MAX_PAYLOAD];
    size_t size;
    int fd;
};

struct mainFrame_struct mainFrame;

////////////////////////////////////////////////
// Alarm
////////////////////////////////////////////////

struct alarmConfigStruct {
    int Counter;
    int timeout;
    int nreTransmissions;
    int Enabled;
};

struct alarmConfigStruct alarmConfig;   // Initialize Alarm

void alarmHandler(int signal) {
    alarmConfig.Counter++;
    alarmConfig.Enabled = FALSE;
    printf("Alarm %d triggered.\n", alarmConfig.Counter);
}

////////////////////////////////////////////////
// State Machine 
////////////////////////////////////////////////

enum State{
    START,		// Initial State
    FLAG_RCV,		// Received Flag
    A_RCV,			// Received Address Field
    C_RCV,			// Received Control Field
    CI_RCV,			// Received Control Field is C_I0 || C_I1
    BCC_OK,			// Received Independent Protection Field and is valid
    STOP_MACHINE,	// End State
    STOP_MACHINE_DC, // End State with Control Field = C_DISC
    PAYLOAD,		// Actual Data being sent
    BCC2_CHECK,	// Check BCC2
    BCC2_OK		// BCC2 check is alright
};


int stateMachine( unsigned char Address, unsigned char Control) {
    unsigned char receive_frame[5];
    
    enum State currentState = START;

    while (currentState != STOP_MACHINE) {
        int bytes = read(mainFrame.fd, receive_frame, 5);
        
        if (alarmConfig.Counter > alarmConfig.nreTransmissions) {
            printf("Number of tries exceeded\n");
            return 1;
        }

        for (int i = 0; i < bytes; i++) {   // iterate over received_frame
            switch (currentState) {
                case START:
                    if (receive_frame[i] == FLAG) {     // if flag, move on to next stage in state machine
                        currentState = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (receive_frame[i] == Address) {
                        currentState = A_RCV;
                    }
                    else if (receive_frame[i] != FLAG) {            // value that is neither FLAG or Address, we restart the machine
                        currentState = START;               
                    }
                    break;
                case A_RCV:
                    if (receive_frame[i] == Control) {
                        currentState = C_RCV;
                    }
                    else if (receive_frame[i] == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    else {
                        currentState = START;
                    }
                    break;
                case C_RCV:
                    if (receive_frame[i] == (Address ^ Control)) {
                        currentState = BCC_OK;
                    }
                    else if (receive_frame[i] == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    else {
                        currentState = START;
                    }
                    break;
                case BCC_OK:
                    if (receive_frame[i] == FLAG) {     // check if received frame ends in the FLAG
                        currentState = STOP_MACHINE;
                    }
                    else {
                        currentState = START;
                    }
                default:
                    break;
            }
        }


    }

    if (currentState != STOP_MACHINE) {
        return 1;
    }

    return 0;
}



////////////////////////////////////////////////
// Frame Functions
////////////////////////////////////////////////

void sendFrame(int fd, unsigned char* frame, int size) {  //sends a frame of data over communication channel (fd = File Descriptor)
    write(fd, frame, size); // writes n bytes from the frame contents in file descriptor
    alarm(alarmConfig.timeout);
}

void buildSupervisionFrame(unsigned char Address, unsigned char Control) {
    // building Supervision frame into mainFrame struct 
    mainFrame.size = 5;
    mainFrame.frame[0] = FLAG;
    mainFrame.frame[1] = Address;
    mainFrame.frame[2] = Control;
    mainFrame.frame[3] = Address ^ Control;
    mainFrame.frame[4] = FLAG;
}

int sendSupervision(int fd, unsigned char Address, unsigned char Control) {
    // write the supervision frame in file descriptor
    buildSupervisionFrame(Address, Control);
    return (write(mainFrame.fd, mainFrame.frame, 5));
}

unsigned char BCC2(const unsigned char* data, int dataSize) {
    unsigned char bcc2_frame = data[0];
    for (int index = 1; index < dataSize; index++) {     // iterate through the data, D1 XOR D2 XOR D3 ... XOR Dn
        bcc2_frame = bcc2_frame ^ data[index];       // XOR operation
    }
    return bcc2_frame;
}

void buildInformationFrame(unsigned char Address, const unsigned char* packet, int packetSize) {
    mainFrame.size = packetSize + 5;        // size of the data plus the HEADER AND TRAILER
    // HEADER
    
    mainFrame.frame[0] = FLAG;
    mainFrame.frame[1] = Address;
    mainFrame.frame[2] = (frameTransmitterControl ? C_I1 : C_I0); // if frameTransmitterControl = 0, the value assigned will be C_I0, otherwise it will be C_I1
    mainFrame.frame[3] = mainFrame.frame[1] ^ mainFrame.frame[2];

    int nextIndex = 4;

    // add packet data to frame

    for (int index = 0; index < packetSize; index++) {
        mainFrame.frame[nextIndex++] = packet[index];
    }

    // TRAILER
    // 
    // build BCC2 after getting the packet
    mainFrame.frame[nextIndex++] = BCC2(packet, packetSize);
    mainFrame.frame[nextIndex] = FLAG;
    
    mainFrame.size = nextIndex;
}
////////////////////////////////////////////////
// STUFFING
////////////////////////////////////////////////


int stuffing(unsigned char* frame, unsigned char* stuffedFrame, int frameSize) {
    int stuffedIndex = 0;   // will keep track of StuffedFrame Size

    for (int index = 0; index < frameSize; index++) {
        if (frame[index] == FLAG) {     //detects flag in frame
            stuffedFrame[stuffedIndex++] = ESCAPE;      //stuffed frame will have ESCAPE VALUE + 0x5E instead of FLAG
            stuffedFrame[stuffedIndex++] = 0x5E;        //++ after stuffedIndex means that after setting value to 0x5E, stuffedIndex increments by 1
        }
        else if (frame[index] == ESCAPE) {  //detects escape in frame
            stuffedFrame[stuffedIndex++] = ESCAPE;  //stuffed frame will have ESCAPE VALUE + 0x5D instead of ESCAPE
            stuffedFrame[stuffedIndex++] = 0x5D;
        }
        else {
            stuffedFrame[stuffedIndex++] = frame[index];    // Not a FLAG or ESCAPE, stuffedFrame is equal to normal frame
        }
    }

    return stuffedIndex;
}

int destuffing(unsigned char* stuffedFrame, unsigned char* destuffedFrame, int stuffedFrameSize) {
    int destuffedIndex = 0;
    int destuffedSize = 1;

    for (int index = 0; index < stuffedFrameSize; index++) {
        if (stuffedFrame[index] == ESCAPE) {        // finds ESCAPE value inside Stuffed Frame
            if (stuffedFrame[index + 1] == 0x5E) {  // ESCAPE value is followed by 0x5E
                destuffedFrame[destuffedIndex++] = FLAG;        // while stuffing, if flag is found, value is changed to ESCAPE+0x5E, so this is "inverse" function
            }
            else if (stuffedFrame[index + 1] == 0x5D) { // ESCAPE value is followed by 0x5D
                destuffedFrame[destuffedIndex++] = ESCAPE;
            }
        }
        else {
            destuffedFrame[destuffedIndex++] = stuffedFrame[index];    // no ESCAPE value, just append the same value that's on the stuffed frame     
        }

        destuffedSize++;    // with each for we add a new character to the destuffedFrame, therefore increasing the size by 1
    }
    
    destuffedSize--;

    return destuffedSize;
}

////////////////////////////////////////////////
// Connection Control 
////////////////////////////////////////////////

int SerialPortHandling(const char* serialPortName, int baudRate) {    // arguments that will be obtained from LinkLayer struct
    // Function based on provided SerialPort function in LAB1 and LAB2

    //open serial port device for reading and writing
    mainFrame.fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (mainFrame.fd < 0) {
        perror(serialPortName);
        return -1;
    }

    struct termios oldtio;
    struct termios newtio;

    // save current port settings
    if (tcgetattr(mainFrame.fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

    //clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo, ...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0; // Blocking read until 5 chars received

    tcflush(mainFrame.fd, TCIOFLUSH); // Flushes data received but not read

    // Set new port settings
    if (tcsetattr(mainFrame.fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    printf("New termios structure set\n");

    return 0;

}


////////////////////////////////////////////////
// LLOPEN 
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters) {
    //Opens file descriptor with Serial Port
    int connectionControl = SerialPortHandling(connectionParameters.serialPort, connectionParameters.baudRate);

    // dealing with errors derived from Serial Port connection
    if (connectionControl< 0) {
        perror("Connection Error");
        return -1;
    }

    //saving connectionParameters to created struct
    alarmConfig.timeout = connectionParameters.timeout;
    alarmConfig.nreTransmissions = connectionParameters.nRetransmissions;

    //llopen will do different tasks depending on the role

    LinkLayerRole role = connectionParameters.role;

    switch (role) {
        case LlTx:
            (void)signal(SIGALRM, alarmHandler);
            //building set buf into mainFrame.frame
            buildSupervisionFrame(A_TX, C_SET);

            sendFrame(mainFrame.fd, mainFrame.frame, mainFrame.size);    // sends set frame

            while (STOP == FALSE && alarmConfig.Counter < alarmConfig.nreTransmissions) {       // can't exceed the number of tries allowed by the program
                //receive 5 chars (bytes), ua connection and check for the values necessary
                int stateMachineControl = stateMachine(A_TX, C_UA);
                
                if (stateMachineControl == 0) {
                    alarm(0);
                    STOP = TRUE;
                }
                else {
                    if (alarmConfig.Enabled == FALSE) {
                        fprintf(stderr, "Error while receiving ua frame. Retrying...");
                        sendFrame(mainFrame.fd, mainFrame.frame, mainFrame.size);    // sends set frame again
                        alarmConfig.Enabled = TRUE;
                    }                    
                }
            }

            return mainFrame.fd;

        case LlRx:
            
            if (stateMachine(A_TX, C_SET) == 1) {
                fprintf(stderr, "Error while receiving ua frame. Retrying...");
            }
            else {
                sendSupervision(mainFrame.fd, A_RX, C_UA);
                break;
            }

        default:
            break;

    }

    
    return mainFrame.fd;

}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////


int llwrite(int fd,const unsigned char* buf, int bufSize)
{
    printf("Attempting to write...");
    // buf = array of characters to transmit

    buildInformationFrame(A_TX, buf, bufSize);

    unsigned char stuffedFrame[2*bufSize];

    int stuffedFrameSize = stuffing(mainFrame.frame, stuffedFrame, mainFrame.size);  // Stuffed frame
    sendFrame(mainFrame.fd, stuffedFrame, stuffedFrameSize + 1); // writing the frame in the file descriptor
    
    unsigned char control_field[5];

    STOP = FALSE;

    while (STOP == FALSE && alarmConfig.Counter<alarmConfig.nreTransmissions){
        int read_bytes = read(mainFrame.fd, control_field, 5);

        if (read_bytes && control_field[0] == FLAG && (control_field[1] == A_TX || control_field[1] == A_RX) && (control_field[2] == C_RR0 || control_field[2] == C_RR1) && control_field[3] == (control_field[1] ^ control_field[2]) && control_field[4] == FLAG) {
            // if control field equals RR0 or RR1 (ready to receive)
            alarm(0);
            frameTransmitterControl = (frameTransmitterControl + 1) % 2;  // guarantees that the value of TransmitterControl is always 1 or 0
            printf("Written Successfully!");
            STOP = TRUE;    // ready to receive ! 
        }
        else if (read_bytes && control_field[0] == FLAG && (control_field[1] == A_TX || control_field[1] == A_RX) && (control_field[2] == C_REJ0 || control_field[2] == C_REJ1) && control_field[3] == (control_field[1] ^ control_field[2]) && control_field[4] == FLAG) {
            // if control field equals REJ0 or REJ1 (receiver rejects information)
            alarm(0);
            printf("Information Frame rejected. Trying again...\n");
            sendFrame(mainFrame.fd, stuffedFrame, stuffedFrameSize + 1);       // attempts to write the frame again
        }
        else {
            if (alarmConfig.Enabled == FALSE) {
                sendFrame(mainFrame.fd, stuffedFrame, stuffedFrameSize + 1);   // attempts to write the frame again
            }
        }
    }

    if (alarmConfig.Counter >= alarmConfig.nreTransmissions) {
        fprintf(stderr, "Alarm Counter surpassed the possible number of Retransmissions");
        llclose(mainFrame.fd, 0);
        return -1;      // negative value because of the occurrence of an error
    }


    return stuffedFrameSize+1;  // number of characters written
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(int fd, unsigned char *packet)
{
    printf("Attempting to read...");

    STOP = FALSE;
    
    unsigned char frame[2000]; // 2000 provisional value
    unsigned char destuffedPayload[2000]; // 2000 provisional value

    enum State currentState = START;

    int currentPos = 0;
    int stuffedPos = 0;
    int destuffedPayloadSize;
    while (STOP == FALSE) {
        unsigned char byte;     // byte to be examined
        int control_Field;      // control field value
        int MachineControl = FALSE;         // False = Machine keeps working, True = Stop the Machine
        int disconnectControl = FALSE;      // Checks if we receive Control field with disconnect order
        unsigned char stuffedPayload[2000];

        while (MachineControl == FALSE) {
            if (read(mainFrame.fd, &byte, 1) > 0) {
                switch (currentState) {
                    case START:
                        if (byte == FLAG) {
                            frame[currentPos++] = byte;
                            currentState = FLAG_RCV;
                        }
                        else {
                            currentPos = 0; // if value is not FLAG, keep trying to read until get flag
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_TX) {
                            frame[currentPos++] = byte;
                            currentState = A_RCV;
                        }
                        else if (byte != FLAG){     // Since current state is flag_rcv, if byte == flag, we stay in the same state
                            currentState = START; // goes back to the start
                            currentPos = 0;
                        }
                        break;
                    case A_RCV:
                        if (byte == C_I0 || byte == C_I1) {     // Control Field of Information Frame
                            frame[currentPos++] = byte;
                            control_Field = byte;
                            currentState = CI_RCV;
                        }
                        else if(byte == C_SET){     // Control Field SET, that was sent by the transmitter to initiate a connection
                            control_Field = byte;
                            currentState = C_RCV;
                        }
                        else if (byte == C_DISC) {      // Control Field that indicates termination of a connection
                            control_Field = byte;
                            currentState = C_RCV;
                            disconnectControl = TRUE;
                        }
                        else if (byte == FLAG) {
                            currentPos = 1;
                            currentState = FLAG_RCV; // Starts Over Process from the Flag_RCV
                        }
                        else {
                            currentState = START;
                            currentPos = 0;     // Starts Over Process from the beginning
                        }
                        break;
                    case C_RCV:
                        if (byte == (frame[1] ^ control_Field)) {
                            frame[currentPos++] = byte;
                            currentState = BCC_OK;
                        }
                        else if (byte == FLAG) {
                            currentState = FLAG_RCV; // Start Over from FLAG
                            currentPos = 1;
                        }
                        else {
                            currentState = START; // Start Over
                            currentPos = 0;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            if (disconnectControl == TRUE) {
                                currentState = STOP_MACHINE_DC;
                            }
                            else {
                                currentState = STOP_MACHINE;
                            }
                        }
                        else {
                            currentState = START; // Start Over
                            currentPos = 0;
                        }
                        break;
                    // beginning to deal in case of payload exchange
                    case CI_RCV:
                        if (byte == (frame[1] ^ control_Field)) {
                            frame[currentPos++] = byte;
                            currentState = PAYLOAD;
                        }
                        else if (byte == FLAG) {
                            currentPos = 1;
                            currentState = FLAG_RCV; // Start Over from FLAG
                        }
                        else {
                            currentPos = 0; 
                            currentState = START;    // Start Over
                        }
                        break;
                    // reading the transferred data
                    case PAYLOAD:
                        if (byte == FLAG) {
                            frame[currentPos++] = byte;     // Start of frame after receiving full data field
                            currentState = BCC2_CHECK;
                        }
                        else {  // saving received payload to custom unsigned char[]
                            stuffedPayload[stuffedPos++] = byte;    //adds byte to stuffed payload char
                            currentPos++;               //frame size also increases
                        }
                        break;
                    case BCC2_CHECK:
                        destuffedPayloadSize = destuffing(stuffedPayload, destuffedPayload, stuffedPos);    // De-stuffing the data received
                        unsigned char bcc2 = BCC2(destuffedPayload, destuffedPayloadSize - 1);      // Checking for error in the data field
                        if (bcc2 == destuffedPayload[destuffedPayloadSize - 1]) {           // if bcc2 check turns out positive
                            currentState = BCC2_OK;
                        }
                        else {          // BCC2 check not successful
                            currentPos = 0;
                            currentState = START;   // Start the Process all Over
                            MachineControl = TRUE;
                            printf("Detected the occurrence of errors in the data field. Retrying...");
                            // send rejected frame with REJ0 or REJ1
                            if (frameReceiverControl == 0) {
                                sendSupervision(mainFrame.fd, A_RX, C_REJ0);
                            }
                            else {
                                sendSupervision(mainFrame.fd, A_RX, C_REJ1);
                            }
                        }
                        break;
                    case BCC2_OK:
                        if (byte == FLAG) {         // ending frame flag found! Success!
                            frame[currentPos++] = byte;     
                            currentState = STOP_MACHINE;
                        }
                        else {
                            currentState = START; // Start Over
                            currentPos = 0;
                        }
                        break;
                    case STOP_MACHINE:
                        if (frameReceiverControl == 0) {    // sends rr1 or rr0
                            sendSupervision(mainFrame.fd, A_TX, C_RR0);          
                        }
                        else {
                            sendSupervision(mainFrame.fd, A_RX, C_RR1);
                        }
                        frameReceiverControl = (frameReceiverControl + 1) & 2;
                        MachineControl = TRUE;
                        STOP = TRUE;
                        break;
                    case STOP_MACHINE_DC:
                        sendSupervision(mainFrame.fd, A_RX, control_Field);   // send disconnection frame to terminate connection
                        MachineControl = TRUE;
                        STOP = TRUE;
                        break;
                    default:
                        break;
                }
            }
        }
    }

    // save the destuffed payload to the unsigned char packet pointer in llread arguments 
    
    for (int i = 0; i < destuffedPayloadSize; i++) {
        packet[i] = destuffedPayload[i];
    }

    return destuffedPayloadSize;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd, int showStatistics)
{
    enum State currentState = START;
 
    (void)signal(SIGALRM, alarmHandler);
    
    buildSupervisionFrame(A_TX, C_DISC);        // creates frame with control field equal to the control disconnect value
    
    sendFrame(mainFrame.fd, mainFrame.frame, 5);    

    // receive disconnect order

    unsigned char byte;

    while (currentState != STOP_MACHINE) {
        if (read(mainFrame.fd, &byte, 1)) {
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
                    if (byte == C_DISC) {
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
                default:
                    break;
            }
        }
    }

    if (currentState != STOP_MACHINE) {           // Previous State Changes didn't work
        return -1;
    }

    // send ua disconnect frame

    sendSupervision(mainFrame.fd, A_TX, C_UA);
    alarm(0);

    return close(mainFrame.fd);
}
