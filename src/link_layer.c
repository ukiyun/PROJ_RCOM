// Link layer protocol implementation

#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


volatile int STOP = FALSE;


////////////////////////////////////////////////
// LLOPEN 
////////////////////////////////////////////////
int llOpenTransmitter(LinkLayer connectionParameters) {

    (void)signal(SIGALRM, alarmHandler);

    buildSupFrame(A_TX, C_SET); // sets initial frame

    sendFrame(mainFrame.fd, mainFrame.frame, mainFrame.size);    // send initial frame

    alarmConfig.alarmEnabled = FALSE;

    int finish = FALSE;

    StateMachine* stM = (StateMachine*)malloc(sizeof(StateMachine));

    unsigned char received_ua_frame[5];

    while (finish == FALSE && alarmConfig.Counter < alarmConfig.nreTransmissions) {
        stateChange(stM, START);	// change machine state to start, beginning of new frame

        int bytes1 = read(mainFrame.fd, received_ua_frame, mainFrame.size);
        stateTransition(stM, mainFrame.frame, bytes1, A_TX, C_UA);

        if (stM->currentState == STOP_MACHINE) {
            alarm(0);
            finish = TRUE;
            printf("Transmitter Successfully Opened");
        }
        else {
            if (alarmConfig.alarmEnabled == FALSE) {
                sendFrame(mainFrame.fd, mainFrame.frame, mainFrame.size);   // reattempts to send frame again
                alarmConfig.alarmEnabled = TRUE;
            }
        }
        alarmConfig.Counter++;
    }

    return mainFrame.fd;
}

int llOpenReceiver(LinkLayer connectionParameters) {

    unsigned char received_frame[5];

    StateMachine* stM = (StateMachine*)malloc(sizeof(StateMachine));
    stateChange(stM, START); // change machine state to start, beginning of new frame
    sleep(4);

    while (STOP == FALSE) {
        int bytes = read(mainFrame.fd, received_frame, 5);
        
        stateTransition(stM, received_frame, bytes, A_TX, C_SET);

        if (stM->currentState == STOP_MACHINE) {
            STOP = TRUE;
        }
    }

    sendSup(mainFrame.fd, A_RX, C_UA);
    free(stM);  // deallocates the memory previously allocated by malloc
    printf("Receiver Successfully Opened");
    return mainFrame.fd;
}


int llopen(LinkLayer connectionParameters){

    if (SerialPortHandling(connectionParameters.serialPort) < 0) {
        fprintf(stderr, "Connection Error!\n");
        return -1;
    }
   
    alarmConfig.Counter = 0;
    alarmConfig.nreTransmissions = connectionParameters.nRetransmissions;
    alarmConfig.timeout = connectionParameters.timeout;
    LinkLayerRole roles = connectionParameters.role;

    switch (roles) {
        case LlTx:
            llOpenTransmitter(connectionParameters);
        case LlRx:
            llOpenReceiver(connectionParameters);
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

    buildInfoFrame(A_TX, buf, bufSize);

    unsigned char new_frame[2* bufSize];

    int new_frame_size = stuffing(mainFrame.frame, new_frame, mainFrame.size);  // Stuffed frame

    sendFrame(mainFrame.fd, new_frame, new_frame_size + 1); // writing the frame in the file descriptor
    
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
            sendFrame(mainFrame.fd, new_frame, new_frame_size + 1);       // attempts to write the frame again
        }
        else {
            if (alarmConfig.alarmEnabled == FALSE) {
                sendFrame(mainFrame.fd, new_frame, new_frame_size + 1);   // attempts to write the frame again
            }
        }
    }

    if (alarmConfig.Counter >= alarmConfig.nreTransmissions) {
        fprintf(stderr, "Alarm Counter surpassed the possible number of Retransmissions");
        llclose(mainFrame.fd, 0);
        return -1;      // negative value because of the occurrence of an error
    }


    return new_frame_size+1;  // number of characters written
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(int fd, unsigned char *packet)
{
    printf("Attempting to read...");

    StateMachine* stMRead = (StateMachine*)malloc(sizeof(StateMachine));
    stateChange(stMRead, START);

    STOP = FALSE;
    
    unsigned char frame[2000]; // 2000 provisional value
    unsigned char destuffedPayload[2000]; // 2000 provisional value
    int currentPos = 0;
    int stuffedPos = 0;
    int destuffedPayloadSize;
    while (STOP == FALSE) {

        unsigned char byte;
        int control_Field;
        int MachineControl = FALSE;         // False = Machine keeps working, True = Stop the Machine
        int disconnectControl = FALSE;
        unsigned char stuffedPayload[2000];
        while (MachineControl == FALSE) {
            if (read(mainFrame.fd, &byte, 1) > 0) {
                switch (stMRead->currentState) {
                    case START:
                        if (byte == FLAG) {
                            frame[currentPos++] = byte;
                            stateChange(stMRead, FLAG_RCV);
                        }
                        else {
                            currentPos = 0; // if value is not FLAG, keep trying to read until get flag
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_TX) {
                            frame[currentPos++] = byte;
                            stateChange(stMRead, A_RCV);
                        }
                        else if (byte == FLAG) {
                            currentPos = 1;  // if byte = FLAG, it stays in the same state with the currPos = 1;
                        }
                        else {
                            stateChange(stMRead, START); // goes back to the start
                            currentPos = 0;
                        }
                        break;
                    case A_RCV:
                        if (byte == C_I0 || byte == C_I1) {     // Control Field of Information Frame
                            frame[currentPos++] = byte;
                            control_Field = byte;
                            stateChange(stMRead, CI_RCV);
                        }
                        else if(byte == C_SET){     // Control Field SET, that was sent by the transmitter to initiate a connection
                            control_Field = byte;
                            stateChange(stMRead, C_RCV);
                        }
                        else if (byte == C_DISC) {      // Control Field that indicates termination of a connection
                            control_Field = byte;
                            stateChange(stMRead, C_RCV);
                            disconnectControl = TRUE;
                        }
                        else if (byte == FLAG) {
                            currentPos = 1;
                            stateChange(stMRead, FLAG); // Starts Over Process from the Flag_RCV
                        }
                        else {
                            stateChange(stMRead, START);
                            currentPos = 0;     // Starts Over Process from the beginning
                        }
                        break;
                    case C_RCV:
                        if (byte == (frame[1] ^ control_Field)) {
                            frame[currentPos++] = byte;
                            stateChange(stMRead, BCC_OK);
                        }
                        else if (byte == FLAG) {
                            stateChange(stMRead, FLAG_RCV); // Start Over from FLAG
                            currentPos = 1;
                        }
                        else {
                            stateChange(stMRead, START); // Start Over
                            currentPos = 0;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            if (disconnectControl == TRUE) {
                                stateChange(stMRead, STOP_MACHINE_DC);
                            }
                            else {
                                stateChange(stMRead, STOP_MACHINE);
                            }
                        }
                        else {
                            stateChange(stMRead, START); // Start Over
                            currentPos = 0;
                        }
                        break;
                    // beginning to deal in case of payload exchange
                    case CI_RCV:
                        if (byte == (frame[1] ^ control_Field)) {
                            frame[currentPos++] = byte;
                            stateChange(stMRead, PAYLOAD);
                        }
                        else if (byte == FLAG) {
                            currentPos = 1;
                            stateChange(stMRead, FLAG_RCV); // Start Over from FLAG
                        }
                        else {
                            currentPos = 0; 
                            stateChange(stMRead, START);    // Start Over
                        }
                        break;
                    // reading the transferred data
                    case PAYLOAD:
                        if (byte == FLAG) {
                            frame[currentPos++] = byte;     // Start of frame after receiving full data field
                            stateChange(stMRead, BCC2_CHECK);
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
                            stateChange(stMRead, BCC2_OK);
                        }
                        else {          // BCC2 check not successful
                            currentPos = 0;
                            stateChange(stMRead, START);    // Start the Process all Over
                            MachineControl = TRUE;
                            printf("Detected the occurrence of errors in the data field. Retrying...");
                            // send rejected frame with REJ0 or REJ1
                            if (frameReceiverControl == 0) {
                                sendSup(mainFrame.fd, A_RX, C_REJ0);
                            }
                            else {
                                sendSup(mainFrame.fd, A_RX, C_REJ1);
                            }
                        }
                        break;
                    case BCC2_OK:
                        if (byte == FLAG) {         // ending frame flag found! Success!
                            frame[currentPos++] = byte;     
                            stateChange(stMRead, STOP_MACHINE);
                        }
                        else {
                            stateChange(stMRead, START); // Start Over
                            currentPos = 0;
                        }
                        break;
                    case STOP_MACHINE:
                        if (frameReceiverControl == 0) {    // sends rr1 or rr0
                            sendSup(mainFrame.fd, A_TX, C_RR0);          
                        }
                        else {
                            sendSup(mainFrame.fd, A_RX, C_RR1);
                        }
                        frameReceiverControl = (frameReceiverControl + 1) & 2;
                        MachineControl = TRUE;
                        STOP = TRUE;
                        break;
                    case STOP_MACHINE_DC:
                        sendSup(mainFrame.fd, A_RX, control_Field);   // send disconnection frame to terminate connection
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

    StateMachine* stMClose= (StateMachine*)malloc(sizeof(StateMachine));
    stateChange(stMClose, START);

    (void)signal(SIGALRM, alarmHandler);
    
    buildSupFrame(A_TX, C_DISC);        // creates frame with control field equal to the control disconnect value
    
    sendFrame(mainFrame.fd, mainFrame.frame, 5);    

    // receive disconnect order

    unsigned char byte;

    while (stMClose->currentState != STOP_MACHINE) {
        if (read(mainFrame.fd, &byte, 1)) {
            switch (stMClose->currentState) {
                case START:
                    if (byte == FLAG) {
                        stateChange(stMClose, FLAG_RCV);
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_RX) {
                        stateChange(stMClose, A_RCV);
                    }
                    else if (byte != FLAG) {
                        stateChange(stMClose, START);
                    }
                    break;
                case A_RCV:
                    if (byte == C_DISC) {
                        stateChange(stMClose, C_RCV);
                    }
                    else if (byte == FLAG) {
                        stateChange(stMClose, FLAG_RCV);
                    }
                    else {
                        stateChange(stMClose, START);
                    }
                    break;
                case C_RCV:
                    if (byte == (A_RX ^ C_DISC)) {
                        stateChange(stMClose, BCC_OK);
                    }
                    else if (byte == FLAG) {
                        stateChange(stMClose, FLAG_RCV);
                    }
                    else {
                        stateChange(stMClose, START);
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        stateChange(stMClose, STOP_MACHINE);
                    }
                    else {
                        stateChange(stMClose, START);
                    }
                default:
                    break;
            }
        }
    }

    if (stMClose->currentState != STOP_MACHINE) {           // Previous State Changes didn't work
        return -1;
    }

    // send ua disconnect frame

    sendSup(mainFrame.fd, A_TX, C_UA);
    alarm(0);

    return close(mainFrame.fd);
}
