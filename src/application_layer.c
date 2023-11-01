// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#define MAX_PAYLOAD_SIZE 1024

unsigned char* sendControlPacket(const unsigned int controlField, const char* filename, long int fileSize, unsigned int* packetSize) {
    
    const unsigned int L1 = sizeof(fileSize);
    const unsigned int L2 = strlen(filename);

    *packetSize = 3 + L1 + 2 + L2;

    printf("Control Packet size in FileName %s is %i\n", filename, *packetSize);
    
    unsigned char* controlPacket = (unsigned char*)malloc(*packetSize);

    unsigned int currentPos = 0;

    // Populating Control Packet

    controlPacket[currentPos++] = controlField;
    controlPacket[currentPos++] = FILE_SIZE;
    controlPacket[currentPos++] = L1;

    for (unsigned char i = 0; i < L1; i++) {
        controlPacket[2 + L1 - i] = fileSize & 0xFF;
        fileSize >>= 8;         // Shifts fileSize 8 positions to the right, e.g if fileSize = 10000000000 in binary, it will become 100. Or decimal value divided by 256
    }
    currentPos += L1;
    controlPacket[currentPos++] = FILE_NAME;
    controlPacket[currentPos++] = L2;

    memcpy(controlPacket + currentPos, filename, L2);   // Copies the characters from the filename to a certain position in the Control Packet

    return controlPacket;
}



unsigned char* sendDataPacket(unsigned char* packetData, int dataSize, int* packetSize) {
    
    // K = number of octets (256 * L2 + L1) in the data field
    int L2 = dataSize / OCTET;
    int L1 = dataSize % OCTET;

    *packetSize = 3 + dataSize;         // C + L1 + L2 + dataSize

    unsigned char* dataPacket = (unsigned char*)malloc(*packetSize);

    dataPacket[0] = CONTROL_DATA;
    dataPacket[1] = L2;
    dataPacket[2] = L1;

    // get packet data field

    memcpy(dataPacket + 3, packetData, dataSize); // copies packet data field after control field, L2 and L1


    return dataPacket;
}

unsigned char* readControlPacket(unsigned char* packet, long* size, unsigned long int* filenameSize) {
    unsigned char fileSizeBytes = packet[6];    // V value (number of octets indicated in L)
    unsigned char* fileSizeAuxiliary = (unsigned char*)malloc(fileSizeBytes);
    memcpy(fileSizeAuxiliary, packet + 7, fileSizeBytes); // copies file Size Auxiliary packet to the end of the Control Packet

    unsigned char fileNameBytes = packet[7 + fileSizeBytes + 1];
    unsigned char* fileName = (unsigned char*)malloc(fileNameBytes);
    memcpy(&fileNameBytes, packet + 7 + fileSizeBytes + 2, fileNameBytes);
    *filenameSize = fileNameBytes;
    return fileName;
}

unsigned char* getInfo(FILE* fileReceived, long int fileSize) {
    unsigned char* data = (unsigned char*)malloc(sizeof(unsigned char) * fileSize);         // creates data array with the size of the file
    fread(data, sizeof(unsigned char), fileSize, fileReceived);         // reads the data from the fileReceived stream, into the array data
    return data;
}

void readDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* frame) {
    memcpy(frame, packet + 4, packetSize - 4);
    frame += packetSize;
}

int sendFileTX(const char* filename) {
    FILE* fileSent = fopen(filename, "rb");		// Opens non-text file for reading

    if (fileSent == NULL) {
        fprintf(stderr, "File not found\n");		//dealing with the case of no file
        return -1;
    }

    fseek(fileSent, 0, SEEK_END);	//  sets the file position indicator to the end of the fileSent
    long fileSentSize = ftell(fileSent);	// returns the current value of the position of the position indicator, since its at the end, we get the fileSize
    fseek(fileSent, 0, SEEK_SET);	// sets the file position indicator in the beginning of the fileSent

    printf("Sent File has the following size: %ld\n", fileSentSize);

    unsigned int startControlPacketSize;
    unsigned char* startControlPacket = sendControlPacket(CONTROL_START, filename, fileSentSize, &startControlPacketSize);

    if (llwrite(mainFrame.fd, startControlPacket, startControlPacketSize) == -1) {  //llwrite error
        fprintf(stderr, "Error while sending Start Control Packet\n");
        return -1;
    }

    printf("Start Control Packet Sent!");

    unsigned char* sentData = (unsigned char*)malloc(MAX_PAYLOAD_SIZE - 3);

    int sentDataSize;
    int dataPacketSize;
    while ((sentDataSize = fread(sentData, 1, MAX_PAYLOAD_SIZE - 3, fileSent)) > 0) {   // reads data from the given stream fileSent and saves them to sentData / reads until there's no more data
        unsigned char* dataPacket = sendDataPacket(sentData, sentDataSize, &dataPacketSize);
        if (llwrite(mainFrame.fd, dataPacket, dataPacketSize) == -1) {
            fprintf(stderr, "Error while sending Data Packet\n");
            return -1;
        }
    }
    unsigned int endControlPacketSize;
    unsigned char* endControlPacket = sendControlPacket(CONTROL_END, filename, fileSentSize, &endControlPacketSize);

    if (llwrite(mainFrame.fd, endControlPacket, endControlPacketSize) == -1) {  //llwrite error
        fprintf(stderr, "Error while sending End Control Packet\n");
        return -1;
    }

    return 0;

}


int getFileRX(const char* filename) {
    unsigned char* packet = (unsigned char*)malloc(MAX_PAYLOAD_SIZE);
    long packetSize;
    unsigned long int *fileNameSize = 0;
    unsigned char* packetFileName = readControlPacket(packet, &packetSize, fileNameSize);    // unused?

    if (llwrite(mainFrame.fd, (const unsigned char*)packet, packetSize) == -1) {  //llwrite error
        fprintf(stderr, "Error while sending Start Control Packet\n");
        return -1;
    }

    printf("Opening File with Name: %s\n", packetFileName);

    FILE* fileGot = fopen(filename,"wb+");

    if (fileGot == NULL){
        fprintf(stderr, "File not found\n");		//dealing with the case of no file
        return -1;
    }

    int fileSize;

    while ((fileSize = llread(mainFrame.fd, packet)) >= 0) {
        if (fileSize == 0) {
            continue;
        }
        if (packet[0] == CONTROL_END) {
            break;
        }
        if (packet[0] != CONTROL_DATA) {
            fprintf(stderr, "Invalid Data Packet\n");
            return -1;
        }

        unsigned long int dataPacketSize = packet[1] * OCTET + packet[2];

        unsigned char* dataPacketBytes = (unsigned char*)malloc(dataPacketSize);

        memcpy(dataPacketBytes, packet + 3, fileSize);  
        
        if (fwrite(dataPacketBytes, sizeof(unsigned char*), dataPacketSize, fileGot) != dataPacketSize) {
            fprintf(stderr, "Failed to write to file\n");
            return -1;
        }

        free(dataPacketBytes);
    }

    printf("File Received!");

    fclose(fileGot);

    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    
    if (strncmp(serialPort, "/dev/ttyS", 9) != 0) {     // compares Serial Port String with /dev/ttyS, 0 if strings are equal.
        fprintf(stderr, "Invalid Serial Port: %s\n", serialPort);
        return;
    }

    if ((strcmp(role, "tx") != 0) && (strcmp(role, "rx") != 0)) {       // check role names
        fprintf(stderr, "Invalid role: %s\n", role);
        return;
    }

    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);      // copies serialPort given into to link layer Serial Port
    if (strcmp(role,(const char*)"tx")) {
        connectionParameters.role = LlTx;
    }
    else {
        connectionParameters.role = LlRx;
    }

    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) < 0) {
        fprintf(stderr, "Failed trying to establish a connection\n");
        return;
    }

    printf("Established a Connection!");

    if (connectionParameters.role == LlTx) {
        if (sendFileTX(filename) == -1) {
            fprintf(stderr, "Failed to Send File\n");
            return;
        }
        printf("File Sent!");
    }
    else {
        if (getFileRX(filename) == -1) {
            fprintf(stderr, "Failed to Receive File\n");
            return;
        }
        printf("File Received!");
    }

    if (llclose(mainFrame.fd, 0) == -1) {
        fprintf(stderr, "Failed trying to close connection\n");
        return;
    }

    printf("Connection Closed!");
}
