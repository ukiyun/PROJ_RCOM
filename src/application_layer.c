// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "macros.h"

// C libraries
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


// #define MAX_PAYLOAD_SIZE 1000


int sendControlPacket(int packetInfo, const char* fileName, long size) {            // packetInfo is either CONTROL_DATA/CONTROL_START/CONTROL_END
    int bits = sizeof(int) * 8 - __builtin_clz(size);       // calculates the number of bits required to represent an int with value size in binary
    unsigned char fileSize = (bits + 7) / 8;                            // L2
    unsigned char fileNameSize = strlen(fileName) + 1;                  // L1
    long packetSize = 5 + fileSize + fileNameSize;               // 5 + L1 + L2

    unsigned char* controlPacket = (unsigned char*)malloc(packetSize);

    unsigned int currentPos = 0;
    controlPacket[currentPos++] = packetInfo;
    controlPacket[currentPos++] = FILE_SIZE;
    controlPacket[currentPos++] = fileSize;
    memcpy(controlPacket + currentPos, &size, fileSize); //little endian 
    currentPos+= fileSize;
    controlPacket[currentPos++] = FILE_NAME;
    controlPacket[currentPos++] = fileNameSize;
    memcpy(controlPacket + currentPos, fileName, fileNameSize);
    printf("Filename: %s\n", fileName);
    printf("Filename Size: %d\n", fileNameSize);
    printf("TESTE %c\n", controlPacket[currentPos]);
    currentPos += fileNameSize;

    if (llwrite(controlPacket, packetSize)<0) {                             // in case it can't write the control Packet
        fprintf(stderr, "Failed while trying to send Control Packet\n");
        free(controlPacket);
        return -1;
    }

    free(controlPacket);            // Memory Allocation


    return 1;               // Successfully sent Control Packet
}

int sendDataPacket(unsigned char* data, int dataSize) {
    // K = number of octets (256 * L2 + L1) in the data field
    int L2 = dataSize / OCTET;
    int L1 = dataSize % OCTET;

    size_t packetSize = dataSize + 3;       // C + L1 + L2 + dataSize

    unsigned char* dataPacket = (unsigned char*)malloc(packetSize);

    dataPacket[0] = CONTROL_DATA;
    dataPacket[1] = L2;
    dataPacket[2] = L1;

    // get packet data field
    memcpy(dataPacket + 3, data, dataSize); // copies packet data field after control field, L2 and L1

    if (llwrite(dataPacket, packetSize) < 0) {                      // Write unsuccessful
        printf("Error trying to send Data Packet");
        fprintf(stderr, "Error trying to send Data Packet");
        return -1;
    }

    return 0;
}

int readControlPacket(unsigned char controlPacket, unsigned char* frame, size_t* fileSize, char* fileName) {
    int frameSize;

    if ((frameSize = llread(frame)) < 0) {                  // error while reading
        fprintf(stderr, "Error while trying to read Control Packet\n");
        return -1;
    }

    if (frame[0] != controlPacket) {                        // error in Control Packet
        fprintf(stderr, "Invalid Control Packet\n");
        return -1;
    }

    int fileNameSize = sizeof(fileName);
    printf("fileName Size: %d\n", fileNameSize);

    int index = 1;                                         // start after frame[0]
    unsigned char type;                                    // TLV

    while (index < frameSize) {
        type = frame[index++];                              // increments index after assigning frame[1] to type
        printf("Frame index %d\n", index);
        //dealing with type
        if (type == FILE_SIZE) {
            printf("FileSize Accessed\n");  
            *fileSize = frame[index];                       // type 0 = FILE SIZE INFO
            index += sizeof(size_t);                        // increment fileSize type size to index
        }
        else if (type == FILE_NAME) {
            printf("FileName Accessed\n");                  
            *fileName = frame[index];                       // type 1 = FILE NAME INFO
            index += fileNameSize;

        }
        else {
            fprintf(stderr, "Invalid Control Packet Type\n");
            return -1;
        }
    }
    return 1;                               // successfully read ControlPacket
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
    
    if (strcmp(role,"tx")== 0) {
        connectionParameters.role = LlTx;
    }
    else {
        connectionParameters.role = LlRx;
    }

    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) < 0) {
        fprintf(stdout, "Failed trying to establish a connection\n");
        return;
    }

    printf("Established a Connection!\n");

    switch (connectionParameters.role) {
        case LlTx: {

            FILE* fileTx = fopen(filename, "r"); // Opens non-text file for reading
            //no file open
            if (fileTx == NULL) {
                fprintf(stderr, "File not found TX\n");		//dealing with the case of no file
                exit(-1);
            }


            int fileStart = ftell(fileTx);    //get beginning position of file
            fseek(fileTx, 0, SEEK_END);	//  sets the file position indicator to the end of the fileSent
            long fileTxSize = ftell(fileTx);	// returns the current value of the position of the position indicator, since its at the end, we get the fileSize
            fseek(fileTx, fileStart, SEEK_SET);	// sets the file position indicator in the beginning of the fileSent

            printf("Trying to send Start Control Packet\n");

            if (sendControlPacket(CONTROL_START, filename, fileTxSize) == -1) {     // error in sending ControlPacket
                fprintf(stderr, "Error trying to send Control Packet");
                return;
            }

            printf("Start Control Packet Sent!\n ");

            unsigned char* dataPacket = (unsigned char*)malloc(sizeof(unsigned char)*fileTxSize) ;
            long dataSize;
            
            while ((dataSize = fread(dataPacket, sizeof(unsigned char),fileTxSize, fileTx)) > 0) {                // reads data from fileTx into dataPacket
                if (sendDataPacket(dataPacket, dataSize)) {
                    fprintf(stderr, "Failed to send Data Packet\n");
                    return;
                }
            }

            fclose(fileTx);

            printf("Sent Data Packet!\n");

            if (sendControlPacket(CONTROL_END, filename, fileTxSize) == -1) {
                fprintf(stderr, "Error trying to send Control Packet\n");
                return;
            }

            printf("Sent End Control Packet!\n");

            if (llclose(1) < 0) {
                fprintf(stderr, "Error Trying to close Link Layer");
                return;
            }

            printf("Link Layer Closed!\n");

            break;
        }

        case LlRx: {
            size_t startControlPacketSize;
            char rxFileName[0xff];
            unsigned char* startControlPacket = (unsigned char*)malloc(MAX_PAYLOAD);

            printf("Attempting to read Start Packet...\n");

            if (readControlPacket(CONTROL_START, startControlPacket, &startControlPacketSize, rxFileName) < 0) {
                fprintf(stderr, "Error trying to read Control Packet\n");
                return;
            }

            printf("Read Control Packet Successfully!\n");

            FILE* fileRx = fopen((char*)filename, "w");

            if (fileRx == NULL) {                             // dealing with the case of no file
                fprintf(stderr, "File Not Found (RX)");
                return;
            }

            printf("File Stream Opened!\n");
            
            int payloadSize = -1;

            while ((payloadSize = llread(startControlPacket)) <0) {           // llread positive, so there are no errors;
                if (payloadSize == 0) {
                    break;      
                }
                else if (startControlPacket[0] == CONTROL_END) {
                    printf("Received End Packet!\n");

                    fwrite(startControlPacket, sizeof(unsigned char), payloadSize-4, fileRx);
                    free(startControlPacket);
                }
                else {
                    break;
                }

            }

            printf("File Received!\n");
            fclose(fileRx);                           // Closes the Stream fileSent
            printf("The File Stream has been closed!\n");
            printf("Closing LinkLayer...\n");
            int disconnect = 1;
            while((disconnect = llread(startControlPacket))<0);
            printf("Disconnecting...\n");
            break;
        }
        default:
            exit(-1);
            break;

    }

}
