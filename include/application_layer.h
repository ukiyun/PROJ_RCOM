// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include "link_layer.h"

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

unsigned char* sendControlPacket(const unsigned int controlField, const char* filename, long int fileSize, unsigned int* packetSize); // Signals the Start and End of File Transfer

unsigned char* sendDataPacket(const unsigned int controlField, unsigned char* data, int dataSize, int *packetSize);    // Contains fragments of the file to be transmitted

unsigned char* readControlPacket(unsigned char* packet, int size, unsigned long int* fileSize);

int sendFileTX(LinkLayer connectionParameters, const char* filename);

int getFileRX(LinkLayer connectionParameters);

#endif // _APPLICATION_LAYER_H_
