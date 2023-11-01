
#ifndef _UTLITIES_H_
#define _UTILITIES_H_

#include "macros.h"
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>


volatile int STOP;

int frameTransmitterControl;    // Control that the transmitter is sending
int frameReceiverControl;       // Control that the receiver is expecting

// Frame Holder
extern struct mainFrame_struct {
	unsigned char frame[MAX_PAYLOAD];
	size_t size;
	int fd;
}mainFrame;

// Alarm Config
extern struct alarmConfig_struct {
	int Counter;
	int timeout;
	int nreTransmissions;
	int alarmEnabled;
}alarmConfig;

void newAlarm();

void alarmHandler();

int SerialPortHandling(char serialPortName[50]); // Handling the SerialPort, based on TP1 and TP2 files

void buildSupFrame(unsigned char Address, unsigned char Control);    // Builds Supervision and Unnumbered Frames

void buildInfoFrame(unsigned char Address, const unsigned char* packet, int packetSize); // Builds Information Frame

void sendFrame(int fd, unsigned char* frame, int n);

int sendSup(int fd, unsigned char Address, unsigned char C);

unsigned char BCC2(unsigned char* frame, int size); // Field to detect the occurrence of errors in the data field

int stuffing(const unsigned char* frame, int frameSize, int* newSize); // Byte Stuffing Mechanism

int destuffing(const unsigned char* stuffedFrame, int frameSize, int* newSize); // Reverse Byte Stuffing Mechanism

#endif /* _UTILITIES_H_ */