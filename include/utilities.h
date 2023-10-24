
#ifndef _UTLITIES_H_
#define _UTILITIES_H_

#include "macros.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

void alarmHandler();

int SerialPortHandling(char serialPortName[50]); // Handling the SerialPort, based on TP1 and TP2 files

unsigned char* stuffing(const unsigned char* frame, int frameSize, int* newSize); // Byte Stuffing Mechanism

unsigned char* destuffing(const unsigned char* stuffedFrame, int frameSize, int* newSize); // Reverse Byte Stuffing Mechanism

#endif /* _UTILITIES_H_ */