#include "receiver.h"
#include "utilities.h"
#include "link_layer.h"
#include "state_machine.h"

volatile int STOP = FALSE;
int alarmCounter = 0;
int alarmEnabled = FALSE;

int llOpenReceiver(LinkLayer connectionParameters) {
	int fd; 

	if (SerialPortHandling(connectionParameters.serialPort) < 0 || fd < 0) {
		return -1; // either there's a problem with the SerialPort or with the File Descriptor
	}

	unsigned char set_frame[5];

	StateMachine* stM = (StateMachine*)malloc(sizeof(StateMachine));

	while (STOP == FALSE) {
		stateChange(stM, START); // change machine state to start, beginning of new frame
		int bytes = read(fd, set_frame, 5);
		set_frame[bytes] = '\0';
		stateTransition(stM, set_frame, bytes, A_TX, C_SET);
		if (stM->currentState == STOP) {
			sendSup(fd, A_RX, C_UA);
			STOP = TRUE;
		}
	}

	free(stM);  // deallocates the memory previously allocated by malloc
	printf("Receiver Successfully Opened");

	return 0;
}