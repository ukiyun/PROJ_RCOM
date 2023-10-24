#include "transmitter.h"
#include "utilities.h"
#include "link_layer.h"
#include "state_machine.h"

int alarmCounter = 0;
int alarmEnabled = FALSE;

int llOpenTransmitter(LinkLayer connectionParameters) {
		(void)signal(SIGALRM, alarmHandler);
	
		int fd; // File Descriptor
	
		if (SerialPortHandling(connectionParameters.serialPort) < 0 || fd < 0) {
			return -1; // either there's a problem with the SerialPort or with the File Descriptor
		}

		unsigned char set_frame[5] = { FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG }; // sets initial frame
		int ended = FALSE;

		StateMachine* stM = (StateMachine*)malloc(sizeof(StateMachine));

		while (ended == FALSE && connectionParameters.nRetransmissions != 0) {
			stateChange(stM, START);	// change machine state to start, beginning of new frame
			int bytes = write(fd, set_frame, 5);	// set size = 5
			alarm(connectionParameters.timeout);
			alarmEnabled = FALSE;
			int bytes1 = read(fd, set_frame, 5);
			stateTransition(stM, set_frame, bytes1, A_RX, C_UA);
			
			if (stM->currentState == STOP) {
				ended = TRUE;
			}
			connectionParameters.nRetransmissions--;
		}

		return -1;

}
