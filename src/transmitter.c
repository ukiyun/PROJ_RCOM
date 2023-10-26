#include "transmitter.h"
#include "utilities.h"
#include "link_layer.h"
#include "state_machine.h"


int llOpenTransmitter(LinkLayer connectionParameters) {
		(void)signal(SIGALRM, alarmHandler);
	
		int fd; // File Descriptor
	
		if (SerialPortHandling(connectionParameters.serialPort) < 0 || fd < 0) {
			return -1; // either there's a problem with the SerialPort or with the File Descriptor
		}
		alarmConfig.timeout = connectionParameters.timeout;
		alarmConfig.nreTransmissions = connectionParameters.nRetransmissions;
		buildSupUnnFrames(A_TX, C_SET); // sets initial frame
		
		int finish = FALSE;
		
		StateMachine* stM = (StateMachine*)malloc(sizeof(StateMachine));

		while (finish == FALSE && alarmConfig.Counter<alarmConfig.nreTransmissions) {
			stateChange(stM, START);	// change machine state to start, beginning of new frame
			sendFrame(fd, mainFrame.frame, mainFrame.size); // sends the connection set
			
			alarmConfig.alarmEnabled = FALSE;
			int bytes1 = read(fd, mainFrame.frame, mainFrame.size);
			stateTransition(stM, mainFrame.frame, bytes1, A_RX, C_UA);
			
			if (stM->currentState == STOP_MACHINE) {
				finish = TRUE;
				printf("Transmitter Successfully Opened");
				return 0;
			}
			alarmConfig.nreTransmissions--;
		}

		return -1;
}
