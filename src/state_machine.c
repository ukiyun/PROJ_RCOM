#include "state_machine.h"

void stateChange(StateMachine* machineState, State state) {
    machineState->currentState = state;
}

void stateTransition(StateMachine* stM, unsigned char* frame, int size, unsigned char Address, unsigned char Control) {
    for (int i = 0; i < size; i++) {
        unsigned char byte = frame[i];

        switch (stM->currentState) {
            case START:
                if (byte == FLAG) {
                    stateChange(stM, FLAG_RCV);
                }
                break;
            case FLAG_RCV:
                if (byte == Address) {
                    stateChange(stM, A_RCV);
                }
                else if (byte != FLAG) {
                    stateChange(stM, START);
                }
                break;
            case A_RCV:
                if (byte == Control) {
                    stateChange(stM, C_RCV);
                }
                else if (byte == FLAG) {
                    stateChange(stM, FLAG_RCV);
                }
                else {
                    stateChange(stM, START);
                }
                break;
            case C_RCV:
                if (byte == (Address ^ Control)) {
                    stateChange(stM, BCC_OK);
                }
                else if (byte == FLAG) {
                    stateChange(stM, FLAG_RCV);
                }
                else {
                    stateChange(stM, START);
                }
                break;
            case BCC_OK:
                if (byte == FLAG) {
                    stateChange(stM, STOP_MACHINE);
                }
                else {
                    stateChange(stM, START);
                }
                break;
            default:
                break;
        }
            
    }
}

int getControlField() {
	unsigned char currentByte = 0;
	unsigned char saveControlByte = 0;

	//initiating state machine
	StateMachine* stM = (StateMachine*)malloc(sizeof(StateMachine));
	stateChange(stM, START);

	while ((stM->currentState != STOP_MACHINE) && (alarmConfig.alarmEnabled == FALSE)) {
		if (read(fd, &currentByte, 1) > 0) {        // will read bytes (if they exist one by one, verifying at every point the main structure of the frame
			switch (stM->currentState) {
			case START:
				if (currentByte == FLAG) {
					stateChange(stM, FLAG_RCV);
				}
				break;
			case FLAG_RCV:
				if (currentByte == A_RX) {
					stateChange(stM, A_RCV);
				}
				else if (currentByte != FLAG) {
					stateChange(stM, START);
				}
				break;
			case A_RCV:
				if (currentByte == C_RR0 || currentByte == C_RR1 || currentByte == C_REJ0 || currentByte == C_REJ1 || currentByte == C_DISC) {			// if control field is found, we save the control indication that will affect how the llwrite works
					saveControlByte = currentByte; 		// Control Byte for Testing	
					stateChange(stM, C_RCV);
				}
				else if (currentByte == FLAG) {
					stateChange(stM, FLAG_RCV);
				}
				else {
					stateChange(stM, START);
				}
				break;
			case C_RCV:
				if (currentByte == (A_RX ^ saveControlByte) {
					stateChange(stM, BCC_OK);
				}
				else if (currentByte == FLAG) {
					stateChange(stM, FLAG_RCV);
				}
				else {
					stateChange(stM, START);
				}
				break;
			case BCC_OK:
				if (currentByte == FLAG) {
					stateChange(stM, STOP_MACHINE);
				}
				else {
					stateChange(stM, START);
				}
				break;
			default:
				break;
			}

		}


	}

	return saveControlByte;
}