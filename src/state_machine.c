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