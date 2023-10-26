#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include <stdio.h>
#include "macros.h"


/*
HEADER DEDICATED TO IMPLEMENTATIONS OF ALL FUNCTIONS/MACROS RELATED TO THE STATE MACHINES
*/

// Enumeration of the possible states of the machine.
typedef enum {
	START,		// Initial State
	FLAG_RCV,		// Received Flag
	A_RCV,			// Received Address Field
	C_RCV,			// Received Control Field
	BCC_OK,			// Received Independent Protection Field and is valid
	STOP_MACHINE	// End State
}State;


// Defining Structure State Machine

typedef struct {  
	State currentState;
}StateMachine;
//some state machine functions

void stateChange(StateMachine* stM, State state); // changes the current state of the machine

void stateTransition(StateMachine* stM, unsigned char* frame, int size, unsigned char Address, unsigned char Control); // Address and Control depend on the role

int getControlField(); // gets the value stored in control field of the frame


#endif /* _STATE_MACHINE_H_ */