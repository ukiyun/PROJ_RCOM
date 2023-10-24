
#ifndef _MACROS_H_
#define _MACROS_H_


//boolean values
#define TRUE 1
#define FALSE 0

#define BUF_SIZE 256

// buffer indexes
#define FLAG 0x7E		// Start or End of frame
#define A_TX 0x03		// Commands sent by Transmitter or replies sent by the Receiver
#define A_RX 0x01		// Commands sent by Receiver or replies sent by the Transmitter
#define C_SET 0x03		// sent by the transmitter to establish a connection
#define C_UA 0x07		// sent by the receiver to confirm the connection


// responses
#define C_RR0 0x05		// indication sent by the receiver that it is ready to receive an information frame nº 0
#define C_RR1 0x85		// indication sent by the receiver that it is ready to receive an information frame nº 1
#define C_REJ0 0x01		// indication sent by the receiver that it has rejected an information frame nº 0 (detected an error)
#define C_REJ1 0x81		// indication sent by the receiver that it has rejected an information frame nº 1 (detected an error)

// commands
#define C_I0 0x00		// I = Information frame number 0
#define C_I1 0x40		// I = Information frame number 1
#define C_DISC 0x0B		// sent by the transmitter to terminate the connection

#define ESCAPE 0x7D		// byte stuffing
#define BAUDRATE B38400 // baud-rate


#endif /* _MACROS_H_ */