#ifndef _TRANSMITTER_H_
#define _TRANSMITTER_H_

int llOpenTransmitter(LinkLayer connectionParameters);

void applicationLayerTransmitter(LinkLayer connectionParameters, const char *filename);

#endif _TRANSMITTER_H_