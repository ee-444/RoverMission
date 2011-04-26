
#include "MissionConsole.h"

Cmissionconsole::Cmissionconsole()
{
	// set the baud rate
	UBRR0 = REG_BAUD_VAL;
	// set the data to 8-bits
	UCSR0C = 6;
}

Cmissionconsole::~Cmissionconsole()
{

}

void Cmissionconsole::enable()
{
	// enable interrupts??  definately TX AND RX enable
	UCSR0B |= 0x18;
}

void Cmissionconsole::disable()
{
	// disable the UART tx and rx
	UCSR0B = 0;
}

void Cmissionconsole::write(uint8_t in)
{
	//UDR0 = in;
}

void Cmissionconsole::write(char* in)
{
	// keep going until all chars are sent
	while (*in != 0x00){
		// wait until the last byte has been transfered
		while ( !( UCSR0A & (1<<5)) );
		// load the transmit buffer
		UDR0 = *in;
		// increment the packet pointer
		in++;
	}
}

void Cmissionconsole::adValue(uint16_t in)
{
	char* tmp = (char*)calloc(64, sizeof(char));
	char* tmp_base = tmp;
	// no room on the heap
	if (tmp == NULL){
		return;
	}
	// build the a/d conversion packet
	sprintf(tmp, "<a/d - %d>\n", in);
	// send it to the communication channel
	write(tmp);
	// release the data on the heap
	free(tmp_base);
}

void Cmissionconsole::longRangeIR(uint16_t in)
{
	char* tmp = (char*)calloc(64, sizeof(char));
	char* tmp_base = tmp;
	// no room on the heap
	if (tmp == NULL){
		return;
	}
	// build the a/d conversion packet
	sprintf(tmp, "<l.r.i.r. - %d>\n", in);
	// send it to the communication channel
	write(tmp);
	// release the data on the heap
	free(tmp_base);
}

void Cmissionconsole::mediumRangeIR(uint16_t in)
{
	char* tmp = (char*)calloc(64, sizeof(char));
	char* tmp_base = tmp;
	// no room on the heap
	if (tmp == NULL){
		return;
	}
	// build the a/d conversion packet
	sprintf(tmp, "<m.r.i.r. - %d>\n", in);
	// send it to the communication channel
	write(tmp);
	// release the data on the heap
	free(tmp_base);
}


