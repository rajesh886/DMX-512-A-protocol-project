// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U1TX (PB1) and U1RX (PB0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef UART1_H_
#define UART1_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart1();
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart1(char c);
void putsUart1(char* str);
char getcUart1();
bool kbhitUart1();

#endif
