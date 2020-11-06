//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM

// Hardware configuration:
// UART Interface:
//   U1TX (PB1) and U1RX (PB0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart1.h"

// Initialize UART1
void initUart1()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTB_DIR_R |= UART_TX_MASK;                   // enable output on UART1 TX pin
    GPIO_PORTB_DIR_R &= ~UART_RX_MASK;                   // enable input on UART1 RX pin
    GPIO_PORTB_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART1 pins
    GPIO_PORTB_AFSEL_R &= ~(UART_TX_MASK | UART_RX_MASK);  // use peripheral to drive PB0, PB1
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M); // clear bits 0-7
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX);
    //GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;
                                                        // select UART1 to drive pins PB0 and PB1: default, added for clarity....

    // Configure UART1 to 250000 baud, 8N2 format
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART1_IBRD_R = 10;                                  // r = 40 MHz / (Nx250kHz)
    UART1_FBRD_R = 00;
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2| UART_LCRH_FEN ;    // configure for 8N2 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_EOT;
                                                        // enable TX, RX,module and end of transmission.

    NVIC_EN0_R |= 1 << (INT_UART1 - 16);
}

// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART1_IBRD_R = divisorTimes128 >> 7;                 // set integer value to floor(r)
    UART1_FBRD_R = ((divisorTimes128 + 1)) >> 1 & 63;    // set fractional value to round(fract(r)*64)
}

