// Name - Rajesh Rayamajhi
// ID   - 1001520360

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "hibernation.h"
#include "eeprom.h"
#include "uart1.h"
#include "timer.h"
#include "wait.h"

#define N 1024                              //maximum length of the buffer

#define MAX_CHARS 80
#define MAX_FIELDS 8

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
}USER_DATA;

uint16_t LED_TIMEOUT_OFF = 0;

uint16_t LED_TIMEOUT_ON = 0;

uint32_t data_received_timer = 0;

uint32_t data_table[513];                   //table for storing the data. max address is 512

uint32_t poll_table[513];                   //table for polling the data. One address is polled per 512 byte.

uint32_t device_address[513];               //list of device address that was polled stored here

char str[100];

uint32_t max = 513;                         //maximum address

bool on = true;                             //variable that determines whether the transmitter is on or off.

bool poll_req = false;                      //Has the controller requested the poll ?

char txbuffer[N];                           //Ring buffer that store the character

uint32_t wr_idx = 0;

uint32_t rd_idx = 0;

uint32_t phase = 0;                         //no of phases for transmitter

uint32_t rx_phase = 0;                      //no of phases for receiver

uint32_t last_phase = 0;                    //counter that keeps track of which address was polled after polling one address at the beginning.

uint32_t last_rx_phase = 0;                 //counter that keeps track when polling is done

uint32_t last_searched_address =  0;

bool polled = false;                        //Is the device in transmitter mode while polling ?

bool polledC = false;                       //Is the controller in receiver mode while polling ?

uint8_t receiver_phase = 0;                 //phases of receiver while it is in transmitter mode when it is in the process of sending ack

uint32_t mode = 0;                          //varaible that determines controller or device

uint32_t add = 0;

uint32_t data = 0;                         //variable that stores the data from the uart data register.

uint32_t poll_data = 0;

uint32_t n = 0;                         //global variable that stores the number of seconds.

uint32_t sortedArray[4][3];             //sorted array to store the task in ascending order.

uint32_t RTCALM = 0;                    //value that will be loaded to the match register.

// Bitband aliases
#define RED_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define DE            (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))


// PortF masks
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8

// PORT C MASK
#define DE_MASK 128

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks on GPIO
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    //Enable clocks on Timer
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    //Enable clock on PWM
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;  // bits 1 is output
    GPIO_PORTF_DR2R_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;  // enable LEDs

    //configure PORTC for Data enable pin
    GPIO_PORTC_DIR_R |= DE_MASK;                                        //DE pin is output.
    GPIO_PORTC_DR2R_R |= DE_MASK;                                       //set drive strength to 2mA.
    GPIO_PORTC_DEN_R |= DE_MASK;                                        //use digital function

}

void initPWM()
{
    GPIO_PORTF_AFSEL_R |=  BLUE_LED_MASK ;
    GPIO_PORTF_PCTL_R &= GPIO_PCTL_PF2_M ;
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;
    // Configure PWM module 1 to drive RGB
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
    SYSCTL_SRPWM_R = 0;
    //PWM1_2_CTL_R = 0;
    PWM1_3_CTL_R = 0;
    //PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;

    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;

    //PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;

    //PWM1_2_LOAD_R = 1024;
    PWM1_3_LOAD_R = 1024;
    PWM1_INVERT_R = PWM_INVERT_PWM6INV ;

    //PWM1_2_CMPB_R = 0;
    //PWM1_3_CMPB_R = 0;
    PWM1_3_CMPA_R = 0;

    //PWM1_2_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM6EN ;
}

void setRgbColor(uint16_t blue)
{
//    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
//    PWM1_3_CMPB_R = green;
}

//This function receives characters from the user interface
void getsUart0(USER_DATA* data)
{
  uint8_t count = 0;
  while(true){

    char c;
    c = getcUart0();

    if(mode == 1)               //if the device is in controller mode
    {
        BLUE_LED = 1;           //turn on the blue LED

        LED_TIMEOUT_OFF = 5;    //Wait for the blue LED to turn off
    }

    if(c==127||c==8)  //checking if the character entered is DEL or backspace
    {
       if(count>0)
       {
         count--;
         continue;
       }
       else{
       continue;
       }
    }

    else if(c==10 || c==13) //checking if the character entered is Line feed or carriage return
    {
       data->buffer[count]='\0';
       return;
    }

    else if(c >= 32)
    {
       if(count==MAX_CHARS) //if the count of characters in the buffer is equal to the maximum, then put the null
                             //character in the end of the buffer. Else, increment the count.
       {
         data->buffer[count]='\0';
         return;
       }

       else{
       data->buffer[count++]=c;
       continue;
       }

     }

  }
}

/* this function checks whether the character is alphabet or not
 * 65-90 = A - Z & 97 - 122 = a - z */
bool alpha(char c)
{
  if((c>=65 && c<=90) || (c>=97 && c<=122))
  {
      return true;
  }
  else{
      return false;
  }
}

/* this function checks whether the character is numeric or not
 * 48 - 57 = 0 - 9, 45 is hyphen & 46 is dot*/
bool numeric(char c)
{
    if((c<=9) || (c>=48 && c<=57) || (c>=45 && c<=46))
    {
       return true;
    }
    else{
       return false;
    }
}

/* This is a function that takes the buffer string from the getsUart0() function and
processes the string in-place and returns information about the parsed fields in
fieldCount, fieldPosition, and fieldType. */
void parseFields(USER_DATA* data)
{

    uint8_t Count = 0; //number of field counts
    uint8_t i = 0;     //keeps track of offset of the field within the buffer

    while( (data->buffer[i] != 0))
    {
        if(data->fieldCount == MAX_FIELDS)
        {

            return;
        }

        if(i == 0 && ( (alpha(data->buffer[i])) || (numeric(data->buffer[i])) ) )
        {
          if( (alpha(data->buffer[i])) )
          {
             data->fieldPosition[Count] = i;
             data->fieldType[Count] = 97;     //record the type of field. 97 - a and 110 - n
             data->fieldCount = Count+1;
             Count++;
          }
          else if( numeric(data->buffer[i]) )
          {

              data->fieldPosition[Count] = i;
              data->fieldType[Count] = 110;
              data->fieldCount = Count+1;
              Count++;

          }

        }

        if( !(alpha(data->buffer[i])) && !(numeric(data->buffer[i])) )
        {
            data->buffer[i] = 0;

            if(alpha(data->buffer[i+1]))
            {

                data->fieldPosition[Count] = i+1;
                data->fieldType[Count] = 97;
                data->fieldCount = Count+1;
                Count++;


            }

            else if(numeric(data->buffer[i+1]))
            {

                data->fieldPosition[Count] = i+1;
                data->fieldType[Count] = 110;
                data->fieldCount = Count+1;
                Count++;

            }
         }

        i++;
    }
}

/* Returns the value of a field requested if the field number is in range or NULL otherwise */
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
    {
       if(data->fieldType[fieldNumber] == 97)
       {
          return &data->buffer[data->fieldPosition[fieldNumber]];
       }
    }

      return '\0';
}

/* Returns a pointer to the field requested if the field number is in range and the field type
is numeric or 0 otherwise */
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    char num[5]={0};
    if(fieldNumber <= data->fieldCount)
    {
        if(data->fieldType[fieldNumber] == 110)
        {
            //this part of code will run until the null char is reached.
            uint8_t count = data->fieldPosition[fieldNumber];
            uint8_t pos=0;
            while(data->buffer[count] != 0)
            {
                num[pos] = data->buffer[count];
                pos++;
                count++;
            }

            int32_t value = atoi(num);          //convert string to integer using c library

            //code and discard
            //int32_t value = (num[0]-'0')*100+(num[1]-'0')*10+ (num[2]-'0');

            return value;
        }
    }

    return 0;
}

//this function compares two strings.
bool compare(char *str1, char *str2)
{
    while (*str1 == *str2 || *str1+32 == *str2)
    {
        if (*str1 == '\0' && *str2 == '\0')
            return true;
        str1++;
        str2++;
    }

    return false;
}

/* This function returns true if the command matches the first field and the number of
arguments (excluding the command field) is greater than or equal to the requested
number of minimum arguments */

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    char *name = &data->buffer[data->fieldPosition[0]];

    if(compare(name, strCommand))
    {
        if((data->fieldCount)-1 == minArguments || (data->fieldCount)-1 >= 5)
        {
            return true;
        }
    }

  return false;
}

//function that sets the mode of device as controller or device from EEPROM at the beginning
void setMode()
{
    writeEeprom(0,1);
}

//function that sets the address of the device
void setAdd(uint32_t mode_num, uint32_t add)
{
    writeEeprom(0, mode_num);
    writeEeprom(1, add);
}

//this function prints the character to the screen using the UART0 Interrupt
void displayUart0(char str[])
{

    bool full;                                  //buffer is full if wr_idx == rd_idx

    full = ((wr_idx+1) % N) == rd_idx;

    if(!full)
    {

        while (str[wr_idx] != '\0')             //write into the buffer until the char is null;
        {
           txbuffer[wr_idx] = str[wr_idx];

           wr_idx++;
        }
        if(UART0_FR_R & UART_FR_TXFE)           //check if the FIFO is empty
        {
           UART0_DR_R = txbuffer[rd_idx] ;      //if empty, write the data into the data register to create an interrupt.

           rd_idx++;
        }

        UART0_IM_R |= UART_IM_TXIM;            //Turn on the interrupt for uart0Isr.
    }
}

void uart0RxIsr()
{
    if(UART0_FR_R & UART_FR_TXFE)               //transition of fifo from non-empty to empty
    {
        UART0_DR_R = txbuffer[rd_idx] ;         //write the value to the data register from the ring buffer

        rd_idx++;

        if(rd_idx == wr_idx)                    //if read index and write index are equal
        {

            UART0_IM_R &= ~UART_IM_TXIM;        //turn off the interrupt

            rd_idx = 0;                         //reset the read index

            wr_idx=0;                           //reset the write index
        }
     }
}

void dmxTX()
{
    DE = 1;                            // enabling the PC7 pin high

    D = 0;                             //transmitting break condition '0'

    phase = 0;                         //first phase is 0 for sending break

    initTimer1();                      //Break is transferred for 176us. Causes Frame Error and Break Error in receiver called "synchronization event"
}

void Timer1Isr()
{
    if(mode == 1)                                   //if the device mode is controller
    {
        if(phase == 0)                              //if the device is done sending the break
        {
            D = 1;                                  //transmit mark after break(MAB) "1" for 12us

            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

            TIMER1_TAILR_R = 480;                   //timer is re-enabled for 12 us

            TIMER1_CTL_R |= TIMER_CTL_TAEN;

            phase = 1;                              //increase the phase
        }
        else if(phase == 1)
        {
            TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;

            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;       //Turning off timer

            TIMER1_TAILR_R = 7040;                 //reload the value in timer to configure the timer for 176us for sending the break once the controller is done sending 512 values

            GPIO_PORTB_AFSEL_R |= UART_TX_MASK ;                //UART1 AFSEL and PCTL on
                                                                //only transmit is enabled if the device is in the controller mode.

            GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX ;


            if(!poll_req)                               //if the poll has not been requested
            {
              UART1_DR_R = 0;                           //write start code of 0
            }

            if(poll_req)                               //if the controller requests the poll
            {
                UART1_DR_R  = 0xF7;                     //write start code of 0xF7
            }

            UART1_IM_R |= UART_IM_TXIM;                 //turn on the transmit interrupt for UART1Isr

            phase = 2;                                  //phase gets incremented to 2 after sending start code to the device
        }

        TIMER1_ICR_R = TIMER_ICR_TATOCINT;              //clear the interrupt flag for timer
    }
    else if(mode == 0xFFFFFFFF && polled == true)       //if the device mode is device
    {

        if(receiver_phase == 0)                         //the device becomes transmitter when polling device
        {
            TIMER1_ICR_R = TIMER_ICR_TATOCINT;          //clear the interrupt flag for timer

            DE = 1;                                     // enabling the PC7 pin high

            D = 0;                                      //transmitting break condition '0'

            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;            //turn off the timer

            TIMER1_TAILR_R = 8000;                      //transmit the break for 176us

            TIMER1_IMR_R |= TIMER_IMR_TATOIM;           //turn on the interrupt

            TIMER1_CTL_R |= TIMER_CTL_TAEN;             //turn on the timer

            GREEN_LED = 0;                              //turn off green LED

            data_received_timer = 0;                    //reset the timer to 0.

            RED_LED = 1;                                //signal the break condition by turning on the RED LED

            LED_TIMEOUT_OFF = 40;                       //RED LED will be turned off after the value decrements to zero.

            receiver_phase = 1;                         //increment the phase for receiver that is sending the ack.
        }

        else if(receiver_phase == 1)
        {
            DE = 0;                                    //release the transmitter

            TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;          //turn off the interrupt

            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;            //turn off the timer

            TIMER1_ICR_R = TIMER_ICR_TATOCINT;          //clear the interrupt flag

            GPIO_PORTB_AFSEL_R |= UART_RX_MASK;         //turn on the AFSEL

            GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB0_U1RX;    //turn on the PCTL

            UART1_IM_R |= UART_IM_RXIM;                 //turn on the receiver interrupt to change the device from transmitter to receiver

            receiver_phase = 0;                         //reset the phase of receiver to 0 to start from the beginning when polling.
        }
    }

}

void uart1Isr()
{
    if(UART1_MIS_R & UART_MIS_TXMIS)                                //if the interrupt was given by transmit interrupt
    {
        if(!polled)                                                 //polling is off
        {
            if(((phase-1)< (max+2)) && !poll_req)                   //start the phase from 1 to 515. +2 was needed because the transmitter was not working properly. polling is off
            {
               while (UART1_FR_R & UART_FR_TXFF);

               UART1_DR_R = data_table[phase-1];                    //write the data to the UART data register.

               phase++;                                             //increment the phase
            }
            else if(((phase-1) < (max+2)) && poll_req)              //polling request is on
            {
                while (UART1_FR_R & UART_FR_TXFF);

                UART1_DR_R = poll_table[phase-1];                   //write the value of 1 from the poll table to the data register. Only one address is polled
                                                                    //each time by sending 1 to the address. The other values are 0.

                phase++;                                            //increment the phase.

                if((phase-2) == 514)                                //if the phase reached to 514
                {
                    UART1_ECR_R = 0;                                //clear the framing error.

                    UART1_ICR_R |= UART_ICR_FEIC;                   //clear the framing mask interrupt.

                    UART1_ICR_R |= UART_ICR_BEIC;                   //clear the breaking mask interrupt.

                    polledC = true;                                 //polling is true for the controller. value that is used to change the controller from transmitter to the receiver
                                                                    //to receive break from the devices.

                    GPIO_PORTB_AFSEL_R &= ~(UART_TX_MASK);          //turn off AFSEL

                    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_U1TX);    //turn off PCTL

                    UART1_IM_R &= ~UART_IM_TXIM;                    //turn off transmit interrupt

                    UART1_ICR_R |= UART_ICR_TXIC;                   //clear the transmit interrupt flag

                    GPIO_PORTB_AFSEL_R |= UART_RX_MASK;             //turn on AFSEL for RX to become receiver

                    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB0_U1RX;        //turn on PCTL

                    UART1_IM_R |= UART_IM_RXIM;                     //turn on receiver interrupt

                }

            }
            else
            {
                UART1_IM_R &= ~UART_IM_TXIM;                        //turn off the transmit interrupt for the transmitter

                GPIO_PORTB_AFSEL_R &= ~(UART_TX_MASK);              //turn off AFSEL for the transmit

                GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_U1TX);         //turn off PCTL for the transmit

                while (UART1_FR_R & UART_FR_BUSY);

                if (on)                                             //if the transmitter is on
                {
                    DE = 0;                                         //release the transmitter

                   dmxTX();

                }
            }
        }
    }
    else if(UART1_MIS_R & UART_MIS_RXMIS)
    {
        if(!polledC)                                                    //if the polling is off for the controller
        {
            data = UART1_DR_R;                                          //read the data register

            if(data & UART_DR_BE)                                       //check to see break error
            {
                if(!poll_req)                                           //if polling is off
                {
                   setRgbColor(data_table[add]);                        //control the PWM of the blue LED on the board.
                }

                rx_phase = 0;
            }

            else
            {

                if( rx_phase < (max +2) )                               //start the receiver phase from 0 to 515. +2 was needed because the receiver was not working properly
                {
                   data_table[rx_phase] = data;                         //starting index 0 always contains the start code.

                   if(data_table[rx_phase] == 247 && rx_phase == 0)     //if the start code is 0xF7 then polling is on for the receiver.
                   {
                      poll_req = true;

                   }
                   else if(data_table[rx_phase] == 0 && rx_phase == 0)  //if the start code is 0 then polling is off the receiver.
                   {
                       poll_req = false;
                   }

                   rx_phase++;                                          //increment the receiver phase.

                   if (rx_phase == 513)                                 //if the receiver phase reaches to the 513 which means after receiving 512 byte
                   {
                       data_received_timer++;                           //44us * 512 * 45 is nearly equal to the 1 second. if the
                                                                        //data was received in the last 1 second, then turn on the green LED.

                       if(data_received_timer == 45)
                       {
                           GREEN_LED = 1;

                           data_received_timer = 0;                     //reset the data received timer for the device.
                       }

                       if( (data_table[add] == 1) && (poll_req == true) )   //if the poll request is on and the value of device addresss is 1 then prepare the device for sending the ack.
                       {
                         sprintf(str,"Sending Ack from %d\r\n",add);

                         displayUart0(str);

                         polled = true;

                         GPIO_PORTB_AFSEL_R &= ~(UART_RX_MASK | UART_TX_MASK);              //turn off AFSEL for transmit and receive.

                         GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX);   //turn off PCTL for transmit and receive.

                         UART1_IM_R &= ~ (UART_IM_RXIM | UART_IM_TXIM);                     //turn off interrupt for the transmit and receive.

                         TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

                         TIMER1_TAILR_R = 640;                                              //reload the timer to create an interrupt after 12us.

                         TIMER1_IMR_R |= TIMER_IMR_TATOIM;

                         TIMER1_CTL_R |= TIMER_CTL_TAEN;
                       }
                   }

                }
            }
        }

        else if(polledC)                                                    //if the controller has requested the poll for the device, then controller becomes receiver to receive break from the device.
        {
            while (UART1_FR_R & UART_FR_BUSY);

            DE = 0;                                                         //release the transmitter

            waitMicrosecond(200);                                           //wait to receive the break from the device.

            poll_data = UART1_DR_R;                                         //read the data register

            if(poll_data & UART_DR_BE)                                      //check to see if the break was received from the device
            {
              if(UART1_RIS_R & UART_RIS_BERIS)                              //check to see if the breaking interrupt mask was set by the de
              {
                 sprintf(str,"    Ack was received from the device with the address %d.\r\n\n",last_phase);

                 displayUart0(str);

                 device_address[last_phase] = last_phase;                  //record the device address into the table.

                 RED_LED = 0;                                              //turn off red led for a moment to turn on the green LED

                 LED_TIMEOUT_ON = 40;

                 GREEN_LED = 1;                                             //turn on green LED to indicate an ack was received from the device by the controller.

                 LED_TIMEOUT_OFF = 20;
              }
            }

            GPIO_PORTB_AFSEL_R &= ~( UART_RX_MASK | UART_TX_MASK);

            GPIO_PORTB_PCTL_R &= ~( GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX);

            UART1_IM_R &= ~( UART_IM_RXIM | UART_IM_TXIM );

            UART1_ICR_R |= UART_ICR_RXIC | UART_ICR_TXIC;

            req_poll();                                                 //polling continues until all the addresses are searched.
        }

    }

}

//This Isr controls the blinking of LEDs on the board.
void Timer2Isr()
{
  if(mode == 1)
  {
      if(LED_TIMEOUT_OFF > 0)
      {
         LED_TIMEOUT_OFF--;
         if(LED_TIMEOUT_OFF == 0)
         {
            BLUE_LED = 0;
            GREEN_LED = 0;
         }
      }

      if(LED_TIMEOUT_ON > 0)
      {
          LED_TIMEOUT_ON--;
          if(LED_TIMEOUT_ON == 0)
          {
             RED_LED = 1;
          }
      }
  }
  if(mode == 0xFFFFFFFF)
  {
      if(LED_TIMEOUT_OFF > 0)
      {
         LED_TIMEOUT_OFF--;
         if(LED_TIMEOUT_OFF == 0)
         {
            RED_LED = 0;
            GREEN_LED = 0;
         }
      }

      if(LED_TIMEOUT_ON > 0)
      {
          LED_TIMEOUT_ON--;
          if(LED_TIMEOUT_ON == 0)
          {
             GREEN_LED = 1;
          }
      }
  }
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;   //clear the interrupt flag for timer
}

void hibernationIsr()
{

    HIB_IC_R |= HIB_IC_RTCALT0;

    data_table[sortedArray[0][1]] = sortedArray[0][2];     //set the value to the data table by reading the first index in the sorted array.

    uint32_t j = 0;
    if(sortedArray[0][0] !=0 )
    {
        for(j = 0; j < 3; j++)                              //move the data in the table
        {
            sortedArray[j][0] = sortedArray[j+1][0];
            sortedArray[j][1] = sortedArray[j+1][1];
            sortedArray[j][2] = sortedArray[j+1][2];
        }
    }

    sortedArray[3][0] = 0;
    sortedArray[3][1] = 0;
    sortedArray[3][2] = 0;

    setAlarm(sortedArray[0][0]);                            //load the match register with the smallest value from the sorted Array.

}

//this function clears the data in the data_table array
void clear()
{
    uint32_t k=0;
    for(k=0; k<513; k++)
    {
        data_table[k]=0;
    }
}

void req_poll()
{
    poll_req = true;

    poll_table[last_phase] = 0;                             //clear the last address which we requested poll

    last_phase++;

    poll_table[last_phase] = 1;                             //poll the next address on the bus

    dmxTX();                                                //continue the transmit for the next address polling

    if(last_phase == 513)                                   //if we are done polling for the all addresses on the bus
    {
        last_phase = 0;                                     //reset the counter

        displayUart0("Polling Done\r\n\n");

        poll_req = false;                                   //disable the polling request

        dmxTX();                                            //continue the normal operation of transmit of sending data to the address.
    }
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    USER_DATA data;

    initHw();

    initUart0();
    setUart0BaudRate(115200, 40e6);

    initUart1();
    setUart1BaudRate(250000, 40e6);

    initEeprom();
    initHibernationHW();


    //Power on Flash by turning on and off of the on board Blue Led
    BLUE_LED = 1;
    waitMicrosecond(200000);
    BLUE_LED = 0;
    waitMicrosecond(200000);
    BLUE_LED = 1;
    waitMicrosecond(200000);
    BLUE_LED = 0;

    //Turning off the data enable pin
    DE = 0;

    mode = readEeprom(0);                               //read the mode of the device

    add = readEeprom(1);                                //read the address of the device

    if(mode == 0xFFFFFFFF)                              //mode is device
    {

        if(add == 0xFFFFFFFF)
        {
            add = 1;
        }

        initPWM();

        initTimer1();

        TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;

        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

        GPIO_PORTB_AFSEL_R |= UART_RX_MASK;

        GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB0_U1RX;

        UART1_IM_R |= UART_IM_RXIM;

        DE = 0;

        displayUart0("Device Mode entered\r\n");

        initTimer2();
    }

    else if(mode == 1)                                      //mode is controller.
    {
        displayUart0("Controller mode was entered\r\n");
        initTimer2();

        if(on)
        {
           RED_LED = 1;
           dmxTX();
        }
    }


    while(true)
    {
       memset(&data, 0, sizeof(data));

       getsUart0(&data);

       displayUart0(data.buffer);

       displayUart0("\r\n");

       parseFields(&data);

       bool valid = false;


       if(isCommand(&data, "set", 2))
       {
           uint32_t add = getFieldInteger(&data, 1);

           uint32_t value = getFieldInteger(&data, 2);

           data_table[add]=value;

           valid = true;
       }

       if(isCommand(&data, "get", 1))
       {
           uint32_t add = getFieldInteger(&data, 1);

           uint32_t value = data_table[add];

           sprintf(str, "%d\n",value);

           displayUart0(str);

           valid = true;
       }

       if(isCommand(&data, "clear", 0))
       {
           clear();

           valid = true;
       }

       if(isCommand(&data, "max", 1))
       {
           uint32_t val = getFieldInteger(&data, 1);

           max = val;

           sprintf(str, "%d\n",max);

           displayUart0(str);

           valid = true;
       }

       if(isCommand(&data, "on", 0))
       {
           on = true;

           RED_LED = 1;

           if(mode == 1)  //if the mode is controller
           {
             dmxTX();
           }

           valid = true;
       }

       if(isCommand(&data, "off", 0))
       {
           on = false;

           RED_LED = 0;

           valid = true;
       }

       if(isCommand(&data, "poll", 0))
       {
           if(mode == 1)                //the polling request is valid if the device is in controller mode
           {
               clear();
               req_poll();
               valid = true;
           }
       }

       if(isCommand(&data, "time", 0))
       {
           get_time(&hr, &min, &sec);

           valid = true;

           //code and discard
           uint32_t time_elapsed = getCurrentSeconds();

           uint32_t hour1 = time_elapsed/3600;

           uint32_t min1 = (time_elapsed%3600)/60;

           char str[200];

           sprintf(str, "seconds:%u    time in hr and min=> %u:%u\r\n",time_elapsed,hour1, min1);

           displayUart0(str);
       }

       if(isCommand(&data, "time", 3))
       {
           set_time(getFieldInteger(&data, 1),getFieldInteger(&data, 2),getFieldInteger(&data, 3));
           n = calculateSeconds(hr, min ,sec, 0, 0);
           setRTC(n);
           valid = true;
       }

       if(isCommand(&data, "date", 0))
       {
           get_date(&month, &day);
           valid = true;
       }

       if(isCommand(&data, "date", 2))
       {
           set_date(getFieldInteger(&data, 1),getFieldInteger(&data, 2));

           n = calculateSeconds(hr, min ,sec, day, month);

           setRTC(n);

           valid = true;
       }

       if(isCommand(&data, "setat", 7))
       {
           uint32_t add = getFieldInteger(&data, 1);

           uint32_t value = getFieldInteger(&data, 2);

           set_time(getFieldInteger(&data, 3),getFieldInteger(&data, 4),getFieldInteger(&data, 5));

           set_date(getFieldInteger(&data, 6),getFieldInteger(&data, 7));

           n = calculateSeconds(hr, min ,sec, day, month);

           addToSortedList(n, add, value, sortedArray);

           sortArray(sortedArray);

           while(!(HIB_CTL_R & 0x80000000));                //Wait for the WRC bit to set before writing to the interface

           HIB_RTCM0_R = sortedArray[0][0];                 //Load the first value from the first index in the sorted array to the match register.

           //code and discard
//           uint8_t i = 0;
//           for( i =0; i<4; i++)
//           {
//              sprintf(str, "%d\r\n",sortedArray[i][0]);
//              displayUart0(str);
//
//           }

           valid = true;
       }

       if(isCommand(&data, "controller", 0))              //controller command sets the device mode as controller
       {
           setMode();

           displayUart0("Controller mode was set.\r\n");

           valid = true;
       }

       if(isCommand(&data, "device", 1))                //device [address] sets the device as device mode and address. The default address is 1 if the address is not set.
       {
           LED_TIMEOUT_OFF = 4;                         //turn off the GREEN LED if the address changes

           device_address[add] = 0;                     //the device address is being changed on the bus so reset the address on the device address found list.

           uint32_t addr = getFieldInteger(&data, 1);

           setAdd(0xFFFFFFFF, addr);

           displayUart0("Device mode was set.\r\n");

           valid = true;

           add = readEeprom(1);

           displayUart0("Add: ");

           sprintf(str, "%d\n",add);

           displayUart0(str);

           displayUart0("\r\n");
       }

       if(isCommand(&data, "devinfo", 0))               //my own command to print the address of the device
       {
           valid = true;

           add = readEeprom(1);

           displayUart0("Add: ");

           sprintf(str, "%d\n",add);

           displayUart0(str);

           displayUart0("\r\n");
       }

       if(isCommand(&data, "list", 0))               //my own command to print the list of addresses that were polled
       {
           valid = true;

           uint16_t index = 0;

           bool found = false;

           for(index = 0; index<513; index++)
           {
              if(device_address[index] != 0)
              {
                  sprintf(str,"Address %d \r\n", device_address[index]);

                  displayUart0(str);

                  found = true;
              }
           }

           if(!found)
           {
               displayUart0("No device has been polled yet by the controller.\r\n");
           }
       }

       displayUart0("\r\n");


       if(!valid)
           displayUart0("Invalid Command\r\n");

     }
}
