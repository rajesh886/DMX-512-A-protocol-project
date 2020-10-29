
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


#define N 1024
#define MAX_CHARS 80
#define MAX_FIELDS 8
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
}USER_DATA;

uint32_t data_table[512];
char str[100];
uint32_t max;
bool on;
bool poll_req = false;
char txbuffer[N];
uint8_t wr_idx = 0;
uint8_t rd_idx = 0;
//uint32_t hr;
//uint32_t min;
//uint32_t sec;

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

// PortF masks
#define RED_LED_MASK 2

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= RED_LED_MASK;  // bits 1 is output
    GPIO_PORTF_DR2R_R |= RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= RED_LED_MASK;  // enable LEDs
}

//This function receives characters from the user interface
void getsUart0(USER_DATA* data)
{
  uint8_t count = 0;
  while(true){

    char c;
    c = getcUart0();

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
            int32_t value = atoi(num);

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
        if((data->fieldCount)-1 == minArguments)
        {
            return true;
        }
    }

  return false;
}

void req_poll(){
    poll_req = true;

    //TO DO: who clears bit
}

void setMode(){
    writeEeprom(0,1);
}

void setAdd(uint32_t mode_num, uint32_t add){
    writeEeprom(1, mode_num);
    writeEeprom(2, add);
}

void displayUart0(char str[])
{
    //TO DO: write code to display the longer characters by the UART0.
    //buffer is full if wr_idx == rd_idx
    bool full;
    full = ((wr_idx+1) % N) == rd_idx;
    if(!full)
    {
        //write into the buffer until the char is null;
        while (str[wr_idx] != '\0')
        {
           txbuffer[wr_idx] = str[wr_idx++];
        }
        if(UART0_FR_R & UART_FR_TXFE) //checking to see if the fifo is empty
        {
           //putcUart0(txbuffer[rd_idx++]);
           UART0_DR_R = txbuffer[rd_idx++] ;
        }
        UART0_IM_R |= UART_IM_TXIM;  //Turning on the interrupt for uart0Isr.
    }
}

void uart0RxIsr()
{
    if(UART0_FR_R & UART_FR_TXFE) //transition of fifo from non-empty to empty
    {
//     //putcUart0(txbuffer[rd_idx++]);
        rd_idx++;
        UART0_DR_R = txbuffer[rd_idx] ;
        if(rd_idx == wr_idx)
        {
            UART0_IM_R &= ~UART_IM_TXIM;
            rd_idx = 0;
            wr_idx=0;
        }

      }
}

void Timer1Isr()
{
    TIMER1_TAILR_R = 480;
    GPIO_PORTB_DATA_R = 1;
    waitMicrosecond(12);
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;
    UART1_DR_R = 0xF7;
    if(UART0_FR_R & UART_FR_TXFE)
    {
        UART1_IM_R |= UART_IM_TXIM;
    }
}

void uart1Isr()
{
    uint8_t n=0;
    while(n!=max)
    {
       UART1_DR_R = data_table[n];
       n++;
    }
    UART0_IM_R &= ~UART_IM_TXIM;
    //TO DO: This should be in a continuous loop
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    USER_DATA data;

    initHw();

    initUart0();

    initEeprom();

    setUart0BaudRate(115200, 40e6);

    while(true)
    {
       memset(&data, 0, sizeof(data));

       getsUart0(&data);

       putsUart0(data.buffer);

       putsUart0("\r\n");

       parseFields(&data);

       bool valid = false;

       if(isCommand(&data, "set", 2))
       {
           uint32_t add = getFieldInteger(&data, 1);
           uint32_t value = getFieldInteger(&data, 2);
           data_table[add]=value;
           displayUart0("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz\r\n");
           //displayUart0("1234567891234567891234567890123456789012345678901234568");
           valid = true;
       }

       if(isCommand(&data, "get", 1))
       {
           uint32_t add = getFieldInteger(&data, 1);
           uint32_t value = data_table[add];
           sprintf(str, "%d\n",value);
           putsUart0(str);
           valid = true;
       }

       if(isCommand(&data, "clear", 0))
       {
           uint32_t k=0;
           for(k=0; k<512; k++){
               data_table[k]=0;
           }
           //putsUart0("Data table cleared successfully.\r\n");
           valid = true;
       }

       if(isCommand(&data, "max", 1))
       {
           uint32_t val = getFieldInteger(&data, 1);
           max = val;
           sprintf(str, "%d\n",max);
           putsUart0(str);
           valid = true;
       }

       if(isCommand(&data, "on", 0))
       {
           on = true;
       }

       if(isCommand(&data, "off", 0))
       {
           on = false;
       }

       if(isCommand(&data, "poll", 1))
       {
           req_poll();
           valid = true;
       }

       if(isCommand(&data, "time", 0))
       {
           get_time(&hr, &min, &sec);
           valid = true;
       }

       if(isCommand(&data, "time", 3))
       {
           set_time(getFieldInteger(&data, 1),getFieldInteger(&data, 2),getFieldInteger(&data, 3));
//           sprintf(str, "Hour: %d\r\n", hr);
//           putsUart0(str);
//
//           sprintf(str, "Minute: %d\r\n", min);
//           putsUart0(str);
//
//           sprintf(str, "Seconds: %d\r\n", sec);
//           putsUart0(str);
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
           valid = true;
       }

       if(isCommand(&data, "setat", 7))
       {
           uint32_t add = getFieldInteger(&data, 1);
           uint32_t value = getFieldInteger(&data, 2);
           data_table[add]=value;
           set_time(getFieldInteger(&data, 3),getFieldInteger(&data, 4),getFieldInteger(&data, 5));
           set_date(getFieldInteger(&data, 6),getFieldInteger(&data, 7));
           valid = true;
       }

       if(isCommand(&data, "controller", 0))
       {
           setMode();
           valid = true;
       }

       if(isCommand(&data, "device", 1))
       {
           uint32_t addr = getFieldInteger(&data, 1);
           setAdd(0, addr);
           valid = true;
       }

//       uint32_t b = readEeprom(1);
//       sprintf(str,"%d\n",b);
//       putsUart0(str);
//       uint32_t c = readEeprom(2);
//       sprintf(str,"%d\n",c);
//       putsUart0(str);

//      Debugging code
//       if(on){
//           putsUart0("success\n");
//       }

       if(isCommand(&data, "run",0))
       {
           initUart1();
           setUart1BaudRate(250000, 40e6);
           initTimer1();
           GPIO_PORTB_DATA_R = 0;
           TIMER2_CTL_R |= TIMER_CTL_TAEN;
           waitMicrosecond(176);
       }

       putsUart0("\r\n");

       if(!valid)
           putsUart0("Invalid Command\r\n");

     }
}
