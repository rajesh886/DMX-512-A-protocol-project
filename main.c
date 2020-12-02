
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
#include "polling.h"

//maximum length of the buffer
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

uint16_t LED_TIMEOUT_OFF = 10;
uint16_t LED_TIMEOUT_ON = 10;
uint32_t data_table[513];           //table for storing the data. max address is 512
uint32_t poll_table[512];
uint32_t device_address[512];
char str[100];
uint32_t max = 512;                 //maximum address
bool on = true;
bool poll_req = false;
char txbuffer[N];
uint32_t wr_idx = 0;
uint32_t rd_idx = 0;
uint32_t phase = 0;
uint32_t rx_phase = 0;
uint32_t last_phase = 0;
uint32_t last_rx_phase = 0;

uint32_t mode = 0;
uint32_t add = 0;
uint32_t data = 0;
uint32_t poll_data = 0;

uint32_t n = 0;                 //global variable that stores the number of seconds.
uint32_t sortedArray[512][3];   //sorted array to store the task in ascending order.
uint32_t RTCALM = 0;

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

    //configure portc for Data enable pin
    GPIO_PORTC_DIR_R |= DE_MASK;   //DE pin is output.
    GPIO_PORTC_DR2R_R |= DE_MASK;  //set drive strength to 2mA.
    GPIO_PORTC_DEN_R |= DE_MASK;    //use digital function

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
        if((data->fieldCount)-1 == minArguments || (data->fieldCount)-1 >= 5)
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
    writeEeprom(0, mode_num);
    writeEeprom(1, add);
}

void displayUart0(char str[])
{
    //buffer is full if wr_idx == rd_idx
    bool full;
    full = ((wr_idx+1) % N) == rd_idx;
    if(!full)
    {
        //write into the buffer until the char is null;
        while (str[wr_idx] != '\0')
        {
           txbuffer[wr_idx] = str[wr_idx];
           wr_idx++;
        }
        if(UART0_FR_R & UART_FR_TXFE) //checking to see if the fifo is empty
        {
           UART0_DR_R = txbuffer[rd_idx] ;
           rd_idx++;
        }

        UART0_IM_R |= UART_IM_TXIM;  //Turning on the interrupt for uart0Isr.
    }
}

void uart0RxIsr()
{
    if(UART0_FR_R & UART_FR_TXFE) //transition of fifo from non-empty to empty
    {
        UART0_DR_R = txbuffer[rd_idx] ;
        if(mode == 1)
        {
            BLUE_LED = 1;
            LED_TIMEOUT_OFF = 10;
            initTimer2();
        }
        rd_idx++;
        if(rd_idx == wr_idx)
        {
            UART0_IM_R &= ~UART_IM_TXIM;
            rd_idx = 0;
            wr_idx=0;
        }
     }
}

void dmxTX()
{
    DE = 1;                            // enabling the PC7 pin high
    D = 0;       //transmitting break condition '0' for 176 us.
    phase = 0;
    initTimer1();
}

void Timer1Isr()
{
    if(phase == 0)
    {
        D = 1;    //transmitting 1 for 12 us.
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        TIMER1_TAILR_R = 480;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        phase = 1;
    }
    else if(phase == 1)
    {
        //Turning off timer
        TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        TIMER1_TAILR_R = 7040;

        //UART1 AFSEL and PCTL on
        //only transmit is enabled if the device is in the controller mode.
        GPIO_PORTB_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;
        GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX ;

        //writing start code
        if(!poll_req)
        {
          while (UART1_FR_R & UART_FR_TXFF);
          UART1_DR_R = 0;
        }

        if(poll_req)
        {
            while (UART1_FR_R & UART_FR_TXFF);
            UART1_DR_R  = 0xF7;
        }

        //turning on the interrupt for UART1Isr
        UART1_IM_R |= UART_IM_TXIM;

        phase = 2;
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;   //clearing the interrupt flag for timer
}

void uart1Isr()
{
    if(UART1_MIS_R & UART_MIS_TXMIS)
    {
        if((phase-2)<max && !poll_req)
        {
           while (UART1_FR_R & UART_FR_TXFF);
           UART1_DR_R = data_table[phase-2];
           phase++;
        }
        else if((phase-2)<max && poll_req)
        {
            poll_table[last_phase] = 1;
            while (UART1_FR_R & UART_FR_TXFF);
            UART1_DR_R = poll_table[phase-2];
            phase++;

            if((phase-2) == 512)
            {
                //waitMicrosecond(8);
                GPIO_PORTB_AFSEL_R &= ~UART_TX_MASK;
                GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB1_U1TX;
                GPIO_PORTB_AFSEL_R |= UART_RX_MASK;
                GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB0_U1RX ;
                DE = 0;
                waitMicrosecond(24);
                while (UART1_FR_R & UART_FR_TXFF);
                //while (UART0_FR_R & UART_FR_RXFE);
                poll_data = UART1_DR_R;
                if(poll_data & UART_DR_BE)
                {
                    device_address[last_phase] = last_phase;
                    //sprintf(str,"%d\r\n",device_address[last_phase]);
                    //displayUart0(str);
                }
                GPIO_PORTB_AFSEL_R |= UART_TX_MASK;
                GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX;
                GPIO_PORTB_AFSEL_R &= ~UART_RX_MASK;
                GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB0_U1RX ;

                poll_table[last_phase] = 0;
                last_phase +=1;
            }

            if(last_phase == 512)
            {
                last_phase = 0;
                displayUart0("last reached\r\n");
                poll_req = false;
                dmxTX();
            }

        }
        else
        {
            UART1_IM_R &= ~UART_IM_TXIM;
            GPIO_PORTB_AFSEL_R &= ~(UART_TX_MASK);
            GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_U1TX);

            while (UART1_FR_R & UART_FR_BUSY);
            if (on)
            {
                DE = 0;
               dmxTX();

            }
        }
    }
    else if(UART1_MIS_R & UART_MIS_RXMIS)
    {
        //LED_TIMEOUT_OFF = 10;
        data = UART1_DR_R;
        if(data & UART_DR_BE)
        {
            if(poll_req == true) last_rx_phase++;

            if(last_rx_phase == 512)
            {
               poll_req = false;
               last_rx_phase = 0;
            }

            if(!poll_req)
            {
               setRgbColor(data_table[add]);
            }
            rx_phase = 0;
        }

        else
        {
            if(data_table[rx_phase] == 247 && rx_phase == 0)
            {
               poll_req = true;
            }


            if( rx_phase < (max+1) )
            {
               data_table[rx_phase] = data;

                 rx_phase++;

               if(rx_phase == 512)
               {
                   displayUart0("success\r\n");
               }

               if(rx_phase == 494)
               {
                   if( (data_table[add] == 1) && (poll_req == true) )
                    {
                       displayUart0("success\r\n");
                       GPIO_PORTB_AFSEL_R &= ~(UART_TX_MASK | UART_RX_MASK);
                       GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX);
                       UART1_IM_R &= ~UART_IM_RXIM;
                       waitMicrosecond(16);

                       DE = 1;
                       D = 0;
                       waitMicrosecond(8);

                       DE = 0;
                       GPIO_PORTB_AFSEL_R |= UART_RX_MASK;
                       GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB0_U1RX;
                       UART1_IM_R |= UART_IM_RXIM;
                    }
               }

            }
        }
    }
}

void Timer2Isr()
{
  if(mode == 1)
  {
      if(LED_TIMEOUT_OFF > 0)
      {
         LED_TIMEOUT_OFF--;
         BLUE_LED ^= 1;
         if(LED_TIMEOUT_OFF == 0)
         {
             BLUE_LED = 0;
         }
      }
  }
//  if(mode == 0xFFFFFFFF)
//  {
//      if(LED_TIMEOUT_ON > 0)
//      {
//         LED_TIMEOUT_ON--;
//         RED_LED ^= 1;
//         if(LED_TIMEOUT_ON == 0)
//         {
//             RED_LED = 0;
////             LED_TIMEOUT_OFF = 10;
//             //TIMER2_IMR_R &= ~TIMER_IMR_TATOIM;
////             TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
//         }
//      }
//  }
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;   //clearing the interrupt flag for timer
}

void hibernationIsr()
{
    NVIC_EN1_R &= ~(1 << (INT_HIBERNATE -16 - 32));
    data_table[sortedArray[0][1]] = sortedArray[0][2];
    uint32_t j = 0;
    if(sortedArray[0][0] !=0 )
    {
        for(j = 0; j<512; j++)
        {
            sortedArray[j][0] = sortedArray[j+1][0];
            sortedArray[j][1] = sortedArray[j+1][1];
            sortedArray[j][2] = sortedArray[j+1][2];
            if(j == 511)
            {
                sortedArray[j][0] = 0;
                sortedArray[j][1] = 0;
                sortedArray[j][2] = 0;
                break;
            }
        }
    }
    HIB_IC_R |= HIB_IC_RTCALT0;
}

void clear()
{
    uint32_t k=0;
    for(k=0; k<512; k++){
        data_table[k]=0;
    }
    //putsUart0("Data table cleared successfully.\r\n");
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

    mode = readEeprom(0);
    add = readEeprom(1);
    if(mode == 0xFFFFFFFF){
        if(add == 0xFFFFFFFF)
        {
            add = 1;
        }
        //RED_LED = 1;
        initPWM();
        GPIO_PORTB_AFSEL_R |= UART_RX_MASK;
        GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB0_U1RX;
        UART1_IM_R |= UART_IM_RXIM;
        DE = 0;

        displayUart0("Device Mode entered\r\n");
    }

    else if(mode == 1)
    {
        displayUart0("Controller mode was entered\r\n");
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
           //sprintf(str, "%d\n",max);
           //putsUart0(str);
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
           if(mode == 1)
           {
               clear();
                on = false;
                dmxTX();
                on = true;
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
           //data_table[add]=value;
           set_time(getFieldInteger(&data, 3),getFieldInteger(&data, 4),getFieldInteger(&data, 5));
           set_date(getFieldInteger(&data, 6),getFieldInteger(&data, 7));
           n = calculateSeconds(hr, min ,sec, day, month);
           addToSortedList(n, add, value, sortedArray);
           sortArray(sortedArray);
           if(n <= sortedArray[0][0] || sortedArray[0][0] == 0)
           {
              NVIC_EN1_R |= (1 << (INT_HIBERNATE -16 - 32));
              HIB_IM_R |= HIB_IM_RTCALT0;
           }

//           while(!(HIB_CTL_R & 0x80000000));
//           HIB_RTCM0_R = sortedArray[0][0];
//           while(!(HIB_CTL_R & 0x80000000));

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
           setAdd(0xFFFFFFFF, addr);
           valid = true;
           add = readEeprom(1);
           displayUart0("Add: ");
           sprintf(str, "%d\n",add);
           displayUart0(str);
           displayUart0("\r\n");
       }

       if(isCommand(&data, "devinfo", 0))
       {
           valid = true;
           add = readEeprom(1);
           displayUart0("Add: ");
           sprintf(str, "%d\n",add);
           displayUart0(str);
           displayUart0("\r\n");
       }

       displayUart0("\r\n");

       if(!valid)
           displayUart0("Invalid Command\r\n");

     }
}
