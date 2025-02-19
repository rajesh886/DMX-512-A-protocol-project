//Rajesh Rayamajhi

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "hibernation.h"
#include "uart0.h"

void initHibernationHW()
{

    SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0;
    _delay_cycles(3);

    //WRC bit has to be set before performing any writes as suggested in the datasheet

    while(!(HIB_CTL_R & 0x80000000));

    HIB_IM_R &= ~HIB_IM_WC;

    HIB_CTL_R |= 0x00000040;

    while(!(HIB_CTL_R & 0x80000000));

    NVIC_EN1_R |= (1 << (INT_HIBERNATE - 16 - 32));

    while(!(HIB_CTL_R & 0x80000000));

    HIB_IM_R |= HIB_IM_RTCALT0;

    while(!(HIB_CTL_R & 0x80000000));

    HIB_CTL_R |= 0x00000041;

}

uint32_t getCurrentSeconds()
{
    uint32_t seconds = HIB_RTCC_R;                           //read the RTCC register to get the seconds

    return seconds;
}

//this function store the time in their respective variables
void set_time(uint32_t hor, uint32_t minu, uint32_t secs)
{
    hr = hor;
    min = minu;
    sec = secs;
}

//function that displays the current hr, min and sec set by the user.
void get_time(uint32_t *hr, uint32_t *min, uint32_t *sec)
{
    sprintf(timestr, "Hour: %d\r\n", *hr);
    displayUart0(timestr);

    sprintf(timestr, "Min: %d\r\n", *min);
    displayUart0(timestr);

    sprintf(timestr, "Seconds: %d\r\n", *sec);
    displayUart0(timestr);
}

//this function stores the date in their respective variables
void set_date(uint32_t m, uint32_t d)
{
    month = m;
    day = d;
}

void get_date(uint32_t *m, uint32_t *d)
{
    sprintf(timestr, "Month: %d\r\n", *m);
    displayUart0(timestr);

    sprintf(timestr, "Day: %d\r\n", *d);
    displayUart0(timestr);

}

uint32_t calculateSeconds(uint32_t hor, uint32_t minu, uint32_t secs, uint32_t m, uint32_t d)
{
    if(m == 0 && d == 0)
    {
        return (hor * 60 * 60 + minu * 60 + secs);
    }
    else
    {
        return (m * 30 * 86400 + d * 86400 + hor * 60 * 60 + minu * 60 + secs);
    }
}

//Load the value into the RTC register
void setRTC(uint32_t N)
{
    while(!(HIB_CTL_R & 0x80000000));
    HIB_RTCLD_R = N;
}

//Load the value into the match register
void setAlarm(uint32_t RTCALM)
{
    while(!(HIB_CTL_R & 0x80000000));
    HIB_RTCM0_R = RTCALM;
}

//Add the time sorted tasks to the queue.
//Task in the queue won't be added if that time is already passed for now.
void addToSortedList(uint32_t n, uint32_t a, uint32_t v, uint32_t sortedArray[4][3])
{
    uint32_t seconds = HIB_RTCC_R;
    if(n < seconds)
    {
      displayUart0("The time has already passed.\r\n");
    }
    else
    {
        sortedArray[ArrayIndex][0] = n;
        sortedArray[ArrayIndex][1] = a;
        sortedArray[ArrayIndex][2] = v;
        ArrayIndex++;
        if(ArrayIndex == 4)
        {
            ArrayIndex = 0;
        }
    }
}


void sortArray(uint32_t sortedArray[4][3])
{
    uint32_t temp;
    uint32_t temp1;
    uint32_t temp2;
    uint32_t i = 0;
    uint32_t j = 0;

    for (i = 0; i < 4; ++i)
    {
      for (j = i + 1; j < 4; ++j)
      {
          //since the empty arrays has 0, will only sort if the number is non zero in the array.
          if ((sortedArray[i][0] > sortedArray[j][0]) && (sortedArray[i][0] != 0) && (sortedArray[j][0] != 0))
          {
              temp  =  sortedArray[i][0];
              temp1 =  sortedArray[i][1];
              temp2 =  sortedArray[i][2];
              sortedArray[i][0] = sortedArray[j][0];
              sortedArray[i][1] = sortedArray[j][1];
              sortedArray[i][2] = sortedArray[j][2];
              sortedArray[j][0] = temp;
              sortedArray[j][1] = temp1;
              sortedArray[j][2] = temp2;

          }
     }
    }
}


