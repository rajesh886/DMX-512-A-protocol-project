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

    HIB_IM_R |= HIB_IM_WC;
    HIB_CTL_R |= HIB_CTL_CLK32EN;
    while(!(HIB_MIS_R & HIB_IM_WC));
    //HIB_IM_R |= HIB_IM_RTCALT0  ;
    HIB_CTL_R |= HIB_CTL_RTCEN ;
    //NVIC_EN1_R |= (1 << (INT_HIBERNATE -16 - 32));
}

uint32_t getCurrentSeconds()
{
    uint32_t seconds = (HIB_RTCC_R);
    return seconds;
}

void set_time(uint32_t hor, uint32_t minu, uint32_t secs)
{
    hr = hor;
    min = minu;
    sec = secs;
    //TO DO: Change this function later in the project
}

void get_time(uint32_t *hr, uint32_t *min, uint32_t *sec)
{
    sprintf(timestr, "Hour: %d\r\n", *hr);
    displayUart0(timestr);

    sprintf(timestr, "Min: %d\r\n", *min);
    displayUart0(timestr);

    sprintf(timestr, "Seconds: %d\r\n", *sec);
    displayUart0(timestr);
}

void set_date(uint32_t m, uint32_t d)
{
    month = m;
    day = d;
    //TO DO: Change this function later in the project
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
        return secs + (60 * ( minu * 60 * (hor * 24 * (d + (30 * m)))));
    }
}


void setRTC(uint32_t N)
{
    while(!(HIB_CTL_R & 0x80000000));
    HIB_RTCLD_R = N;
    while(!(HIB_CTL_R & 0x80000000));
}


void addToSortedList(uint32_t n, uint32_t a, uint32_t v, uint32_t sortedArray[512][3])
{
   ArrayIndex = 0;
   sortedArray[ArrayIndex][0] = n;
   sortedArray[ArrayIndex][1] = a;
   sortedArray[ArrayIndex][2] = v;
   ArrayIndex++;
}

void sortArray(uint32_t sortedArray[512][3])
{
    uint32_t temp;
    uint32_t temp1;
    uint32_t temp2;
    uint32_t i = 0;
    uint32_t j = 0;

    for (i = 0; i < 512; ++i)
    {
      for (j = i + 1; j < 512; ++j)
      {
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


