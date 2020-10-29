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
    HIB_CTL_R |= HIB_CTL_RTCEN;
}

int32_t getCurrentSeconds()
{
    int32_t seconds = (HIB_RTCC_R%86400);
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
    putsUart0(timestr);

    sprintf(timestr, "Min: %d\r\n", *min);
    putsUart0(timestr);

    sprintf(timestr, "Seconds: %d\r\n", *sec);
    putsUart0(timestr);
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
    putsUart0(timestr);

    sprintf(timestr, "Day: %d\r\n", *d);
    putsUart0(timestr);

}
