#ifndef HIBERNATION_H
#define HIBERNATION_H

//global variables
uint32_t hr;
uint32_t min;
uint32_t sec;
uint32_t day;
uint32_t month;
char timestr[100];

void initHibernationHW();
int32_t getCurrentSeconds();
void set_time(uint32_t hr, uint32_t min, uint32_t sec);
void get_time(uint32_t *hr, uint32_t *min, uint32_t *sec);
void set_date(uint32_t m, uint32_t d);
void get_date(uint32_t *m, uint32_t *d);

#endif
