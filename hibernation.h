//Rajesh Rayamajhi

#ifndef HIBERNATION_H
#define HIBERNATION_H

//global variables
uint32_t hr;

uint32_t min;

uint32_t sec;

uint32_t day;

uint32_t month;

uint32_t ArrayIndex;

char timestr[100];

//functions
void initHibernationHW();

uint32_t getCurrentSeconds();

void set_time(uint32_t hr, uint32_t min, uint32_t sec);

void get_time(uint32_t *hr, uint32_t *min, uint32_t *sec);

void set_date(uint32_t m, uint32_t d);

void get_date(uint32_t *m, uint32_t *d);

uint32_t calculateSeconds(uint32_t hor, uint32_t minu, uint32_t secs, uint32_t m, uint32_t d);

void setRTC(uint32_t N);

void setAlarm(uint32_t RTCALM);

void addToSortedList(uint32_t n, uint32_t a, uint32_t v, uint32_t sortedArray[][]);

void sortArray(uint32_t sortedArray[][]);

#endif
