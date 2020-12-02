/*
 * polling.c
 *
 *  Created on: Nov 30, 2020
 *      Author: raj
 */

#include "polling.h"

void sendAcks(int data_table[])
{
    int max_address = 512;
    int i = 0;
    while(max_address >=0)
    {
        data_table[i] = 1;
        max_address--;
        i++;
    }
}
