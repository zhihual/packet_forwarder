/*
        define the common func and ...
*/


#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include "global_type.h"


static struct timeval st_tv;
static struct timeval end_tv;

void recordStartTime(void)
{
	do_gettimeofday(&st_tv);
	return;
}

void recordEndTime(uint8 debug)
{
	do_gettimeofday(&end_tv);
	
	if(1 == debug)
	{
		if(end_tv.tv_usec > st_tv.tv_usec)
		{	
			printk("time taken =	%lds %ldus\n",(end_tv.tv_sec - st_tv.tv_sec),(end_tv.tv_usec - st_tv.tv_usec));
		}else{
			printk("time taken =	%lds %ldus\n",(end_tv.tv_sec - st_tv.tv_sec)-1,1000000+(end_tv.tv_usec - st_tv.tv_usec));
		}
	}
	return;
}

