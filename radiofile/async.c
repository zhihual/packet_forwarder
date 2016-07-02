#include <linux/fs.h>
#include <linux/sched.h>


#include "async.h"


//首先是定义一个结构体，其实这个结构体存放的是一个列表，这个
//列表保存的是一系列设备文件，SIGIO信号就发送到这些设备上
static struct fasync_struct *fasync_queue;

//fasync方法的实现
int hal_fasync(int fd, struct file * filp, int on)
{
    int retval;  
    //将该设备登记到fasync_queue队列中去
    retval=fasync_helper(fd,filp,on,&fasync_queue);  
    if(retval<0)
    {
        return retval;
    }
    return 0;
}

void signalRX(void)
{
	kill_fasync(&fasync_queue, SIGIO, POLL_IN);
}




