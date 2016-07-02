#include <linux/fs.h>
#include <linux/sched.h>


#include "async.h"


//�����Ƕ���һ���ṹ�壬��ʵ����ṹ���ŵ���һ���б����
//�б������һϵ���豸�ļ���SIGIO�źžͷ��͵���Щ�豸��
static struct fasync_struct *fasync_queue;

//fasync������ʵ��
int hal_fasync(int fd, struct file * filp, int on)
{
    int retval;  
    //�����豸�Ǽǵ�fasync_queue������ȥ
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




