#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include <linux/ioctl.h>
#include "main.h"

t_global_control globalCntl;
int globalSiFd;


static globalCntl_init(void)
{
        globalCntl.sendtest = 0;
        globalCntl.rxmode = 0;
        globalCntl.per = 0.0;
        globalCntl.rx_length = 0;
        globalCntl.getPer = 0;
}

uint32 getTotalMemSize(int fd)
{
	int cmd;	
	uint32 size;
	
	cmd = VNIC_IOCGET_TOTAL_MEM_SIZE;
	if (ioctl(fd, cmd,&size) < 0)
	{
		printf("ioctl cmd 0x%x fail\n",cmd);
		return -1;
	}
	return size;
}

void enterCCAmode(int fd)
{
        int cmd;
        cmd = VNIC_IOCCCA;
        
	if (ioctl(fd, cmd,NULL) < 0)
	{
		printf("ioctl cmd 0x%x fail\n",cmd);
		return -1;
	}
        return;
}

bool isLinkTestPkt(uint8 type)
{
    if(LINK_TEST_PKT == type)
       return true;
    else
       return false;
}

bool isLinkTestResultPkt(uint8 type)
{
    if(LINK_TEST_RESULT_PKT == type)
      return true;
    else
      return false;
}


void calcTestDataPER(uint8* buff, uint32 length)
{
        int i;
        int good_pkt = 0;
        int error_pkt = 0;
        uint8 checkVal = 0;
        float per = 0.0;
        printf("%s, length = %d\n",__func__,length);

        for(i=0;i<length;i++,checkVal++)
        {
                if(checkVal == buff[i])
                {
                    good_pkt++;
                }else{
                    error_pkt++;
                }                
        }
        
        per = ((float)(error_pkt)/(float)(length))*100;
        globalCntl.per = per;
        globalCntl.rx_length = length;
        
        printf("PER: Total(%d), Error(%d), PER(%.2f%%)\n",length,error_pkt,per);

        return;
}

static void sendSendTestResultPkt(int fd)
{
        t_pktHeader pktHeader;
        t_pkt_link_result linkResult;
        uint8 tx_len = sizeof(t_pktHeader) + sizeof(t_pkt_link_result);
        uint8 *p = malloc(tx_len);
        int ret = 0;
        
        pktHeader.type = LINK_TEST_RESULT_PKT;
        linkResult.per = globalCntl.per;
        linkResult.rx_length = globalCntl.rx_length;
        memcpy(p,&pktHeader,sizeof(t_pktHeader));
        memcpy(p+sizeof(t_pktHeader),&linkResult,sizeof(t_pkt_link_result));
        
        ret = write(fd, p, tx_len);
        if(ret < 0)
        {
           printf("%s, error_code = 0x%d\n",__func__,ret);
        }
        
        //clear the results
        globalCntl.per = 0.0;
        globalCntl.rx_length = 0;
        free(p);
	return;
}



static void linkTesHandler(int fd,uint8 *payload, uint32 length)
{
        calcTestDataPER(payload,length);
        sendSendTestResultPkt(fd);
}

static void linkTestResultHandler(int fd,uint8 *payload, uint32 length)
{
        t_pkt_link_result *pLinkResult = payload;
        globalCntl.per = pLinkResult->per;
        globalCntl.rx_length = pLinkResult->rx_length;
        globalCntl.getPer = 1;
        //printf("PER: RX(%d), PER(%.2f%%)\n",pLinkResult->rx_length,pLinkResult->per);
}

static void pktHandler(int fd,uint8 *buff, uint32 length)
{
        t_pktHeader *pHd = buff;
        if(isLinkTestPkt(pHd->type))
        {
                linkTesHandler(fd,buff+LINK_PKT_HEADER_SIZE,length - LINK_PKT_HEADER_SIZE);
        }else if(isLinkTestResultPkt(pHd->type))
        {
                linkTestResultHandler(fd,buff+LINK_PKT_HEADER_SIZE,length - LINK_PKT_HEADER_SIZE);
        }else
        {
                printf("unkown type = 0x%02x\n",pHd->type);
        }
}

//处理函数，没什么好讲的，用户自己定义
static void rx_input_handler(int num)
{
        uint32 rx_length = 0;
        uint8 *pRxMem = NULL;
        int read_len = 0;
        int i;
        
        rx_length = getTotalMemSize(globalSiFd);
        //printf("%s, rx_length = %d\n",__func__,rx_length);

        pRxMem = malloc(rx_length);
        memset(pRxMem,0,rx_length);
	//printf("APP got mem, length = %d\n",total_mem_size);

	    read_len = read(globalSiFd, pRxMem, rx_length);
        printf("%s, rx_length = %d, read_len = %d\n",__func__,rx_length,read_len);
#if 0
        printf("RX(%d): ",read_len);
        for(i=0;i<read_len;i++)
        {
                printf("0x%02x\t",pRxMem[i]);
        }
        printf("\n");
#endif

        usleep(40000);  //sleep xx ms to make sender switch to RX ready.  5ms for 30Kbps,40ms for 10Kbps, 500ms for 1Kbps,
                        //if we used the high rate send and low rate receive, then we could comment the delay...


        pktHandler(globalSiFd,pRxMem,read_len);

        free(pRxMem);
        
}

static void rxSignalInit(int fd)
{
        int oflags;

        //启动信号驱动机制,将SIGIO信号同input_handler函数关联起来,一旦产生SIGIO信号,就会执行input_handler
        signal(SIGIO, rx_input_handler);    

        //STDIN_FILENO是打开的设备文件描述符,F_SETOWN用来决定操作是干什么的,getpid()是个系统调用，
        //功能是返回当前进程的进程号,整个函数的功能是STDIN_FILENO设置这个设备文件的拥有者为当前进程。
        fcntl(fd, F_SETOWN, getpid());    

        //得到打开文件描述符的状态
        oflags = fcntl(fd, F_GETFL);

        //设置文件描述符的状态为oflags | FASYNC属性,一旦文件描述符被设置成具有FASYNC属性的状态，
        //也就是将设备文件切换到异步操作模式。这时系统就会自动调用驱动程序的fasync方法。
        fcntl(fd, F_SETFL, oflags | FASYNC);  
}


#define WAIT_SENDTEST_PER_TIMEOUT 1000


static void sendTest(int fd,int length)
{
        uint32 i = 0;
        uint8 *buff;
        uint8 *p;
        int ret = 0;
        t_pktHeader pktHeader;
        
        uint16 cnt = 0;

        buff = malloc(length+LINK_PKT_HEADER_SIZE);
        memset(buff,0,length+LINK_PKT_HEADER_SIZE);
        
        pktHeader.type = LINK_TEST_PKT;
        memcpy(buff,&pktHeader,LINK_PKT_HEADER_SIZE);
        p = buff + LINK_PKT_HEADER_SIZE;
        
        for(i=0;i<length;i++)
        {
                p[i] = i;
        }
        
        ret = write(fd, buff, length+LINK_PKT_HEADER_SIZE);
        if(ret < 0)
        {
                printf("%s, error_code = 0x%d\n",__func__,ret);
        }

        while(cnt++ < WAIT_SENDTEST_PER_TIMEOUT)
        {
		if(1 == globalCntl.getPer)
		{               
                        printf("PER Result: TX(%d) RX(%d), PER(%.2f%%)\n",length,globalCntl.rx_length,globalCntl.per);
                        break;
		}
                usleep(1000);
	}
        //printf("cnt = %d\n",cnt);
    if(cnt >= WAIT_SENDTEST_PER_TIMEOUT)
	{
		printf("---> wait for test result timeout!!!\n");
	}
        //clear the results
    globalCntl.per = 0.0;
    globalCntl.rx_length = 0;
    globalCntl.getPer = 0;
}

static int parseCmd(int fd, int argc, char**argv)
{
	int i;
	int length;

	if(argc > 1)
	{
		for(i = 0;i < argc; i++)
		{	
			if(0 == strcmp("-s",argv[i])){
				length = atoi(argv[i+1]);
				if(length <= 0) 
				{
					printf("length is not valid, length = %s, EXIT(0)\n",(argv[i+1]));
					exit(0);
				}
				printf("start sendtest test, memsize = %d\n",length);
				sendTest(fd,length);
			}	
            if(0 == strcmp("-r",argv[i])){
               globalCntl.rxmode = 1;
			}
            if(0 == strcmp("-c",argv[i])){
               enterCCAmode(fd);
            }

		}
	}
	return 0;
}




int main(int argc, char**argv)
{
	int fd;		
        globalCntl_init();
        
	if((fd=open("/dev/loradio",O_RDWR)) == -1) //打开memdev设备
	{
		printf("open SPI WRONG!\n");
                return 1;
	}
	else
	{
		printf("open SPI SUCCESS!\n");
	}
        globalSiFd = fd;
    
        rxSignalInit(fd);

        parseCmd(fd,argc,argv);

        //最后进入一个死循环，程序什么都不干了，只有信号能激发input_handler的运行
        //如果程序中没有这个死循环，会立即执行完毕
        if(1 == globalCntl.rxmode)
        {
                printf("Entering Rx State... ...\n");
                while (1)
                {
                        sleep(1);
                }
        }
        
	close(fd);
        return 0;
}






