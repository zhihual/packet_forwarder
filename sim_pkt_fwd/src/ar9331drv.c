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


#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf */
#include <string.h>		/* memcpy */

#include "parson.h"
#include "base64.h"
#include "loragw_hal.h"
#include "loragw_aux.h"

#include "ar9331drv.h"

#define F_SETOWN        8
#define FASYNC          00020000


struct FILE * g_fd = 0;

static pthread_mutex_t mux_readpkt = PTHREAD_MUTEX_INITIALIZER; /* control readbuf */

#define ARRAY_SIZE 128

struct PktArrayItem
{
   int    waitRead;
   struct lgw_pkt_rx_s pkt;
};

struct PktArray {
	struct PktArrayItem Item[ARRAY_SIZE];
    int lastReadIdx;
    int lastWriteIdx;
    int readyNum;
    int InNum;  // for debug 
    int OutNum; // for debug
};

struct PktArray g_RecPool;

#if 0
static void calcTestDataPER(uint8* buff, uint32 length)
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
    
    printf("PER: Total(%d), Error(%d), PER(%.2f%%)\n",length,error_pkt,per);

    return;
}
#endif

static uint32 getTotalMemSize(struct FILE* fd)
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


static int GetReadyBufNum()
{
   return g_RecPool.readyNum;
}

static int PutPktIntoArray(uint8* pRxMem, int rx_length, int read_len)
{
   struct PktArrayItem* p = NULL;
   int writeIndex = 0;
   int ret = 0;
   
   if(g_RecPool.readyNum >= ARRAY_SIZE)
   {
     printf("recev pool is fully, skip in pool\n");
   }

   if(g_RecPool.lastWriteIdx >= (ARRAY_SIZE-1) )
     writeIndex = 0;
   else
     writeIndex = g_RecPool.lastWriteIdx+1;

   p = &g_RecPool.Item[writeIndex];

   if(p->waitRead)
   {
     printf("why data not been read out. overflow\n");
     return -1;
   }

   rx_length = rx_length;

   printf("write %d \n", writeIndex);
   memcpy(p->pkt.payload, pRxMem, read_len);
   p->pkt.size = read_len;
   p->waitRead = 1; // set flag
   
   g_RecPool.lastWriteIdx = writeIndex;
   g_RecPool.InNum++;
   g_RecPool.readyNum++;

   // fill fake packet structure
   p->pkt.freq_hz=470;	
   p->pkt.if_chain=10;
   p->pkt.status=STAT_CRC_OK;
   p->pkt.count_us= 1;
   p->pkt.rf_chain=4;
   p->pkt.modulation=MOD_LORA;
   p->pkt.bandwidth=BW_500KHZ;
   p->pkt.datarate=DR_LORA_SF7;
   p->pkt.coderate=CR_LORA_4_5;
   p->pkt.rssi=8;
   p->pkt.snr=5;
   p->pkt.snr_min=1;
   p->pkt.snr_max=10;
   p->pkt.crc=0;

   return ret;
}

static int PopPktFromArray(int num, struct lgw_pkt_rx_s *pkt_data)
{
   int i = 0;
   struct lgw_pkt_rx_s* dst = NULL;
   struct PktArrayItem* src = NULL;
   int ret = 0;
   
   int readInx = 0;

   dst = pkt_data;

   for(i=0; i<num; i++)
   {
      readInx = g_RecPool.lastReadIdx+1;
      if(readInx >= (ARRAY_SIZE-1))
        readInx = 0;

      printf("readinx :%d\n", readInx);
      src = &g_RecPool.Item[readInx];

      if(!src->waitRead)
      {
        printf("why packet not ready to read!\n");
        ret = -1;
        return ret;
      }

      memcpy((void*)dst,(void*)&src->pkt, sizeof(struct lgw_pkt_rx_s));

      dst++;
      src->waitRead = 0;

      g_RecPool.OutNum++;
      g_RecPool.readyNum--;
      g_RecPool.lastReadIdx = readInx;

   }

   return ret;

}

static void rx_input_handler(int num)
{
   uint32 rx_length = 0;
   uint8 *pRxMem = NULL;
   int read_len = 0;
   int i;
        
   rx_length = getTotalMemSize(g_fd);
   //printf("%s, rx_length = %d\n",__func__,rx_length);

   pRxMem = malloc(rx_length);
   memset(pRxMem,0,rx_length);
   //printf("APP got mem, length = %d\n",total_mem_size);

   read_len = read(g_fd, pRxMem, rx_length);
   if (rx_length ||read_len)
   printf("%s, rx_length = %d, read_len = %d\n",__func__,rx_length,read_len);

   // can dump packet here

   usleep(400000);  //sleep xx ms to make sender switch to RX ready.  5ms for 30Kbps,40ms for 10Kbps, 500ms for 1Kbps,
                   //if we used the high rate send and low rate receive, then we could comment the delay...
   
   
   // call into output here
   //pktHandler(globalSiFd,pRxMem,read_len);
   if(read_len)
   {
     pthread_mutex_lock(&mux_readpkt);
     PutPktIntoArray(pRxMem, rx_length,read_len);                  
     pthread_mutex_unlock(&mux_readpkt);
   }
   free(pRxMem);
        
}

static void rxSignalInit(int fd)
{
#if 1
   int oflags;

   signal(SIGIO, rx_input_handler);    
   fcntl(fd, F_SETOWN, getpid());    
   oflags = fcntl(fd, F_GETFL);
   fcntl(fd, F_SETFL, oflags | FASYNC);  
#endif
}

static int SendPkt(int fd,int length, uint8* buff)
{
  uint8 *p = NULL;
  int ret = 0;
  uint16 cnt = 0;

  p = buff;
        
  ret = write(fd, buff, length);
  if(ret < 0)
  {
    printf("%s, error_code = 0x%d\n",__func__,ret);
  }

  return ret;
}

int AR9331Drv_SendPkt(struct lgw_pkt_tx_s pkt_data)
{

   printf("AR9331Drv_SendPkt  Send=%p (size=%d)\n)", pkt_data.payload,pkt_data.size);
   SendPkt(g_fd, pkt_data.payload, pkt_data.size);
}

int AR9331Drv_RecvPkt(uint8 max_pkt, struct lgw_pkt_rx_s *pkt_data)
{
   uint8*p = NULL;
   int readNum = 0;
   pthread_mutex_lock(&mux_readpkt);
   readNum = GetReadyBufNum();
   
   if(readNum > 0)
   {
      printf("readNum = %d\n", readNum);
      if (readNum >max_pkt)
        readNum = max_pkt;
      PopPktFromArray(readNum,pkt_data);
   }
    
   pthread_mutex_unlock(&mux_readpkt);

   rx_input_handler(0);
   return readNum;
}

#if 0
void AR9331Drv_EnableCCAmode(int fd)
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
#endif


int AR9331Drv_Open()
{
  struct FILE * fd = NULL;		
        
  if((fd=open("/dev/spidev",O_RDWR)) == -1)
  {
	printf("open SPI WRONG!\n");
    return -1;
  }
  else
  {
	printf("open SPI SUCCESS!\n");
  }
  
  rxSignalInit(fd);
 
  g_fd = fd;

  g_RecPool.InNum = 0;
  g_RecPool.lastReadIdx =-1;
  g_RecPool.lastWriteIdx = -1;
  g_RecPool.OutNum = 0;
  g_RecPool.readyNum = 0;

  return 0;
}

int AR9331Drv_Close()
{
  int ret = 0;
  if(g_fd)
  {
    fclose(g_fd);
    g_fd = 0;
  }else{
    ret = -1;
  } 

  
  
  return ret;
}




