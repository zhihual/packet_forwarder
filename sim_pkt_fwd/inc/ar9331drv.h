#ifndef _H_AR9331_DRV
#define _H_AR9331_DRV

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;


#define MEM_SIZE 4096
#define TX_TIME_OUT (1000)  //2s
#define RX_TIME_OUT (5000)  //10
#define K (1024)

#define RX_MEM_SIZE (20*K)


#define MEM_CLEAR_CMD 0x1 

#define MEMDEV_IOC_MAGIC  'k'

#define VNIC_IOCGET_TOTAL_MEM_SIZE _IO(MEMDEV_IOC_MAGIC, 1)


#define VNIC_IOCCCA _IO(MEMDEV_IOC_MAGIC, 2)



// Export Interface
int AR9331Drv_Open(int* pfd);
int AR9331Drv_Close(int* fd);

int AR9331Drv_SendPkt(struct lgw_pkt_tx_s pkt_data);
int AR9331Drv_RecvPkt(uint8 max_pkt, struct lgw_pkt_rx_s *pkt_data);

//void AR9331Drv_EnableCCAmode(int fd);

#endif

