
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;


#define MEM_SIZE 4096
#define TX_TIME_OUT (1000)  //2s
#define RX_TIME_OUT (5000)  //10
#define K (1024)

#define RX_MEM_SIZE (20*K)

typedef struct __global_control
{
	uint8 sendtest;
	uint8 rxmode;
        float per;
        uint32 rx_length;
        uint8 getPer;
}t_global_control;

typedef struct __PKT_HEADER
{
        uint8 type;
}t_pktHeader;

typedef struct __PKT_LINK_RESULT
{
        float per;
        uint32 rx_length;
}t_pkt_link_result;



#define LINK_PKT_HEADER_SIZE sizeof(t_pktHeader)

typedef enum 
{
	LINK_INVALID_DOWN = 0x19,	
	
	LINK_TEST_PKT,
    
	LINK_TEST_RESULT_PKT,

	LINK_ACK,

    LINK_SEND_TEST_RESULT,
        
    LINK_SEND_TEST_REQUEST,

	LINK_INVALID_TOP,
}PKT_TYPE;



#define MEM_CLEAR_CMD 0x1 /*清零全局内存*/

/* 定义幻数 */
#define MEMDEV_IOC_MAGIC  'k'

/* 定义命令 */
#define VNIC_IOCGET_TOTAL_MEM_SIZE _IO(MEMDEV_IOC_MAGIC, 1)


#define VNIC_IOCCCA _IO(MEMDEV_IOC_MAGIC, 2)





