#ifndef _HAL_H_

#define _HAL_H_

#include "sx1276.h"



#define FLAG_WRITE_OK 0
#define FLAG_READ_OK  1

typedef struct rx_node
{
	int index;
	int flag;  
	uint8 *pPayload;
	uint8 length;
	uint8 *pRxMem;
	struct rx_node* pNext;
}rxDescriptor;

typedef struct __hal_state
{
	uint8 total_pkt; 
    uint8 last_seq_number;
    uint32 mem_size;
}hal_state_type;





//#define PKT_HEADER_SIZE sizeof(pktHeader)   
//#define PKT_PAYLOAD_MAX_SIZE (sx1276_BUFF_LEN - PKT_HEADER_SIZE)

//#define K (1024)
#define MAX_RX_MEM_SIZE  (63750)
//#define RX_MEM_BLK_TOTAL_NUMBER  (MAX_RX_MEM_SIZE/sx1276_BUFF_LEN)

#define RX_MEM_BLK_TOTAL_NUMBER  250


void hal_init(void);
void hal_release(void);
int hal_tx(uint8 *tx_mem, int size);
void hal_start_rx(void);
void hal_rx(unsigned long dev_id);
uint32 getTotalMemSize(void);
uint8 getTotalPktNumber(void);
rxDescriptor *getRxDesc(uint8 index);
void hal_state_reset(void);
void hal_cca(void);

#endif


