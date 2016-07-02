#include "sx1276.h"

typedef struct PKT_HEADER_SIZE
{
	//uint8 length;      //length + type + seq_number + payload
	uint8 type;
	uint8 seq_number;  //16K Bytes Maxism
}pktHeader;

typedef struct tx_node
{
	int index;
	uint8 pkt_length;
	pktHeader header;
	uint8 *pPayload;
	struct tx_node* pNext;
}txDescriptor;

typedef struct rx_node
{
	int index;
	pktHeader header;
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



typedef enum 
{
	A1_AGGREGATION = 1,   //audio talking
	
	A1_AGGREGATION_END = 2,
	
	A1_SINGLE = 3,	

	LP_AGGREGATION,   //loopback test
	
	LP_AGGREGATION_END,
	
	LP_SINGLE,	

	A1_INVALID,
}PKT_TYPE;


#define PKT_HEADER_SIZE sizeof(pktHeader)   
#define PKT_PAYLOAD_MAX_SIZE (sx1276_BUFF_LEN - PKT_HEADER_SIZE)

#define K (1024)
#define MAX_RX_MEM_SIZE  (15*K)
#define RX_MEM_BLK_TOTAL_NUMBER  (MAX_RX_MEM_SIZE/sx1276_BUFF_LEN)

void hal_init(void);
void hal_release(void);
int hal_tx(uint8 *tx_mem, int size);
void hal_start_rx(void);
void hal_rx(unsigned long dev_id);
uint32 getTotalMemSize();
uint8 getTotalPktNumber();
rxDescriptor *getRxDesc(uint8 index);
void hal_state_reset(void);
void hal_cca(void);




