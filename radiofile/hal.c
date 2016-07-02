#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/sched.h>  

#include <linux/slab.h>
#include <linux/timer.h> 
#include <linux/time.h>


#include "global_type.h"

#include "common.h"
#include "sx1276.h"
#include "hal.h"
#include "async.h"
#include "common.h"


static txDescriptor *txHead = NULL;
static rxDescriptor gRxDescArray[RX_MEM_BLK_TOTAL_NUMBER] = {0};
static hal_state_type hal_state;

// we build a rx descriptor array firstly, 
static int createRxDescriptorArray()
{
	uint32 i;

	printk("%s, rx_mem_blk_number = %d, sizeof(gRxDescArray)=%d, sizeof(rxDescriptor)=%d\n",\
		__func__,RX_MEM_BLK_TOTAL_NUMBER,sizeof(gRxDescArray),sizeof(rxDescriptor));

	//allocate the mem firstly to the rx_mem
	for(i = 0;i < RX_MEM_BLK_TOTAL_NUMBER; i++)
	{
		gRxDescArray[i].pRxMem = (uint8 *)kmalloc(sx1276_BUFF_LEN,GFP_KERNEL);
		if (!gRxDescArray[i].pRxMem)
		{
			printk("%s-> rxMem alloc failed\n",__func__);
			return 1;
		}
		//to clear the pRxMem, avoid the random value
		memset(gRxDescArray[i].pRxMem, 0, sx1276_BUFF_LEN);
	}

	return 0;
}

static rxDescriptor* getRxDescByIndex(uint32 index)
{
	if(index > RX_MEM_BLK_TOTAL_NUMBER)
	{
		printk("%s: invalid index %d\n",index);
		return NULL;
	}
	return &gRxDescArray[index];
}

void hal_state_reset(void)
{
        hal_state.total_pkt = 0;
        hal_state.last_seq_number = 0;
        hal_state.mem_size = 0;
}

void hal_init(void)
{
        sx1276_init();
        createRxDescriptorArray();
        hal_state_reset();

}

void hal_release(void)
{
        sx1276_release();
}

bool createTxHead(void)
{
	txHead = (txDescriptor*)kmalloc(sizeof(txDescriptor),GFP_KERNEL);
	if(NULL == txHead)
	{
		printk("%s: txHead is NULL\n",__func__);
		return false;
	}
	else
	{
		txHead->index = -1;
		txHead->pNext = NULL;
		return true;
	}
}

bool addNode(txDescriptor* tx_desc)
{
	if(NULL == txHead)
	{
		printk("%s: txHead is NULL\n",__func__);
		return false;
	}
	txDescriptor* p = txHead->pNext;
	txDescriptor* q = txHead;
	while(NULL != p)
	{
		q = p;
		p = p->pNext;
	}
	q->pNext = tx_desc;
	tx_desc->pNext = NULL;
	return true;	
}

void destroyTxNodeList()
{
	if(NULL == txHead)
	{
		return;
	}
	if(NULL == txHead->pNext)
	{
		kfree(txHead);
		txHead = NULL;
		return;
	}
	txDescriptor* p = txHead->pNext;
	while(NULL != p)
	{
		txDescriptor* tmp = p;
		p = p->pNext;
		kfree(tmp);
	}
	kfree(txHead);
	txHead = NULL;
}


void sendTxDescriptor(txDescriptor* tx_desc)
{
	uint8 buff[sx1276_BUFF_LEN];
	int i;

#if 1
	memcpy(buff,&(tx_desc->header),PKT_HEADER_SIZE);
	memcpy(buff+PKT_HEADER_SIZE,tx_desc->pPayload,(tx_desc->pkt_length - PKT_HEADER_SIZE));
#endif

#if 0
	printk("------------------buff_len(%d)---------------------------\n",tx_desc->header.length);
	for(i=0;i<tx_desc->header.length;i++)
	{
		printk("%02d\t",buff[i]);
	}
	printk("\n");
	printk("---------------------------------------------\n");
#endif

	sx1276_Send_Packet(buff,tx_desc->pkt_length);

	return;
}


void sendTxDesciprotList()
{
	txDescriptor* p = txHead->pNext;
	txDescriptor* q = txHead;
	while(NULL != p)
	{
		q = p;
		p = p->pNext;
		sendTxDescriptor(q);
		printk(".");
	}
	printk("\n");
	
	return;
}

uint8 isSiLoopbackTest()
{
	return 0;
}


int createTxDescriptorList(uint8 *p, uint32 mem_size)
{
	uint32 integer = 0;
	uint8 mod = 0;
	uint32 index = 0;
	int result = 0;
	txDescriptor *txDesc = NULL;
	uint32 total_cnt = 0;
	
	integer = mem_size/PKT_PAYLOAD_MAX_SIZE;
	mod = mem_size%PKT_PAYLOAD_MAX_SIZE;
	printk("--> integer = %d, mod = %d\n",integer,mod);
	total_cnt = integer + (mod?1:0);
	for(index = 0;index < total_cnt;index++)
	{
		txDesc = (txDescriptor*)kmalloc(sizeof(txDescriptor),GFP_KERNEL);
		if (!txDesc)
		{
			printk("%s-> txDesc alloc failed\n",__func__);
		}
		
		txDesc->index = index;
		if((0 != mod)&&((total_cnt-1) == index))	
			txDesc->pkt_length = mod + PKT_HEADER_SIZE;
		else
			txDesc->pkt_length = PKT_PAYLOAD_MAX_SIZE + PKT_HEADER_SIZE;

		if(total_cnt == 1)
		{
			if(isSiLoopbackTest())
				txDesc->header.type = LP_SINGLE;
			else
				txDesc->header.type = A1_SINGLE;
		}else if((total_cnt-1) == index)
		{
			if(isSiLoopbackTest())
				txDesc->header.type = LP_AGGREGATION_END;   //
			else
				txDesc->header.type = A1_AGGREGATION_END;   //
		}else
		{
			if(isSiLoopbackTest())
				txDesc->header.type = LP_AGGREGATION;
			else
				txDesc->header.type = A1_AGGREGATION;
		}
		
		//txDesc->header.length = txDesc->pkt_length; // length = length + type + seq_number + payload	

		txDesc->header.seq_number = index;
		
		txDesc->pPayload= p+(index*PKT_PAYLOAD_MAX_SIZE);
		txDesc->pNext = NULL;
		addNode(txDesc);
	}
	
	return result;
}





int hal_tx(uint8 *tx_mem, int size)
{
        int result = 0;
    
        if(size <= 0){
                printk("mem_size is invalid size = %d \n",size);
                return -EINVAL;
        }

        if(size >= MAX_RX_MEM_SIZE)
        {
                printk("mem_size(%d) overload the MAX_RX_MEM_SIZE(%d)\n",size,MAX_RX_MEM_SIZE);
                return -EINVAL;
        }
        
        //allocate tx_descriptor linked list
        createTxHead();
        createTxDescriptorList(tx_mem,size);
        //printTxDescriptorList();
        sendTxDesciprotList();
        //destroy txNodeList
        destroyTxNodeList();
    
        //after sent done, go back to RX status
        hal_start_rx(); 
        return result;
}

static rxDescriptor *copyRxBuff2RxMem(uint8 rx_length)
{
        uint8 rxBuff[sx1276_BUFF_LEN];
        int i;
        rxDescriptor *rxDesc;
        uint8 seq_number;
        
        sx1276_getRecvBuff(rxBuff,rx_length);
        
        seq_number = rxBuff[RX_HEADER_SEQ_NUMBER_INDEX];
        rxDesc = getRxDescByIndex(seq_number);
        if(NULL == rxDesc)
        {
            printk("%s, NULL == rxDesc\n",__func__);
            goto end_loop;
        }
        memcpy(rxDesc->pRxMem,rxBuff,rx_length);
        rxDesc->length = rx_length;
        hal_state.last_seq_number = seq_number;

end_loop:
        return rxDesc;
        
}

static bool isSingleOrLastPkt(rxDescriptor *rxDesc)
{
	if( A1_SINGLE == rxDesc->pRxMem[RX_HEADER_TYPE_INDEX] || \
		A1_AGGREGATION_END == rxDesc->pRxMem[RX_HEADER_TYPE_INDEX] || \
		LP_SINGLE == rxDesc->pRxMem[RX_HEADER_TYPE_INDEX] || \
		LP_AGGREGATION_END == rxDesc->pRxMem[RX_HEADER_TYPE_INDEX] ) 
	{
		return true;
	}else{
		return false;
	}
}

static void lastPktHandle(rxDescriptor *rxDesc)
{

	hal_state.total_pkt = rxDesc->pRxMem[RX_HEADER_SEQ_NUMBER_INDEX] + 1;
	hal_state.mem_size = (hal_state.total_pkt - 1)*PKT_PAYLOAD_MAX_SIZE + (rxDesc->length - PKT_HEADER_SIZE);
	signalRX();

	
	return;
}

uint32 getTotalMemSize()
{
	return hal_state.mem_size;
}

uint8 getTotalPktNumber()
{
        return hal_state.total_pkt;
}


rxDescriptor *getRxDesc(uint8 index)
{
        return &gRxDescArray[index];
}


void hal_rx(unsigned long dev_id)
{
        uint8 rx_length = 0;
        rxDescriptor *rxDesc;
        int rssi = 0;

        
        /*****
        get the rx_length
        *****/
        rx_length = sx1276_getRXLength();

        /*****
        copy the rx_buff to the involved desc
        *****/
        rxDesc = copyRxBuff2RxMem(rx_length);
        if(NULL == rxDesc)
        {
                goto fail_receive;
        }
        printk("|%d-%d-%d|",rxDesc->pRxMem[RX_HEADER_TYPE_INDEX], \
                                            rxDesc->pRxMem[RX_HEADER_SEQ_NUMBER_INDEX],\
                                            rxDesc->length);

        /*****
            last pkt handle
        *****/
        if(isSingleOrLastPkt(rxDesc))
	{
		//printRxDescriptorList();
		lastPktHandle(rxDesc);
                /*****
                    print pkt RSSI
                ******/
                //s46_Get_RSSI(&(tr_status.rssi1),&(tr_status.rssi2),1);
                
                rssi = SX1276LoRaReadRssi();
                printk("-> (%d dBm)\n", rssi);			//print rssi
	}
        

fail_receive:
        //go back to receive mode
        hal_start_rx();
}

void hal_start_rx(void)
{
        rx_init();
}

static ktime_t ktime;
static struct hrtimer hr_timer;
#define us 1000
#define ms 1000000
/*
        SF = 8, BW = 500KHz,
        symbol_time = (2^SF+32)/BW = 576us
        time the sample time to 500us
*/
static unsigned int interval=(500*us); //1000000; /* unit: ns */
wait_queue_head_t timeout_wait_queue;
uint8 timeout = 0;

#define TX_TIME_OUT 100 // unit is 10ms, timeout is 100ms for 30Kbps, 500ms for 10Kbps,
int rssi;


enum hrtimer_restart my_hrtimer_callback( struct hrtimer *timer )
{
	int i;
	static uint8 clk_count = 0;
	//printk( "my_hrtimer_callback called (%ld) -> %d.\n", jiffies ,clk_count++);
	timeout = 1;
        //wake_up(&timeout_wait_queue);
        rssi = SX1276LoRaReadRssi();
        printk("rssi = %d dBm\n",rssi);

	return HRTIMER_NORESTART;  //HRTIMER_RESTART
}

//clear channel assessment
void hal_cca(void)
{
        int count = 0;
        uint8 t;
        int ret;

        
        ktime = ktime_set( 0, interval);
        hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	hr_timer.function = &my_hrtimer_callback;
	init_waitqueue_head(&timeout_wait_queue);
        timeout = 0;
        
        printk("<--%s-->\n",__func__);
        sx1276_cad_init();
        
        hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );

        recordStartTime();
        while(1)
        {
                msleep(1);
                if( 0 == timeout ) continue;
                t = sx1276IntrFlag();
                //printk("t-> 0x%02x\n",t);
                if(t & RFLR_IRQFLAGS_CADDONE)
                        break;
        }
        recordEndTime(1);

        printk("t = 0x%02x, current mode = 0x%02x\n",t,sx1276LoRaGetOpMode());
        
        SPIWriteReg(LR_RegIrqFlags,RFLR_IRQFLAGS_CADDONE);  //clear irq
        
}

void hal_conTx(void)
{}


