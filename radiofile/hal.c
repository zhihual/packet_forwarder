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



static rxDescriptor gRxDescArray[RX_MEM_BLK_TOTAL_NUMBER] = {0};
static hal_state_type hal_state;

int lastWIdx = -1;
int lastRIdx = -1;
int RecvPktCnt = 0;

// we build a rx descriptor array firstly, 
static int createRxDescriptorArray(void)
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
    lastWIdx = -1;
    lastRIdx = -1;
	return 0;
}

void desctoryRxDescriptorArray(void)
{
    uint32 i;

	
	//allocate the mem firstly to the rx_mem
	for(i = 0;i < RX_MEM_BLK_TOTAL_NUMBER; i++)
	{
		
		if (gRxDescArray[i].pRxMem)
		{
           printk("i=%d mem=%x\n", i, gRxDescArray[i].pRxMem);
		   kfree(gRxDescArray[i].pRxMem);
		}
	
	}
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
    int seq;
    rxDescriptor * pRxDesc; 

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
   desctoryRxDescriptorArray();
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

  sx1276_tx(tx_mem,size);      


  //after sent done, go back to RX status
  printk("Done, turn to receive\n");
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

        seq_number = lastWIdx+1;

    
        if(seq_number>=RX_MEM_BLK_TOTAL_NUMBER)
        {
           seq_number = 0;
           printk("[DrvRx] writindx roveover\n");
        }

        
    
        rxDesc = getRxDescByIndex(seq_number);
        if((NULL == rxDesc) || (rxDesc->flag == FLAG_READ_OK))
        {
            printk("[DrvRx] rxDesc Fail indx=%d\n",seq_number);
            goto end_loop;
        }
        else
        {
            printk("[DrvRx] Rec Data Idx=%d\n", seq_number);
            rxDesc->flag = FLAG_READ_OK;

            memcpy(rxDesc->pRxMem,rxBuff,rx_length);
            rxDesc->length = rx_length;
            lastWIdx = seq_number;
            RecvPktCnt++;
            hal_state.last_seq_number = seq_number;
            
        }
        
end_loop:
        return rxDesc;
        
}


static void lastPktHandle(rxDescriptor *rxDesc)
{
    signalRX();
	
	return;
}

uint32 getTotalMemSize()
{
    int seq;

    rxDescriptor * pRxDesc = NULL;
    if(RecvPktCnt>0)
    {
       hal_state.total_pkt = 1;
       hal_state.last_seq_number = lastRIdx;

       seq = lastRIdx+1;
       if(seq>= RX_MEM_BLK_TOTAL_NUMBER)
       {
          seq = 0;
       }
       printk("[DrvRx]Refill to read seq=%d\n", seq);
       pRxDesc = getRxDesc(seq);
       hal_state.mem_size = pRxDesc->length;
    }

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
        
    

        /*****
            last pkt handle
        *****/

   // if(isSingleOrLastPkt(rxDesc))
	  {
		//printRxDescriptorList();
		lastPktHandle(rxDesc);
        /***** print pkt RSSI ******/
        //s46_Get_RSSI(&(tr_status.rssi1),&(tr_status.rssi2),1);
                
        rssi = SX1276LoRaReadRssi();
        printk("recv-> (%d dBm)\n", rssi);			//print rssi
        
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


