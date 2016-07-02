#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/types.h>


#include "global_type.h"
#include "common.h"

#include "sx1276.h"
#include "hal.h"

#include <asm/mach-ath79/ar71xx_regs.h>
#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/irq.h>


static uint8 gpioToIrq =  ATH79_MISC_IRQ(2); 
wait_queue_head_t txdone_wait_queue;
static unsigned long gpio_both_edge = 0;
static void __iomem *base = 0xb8040000;	
static uint8 txSentDone;
static struct tasklet_struct rxtasklet;
static struct _sx1276_status sx1276_status;


// spi read and write one byte
static unsigned char SpiInOut(unsigned char data) 
{
	unsigned char i;
	
	for (i = 0; i < 8; i++)		
	{				
		if (data & 0x80)
			//MOSI = 1;
                        gpio_set_value(SX1276_MOSI_GPIO, 1); 
		else
			//MOSI = 0;
                        gpio_set_value(SX1276_MOSI_GPIO, 0); 
			
		data <<= 1;
		//SCK = 1;
                gpio_set_value(SX1276_SCK_GPIO, 1); 
                //mdelay(1);
		
		//if (MISO)
                if(gpio_get_value(SX1276_MISO_GPIO))
			data |= 0x01;
		else
			data &= 0xfe;
			
		//SCK = 0;
                gpio_set_value(SX1276_SCK_GPIO, 0); 
                //mdelay(1);
	}	
	return (data);	
}

// SPI write register
void SPIWriteReg(unsigned char addr, unsigned char value)                
{                                                       
	addr |= 0x80;			// write register,MSB=1

	//SCK=0;
        gpio_set_value(SX1276_SCK_GPIO, 0); 
	//nCS=0;
        gpio_set_value(SX1276_NSS_GPIO, 0); 

	SpiInOut(addr);		// write register address
	SpiInOut(value);		// write register value

	//SCK=0;
        gpio_set_value(SX1276_SCK_GPIO, 0); 
	//MOSI=1;
        gpio_set_value(SX1276_MOSI_GPIO, 1); 
	//nCS=1;
        gpio_set_value(SX1276_NSS_GPIO, 1); 
    
}

// SPI read register
static unsigned char SPIReadReg(unsigned char addr)
{
	unsigned char data; 
	
	//MOSI=0;
        gpio_set_value(SX1276_MOSI_GPIO, 0); 
	//SCK=0;
        gpio_set_value(SX1276_SCK_GPIO, 0); 
	
	//nCS=0;
        gpio_set_value(SX1276_NSS_GPIO, 0); 
    
	SpiInOut(addr);		// write register address
	data = SpiInOut(0);		// read register value
	//nCS=1;
        gpio_set_value(SX1276_NSS_GPIO, 1); 
	return(data);
}

// spi burst write
void SPIBurstRead(unsigned char addr, unsigned char *ptr, unsigned char len)
{
	unsigned char i;
	if(len<=1)			// length>1,use burst mode
		return;
	else
	{
		//SCK=0; 
                gpio_set_value(SX1276_SCK_GPIO, 0); 
		//nCS=0;
                gpio_set_value(SX1276_NSS_GPIO, 0); 

		SpiInOut(addr);
		for(i=0;i<len;i++)
			ptr[i] = SpiInOut(0);
		//nCS=1;  
                gpio_set_value(SX1276_NSS_GPIO, 1); 
	}
}

// spi burst read
void SPIBurstWrite(unsigned char addr, unsigned char *ptr, unsigned char len)
{ 
	unsigned char i;
	
	addr |= 0x80;  

	if(len<=1)			// length>1,use burst mode
		return;
	else  
	{   
		//SCK=0;
                gpio_set_value(SX1276_SCK_GPIO, 0); 
		//nCS=0;        
                gpio_set_value(SX1276_NSS_GPIO, 0); 
		SpiInOut(addr);
		for(i=0;i<len;i++)
			SpiInOut(ptr[i]);
		//nCS=1;  
               gpio_set_value(SX1276_NSS_GPIO, 1); 
	}
}

void sx1276_GPIO_Init()
{
	int result;
	result = gpio_request(SX1276_SCK_GPIO, SX1276_SCK_GPIO_NAME);
	if (result < 0){
		printk("###gpio_request ERROR: can't request %d pin for output###\n",SX1276_SCK_GPIO);
		return -1;
	}else
	{
		printk("GPIO request OK: GPIO %d\n",SX1276_SCK_GPIO);
	}
	gpio_direction_output(SX1276_SCK_GPIO, 0);

	result = gpio_request(SX1276_MOSI_GPIO, SX1276_MOSI_GPIO_NAME);
	if (result < 0){
		printk("###gpio_request ERROR: can't request %d pin for output###\n",SX1276_MOSI_GPIO);
		return -1;
	}else
	{
		printk("GPIO request OK: GPIO %d\n",SX1276_MOSI_GPIO);
	}
	gpio_direction_output(SX1276_MOSI_GPIO, 0);

	result = gpio_request(SX1276_MISO_GPIO, SX1276_MOSI_GPIO_NAME);
	if (result < 0){
		printk("###gpio_request ERROR: can't request %d pin for input###\n",SX1276_MISO_GPIO);
		return -1;
	}else
	{
		printk("GPIO request OK: GPIO %d\n",SX1276_MISO_GPIO);
	}
	gpio_direction_input(SX1276_MISO_GPIO);

	result = gpio_request(SX1276_NSS_GPIO, SX1276_NSS_GPIO_NAME);
	if (result < 0){
		printk("###gpio_request ERROR: can't request %d pin for output###\n",SX1276_NSS_GPIO);
		return -1;
	}else
	{
		printk("GPIO request OK: GPIO %d\n",SX1276_NSS_GPIO);
	}
	gpio_direction_output(SX1276_NSS_GPIO, 1);

	result = gpio_request(SX1276_NRESET_GPIO, SX1276_NRESET_GPIO_NAME);
	if (result < 0){
		printk("###gpio_request ERROR: can't request %d pin for input###\n",SX1276_NRESET_GPIO);
		return -1;
	}else
	{
		printk("GPIO request OK: GPIO %d\n",SX1276_NRESET_GPIO);
	}
	//gpio_direction_input(Si4463_CTS_GPIO);
	gpio_direction_output(SX1276_NRESET_GPIO, 0);

	//initGPIO_IRQ();
	
	return;
}

void sx1276_GPIO_Release()
{
	gpio_free(SX1276_SCK_GPIO);
	gpio_free(SX1276_MOSI_GPIO);
	gpio_free(SX1276_MISO_GPIO);
	gpio_free(SX1276_NSS_GPIO);
	gpio_free(SX1276_NRESET_GPIO);
    
	//freeGPIO_irq();
	printk("*******%s*************\n",__func__);
}

void reset_sx1276(void)
{
	//tx_en=0;		// close antenna switch
	//rx_en=0;

	//RF_RST=0;
        gpio_set_value(SX1276_NRESET_GPIO,0);
	//delay_x10ms(1);		// delay 10ms
	msleep(10);
	//RF_RST=1;
        gpio_set_value(SX1276_NRESET_GPIO,1);
	//delay_x10ms(2);		// delay 20ms
	msleep(20);
}   

// length of packet
//#define payload_length    64

// tx packet
//unsigned char  txbuf[payload_length]={'s','e','n','d','_','t','e','s','t'};
// rx packet

/*
    1. 根据数据包的大小设置超时时间，
    2. wait_event_timeout中的timeout的单位是10ms
    3. 实验数据得出，小包时速率为1.6Kbps，大包速率为5.8Kbps，
    4. timeout = (size*8)/bps
    5. 内核中不推荐使用浮点计算，故速率*10处理，
*/

#define TX_TIMEOUT_OFFSET 200 //add offset of tx time

static uint8 calcTimeOut(uint8 datasize)
{
        uint8 timeout;  //unit is 10ms
        if(datasize < 30)
        {
                timeout = (((datasize*8)/(16)+TX_TIMEOUT_OFFSET/10)*10)/10; 
        }else
        {
                timeout = (((datasize*8)/(58)+TX_TIMEOUT_OFFSET/10)*10)/10;
        }
        //printk("calc time out is %d ms\n",timeout*10);
        return timeout;
}


/*
static uint8 calcTimeOut(uint8 datasize)
{
        uint8 timeout;  //unit is 10ms
        if(datasize < 30)
        {
                timeout = (((datasize*8)/(5)+TX_TIMEOUT_OFFSET/10)*10)/10; 
        }else
        {
                timeout = (((datasize*8)/(19)+TX_TIMEOUT_OFFSET/10)*10)/10;
        }
        //printk("calc time out is %d ms\n",timeout*10);
        return timeout;
}
*/

static void waitSentDone(uint8 datasize)
{
        int ret;
        uint8 timeout = calcTimeOut(datasize);
        
        //recordStartTime();

	ret = wait_event_timeout(txdone_wait_queue,(1 == txSentDone),timeout);  //
	if((1 == ret) || (0 == ret))
	{
                printk("*");
		//printk("%s, timeout %dms, keep sending\n",__func__,TX_TIME_OUT*10);
                SPIWriteReg(LR_RegIrqFlags,0xff);	// clear interrupt
                //SPIWriteReg(LR_RegOpMode,0x09);		// enter Standby mode
                sx1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);   // enter Standby mode
                reset_sx1276();						// reset RF
                SX1276_Config();  			// initialize RF module
	}
	
	txSentDone = 0;
    
        //recordEndTime(1);
	return;
}


static void sx1276_tx(uint8 *txbuf, unsigned int length)
{
	unsigned char addr,temp;
	
	//tx_en=1;						// open tx antenna switch
	//rx_en=0;
	//printk("current mode = 0x%02x, length = %d\n",SX1276_GetOpMode(),length);

	SPIWriteReg(REG_LR_DIOMAPPING1,0x41); 			// DIO0=TxDone,DIO1=RxTimeout,DIO3=ValidHeader
		
	SPIWriteReg(LR_RegIrqFlags,0xff);			// clear interrupt
	SPIWriteReg(LR_RegIrqFlagsMask,0xf7);			// enable txdone
	SPIWriteReg(LR_RegPayloadLength,length);	// packet length

	addr = SPIReadReg(LR_RegFifoTxBaseAddr);		// read TxBaseAddr      
	addr =0x0;
        SPIWriteReg(LR_RegFifoTxBaseAddr,addr);	
	//printk("LR_RegFifoTxBaseAddr = 0x%02x\n",addr);

	SPIWriteReg(LR_RegFifoAddrPtr,addr);			// TxBaseAddr->FifoAddrPtr          
	
	SPIBurstWrite(0x00,txbuf,length);		// fill data into fifo
	//SPIWriteReg(LR_RegOpMode,0x0b);					// enter tx mode
	sx1276LoRaSetOpMode(RFLR_OPMODE_TRANSMITTER);                          // enter tx mode

        waitSentDone(length);
        
	//SPIWriteReg(LR_RegOpMode,0x09);  			// enter standby mode
	sx1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);    // enter standby mode
	
}

void sx1276_Send_Packet(uint8 *txbuf, unsigned int length)
{
        int i;
        
#if 0
        printk("------------------buff_len(%d)-----------------\n",length);
        for(i=0;i<length;i++)
        {
            printk("%02d\t",txbuf[i]);
            if((i+1)%8 == 0) printk("\n");
        }
        printk("\n");
        printk("---------------------------------------------\n");
#endif
        sx1276_tx(txbuf,length);

}



void rx_init(void)
{
	unsigned char addr; 
        uint8 RegDioMapping1;
        uint8 RegIrqFlagsMask;
	
	//tx_en=0;
	//rx_en=1;						// open rx antenna switch
	
	//Flag.is_tx = 0;
	    
	//SPIWriteReg(REG_LR_DIOMAPPING1,0x01);			//DIO0=00, DIO1=00, DIO2=00, DIO3=01  DIO0=00--RXDONE
	                                        // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
        SPIWriteReg(REG_LR_DIOMAPPING1,RegDioMapping1);
	  
	//SPIWriteReg(LR_RegIrqFlagsMask,0x3f);			// enable rxdone and rxtimeout

        RegIrqFlagsMask = //RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    RFLR_IRQFLAGS_CADDETECTED;
                                    
        SPIWriteReg( LR_RegIrqFlagsMask, RegIrqFlagsMask );
	SPIWriteReg(LR_RegIrqFlags,0xff);			// clear interrupt

	addr = SPIReadReg(LR_RegFifoRxBaseAddr);		// read RxBaseAddr
	addr = 0x0;
        SPIWriteReg(LR_RegFifoRxBaseAddr,addr);
	//printk("LR_RegFifoRxBaseAddr = 0x%02x\n",addr);
	SPIWriteReg(LR_RegFifoAddrPtr,addr);			// RxBaseAddr->FifoAddrPtr
	//SPIWriteReg(LR_RegOpMode,0x0d);				// enter rx continuous mode
	sx1276LoRaSetOpMode(RFLR_OPMODE_RECEIVER);				// enter rx continuous mode
}



uint8 sx1276_test()
{
        uint8 temp;
        int i;
        temp=SPIReadReg(REG_LR_VERSION);
        printk("REG_LR_VERSION = 0x%02x\n",temp);
/*
        SPIWriteReg(REG_LR_DIOMAPPING1,0x55);
        temp=SPIReadReg(REG_LR_DIOMAPPING1);
        printk("REG_LR_DIOMAPPING1 = 0x%02x\n",temp);
*/

/*
        for(i=0;i<10;i++)
        {
                tx_mode();                   // transmit packet
                rx_init();                      // wait for the reply
                msleep(100);
        }
*/
	return 0;
}

void SX1276_Config(void)
{
	// In setting mode, RF module should turn to sleep mode
	//SPIWriteReg(LR_RegOpMode,0x08);		// low frequency mode，sleep mode
	SPIWriteReg(LR_RegOpMode,0x08);		// low frequency mode，
	sx1276LoRaSetOpMode(RFLR_OPMODE_SLEEP);		// sleep mode
	//nop_10();
	msleep(10);

	SPIWriteReg(REG_LR_TCXO,0x09);		// external Crystal
	SPIWriteReg(LR_RegOpMode,0x88);		// lora mode

	SPIWriteReg(LR_RegFrMsb,0x7A);         // 0x75 -> 470MHz   0x6c->434MHz, 0x7A -> 490MHz
	SPIWriteReg(LR_RegFrMid,0x80);
	SPIWriteReg(LR_RegFrLsb,0x00);		// frequency：434Mhz					 

	SPIWriteReg(LR_RegPaConfig,0xff);	// max output power

	SPIWriteReg(LR_RegOcp,0x0B);		// close ocp
	SPIWriteReg(LR_RegLna,0x23);		// enable LNA

	SPIWriteReg(LR_RegModemConfig1,0x92);	// signal bandwidth：500kHz,coding rate = 4/5,explicit Header mode
	SPIWriteReg(LR_RegModemConfig2,0x87);	// spreading factor：7
	SPIWriteReg(LR_RegModemConfig3,0x08);	// LNA
	  
	SPIWriteReg(LR_RegSymbTimeoutLsb,0xFF); // max rx timeout

	SPIWriteReg(LR_RegPreambleMsb,0x00);
	SPIWriteReg(LR_RegPreambleLsb,16);      // preamble 16 bytes  	

	SPIWriteReg(REG_LR_PADAC,0x87);         // tx power 20dBm
	SPIWriteReg(LR_RegHopPeriod,0x00);      // no hopping

	SPIWriteReg(REG_LR_DIOMAPPING2,0x01);   // DIO5=ModeReady,DIO4=CadDetected
	//SPIWriteReg(LR_RegOpMode,0x09);         // standby mode
	sx1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);         // standby mode
}


void sx1276_cad_init(void)
{
        
        uint8 RegIrqFlagsMask;
        uint8 RegDioMapping1,RegDioMapping2;
        printk("%s\n",__func__);
        sx1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    
        RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    //RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        SPIWriteReg( LR_RegIrqFlagsMask, RegIrqFlagsMask );
           
                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        //RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_10 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
        RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SPIWriteReg( REG_LR_DIOMAPPING1, RegDioMapping1 );
        SPIWriteReg( REG_LR_DIOMAPPING2, RegDioMapping2 );
            
        sx1276LoRaSetOpMode( RFLR_OPMODE_CAD );
}


static void sx1276StatusInit(void)
{
        sx1276_status.currState = 0x00;
}



void sx1276_init(void)
{
        sx1276_GPIO_Init();
        initGPIO_IRQ();
        
        reset_sx1276();		// reset RF module
        sx1276StatusInit();
        SX1276_Config();        
        
        ath79_gpio_irq_enable(SX1276_nIRQ_GPIO);  //enable the gpio interrupt
        sx1276_test();
}



void sx1276_release(void)
{
        sx1276_GPIO_Release();
        freeGPIO_irq();
}


uint8 SX1276_GetOpMode( void )
{
        uint8 temp;
        temp=SPIReadReg(LR_RegOpMode);
    
        //return temp & 0x07;
        return temp;
}


void sx1276LoRaSetOpMode( uint8 opMode )
{
        static uint8 first_touch = 0;
        if( (opMode != (sx1276_status.currState & ~RFLR_OPMODE_MASK)) || \
                    (0 == first_touch))
        {
                sx1276_status.currState = ( sx1276_status.currState & RFLR_OPMODE_MASK ) | opMode;
                SPIWriteReg( LR_RegOpMode, sx1276_status.currState ); 
                //printk("%s--> 0x%02x\n",__func__,sx1276_status.currState);
        }
        first_touch++;
}


uint8 sx1276LoRaGetOpMode( void )
{
        uint8 currentMode;
        currentMode = SPIReadReg(LR_RegOpMode);
        return  (currentMode & ~RFLR_OPMODE_MASK);
}

uint8 sx1276IntrFlag(void)
{
        return  SPIReadReg(LR_RegIrqFlags);
}

int SX1276LoRaReadRssi( void )
{
        uint8 RegRssiValue;
        // Reads the RSSI value
        RegRssiValue = SPIReadReg( LR_RegRssiValue );
        //printk("RegRssiValue = %d\n",RegRssiValue);

        if( 1 )  // LF
        {
                return (int)RSSI_OFFSET_LF + RegRssiValue;
        }
        else
        {
                return (int)RSSI_OFFSET_HF + RegRssiValue;
        }
}

void sx1276_getRSSIValue(struct _rssi_value *rssi_value)
{
        uint8 rxSnrEstimate;
        int8 RxPacketSnrEstimate;
        uint8 RegPktRssiValue;
        int rssiValue;
        
        rxSnrEstimate = SPIReadReg( LR_RegPktSnrValue );
        
        printk("rxSnrEstimate = %02x\n",rxSnrEstimate);
        if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
        {
                // Invert and divide by 4
                RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                RxPacketSnrEstimate = -RxPacketSnrEstimate;
        }
        else
        {
                // Divide by 4
                RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
        }

        
        RegPktRssiValue = SPIReadReg( LR_RegPktRssiValue );
        printk("RegPktRssiValue = %d\n",RegPktRssiValue);

        //LF
        {    
                if( RxPacketSnrEstimate < 0 )
                {
                        rssi_value->integer = (int)RSSI_OFFSET_LF + (RegPktRssiValue) + RxPacketSnrEstimate;
                        rssi_value->fraction = 0;
                }
                else
                {
                        //RxPacketRssiValue = RSSI_OFFSET_LF + ( 1.0666 * ( ( double )RegPktRssiValue ) );
                        rssiValue = ((int)RSSI_OFFSET_LF)*10000 + ( 10666 * ( RegPktRssiValue ) );
                        rssi_value->integer = rssiValue/10000;
                        if(rssiValue < 0) {rssiValue = 0 - rssiValue;}
                        rssi_value->fraction = rssiValue%10000;
                }
        }


        return;
}


static void receiveHandler(unsigned long dev_id)
{	
        uint8 temp;
        uint8 packet_size;
        struct _rssi_value rssi;
        
        //printk("%s\n",__func__);
        temp = SPIReadReg(LR_RegFifoRxCurrentaddr);	// read RxCurrentaddr
	SPIWriteReg(LR_RegFifoAddrPtr,temp);		// RxCurrentaddr -> FiFoAddrPtr

	packet_size = SPIReadReg(LR_RegRxNbBytes);	// read length of packet  
	sx1276_getRSSIValue(&rssi);
	printk("rx packet_size = %d rssi = %d.%d dBm\n",packet_size,rssi.integer,rssi.fraction);

fail_receive:
        rx_init();
        return;
}

void sx1276_getRecvBuff(uint8 *p,uint8 length)
{
        SPIBurstRead(0x00, p, length);     // read from fifo
        
#if 0
                printk("%s, rx_length = %d\n",__func__,rx_length);
        
                for(i=0;i<rx_length;i++)
                {
                        printk("%02d\t",rxBuff[i]);
                        if((i+1)%8 == 0) printk("\n");
                }
                printk("\n");
#endif
}


uint8 sx1276_getRXLength(void)
{
        uint8 temp;
        uint8 packet_size;
        temp = SPIReadReg(LR_RegFifoRxCurrentaddr);	// read RxCurrentaddr
	SPIWriteReg(LR_RegFifoAddrPtr,temp);		// RxCurrentaddr -> FiFoAddrPtr

	packet_size = SPIReadReg(LR_RegRxNbBytes);	// read length of packet  
	return packet_size;
}

static irqreturn_t PA0IntHandler(int irq, void *dev_id)
{
        uint8 t=0;
        int rssi = 0;

        //spin_lock_irqsave(&ath79_gpio_lock, flags);
        // clear the GPIO interrupt status
        unsigned long stat = __raw_readl(base + AR71XX_GPIO_REG_INT_PENDING);
        unsigned long gpio_level = 0x1 & (__raw_readl(base + AR71XX_GPIO_REG_IN)>>SX1276_nIRQ_GPIO);
        //unsigned long gpio_level = gpio_get_value(Si4463_nIRQ_GPIO);
        //printk("Irq=%d stat = 0x%x,\t(%d)gpio_lvl = 0x%x\n",irq,stat,SX1276_nIRQ_GPIO,gpio_level); 

	if((gpio_level == 1)&& \ 
		((stat & (1 << SX1276_nIRQ_GPIO)) == (1 << SX1276_nIRQ_GPIO)))
        {
                t=SPIReadReg(LR_RegIrqFlags);			// read interrupt 
                //printk("LR_RegIrqFlags = 0x%02x\n",t);
                if(t & RFLR_IRQFLAGS_TXDONE)
                {
                        //printk("tx done!\n");
                        SPIWriteReg(LR_RegIrqFlags,RFLR_IRQFLAGS_TXDONE);  //clear irq
                        txSentDone = 1;
			wake_up(&txdone_wait_queue);
                }else if(t & RFLR_IRQFLAGS_RXDONE)
                {
                        //printk("rx done!\n");
                        
                        //printk("rx done 0x%02x\n", t);
                        SPIWriteReg(LR_RegIrqFlags,RFLR_IRQFLAGS_RXDONE);  //clear irq
#if 1
                        //底半部实现一：tasklet（由内核线程——软中断（ksoftirqd/0）调度执行），不能睡眠
                        tasklet_init(&rxtasklet, hal_rx,(unsigned long)dev_id); //中断底半部，tasklet实现方式中、底半部也不能睡眠
                        tasklet_schedule(&rxtasklet); //调度底半部
                        //此时，底半部在合适时机运行与软中断上下文               
#endif
                }else if(t & RFLR_IRQFLAGS_CADDONE)
                {
                        printk("Intr CAD Done!\n");
                        SPIWriteReg(LR_RegIrqFlags,RFLR_IRQFLAGS_CADDONE);  //clear irq
                        
                        printk("***0x%02x\n", t);
                        
                        rssi = SX1276LoRaReadRssi();
                        printk("-> (%d dBm)\n", rssi);			//print rssi
                }
                else
                {
                        printk("unknown IrqFlags = 0x%02x\n",t);
                }
        }   
        
        return IRQ_HANDLED;
        
}

/*  
    说明...
	这个历程已经实现了GPIO的中断处理，主要的中断配置函数是，ath79_gpio_irq_type， 注意只验证了IRQ_TYPE_EDGE_RISING部分，其他部分未做验证。
	测试使用的GPIO为Si4463_GP14_GPIO，现在能触发中断，但是每次上升沿会触发两次，下降沿也会触发两次，真TMD奇怪。
	对应的中断状态分别为：
					AR71XX_GPIO_REG_INT_PENDING		AR71XX_GPIO_REG_IN
	rising_edge	    0x4000 								1
					0x0									1
	falling_dege	0x4000								0					
					0x0									0
	为了赶进度，只能通过上面的排列组合，来做中断处理了，
	ps：可以确定是边缘触发的，因为长按按钮，不会一直触发中断，并且，改变ath79_gpio_irq_type中的配置，
	可以实现电平触发的机制。
*/

/*
  下面的type全部都写错了，tmd要改，先记着。。。谁tmd写!!!
*/

int ath79_gpio_irq_type(int gpio, unsigned type)
{	
	int offset = gpio;	
	unsigned long flags;	
	unsigned long int_type;	
	unsigned long int_polarity;	
	unsigned long bit = (1 << offset);	
	//spin_lock_irqsave(&ath79_gpio_lock, flags);	
	int_type = __raw_readl(base + AR71XX_GPIO_REG_INT_TYPE);	
	int_polarity = __raw_readl(base + AR71XX_GPIO_REG_INT_POLARITY);	
	gpio_both_edge &= ~bit;	
	switch (type) {	
		case IRQ_TYPE_EDGE_RISING:		
			int_type &= ~bit;		
			int_polarity |= bit;
			break;	
		case IRQ_TYPE_EDGE_FALLING:		
			int_type &= ~bit;		
			int_polarity &= ~bit;	
			break;	
		case IRQ_TYPE_LEVEL_HIGH:		
			int_type |= bit;		
			int_polarity |= bit;		
			break;	
		case IRQ_TYPE_LEVEL_LOW:		
			int_type |= bit;		
			int_polarity &= ~bit;		
			break;	
#if 0
		case IRQ_TYPE_EDGE_BOTH:		
			int_type |= bit;		
			/* set polarity based on current value */		
			if (gpio_get_value(offset)) {			
				int_polarity &= ~bit;		
			} else {			
				int_polarity |= bit;	
			}		
			/* flip this gpio in the interrupt handler */		
			gpio_both_edge |= bit;		
			break;	
#endif
		default:		
			//spin_unlock_irqrestore(&ath79_gpio_lock, flags);	
			printk("*****%s***** error\n",__func__);
			return -EINVAL;	
	}
	__raw_writel(int_type, base + AR71XX_GPIO_REG_INT_TYPE);	
	__raw_writel(int_polarity, base + AR71XX_GPIO_REG_INT_POLARITY);
	//printk("int_type = 0x%x, int_polarity = 0x%x\n",int_type,int_polarity);

	__raw_writel(__raw_readl(base + AR71XX_GPIO_REG_INT_MODE) | (1 << offset),		     
		base + AR71XX_GPIO_REG_INT_MODE);	

	__raw_writel(__raw_readl(base + AR71XX_GPIO_REG_INT_ENABLE) & ~(1 << offset),		     
		base + AR71XX_GPIO_REG_INT_ENABLE);	
	//spin_unlock_irqrestore(&ath79_gpio_lock, flags);	
	return 0;
}


void initGPIO_IRQ()
{
	int result;
	int ret;

	result = gpio_request(SX1276_nIRQ_GPIO, SX1276_nIRQ_GPIO_NAME);
	if (result < 0){
		printk("###gpio_request ERROR: can't request %d pin for input###\n",SX1276_nIRQ_GPIO);
		return -1;
	}else
	{
		printk("GPIO request OK: GPIO %d\n",SX1276_nIRQ_GPIO);
	}
	gpio_direction_input(SX1276_nIRQ_GPIO); 	

	printk("---> gpioToIrq = %d\n",gpioToIrq);
	/* 
	 1. 该版本的中断触发，最后还是采用低电平触发的模式，\
	    原因是若采用边沿触发，接收时会出现丢中断的情况，\
		从而导致接收超时，
	*/
	ath79_gpio_irq_type(SX1276_nIRQ_GPIO,IRQ_TYPE_LEVEL_HIGH);
	
	ret = request_irq(gpioToIrq, PA0IntHandler,0, "Lora_intr", NULL);
	printk("request_irq ret = %d\n",ret);

	init_waitqueue_head(&txdone_wait_queue);

	return;
}


void freeGPIO_irq()
{
	gpio_free(SX1276_nIRQ_GPIO);
	free_irq(gpioToIrq,NULL);
}

void ath79_gpio_irq_enable(int gpio)
{	
	int offset = gpio;	
	void __iomem *base = ath79_gpio_base;	
	__raw_writel(__raw_readl(base + AR71XX_GPIO_REG_INT_ENABLE) | (1 << offset),		     
		base + AR71XX_GPIO_REG_INT_ENABLE);

}

void ath79_gpio_irq_disable(int gpio)
{	
	int offset = gpio;	
	void __iomem *base = ath79_gpio_base;	
	__raw_writel(__raw_readl(base + AR71XX_GPIO_REG_INT_ENABLE) & ~(1 << offset),			
		base + AR71XX_GPIO_REG_INT_ENABLE);
}


