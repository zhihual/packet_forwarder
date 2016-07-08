#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>


#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include "global_type.h"
#include "main.h"

#include "hal.h"
#include "async.h"

#define GLOBALMEM_SIZE MAX_RX_MEM_SIZE  /*ȫ���ڴ���� ...*/

#define MEM_CLEAR_CMD 0x1 /*����ȫ���ڴ�*/
#define GLOBALMEM_MAJOR 252 /*Ԥ��� globalmem �����豸��*/

int8    rssi1           = 0;                                    // �����ź�ǿ��������������
uint8   rssi2           = 0;                                    // �����ź�ǿ��С����������

#define MODE_WAIT_DELAY   3*10                                  // �ϵ��������ʱ��(n*100ms)
uint16  mode_wait_time  = MODE_WAIT_DELAY;                      // �ϵ��������ʱ�����
uint8     rf_mode         = 1;                                    // ����ģʽ��־(0=rx,1=tx)
uint8   sendout_status  = 1;                                    // ���ݷ���״̬��־(1~4)
uint8     sendout_flag    = 0;                                    // ���ݷ��Ͷ�ʱ��־

static int globalmem_major = GLOBALMEM_MAJOR;

static int timer_cnt = 8;

extern int lastWIdx;
extern int lastRIdx;
extern int RecvPktCnt;

extern void desctoryRxDescriptorArray(void);

/*globalmem �豸�ṹ��*/
struct globalmem_dev
{
	struct cdev cdev; /*cdev �ṹ��*/
	unsigned char mem[GLOBALMEM_SIZE]; /*ȫ���ڴ�*/
};
struct globalmem_dev *globalmem_devp; /*�豸�ṹ��ָ��*/


/*�ļ��򿪺���*/
int globalmem_open(struct inode *inode, struct file *filp)
{
	printk("%s +\n",__func__);
	/*���豸�ṹ��ָ�븳ֵ���ļ�˽������ָ��*/
	filp->private_data = globalmem_devp;
	printk("%s -\n",__func__);

	return 0;
}

/*�ļ��ͷź���*/
int globalmem_release(struct inode *inode, struct file *filp)
{
	printk("%s \n",__func__);
    filp->private_data = NULL;
	return 0;
}

#define I2C_IOCTL_CMD_GET(cmd)      ( _IOC_NR(cmd) ) 
#define PRINT_MEM 10
#define I2C_IOCTL_CMD_IS_VALID(cmd) ( (_IOC_TYPE(cmd) == I2C_DRV_MAGICNUM ) ? 1 : 0)


/* ioctl �豸���ƺ��� */
//static int globalmem_ioctl(struct inode *inodep, struct file *filp, unsigned int cmd, unsigned long arg)
//refer to  http://blog.csdn.net/cbl709/article/details/7295772
static int globalmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	uint32 total_mem_size = 0;

        switch (cmd)
	{
	        case VNIC_IOCGET_TOTAL_MEM_SIZE: 
                        total_mem_size = getTotalMemSize();
                        if(copy_to_user((uint32 *)arg, &total_mem_size,sizeof(uint32)))
			{
				printk("%s: copy_to user failed\n",__func__);
				return EFAULT;
			}
                        break;
                case VNIC_IOCCCA: 
                        hal_cca();
                        break;
                default:
			return EINVAL;
        }
}

/*������*/
static ssize_t globalmem_read(struct file *filp, char __user *buf, size_t size,loff_t *ppos)
{
        unsigned long p = *ppos;
        unsigned int count = size;
        int ret = 0;
        // uint32 total_mem_size = getTotalMemSize();
        uint32 total_mem_size = size;
        uint8 *p_mem;
        int i = 0;
        uint8 seq_num;
        rxDescriptor *pRxDesc = NULL;
        
        /*�����ͻ�ȡ��Ч��д����*/
        if (p >= GLOBALMEM_SIZE)
                return count ? ENXIO:0;
        if (count > GLOBALMEM_SIZE)
                return count ? ENXIO:0;
    
        p_mem = kmalloc(total_mem_size, GFP_KERNEL);
        /*����ʧ��*/
        if (!p_mem)
        {
           printk("globalmem_read  alloc memory fail\n");
           ret = -ENOMEM;
           return ret;
        }
        memset(p_mem,0,total_mem_size);

        
            
        for(i = 0; i < 1;i++)
        {
            seq_num = lastRIdx+1;
            pRxDesc = getRxDesc(seq_num);
            //seq_num = 0 ;//= pRxDesc->pRxMem[RX_HEADER_SEQ_NUMBER_INDEX];
            //if pRxDesc->length <= PKT_HEADER_SIZE, usually 0, that means the pkt is an invalid pkt, skip it.
           
            if(seq_num >=RX_MEM_BLK_TOTAL_NUMBER )
            {
               printk("[DrvRx]ReadIdx overloop\n");
               seq_num = 0;
            }
            
            if(pRxDesc->length > 0)
            {
               memcpy(p_mem,(uint8 *)(pRxDesc->pRxMem),pRxDesc->length);
               ret += pRxDesc->length;
               printk("[DrvRx]Pop to APP len %d:\n", ret);
               lastRIdx = seq_num;
               RecvPktCnt--;
               pRxDesc->flag = FLAG_WRITE_OK;
               memset(pRxDesc->pRxMem, 0,pRxDesc->length);
            }
        }
        /*
                ret ��ʵ���յ�����Ч���ݳ���
        */
        
        /*�ں˿ռ���û��ռ�*/
        if (copy_to_user(buf, (void*)(p_mem), ret)) /*���ز��ܸ��Ƶ��ֽ���*/
        {
           printk("io_read pop to userspace error\n");
           ret = EFAULT;
        }
        else
        {
           if (ret != 0)
           {
            printk("read %d bytes(s) to %d\n", ret, buf);

           }
        }
        hal_state_reset();
        kfree(p_mem);
        return ret;
}



static ssize_t globalmem_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    uint8 *p_mem;
	int ret = 0;

    printk("%s+ %d\n",__func__, size);
	/* ���� �ڴ�*/

    
	p_mem = kmalloc(size, GFP_KERNEL);
	/*����ʧ��*/
	if (!p_mem)
	{
	    printk("write alloc mem fail\n");
		ret = -ENOMEM;
		return ret;
	}

	/*�û��ռ���ں˿ռ�*/
	if (copy_from_user(p_mem, buf, size))
	{
	    printk("write copy to usersapce fail\n");
		ret = EFAULT;
	}
	else
	{
		ret = size;
	}

    ret = hal_tx(p_mem,size); 
        
    kfree(p_mem);
    return ret;    
}


/*�ļ������ṹ��*/
static const struct file_operations globalmem_fops =
{
	.owner = THIS_MODULE,
	.read = globalmem_read,
	.write = globalmem_write,
	.unlocked_ioctl = globalmem_ioctl,
	.open = globalmem_open,
	.fasync		= hal_fasync,
	.release = globalmem_release,
};

/*��ʼ����ע�� cdev*/
static void globalmem_setup_cdev(struct globalmem_dev *dev, int index)
{
	int err, devno = MKDEV(globalmem_major, index);
	cdev_init(&dev->cdev, &globalmem_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &globalmem_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding device %d", err, index);
}




/*�豸����ģ����غ���*/
int globalmem_init(void)
{
	int result;
	dev_t devno = MKDEV(globalmem_major, 0);
	/* �����豸��*/
	if (globalmem_major)
		result = register_chrdev_region(devno, 1, "loramem");
	else /* ��̬�����豸�� */
	{
		result = alloc_chrdev_region(&devno, 0, 1, "loramem");
		globalmem_major = MAJOR(devno);
	}
	if (result < 0)
		return result;
	
	/* ��̬�����豸�ṹ����ڴ�*/
	globalmem_devp = kmalloc(sizeof(struct globalmem_dev), GFP_KERNEL);
	
	/*����ʧ��*/
	if (!globalmem_devp)
	{
		result = ENOMEM;
		goto fail_malloc;
	}
	memset(globalmem_devp, 0, sizeof(struct globalmem_dev));
	globalmem_setup_cdev(globalmem_devp, 0);
        printk("%s, %s\n",__func__,THIS_MODULE->name);

    hal_init();

    printk("put into rx default\n");
    hal_start_rx();

	return 0;
fail_malloc: unregister_chrdev_region(devno, 1);
	return result;
}

/*ģ��ж�غ���*/
void globalmem_exit(void)
{
  
	cdev_del(&globalmem_devp->cdev); /*ע�� cdev*/
        hal_release();
#if 0
	gpio_set_value(Si4463_SCK_GPIO, 0);
	gpio_set_value(Si4463_SDI_GPIO, 0);
	cleanup_spi_timer();
	cleanup_tx_timer();
	cleanup_rx_timer();
	s46_GPIO_Release();
#endif
	/*�ͷ��豸�ṹ���ڴ�*/
	
	kfree(globalmem_devp);
	unregister_chrdev_region(MKDEV(globalmem_major, 0), 1); /*�ͷű���*/
}

MODULE_AUTHOR("Song Baohua");
MODULE_LICENSE("Dual BSD/GPL");
module_param(globalmem_major, int, S_IRUGO);
module_param(timer_cnt, int, S_IRUGO);

module_init(globalmem_init);
module_exit(globalmem_exit);

