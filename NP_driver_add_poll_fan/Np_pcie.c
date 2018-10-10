/****
Data:   2015.4.20
Author: 706.ykx
****/
// #include <linux/config.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/irq.h> 
#include <asm/signal.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>
#include <asm/errno.h>
#include <linux/unistd.h>   
#include <linux/time.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/sched.h> 
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/device.h> 
//#include "pcie56Drv_v1.0.h"
#define READ_DATA 		0x00080001	//sf read data	interface
#define WRITE_DATA 		0x00080002	//sf write data interface
#define READ_VERSION 	0x00080003	//read fpga software verison interface
#define FPGA_RESET		0x00080004	//fpga software reset interface
#define FPGA_CONFIG		0x00080005	//fpga config	interface
#define SFIO_FLAG		0x00080006	//slect sf zone interface



#define Version2

#define Server 0
#define DEBUG

#ifdef DEBUG
#define PRINTK(args...) printk(args)
#else
//#define for(;;)             do{}while(0)
#define PRINTK(args...) do{}while(0)
#endif

#define uchar unsigned char

#ifndef CONFIG_PCI
#define CONFIG_PCI
#endif
#define DEVICE_Np1_ID     0x1114       //pcie56 device id
#define DEVICE_Np2_ID     0x1116
#define DEVICE_LS_ID      0x7124

#define NP1_ID 0x01
#define NP2_ID 0x02


#define VENDOR_ID    		 0xd6fa        //pcie56 vendor id
#define DEVICE_MAJOR           66

MODULE_LICENSE("GPL");


#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36) && !defined(init_MUTEX)
#define init_MUTEX(sem)     sema_init(sem, 1)
#endif


/////////////////////////////////////////////////////
#define MAXENCRYPT                32
#define MAXRECVQL			1024
#define MAXSENDQL			1024
#define FRAMELEN			4096
#define DMA_FIFO_SIZE		FRAMELEN    //DMA BUFFER,default is 8K

#define GLOBALMEM_SIZE        	4096		//32KB


#define IP_ADDR_BASE			(unsigned long)(0x00000000)
#define DMA_SND_ADD32      		0x00  			
#define DMA_SND_ADD64 			0x04
#define DMA_SND_SIZE			0x08			
#define DMA_SND_CTRL 			0x0c			
#define DMA_SND_START 			0x00000004			
#define DMA_SND_ABORT 			0x00000008			
#define DMA_SND_SG 				0x00000010			
#define DMA_SND_BUSY			0x00000004			
#define DMA_RCV_ADD32			0x10			
#define DMA_RCV_ADD64 			0x14                   
#define DMA_RCV_SIZE_SET		0x18			
#define DMA_RCV_SIZE_GET		0x24			
#define DMA_RCV_CTRL			0x1c			
#define DMA_RCV_START 			0x00000004			
#define DMA_RCV_ABORT 			0x00000008			
#define DMA_RCV_SG 				0x00000010			
#define DMA_RCV_BUSY			0x00000004			
#define DMA_INT_EN				0x38			
#define DMA_INT_ENALL			0x80000000			
#define DMA_SND_EN				0x00000001			
#define DMA_RCV_EN				0x00000002			
//#define DMA_RCV_RDY_EN			0x00000008			
#define DMA_INT_STAT			0x34						
#define DMA_SND_INT				0x00000001			
#define DMA_RCV_INT				0x00000002			
#define DMA_RCV_RDY_INT		0x00000008			
#define DMA_INT_ALL				0x0000000B
#define DMA_RCV_LIST_RESET       0x7fffffff
#define DMA_RCV_LIST_FLAG		0x80000000
#define SND_LIST_END                   0x00000001
#define SND_LIST_RESET			0xFEFFFFFF
#define RECV_OTHER_COUNT		0x54	//0-15:the count of head on other side,16-31:the count of tail on other side.(RD)
#define RECV_OWN_HEAD			0x58	//0-15:the count of head on own side.(WR)




#define FPGA_SOFT_VERISON		0x80
#define FPGA_SW_RST                   0x3C
#define FPGA_CH_MODE			0x44
#define CUR_MODE_SET			0x48			
#define FPGA_TEST			0x100			


#define SUCCESS						0			
#define FAULT						-1			
#define ERR_SND_DMA_BUSY			-2			
#define ERR_RCV_DMA_BUSY			-3			
#define ERR_SND_QUEUE_FULL			-4			
#define ERR_RCV_QUEUE_EMPTY		-5			
#define ERR_SND_FIFO_FULL			-6			
#define ERR_NO_MEMORY				-7			
#define ERR_PARAMETER				-8			
#define ERR_FRAME_TOO_LONG		-9			
#define ERR_SND_TIMEOUT			-10			
#define ERR_RCV_TIMEOUT			-11			
#define ERR_IRQ_UNDEF				-12			
#define ERR_IRQ_NONE				-13			

#define SEND_TIMEOUT			6000
#define RECV_TIMEOUT			6000


#define UINT8	unsigned char 
#define UINT		unsigned int
#define UINT64	unsigned long
int pcie56_fasync(int fd, struct file *filp, int mode);
int pcie56_release(struct inode *inode, struct file *filp);
int pcie56_open(struct inode *inode, struct file *filp);
ssize_t pcie56_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t pcie56_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
unsigned int  pcie56_poll(struct file *filp, poll_table *wait);
#ifdef Server
long  pcie56_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
int pcie56_ioctl(struct inode *in, struct file *filp, unsigned int cmd ,unsigned long arg);
#endif
		
void pcie56_int_enable(void);
void pcie56_int_disable(void);
void write_BAR0(phys_addr_t   offset, unsigned int data);

struct fasync_struct *async_queue;	
 wait_queue_head_t sendoutq; 			
 wait_queue_head_t recvinq;			
 spinlock_t lock;         			 		/* mutual exclusion semaphore */
 struct semaphore en_sem; 			
struct semaphore  write_sem;
struct semaphore  read_sem;

struct file_operations pcie56Drv_fops =
{
	.owner = THIS_MODULE,
	.open = pcie56_open,
	.release = pcie56_release,
	.read = pcie56_read,
	.write = pcie56_write,
	.poll = pcie56_poll,
//	.fasync = pcie56_fasync,
#ifdef Server
	.compat_ioctl = pcie56_ioctl,
#else
	//.unlocked_
	.ioctl = pcie56_ioctl,
#endif
};

struct SendPacket
{
    int length;
    char *buf;
    int *next_Send_packet;
};

struct class *pcie56_device;
struct pci_dev *pcie56=NULL;        //pci device struct
struct cdev pcie56_cdev;       		 //pci device struct
struct class *pcie56_class;
int pcie56_major = DEVICE_MAJOR;
static int pcie56_opens = 0;
dma_addr_t sendlistPh,recvlistPh;
static unsigned int *pcie56_BAR0_Addr=NULL;     //base0


unsigned int tc;
void do_int_tasklet_recv(unsigned long tc);
//void do_int_tasklet_b2a(unsigned long tc);
//void do_int_tasklet_encrypt(unsigned long tc);

DECLARE_TASKLET(int_tasklet_recv,do_int_tasklet_recv,(unsigned long)&tc);
//DECLARE_TASKLET(int_tasklet_b2a,do_int_tasklet_b2a,(unsigned long)&tc);


DECLARE_WAIT_QUEUE_HEAD(SendQ_Full);
DECLARE_WAIT_QUEUE_HEAD(RecvQ_Empty);

UINT  			Recv_Ready = 0;
volatile UINT		RHead = 0;					
volatile UINT		RTail = 0;					
volatile UINT		SHead = 0;					
volatile UINT		STail = 0;					
volatile UINT 		Slisthead = 0;				
volatile UINT   	Slisttail = 0;					
volatile UINT 		Rlisthead = 0;				
volatile UINT   	Rlisttail = 0;					
volatile UINT 		Renhead = 0;				
volatile UINT   	Rentail = 0;		
volatile UINT		Recv_count = 0;
volatile UINT		Send_count = 0;
volatile UINT         Own_head = 0;


char  *dma_buffer_wr = NULL;          //global buffer for dma
char  *dma_buffer_rd = NULL; 
UINT   rcv_len = 0;
UINT   DMA_Abort_Flag=0;		//DMA recv abort
UINT   DMA_First_Flag = 1;		
UINT   Rcv_wait = 0;		
UINT   Next_Send_count =0;
UINT   DstID = 0;
UINT   LocalID = 0;
UINT   Count = 0;
UINT   Read_FLag = 0;
UINT   Recv_Flag = 0;
UINT   SCount = 0;
UINT   Dma_ready = 1;
struct queues{
	unsigned char *Buffer;
	dma_addr_t  BufferPh;
	int     len;
} RcvQ[MAXRECVQL], SndQ[MAXSENDQL],Rencrycp[MAXENCRYPT];

void Change_BELE(unsigned char *p)
{
	UINT8 b;
	b = p[0];
	p[0] = p[3];
	p[3] = b;
	b = p[1];
	p[1] = p[2];
	p[2] = b;
}
void SetBuffer_BYTE_ChgBELE(const unsigned char *p_data, unsigned int len)
{
	unsigned int i = 0;	
	if (len == 0)
		return;
	if (len % 4 != 0){
		len += 4 - len % 4;
	}
	for (i=0;i<len;i+=4){
		Change_BELE((char*)(p_data+i));
	}
}


//recv  descriptor list for S/G DMA

struct recv_descriptor{
	u32 PhAddr_low;				//send buffer busical address 0 -31  bit ;the bit[1:0] must be '00'
	u32 PhAddr_hig;				//send buffer busical address 32 - 63 bit
	u32 status;					//the bit[32] ='1',the buffer had been filled;the bit[0-31] is the length of the buffer had receved
	u32 NextDesc_low;				//next descriptor address 0-31bit  ;if bit[0]=1,this the end descriptor
	u32 NextDesc_hig;				//nest descriptor address 32-63bit
	u32 RecvFlagtag;
	u32 RecvFlagtag1;
	u32 RecvFlagtag2;
	}*recv_list;
//send descriptor list for S/G DMA
struct send_descriptor{
	u32 PhAddr_low;				//send buffer busical address 0-31 bit ;the bit[1:0] must be '00'
	u32 PhAddr_hig;				//send buffer busical address 32-63 bit
	u32 length;					// the send buffer length
	u32 NextDesc_low;				//next descriptor address 0-31bit  ;if bit[0]=1,this the end descriptor
	u32 NextDesc_hig;				//nest descriptor address 32-63bit
	u32 SendFlagtag;
	u32 SendFlagtag1;
	u32 SendFlagtag2;
	}*send_list;

spinlock_t sendLock;
spinlock_t recvLock;
//spinlock_t sQLock = SPIN_LOCK_UNLOCKED;
//spinlock_t rQLock = SPIN_LOCK_UNLOCKED;

//static DEFINE_SPINLOCK(sendLock);
//static DEFINE_SPINLOCK(recvLock);
//static DEFINE_SPINLOCK(sQLock);
//static DEFINE_SPINLOCK(rQLock);

UINT SendLastspace(UINT head,UINT tail)

{
	if(tail==head)
		return MAXSENDQL;
	else
		{
		if(head >tail)
	 		return (tail+MAXSENDQL-head)%MAXSENDQL;
		else
			return (tail-head);
		}
}
UINT RecvLastspace(UINT head,UINT tail)
{
	 return (head+MAXRECVQL-tail)%MAXRECVQL;
}

//	write fpga reg
 void write_BAR0(phys_addr_t  offset, unsigned int data)
{	
  	 iowrite32(data, (unsigned char *)pcie56_BAR0_Addr + offset);
}
//	read fpga reg
unsigned int read_BAR0(phys_addr_t  offset)
{
	 volatile unsigned int data = 0;
  	 data = ioread32((unsigned char *)pcie56_BAR0_Addr + offset);
 	 return data;
}
//	malloc the space for buffer
unsigned long dma_mem_alloc(int size)
{
	int order = get_order(size);
	return __get_dma_pages(GFP_KERNEL,order);
}

//	free the space for buffer
void dma_mem_free(void *buffer,int size)
{
	int order = get_order(size);
	return free_pages((unsigned long)buffer,order);
}
//	disable interrupt
void pcie56_int_disable() 
{
	unsigned int reg;
	reg = read_BAR0(DMA_INT_EN);
	reg &= 0x00000000;
	write_BAR0(DMA_INT_EN, reg);
}
//	enable interrupt
void pcie56_int_enable()
{
	unsigned int reg;
//		PRINTK("======><pcie56_int_enable> \n");
	reg = read_BAR0(DMA_INT_EN);
	reg = DMA_INT_ENALL | DMA_SND_EN | DMA_RCV_EN;
	write_BAR0(DMA_INT_EN, reg);
//		PRINTK("<======<pcie56_int_enable> \n");
}  

/*******************************************************************
 read the reg of other side driver recv
********************************************************************/
#define MAX_PACK_RECV_SAFE 100
int Other_Side_Recv(void)
{
	int ret;
	
	if(RHead >= RTail)
	{
		ret = RTail+MAXRECVQL-RHead;
	}
	else
	{
		ret = RTail-RHead;
	}
	
	return (ret > MAX_PACK_RECV_SAFE); 
}

#define MAX_PACK_SEND_SAFE 100

int Other_Side_Send(void)
{
	int ret;
	if(SHead >= STail)
	{
		ret = RTail+MAXRECVQL-RHead;
	}
	else
	{
		ret = RTail-RHead;
	}
	
	return (ret > MAX_PACK_SEND_SAFE); 
}

//	DMA0 Start
/***********************************************************************
		DMA0 
************************************************************************/
#define MAX_READ_BUSY_COUNT  100
int start_dma0(void)
{
	unsigned long u64addr;
	UINT ret;
	UINT  TDMA_Stat = 0;
	int i=0;
	u64addr =(UINT64)(sendlistPh+sizeof(struct send_descriptor)*STail);
	write_BAR0(DMA_SND_ADD32, (unsigned int)u64addr);	
	write_BAR0(DMA_SND_ADD64, (unsigned int)(u64addr>>32));
	ret = DMA_SND_START|DMA_SND_SG;
	write_BAR0(DMA_SND_CTRL,ret );


	TDMA_Stat = read_BAR0(DMA_SND_CTRL);        
	TDMA_Stat= TDMA_Stat&DMA_SND_BUSY;
	
	while((((read_BAR0(DMA_SND_CTRL))&DMA_SND_BUSY) == 0) && ((i++)<MAX_READ_BUSY_COUNT))
	{ 	
		
	}
	STail=(STail+1)%MAXSENDQL;	
	Slisttail = STail;
	return SUCCESS;
}
//	DMA1 Start
/***********************************************************************
			DMA1 
************************************************************************/
void start_dma1(void)
{
	unsigned long u64addr;
	UINT ret;
	u64addr = (unsigned long)(recvlistPh+sizeof(struct recv_descriptor)*Rlisthead);
	write_BAR0(DMA_RCV_ADD32, (unsigned int)u64addr);
	write_BAR0(DMA_RCV_ADD64, (unsigned int)(u64addr>>32));
	ret = DMA_RCV_START|DMA_RCV_SG;
	write_BAR0(DMA_RCV_CTRL, ret);
}
/***********************************************************************
	
************************************************************************/
void recv_encrypt_data(void)
{	




wake_up_interruptible(&recvinq);

/*	int  ret,length;
	
	ret = *(unsigned int *)(&RcvQ[Rlisthead].Buffer[0]);
	length = recv_list[Rlisthead].status;	
	Change_BELE((unsigned char *)&length);
	Change_BELE((unsigned char *)&ret);
	ret &= 0xff;
	length &= 0x7fffffff;
	//PRINTK("<recv_encrypt_data>:in the func!\n");
	if(ret > 0x02)
	{
			memcpy(RcvQ[Rlisthead].Buffer,Rencrycp[Renhead].Buffer,length);
			recv_list[Rlisthead].RecvFlagtag = 1;
			Rencrycp[Renhead].len = length;
			Renhead = (Renhead+1)%MAXENCRYPT;
			//PRINTK("<recv_encrypt_data>:after copy to enBuffer!\n");
			//if (async_queue)
			//	kill_fasync(&async_queue, SIGIO, POLL_IN);
	
	}
	*/
}
/**********************************************************************
 			tasklet 定义
**********************************************************************/
void do_int_tasklet_recv(unsigned long tc)
{
	recv_encrypt_data();
}

/**********************************************************************
			fasync 初始化定义
**********************************************************************/
/*
int pcie56_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &async_queue);
}
*/
/**********************************************************************
			中断处理函数
**********************************************************************/
irqreturn_t pcie56Drv_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{

	int IntStat= 0;
	//int pkt_id = 0;
	//int pkt_ids = 0;
	pcie56_int_disable();
	IntStat = read_BAR0(DMA_INT_STAT);
	write_BAR0(DMA_INT_STAT, IntStat);
    	pcie56_int_enable();

#if 0
	if((IntStat & DMA_INT_ALL) == 0){
		return ERR_IRQ_NONE;
	}
#endif
	
	if(IntStat & DMA_SND_INT){
	//PRINTK("<pcie56_interrupt_send>:send complete interrupt!\n");
		while(!(send_list[STail].NextDesc_low&SND_LIST_END)){	
			//pkt_ids = *(unsigned int *)(SndQ[STail].Buffer+44);
			//if(pkt_ids != SCount)
				{
			//	 PRINTK("****************<INT   write> SCount  is %d    pkt_id  is %d SHead is %d \n",SCount,pkt_ids,STail);
				}
			//SCount++;
	
			STail=(STail+1)%MAXSENDQL;	
		}
		
	//	SCount++;
	//	pkt_ids = *(unsigned int *)(SndQ[0].Buffer+44);
		//	if(pkt_ids != SCount)
			//	{
		//		 PRINTK("****************<INT   write> SCount  is %d    pkt_id  is %d \n",SCount,pkt_ids);
		//		}
		
		
		PRINTK("<pcie56_interrupt_send>:Slisttail is : %d\n",STail);
		//spin_lock_bh(&sendLock);
		if((SHead != STail)&&(0==(read_BAR0(DMA_SND_CTRL)&DMA_SND_BUSY))){
			start_dma0();
		}
		else
		{
			Dma_ready = 1;
		}
		//spin_unlock_bh(&sendLock);
		wake_up_interruptible(&sendoutq);
		
	}


	if(IntStat & DMA_RCV_INT){ 
		IntStat = RHead;
		Recv_count =0;
		//PRINTK("<pcie56_interrupt_recv>:recv complete interrupt!\n");
		while(recv_list[Rlisthead].status&DMA_RCV_LIST_FLAG){
			//pkt_id =  *(unsigned int *)(RcvQ[Rlisthead].Buffer+40);
		//	if(Recv_Flag != pkt_id)
		//	{
			//PRINTK("<*****INT*****> RHead is %d Recv_count is %d \n",Rlisthead,Recv_Flag);
		//	}
			//tasklet_schedule(&int_tasklet_recv);
			Rlisthead = (Rlisthead + 1)%MAXRECVQL;
			//Recv_Flag = (Rlisthead < Rlisttail);
			Recv_count ++;
		}
		//Rlisthead = (Rlisthead +MAXRECVQL- 1)%MAXRECVQL;
		//Rlisthead = (Rlisthead + 1)%MAXRECVQL;		
		RHead = Rlisthead; 
		Recv_Flag = (Rlisthead != Rlisttail);
		PRINTK("<pcie56_interrupt_recv>:old:%d,new:%d,count:%d,flag:%d!\n",IntStat,RHead,Recv_count,Recv_Flag);
		wake_up_interruptible(&recvinq);
	}
	return IRQ_HANDLED;
}

/***********************************************************************
	ioctl函数: 主要有加解密数据处理，配置FPGA，	
			     查询FPGA状态
************************************************************************/
#ifdef Server
 long pcie56_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
int pcie56_ioctl(struct inode *in, struct file *filp, unsigned int cmd ,unsigned long arg)
#endif
{
	//PRINTK("<pcie56_ioctld>:into  ioctl   !\n");
	int ret;
	int slen;
	int nolen;
	unsigned int TDMA_Stat;
	switch(cmd){

	case READ_DATA:
			if (down_interruptible(&en_sem)) 
				{ return -EAGAIN;
				 PRINTK("<pcie56_write>: get sem error!\n");
				}
			if(Rentail == Renhead){
				up(&en_sem);
				return  -EAGAIN;
				}
		PRINTK("<pcie56_ioctl_read>:before copy to user!\n");
			SetBuffer_BYTE_ChgBELE(RcvQ[RTail].Buffer,Rencrycp[Rentail].len);
			ret = __copy_to_user((char*)arg, Rencrycp[Rentail].Buffer, Rencrycp[Rentail].len);
			if(ret){
					PRINTK("<pcie56_ioctl_read>:copy to user error!\n");
					up(&en_sem);
					return -ENOMEM;		
	   	 		}
			//PRINTK("Rencrycp[%d].len is %d!\n",Rentail,Rencrycp[Rentail].len);
			ret = Rencrycp[Rentail].len;
			Rentail = (Rentail+1)%MAXENCRYPT;
			up(&en_sem);
			break;
	case WRITE_DATA:	
			if (down_interruptible(&write_sem))
				{
				 PRINTK("<pcie56_write>: get sem error!\n");
				 return -EAGAIN;
				
				}
			if((SHead+1)%MAXSENDQL == STail)
				{
				 PRINTK("<pcie56_write>: no space to write!\n");
				up(&write_sem);
				return -EAGAIN;
				}
			 slen = *(unsigned int *)(arg+4);
			 slen &= 0x0000ffff;
			 nolen = slen;
			 slen = ((slen + 7) / 8) * 8;
			if (GLOBALMEM_SIZE < slen){
				PRINTK("<pcie56_ioct_writel>: write len is too long, Write Data: %d\n", slen);
				up(&write_sem);
				return -EINVAL;
			}
			PRINTK("<pcie56_ioctl_write>:before copy from user!\n");
		        // then copy the data filed
		        if ( copy_from_user(SndQ[SHead].Buffer,(char *)arg,nolen)){
		        	 PRINTK("<pcie56_write>: copy from user error!\n");
				up(&write_sem);
				return -ENOMEM;	   
		        }
			 SndQ[SHead].len = slen;
			SetBuffer_BYTE_ChgBELE(SndQ[SHead].Buffer,nolen);	
			Change_BELE((char *)&SndQ[SHead].len);	
			send_list[SHead].length = SndQ[SHead].len;
			send_list[SHead].NextDesc_low = send_list[SHead].NextDesc_low & SND_LIST_END;
			SHead = (SHead + 1) % MAXSENDQL;
			up(&write_sem);
			TDMA_Stat = read_BAR0(DMA_SND_CTRL);
			PRINTK("<ioctl_write>:TDMA_Stat is 0x%08x\n",TDMA_Stat);           
			TDMA_Stat = TDMA_Stat&DMA_SND_BUSY;
			if(TDMA_Stat==0){
				start_dma0();
				}
			ret = nolen;
			break;
	case READ_VERSION:
			*(unsigned int *)arg = read_BAR0(FPGA_SOFT_VERISON);
			ret= SUCCESS;
			break;
	case SFIO_FLAG:
			*(unsigned int *)arg = 0;
			if((SHead+1)%MAXSENDQL!=STail)
			{
				*(unsigned int *)arg |= 0x01;
				 PRINTK("<pcie56_ioctl>: sf send buffer can  write !\n");
			}
			if(Renhead != Rentail)
			{
				*(unsigned int *)arg |= 0x02; 
				PRINTK("<pcie56_ioctl>: sf  recv buffer can  read !\n");
			}
			ret = SUCCESS;
			break;
	case FPGA_RESET:
			write_BAR0(FPGA_SW_RST,0x5a5a5a5a);
			 PRINTK("<pcie56_ioctl>:FPGA RESET !\n");
			ret = SUCCESS;
			break;
	default:
			ret = -ECHRNG;
			break;
	}
	return ret;
}

/***********************************************************************
		
************************************************************************/
ssize_t pcie56_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int len;//,i;					
	int ret;
	//int num_count;
	int datalen,length;
	printk(KERN_DEBUG"[pcie56_read]: into the function.\n");
	if (down_interruptible(&read_sem))
		return -EAGAIN;
	//printk(KERN_DEBUG"[pcie56_read]2: into the function.\n");
/*	while(recv_list[Rlisttail].RecvFlagtag==1){
		if( !(recv_list[Rlisttail].status&DMA_RCV_LIST_FLAG)){
			up(&read_sem);
			//PRINTK("1.there is no Data to Read!\n");
			return -EAGAIN;
		 	}
		recv_list[Rlisttail].RecvFlagtag = 0;
		recv_list[Rlisttail].status &= DMA_RCV_LIST_RESET;
		RTail = RTail + 1;
		if(RTail>1023)
			{
				RTail %= MAXRECVQL;
				Own_head = (Own_head&0x8000)+RTail +0x8000;
			}
		else
			Own_head = (Own_head&0x8000)+RTail;
		Rlisttail = RTail;
			write_BAR0(RECV_OWN_HEAD, (Own_head&0xffff));
		}
*/
	PRINTK("recv_list[%d].status is 0x%08x \n",Rlisttail,recv_list[Rlisttail].status);

	if( !(recv_list[Rlisttail].status&DMA_RCV_LIST_FLAG)){
		up(&read_sem);
		//PRINTK("2.there is no Data to Read!\n");
		return -EAGAIN;
		}
	if(!Recv_Flag)
	{
		PRINTK("READ data in, but not IRQ!\n");
		spin_lock_irq(&recvLock);
		Recv_count =0;
		ret = Rlisthead;
		//PRINTK("<pcie56_interrupt_recv>:recv complete interrupt!\n");
		while(recv_list[Rlisthead].status&DMA_RCV_LIST_FLAG){
			//pkt_id =  *(unsigned int *)(RcvQ[Rlisthead].Buffer+40);
		//	if(Recv_Flag != pkt_id)
		//	{
			//PRINTK("<*****INT*****> RHead is %d Recv_count is %d \n",Rlisthead,Recv_Flag);
		//	}
			//tasklet_schedule(&int_tasklet_recv);
			Rlisthead = (Rlisthead + 1)%MAXRECVQL;
			//Recv_Flag = (Rlisthead < Rlisttail);
			Recv_count ++;
		}
		//Rlisthead = (Rlisthead +MAXRECVQL- 1)%MAXRECVQL;
		//Rlisthead = (Rlisthead + 1)%MAXRECVQL;		
		RHead = Rlisthead; 
		Recv_Flag = (Rlisthead != Rlisttail);
		spin_unlock_irq(&recvLock);

		PRINTK("<pcie56_read>:old:%d,new:%d,count:%d,flag:%d!\n",ret,RHead,Recv_count,Recv_Flag);
		//up(&read_sem);
		//return -EAGAIN;
	}
	
//	SetBuffer_BYTE_ChgBELE(RcvQ[RTail].Buffer,8);
	
	length = recv_list[Rlisttail].status & DMA_RCV_LIST_RESET;
//	Change_BELE((unsigned char *)(&length));
	RcvQ[RTail].len = length;
	datalen = *(unsigned int *)(RcvQ[RTail].Buffer+4);
//	Change_BELE((unsigned char *)(&datalen));
	datalen &= 0xffff;
	
	/*if(((datalen+15)&0xfffffff0) != RcvQ[RTail].len)
		{
	/	for(i=0;i<64;i++)
			{
			PRINTK(" 0x%x ",RcvQ[RTail].Buffer[i]);
		}
		PRINTK("\n");
		
		PRINTK("<pcie56_read>  length is error!  datalen is %d , RcvQ[%d].len is %d \n",datalen,RTail,RcvQ[RTail].len);
		
		up(&read_sem);
		return -EAGAIN;

		}
	*/
	//PRINTK("******************<pcie56_read> Count  is %d ,    slen is %d \n",*(unsigned int *)(&RcvQ[RTail].Buffer[8]), RcvQ[RTail].len );
	//PRINTK("<pcie56_read>:len is 0x%08x  the paket is 0x%08x   RTail is  0x%08x\n",RcvQ[RTail].len,*(unsigned int *)(RcvQ[RTail].Buffer+44),RTail);
	//RcvQ[RTail].len = RcvQ[RTail].len >>16 ;
	PRINTK("RcvQ[%d].len is 0x%08x\n",RTail,RcvQ[RTail].len);
	if(RcvQ[RTail].len>FRAMELEN) 
	{
	PRINTK("<pcie56_read>:RcvQ[RTail].len too large %d\n",RcvQ[RTail].len);
		up(&read_sem);
		return -EIO;
	}
	datalen-=8;
	if(datalen > FRAMELEN)
	{
		PRINTK("<pcie56_read>:datalen too large %d\n",datalen);
			up(&read_sem);
			return -EIO;
	}
	ret = __copy_to_user(buf, &RcvQ[RTail].Buffer[8],datalen);
	if(ret){
		up(&read_sem);
		PRINTK("<pcie56_read>:copy to user error! ret is %d \n",ret);
		return -EIO;		
    		}
	//num_count = *(unsigned int *)(RcvQ[RTail].Buffer+40);
	//if(num_count != Read_FLag)
	//	{
	//	PRINTK("Rtail is %d,  num_count is %d  , Read_Flag is %d \n",RTail,num_count,Read_FLag);
	//}
	//Read_FLag ++;
	//PRINTK("******************<pcie56_read> Count  is %d ,    slen is %d \n",*(unsigned int *)(&RcvQ[RTail].Buffer[8]), RcvQ[RTail].len );
	//PRINTK("<pcie56_read>:len is 0x%08x  the paket is 0x%08x   RTail is  0x%08x\n",RcvQ[RTail].len,*(unsigned int *)(RcvQ[RTail].Buffer+44),RTail);
	
	spin_lock_irq(&recvLock);
	recv_list[Rlisttail].status = DMA_RCV_LIST_RESET;
	len = RcvQ[RTail].len;
	memset(RcvQ[RTail].Buffer,0,len);
	RTail = (RTail + 1)%MAXRECVQL;
	Rlisttail = RTail;
	Recv_Flag = (Rlisthead != Rlisttail);
	spin_unlock_irq(&recvLock);
	
	//write_BAR0(RECV_OWN_HEAD, (Own_head&0xffff));
	//datalen = datalen -32;
	up(&read_sem);
	PRINTK("<======================<PCIe_read> datalen:%d, new RTail:%d,len:%d,flag:%d\n",datalen,RTail,len,Recv_Flag);
	return datalen;
}
/***********************************************************************
		
************************************************************************/
u32 buffCount=0;
ssize_t pcie56_write(struct file * filp,const char __user * buf,size_t count,loff_t * f_pos)
{	
 	int ret = 0;
    	//u32 i = 0;
	u32 slen;
	//int pkt_id;
	UINT  TDMA_Stat = 0;
	if (down_interruptible(&write_sem)) 								{
	
		printk("pcie56_write line:%d\n",__LINE__);
		return - EAGAIN;	
	}
#if 1	
	ret=Other_Side_Recv();
	if(ret <= 0)
	{
		up(&write_sem);
		PRINTK("<pcie56_write>:Other_Side_Recv fail, head:%d, tail:%d!",RHead ,RTail );
		return -EAGAIN;
	}

	ret = Other_Side_Send();
	if(ret <= 0)
	{
		up(&write_sem);
		PRINTK("<pcie56_write>:Other_Side_Send fail, head:%d, tail:%d!",SHead ,STail );
		return -EAGAIN;
	}
#endif
#if 0
	TDMA_Stat = read_BAR0(DMA_SND_CTRL);        
	//printk("read DMA_SND_CTRL:0x%x,data:0x%x\n",DMA_SND_CTRL,TDMA_Stat);
	TDMA_Stat= TDMA_Stat&DMA_SND_BUSY;
	//spin_lock_bh(&sendLock);
	if(TDMA_Stat != 0)
	{
		//PRINTK("<pcie56_write>:TDMA_Stat is working\n");
		printk("pcie56_write line:%d\n",__LINE__);
		up(&write_sem);
		return -EAGAIN;
	
	}
#endif
	slen=count+8;
	printk("count:%x slen:%x\n",count,slen);


	*(unsigned int *)(&SndQ[SHead].Buffer[0]) = ((0xd6fa<<16)|((buffCount++)&0xffff));
	//*(unsigned int *)(&SndQ[0].Buffer[4]) = (slen<<16)+slen;
	*(unsigned int *)(&SndQ[SHead].Buffer[4]) = slen&0xffff;

	slen = (slen + 7)&0xfffffff8 ;
	if (GLOBALMEM_SIZE < slen)
	{
		up(&write_sem);
		printk("pcie56_write line:%d\n",__LINE__);
		return -EIO;
	}
	//	printk("pcie56_write line:%d count:%d\n",__LINE__,count);
	 ret = __copy_from_user(&SndQ[SHead].Buffer[8], buf, count);
	 if(ret)
    	 {	up(&write_sem);
		PRINTK("<pcie56_write>:copy from user error!\n");
		memset(SndQ[SHead].Buffer, 0, FRAMELEN);
		printk("pcie56_write line:%d count:%d\n",__LINE__,count);
		return -EIO;		
       }
	//pkt_id = *(unsigned int *)(SndQ[0].Buffer+40);
	//if(pkt_id != Count)
	//	{

	//	 PRINTK("****************<pcie56_write> Count  is %d    pkt_id  is %d \n",Count,pkt_id);
	///	}
	Count++;
	
	PRINTK("****************<pcie56_write> Count  is %d  datalen is %d  slen is %d  SHead:%d \n",Count,count,slen,SHead);
//	SetBuffer_BYTE_ChgBELE(SndQ[0].Buffer,8);	
	SndQ[SHead].len = slen;
//	Change_BELE((char *)&SndQ[0].len);
	spin_lock_irq(&sendLock);
	send_list[SHead].length = SndQ[SHead].len;
	send_list[SHead].NextDesc_low |=SND_LIST_END ;
	send_list[(SHead+MAXSENDQL-1)%MAXSENDQL].NextDesc_low &=(SND_LIST_END^0xffffffff);
	Slisthead = SHead = (SHead + 1) % MAXSENDQL;

	
   	TDMA_Stat = read_BAR0(DMA_SND_CTRL);        
	TDMA_Stat= TDMA_Stat&DMA_SND_BUSY;
	//spin_lock_bh(&sendLock);
	if(TDMA_Stat==0)
	{
		start_dma0();
	}
	else
	{
	//	up(&write_sem);
	//	return -EAGAIN;
	}
	spin_unlock_irq(&sendLock);

	up(&write_sem);
		printk("pcie56_write len:%d,count:%dline:%d\n",slen,count,__LINE__);
	//spin_unlock_bh(&sendLock);
    	return count;
	
}

/***********************************************************************
	poll

************************************************************************/
unsigned int pcie56_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	int ret,ret2;
	/*
	 * The buffer is circular; it is considered full
	 * if "wp" is right behind "rp" and empty if the
	 * two are equal.
	 */
	//PRINTK("<pcie56_poll>:in poll" );
	//spin_lock_bh(&lock);
	poll_wait(filp, &sendoutq,  	wait);
	poll_wait(filp, &recvinq, 	wait);
	
	//PRINTK("<pcie56_poll>: SHead is %d  STail  is %d   RHead is  %d   RTail  is  %d  Renhead is  %d   Rentail  is  %d\n",SHead,STail,RHead,RTail,Renhead,Rentail);
	//if((SHead+1)%MAXSENDQL!=STail)

	//if (down_interruptible(&read_sem)==0) 								
	{
		//PRINTK("<pcie56_poll>: pollin  select!\n");
		if( recv_list[Rlisttail].status & DMA_RCV_LIST_FLAG )
			{
				mask |= POLLIN|POLLRDNORM;
				//PRINTK("<pcie56_poll>:can read!\n" );
			}
			else
			{
				PRINTK("<pcie56_poll>:can not read! head:%d, tal:%d\n", RHead ,RTail);
			}
		//up(&read_sem);
	}

	//if (down_interruptible(&write_sem)==0) 									  
	{
		ret = Other_Side_Recv();
		ret2 = Other_Side_Send();

		//PRINTK("<pcie56_poll>:write ret is %d!\n",ret );
		if((ret>0) && (ret2>0))
		{
			mask |=  POLLOUT|POLLWRNORM;
			//PRINTK("<pcie56_poll>:can write!\n" );
		}
		else
		{
			PRINTK("<pcie56_poll>:Other_Side_Recv status: head:%d, tail:%d; Other_Side_Send status: head:%d, tail:%d!\n",RHead,RTail,SHead,STail);
		}

		//up(&write_sem);
	}

	//spin_unlock_bh(&lock);
	//PRINTK("<******pcie56_poll*********>:befor return ! mask is 0x%x \n",mask );
	return mask;
}
/***********************************************************************
	open dev

************************************************************************/
int pcie56_open(struct inode *inode, struct file *filp)
{
  	//PRINTK("<pcie56_open>***************>\n");
	if (pcie56 == NULL){
		//PRINTK("<pcie56_open>: pcie56 open error\n");
		return -1;
	}	
	filp->f_op = &pcie56Drv_fops;
	pcie56_opens++;
	Read_FLag = 0;
	//Recv_Flag = 0;
	Count = 0;
	SCount = 0;
//	enable interrupt
//	pcie56_int_enable();	
	PRINTK("open success!\n");
	return 0;
}

/***********************************************************************
	close dev
************************************************************************/
int pcie56_release(struct inode *inode, struct file *filp)
{
	pcie56_opens--;
	if(pcie56_opens == 0)
	{
		//pcie56_fasync(-1, filp, 0);		//关闭信号通道
	}
	return 0;
}
/***********************************************************************
	setup cdev
************************************************************************/
int pcie56Drv_setup_cdev(struct cdev *dev, int index)
{
	int err;
	/*注册字符设备*/
	int devno = MKDEV(pcie56_major, index);
	cdev_init(dev, &pcie56Drv_fops);
	dev->owner = THIS_MODULE;
	dev->ops = &pcie56Drv_fops;
	err = cdev_add (dev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		PRINTK("Error %d adding pcie56Drv%d\n", err, index);
	pcie56_class = class_create(THIS_MODULE,"pcie56_udev");
	device_create(pcie56_class,NULL,devno,NULL,"pcie56");
	return err;
}

/***********************************************************************
INIT

************************************************************************/
static int __init pcie56Drv_init(void)
{
	char *dma_buffer = NULL;          //global buffer for dma
	char *Sendlist = NULL;
	char *Recvlist = NULL;
	char *encrypt = NULL;
	int i,ret;
	phys_addr_t   base0start = 0;			
	unsigned int base0len = 0;
	phys_addr_t   endsrc0 = 0;
	dev_t dev;
	PRINTK("###################################################\n");
       PRINTK("<pcie56Drv_init> enter into drv Init!\n");
	//init wait queue and lock for poll
	init_waitqueue_head(&sendoutq);
	init_waitqueue_head(&recvinq);
	spin_lock_init(&lock);
	spin_lock_init(&sendLock);
	spin_lock_init(&lock);

	pcie56=pci_get_device(VENDOR_ID , DEVICE_LS_ID ,NULL);

	if (!pcie56) {
		pcie56=pci_get_device(VENDOR_ID , DEVICE_Np1_ID ,NULL);
		if (!pcie56) {	
			pcie56=pci_get_device(VENDOR_ID , DEVICE_Np2_ID ,NULL);
			if (!pcie56) {
				PRINTK("<pcie56Drv_init>: No pcie56Drv Card Found!\n");
				return FAULT;
			}
			else{
				 LocalID = NP2_ID;
				 DstID = NP1_ID;
				PRINTK("<pcie56Drv_init>: pcie56Drv Card Found!The DeviceID is 0x%x\n",DEVICE_Np2_ID);
			}
		}
		else{
			 	 LocalID = NP1_ID;
				 DstID = NP2_ID;
			PRINTK("<pcie56Drv_init>: pcie56Drv Card Found!The DeviceID is 0x%x\n",DEVICE_Np1_ID);
		}
	
	}
	else{
		PRINTK("<pcie56Drv_init>: pcie56Drv Card Found!The DeviceID is 0x%x\n",DEVICE_LS_ID);
	}
	
	if (pci_enable_device(pcie56)){
		PRINTK("<pcie56Drv_init>:pci_enable_device err! return -EIO\n");
		return -EIO;
	}
	/* base 0 */
	base0start=pci_resource_start(pcie56,0);
	endsrc0=pci_resource_end(pcie56,0);
	base0len=endsrc0-base0start;
	request_mem_region(base0start,base0len,"pcie56");
	pcie56_BAR0_Addr = ioremap(base0start,base0len);   
	PRINTK("<pcie56_init>: base0 addr Range 0x%08X - 0x%08X Length:0x%16X\n", (int)base0start, (int)(base0start + base0len), 
	base0len);
//	alloc register device
	ret = alloc_chrdev_region(&dev, 0, 1, "pcie56");
	pcie56_major = MAJOR(dev);
/*  
	// dev= MKDEV(pcie56_major, 0);
	//ret = register_chrdev_region(dev,1,"pcie56");
*/
	if (ret <0){
		PRINTK(" register pcie56Drv device number error\n");
		return ret;
	}
	ret = pcie56Drv_setup_cdev(&pcie56_cdev, 0);
	if (ret){
		PRINTK("<pcie56_init>: pcie56Drv_setup_cdev Error return 0x%08X\n", ret);
	}
 	PRINTK("<pcie56Drv_init>: pcie56_cdev major is %d. \n", pcie56_major);

//	DMA Configure    
	//pcie56_int_disable();
	ret=pci_set_dma_mask(pcie56, DMA_BIT_MASK(64));
	 if(ret!=0){
		PRINTK("<pcie56_init>: set mask Error return 0x%08X\n", ret);
	 }
  	ret= pci_set_consistent_dma_mask(pcie56, DMA_BIT_MASK(64));
	 if(ret!=0){
		PRINTK("<pcie56_init>: set mask 2 Error return 0x%08X\n", ret);
	 }
   	 pci_set_master(pcie56);


//	alloc DMA buffer for send Queue;
	dma_buffer = (void *)dma_mem_alloc(DMA_FIFO_SIZE*MAXSENDQL);
	memset(dma_buffer,0,DMA_FIFO_SIZE*MAXSENDQL);
	SndQ[0].BufferPh = dma_map_single((struct device *)pcie56,dma_buffer,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	PRINTK("<pcie56Drv_init>: SEND Dma Buffer is Create %08x\n,SndQ[0].BufferPh is %08x \n",(int)dma_buffer,(int)SndQ[0].BufferPh);
//	PRINTK("<pcie56Drv_init>: SndQ[0].BufferPh is Create 0x%08x\n",(int)SndQ[0].BufferPh);
	if(!dma_buffer){
		PRINTK("<pcie56Drv_init>: Malloc Send buffer ERROR!\n" );
		goto fail_alloc_sendq;
	}
	for (i=0;i<MAXSENDQL;i++){
		SndQ[i].Buffer = dma_buffer+i*FRAMELEN;
		SndQ[i].BufferPh = SndQ[0].BufferPh + i*FRAMELEN;
//		SndQ[i].BufferPh = virt_to_bus(SndQ[i].Buffer);
		SndQ[i].len=0;
	}
//	initialize send S/G descriptor list
	Sendlist = (void *)dma_mem_alloc(sizeof(struct send_descriptor)*MAXSENDQL);
//	Sendlist = pci_alloc_consistent(pcie56, sizeof(struct send_descriptor)*MAXRECVQL ,&sendlistPh);
	if(!Sendlist){
		printk(KERN_ERR "<pcie56_init_init>: Malloc Recv list buffer ERROR!\n" );
		goto fail_alloc_sendlist;
	}
	memset(Sendlist,0,sizeof(struct send_descriptor)*MAXSENDQL);
	sendlistPh=dma_map_single((struct device *)pcie56,Sendlist,sizeof(struct send_descriptor)*MAXSENDQL,DMA_BIDIRECTIONAL);
//	sendlistPh= virt_to_bus(Sendlist);

	printk("<pcie56_init_init>: sendlistBufferPh = 0x%x\n",(int)sendlistPh);
	printk("<pcie56_init_init>: sendlist_Buffer = 0x%x\n",(int)Sendlist);
	send_list = (struct send_descriptor *)Sendlist;
	for (i=0;i<MAXSENDQL-1;i++){
		send_list[i].length = 0;
		send_list[i].PhAddr_low = (UINT)SndQ[i].BufferPh;
		send_list[i].PhAddr_hig =((UINT64)SndQ[i].BufferPh)>>32;
		send_list[i].NextDesc_low = sendlistPh+ (i+1)*sizeof(struct send_descriptor);
		send_list[i].NextDesc_hig = (sendlistPh+ (i+1)*sizeof(struct send_descriptor))>>32;
		send_list[i].SendFlagtag = 0;
		send_list[i].SendFlagtag1 = 0;
		send_list[i].SendFlagtag2 = 0;
	}
	send_list[i].length = 0;
	send_list[i].PhAddr_low = (UINT)SndQ[i].BufferPh;
	send_list[i].PhAddr_hig = ((UINT64)SndQ[i].BufferPh)>>32;
	send_list[i].NextDesc_low = sendlistPh ;
	send_list[i].NextDesc_hig = ((UINT64) sendlistPh)>>32;
	send_list[i].SendFlagtag = 0;
	send_list[i].SendFlagtag1 = 0;
	send_list[i].SendFlagtag2 = 0;
	
//	alloc DMA buffer for recv Queue
	dma_buffer = (void *)dma_mem_alloc(DMA_FIFO_SIZE*MAXRECVQL);
	memset(dma_buffer,0,DMA_FIFO_SIZE*MAXRECVQL);
	RcvQ[0].BufferPh = dma_map_single((struct device *)pcie56,dma_buffer,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
	PRINTK("<pcie56Drv_init>: RECV Dma Buffer is Create %x\n,RcvQ[0].BufferPh is %0x \n",(int)dma_buffer,RcvQ[0].BufferPh);
//	PRINTK("<pcie56Drv_init>: RcvQ[0].BufferPh is Create 0x%08x\n",(int)RcvQ[0].BufferPh);
	if(!dma_buffer){
		PRINTK(KERN_ERR "<pcie56Drv_init>: Malloc Recv buffer ERROR!\n" );
		goto fail_alloc_recvq;
	}
	
	for (i=0;i<MAXRECVQL;i++){ 
	  	RcvQ[i].Buffer = dma_buffer+i*FRAMELEN;
		RcvQ[i].BufferPh = RcvQ[0].BufferPh+i*FRAMELEN;
//		RcvQ[i].BufferPh = virt_to_bus(RcvQ[i].Buffer);
		RcvQ[i].len=0;
	}
//	 initialize recv S/G descriptor list
	Recvlist = (void *)dma_mem_alloc(sizeof(struct recv_descriptor)*MAXRECVQL);
//	Recvlist = pci_alloc_consistent(pcie56, sizeof(struct recv_descriptor)*MAXRECVQL ,&recvlistPh);
	if(!Recvlist){
		printk(KERN_ERR "<pcie56_init_init>: Malloc Recv list buffer ERROR!\n" );
		goto fail_alloc_recvlist;
		}
	memset(Recvlist,0,sizeof(struct recv_descriptor)*MAXRECVQL);
	recvlistPh=dma_map_single((struct device *)pcie56,Recvlist,sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
//	recvlistPh= virt_to_phys(Recvlist);
	printk("<NP1_init>: recvlistBufferPh = 0x%x\n",(int)recvlistPh);
	printk("<pcie56_init>: recvlist_Buffer = 0x%x\n",(int)Recvlist);
	recv_list= (struct recv_descriptor *)Recvlist;
	for (i=0;i<MAXRECVQL-1;i++){
		recv_list[i].status = 0;
		recv_list[i].PhAddr_low = (UINT)RcvQ[i].BufferPh;
		recv_list[i].PhAddr_hig = ( (UINT64)RcvQ[i].BufferPh)>>32;
		recv_list[i].NextDesc_low =(recvlistPh + ((i+1)*sizeof(struct recv_descriptor)));	
		recv_list[i].NextDesc_hig =  ((UINT64)(recvlistPh + ((i+1)*sizeof(struct recv_descriptor))))>>32;	
		recv_list[i].RecvFlagtag = 0;
		recv_list[i].RecvFlagtag1 = 0;
		recv_list[i].RecvFlagtag2 = 0;
		}
	recv_list[i].PhAddr_low = (UINT)RcvQ[i].BufferPh;
	recv_list[i].PhAddr_hig = ((UINT64)RcvQ[i].BufferPh)>>32;
	recv_list[i].status =0;
	recv_list[i].NextDesc_low =recvlistPh;
	recv_list[i].NextDesc_hig = ((UINT64) recvlistPh)>>32;
	recv_list[i].RecvFlagtag = 0;
	recv_list[i].RecvFlagtag1 = 0;
	recv_list[i].RecvFlagtag2 = 0;
	//alloc encrypt Buffer
	encrypt= (void *)dma_mem_alloc(DMA_FIFO_SIZE*MAXENCRYPT);
	memset(encrypt,0,DMA_FIFO_SIZE*MAXENCRYPT);
	PRINTK("<pcie56Drv_init>: RECV Dma Buffer is Create %x\n ",(int)encrypt);
//	PRINTK("<pcie56Drv_init>: RcvQ[0].BufferPh is Create 0x%08x\n",(int)RcvQ[0].BufferPh);
	if(!encrypt){
		PRINTK(KERN_ERR "<pcie56Drv_init>: Malloc Recv buffer ERROR!\n" );
		goto fail_alloc_encrypt;
	}
	for (i=0;i<MAXENCRYPT;i++){ 
	  	Rencrycp[i].Buffer = encrypt+i*FRAMELEN;
//		Rencrycp[i].BufferPh = RcvQ[0].BufferPh+i*FRAMELEN;
		Rencrycp[i].BufferPh = virt_to_bus(Rencrycp[i].Buffer);
		Rencrycp[i].len=0;
	}
	//SetBuffer_BYTE_ChgBELE(recv_list, sizeof(struct recv_descriptor)*MAXRECVQL);
	//SetBuffer_BYTE_ChgBELE(send_list, sizeof(struct send_descriptor)*MAXSENDQL);
//	Init Send Queue head and tail, Receive Queue head and tail
	init_MUTEX(&en_sem);
	init_MUTEX(&write_sem);
	init_MUTEX(&read_sem);
	SHead = 0;
	STail = 0;
	RHead = 0;
	RTail = 0;
	Slisthead = 0;
	Slisttail = 0;
	Rlisthead = 0;
	Rlisttail = 0;
	Renhead = 0;				
   	Rentail = 0;		
//	interrupt register/disable/enable
	ret = request_irq(pcie56->irq,
			pcie56Drv_interrupt,
			IRQF_SHARED,
			"pcie56",
			pcie56);	  
	if(ret) {
			PRINTK("<pcie56Drv_init>: Could not register interrupt\n");
			goto fail_req_irq;
		} 
	else {
			PRINTK("<pcie56Drv_init>: Register interrupt successful,irq 0x%lx\n",(unsigned long )pcie56->irq);
		}
	ret = read_BAR0(FPGA_SOFT_VERISON);
	//PRINTK("FPGA SOFTWARE VERISON is %08x\n",ret);
	PRINTK("FPGA SOFTWARE VERISON is %02d%02d%02d%02d%02d%02d\n",((ret>>17)&0x3f),((ret>>23)&0xf),((ret>>27)&0x1f),((ret>>12)&0x1f),((ret>>6)&0x3f),((ret>>0)&0x3f));
#if 0
	for(i=0x80;i<=0x17c;i+=4)
	{
	ret = read_BAR0(i);
        PRINTK("FPGA test read user reg:0x%x is 0x%08x\n",i,ret);
	write_BAR0(i,i*5);	
        PRINTK("FPGA test write user reg:0x%x is 0x%08x\n",i,i*5);
	ret = read_BAR0(i);
        PRINTK("FPGA test read2 user reg: 0x%x is 0x%08x\n",i,ret);
	}
#endif
	pcie56_int_enable();
	
 	PRINTK("<pcie56Drv_init>: pcie56_cdev major is %d. \n", pcie56_major);
	start_dma1(); 
	return SUCCESS;

fail_req_irq:
	 dma_mem_free(encrypt, FRAMELEN*MAXSENDQL);
fail_alloc_encrypt:
	 dma_unmap_single((struct device *)pcie56,recvlistPh,sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
	 dma_mem_free(recv_list,sizeof(struct recv_descriptor)*MAXRECVQL);
fail_alloc_recvlist:
	 dma_unmap_single((struct device *)pcie56, RcvQ[0].BufferPh,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
	 dma_mem_free(RcvQ[0].Buffer, FRAMELEN*MAXSENDQL);
fail_alloc_recvq:
	 dma_unmap_single((struct device *)pcie56,sendlistPh,sizeof(struct send_descriptor)*MAXSENDQL,DMA_BIDIRECTIONAL);
	 dma_mem_free(send_list,sizeof(struct send_descriptor)*MAXRECVQL);
fail_alloc_sendlist:
	 dma_unmap_single((struct device *)pcie56,SndQ[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 dma_mem_free(SndQ[0].Buffer, FRAMELEN*MAXSENDQL);
fail_alloc_sendq:
 	 device_destroy(pcie56_class, pcie56_cdev.dev);
	 class_destroy(pcie56_class);
	 cdev_del(&pcie56_cdev);
	 unregister_chrdev_region(dev, 1);
	return FAULT;
}

/***********************************************************************
	设备注销接口
***********************************************************************/
static void __exit pcie56Drv_cleanup(void)
{
	pcie56_opens = 0;
	
	if(pcie56 != NULL){
		
		 dma_unmap_single((struct device *)pcie56,recvlistPh,sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
		 dma_unmap_single((struct device *)pcie56, RcvQ[0].BufferPh,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
		 dma_unmap_single((struct device *)pcie56,sendlistPh,sizeof(struct send_descriptor)*MAXSENDQL,DMA_BIDIRECTIONAL);
		 dma_unmap_single((struct device *)pcie56,SndQ[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	
		 dma_mem_free(Rencrycp[0].Buffer, FRAMELEN*MAXENCRYPT);
		 dma_mem_free(SndQ[0].Buffer, FRAMELEN*MAXSENDQL);
		 dma_mem_free(RcvQ[0].Buffer, FRAMELEN*MAXRECVQL);
		 dma_mem_free(recv_list,sizeof(struct recv_descriptor)*MAXRECVQL);
		 dma_mem_free(send_list,sizeof(struct send_descriptor)*MAXSENDQL);
		 pcie56_int_disable();
	 	 write_BAR0(DMA_RCV_CTRL,DMA_RCV_ABORT);
	
		 free_irq(pcie56->irq, pcie56);
		 write_BAR0(FPGA_SW_RST,0x5a5a5a5a);
 	 	 device_destroy(pcie56_class, pcie56_cdev.dev);
		 class_destroy(pcie56_class);
		 cdev_del(&pcie56_cdev);
		 unregister_chrdev_region(MKDEV (pcie56_major, 0), 1);  
		  iounmap(pcie56_BAR0_Addr);
		 pci_dev_put(pcie56);
		
	}
	//write_BAR0(DMA_RCV_CTRL, DMA_RCV_ABORT);
	PRINTK("<pcie56Drv_cleanup> over!\n");
	PRINTK("###################################################\n");
}

/*module init */
module_init(pcie56Drv_init);
/* module_exit*/
module_exit(pcie56Drv_cleanup);

