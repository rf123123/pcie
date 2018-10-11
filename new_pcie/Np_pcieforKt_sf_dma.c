/****
Data:   2015.9.22
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
#include <linux/kthread.h>
#include <linux/time.h>


#define KERNEL_SLEEP 1


/*
#include <asm/octeon/octeon-ethernet-user.h>
#undef OCTEON_MODEL
#define USE_RUNTIME_MODEL_CHECKS 1
#include <asm/octeon/cvmx.h>
#include <asm/octeon/cvmx-wqe.h>
*/
//#include "pcie56Drv_v1.0.h"
#define READ_DATA 		0x00080001	//sf read data	interface
#define WRITE_DATA 		0x00080002	//sf write data interface
#define READ_VERSION 	0x00080003	//read fpga software verison interface
#define FPGA_RESET		0x00080004	//fpga software reset interface
#define FPGA_CONFIG		0x00080005	//fpga config	interface
#define SFIO_FLAG		0x00080006	//slect sf zone interface
#define CHANEL_CONFIG	0x00080007
#define ECC_TRXSET0		0x00080008
#define ECC_TRXSET1		0x00080009
#define FPGA_DESTROY	0x0008000a


#define DEVICE_COUNT 1   //6
#define GLRZ_MAX   0x11;

#define Version2

//#define Server 0
#define DEBUG

#define PCIE_INT 0


#ifdef DEBUG
#define PRINTK(args...) printk(args)
#else
//#define for(;;)             do{}while(0)
#define PRINTK(args...) do{}while(0)
#endif

#define uchar unsigned char

#define MIN(a,b)                   (((a)<(b))?(a):(b))

#ifndef CONFIG_PCI
#define CONFIG_PCI
#endif
#define DEVICE_Np1_ID     0x1114       //pcie56 device id
#define DEVICE_Np2_ID     0x1116
#define DEVICE_LS_ID      	0x7124

#define NP1_ID 0x01
#define NP2_ID 0x02

//#define VENDOR_ID 0x7024
#define VENDOR_ID    		 0xd6fa        //pcie56 vendor id
#define DEVICE_MAJOR           66

MODULE_LICENSE("GPL");


/////////////////////////////////////////////////////
#define MAXENCRYPT               1024
#define MAXRECVQL			1024
#define MAXSENDQL			1024
#define MAX_NUM 			0x3ff
#define FRAMELEN			4096
#define DMA_FIFO_SIZE		FRAMELEN    //DMA BUFFER,default is 8K

#define GLOBALMEM_SIZE        	4096		//
#define SENDMAX                         8


// the address base of reg
#define IP_ADDR_BASE			(unsigned long)(0x00000000)

//dma0
#define DMA_SND_ADD32      		0x00  			
#define DMA_SND_ADD64 			0x04
#define DMA_SND_SIZE			0x08			
#define DMA_SND_CTRL 			0x0c	

//dma1
#define DMA_RCV_ADD32			0x10			
#define DMA_RCV_ADD64 			0x14                   
#define DMA_RCV_SIZE_SET		0x18			
#define DMA_RCV_SIZE_GET		0x24			
#define DMA_RCV_CTRL			0x1c			

//dma3  sf send dma
#define DMA_SFSND_ADD32      		0x60  			
#define DMA_SFSND_ADD64 			0x64
#define DMA_SFSND_SIZE				0x68			
#define DMA_SFSND_CTRL 			0x6c			

// dma4 sf recv dma
#define DMA_SFRCV_ADD32			0x70			
#define DMA_SFRCV_ADD64 			0x74                   
#define DMA_SFRCV_SIZE_SET			0x78			
#define DMA_SFRCV_SIZE_GET			0x78			
#define DMA_SFRCV_CTRL				0x7c
//dma status

#define DMA_STATUS_START 			0x00000004			
#define DMA_STATUS_ABORT 			0x00000008			
#define DMA_STATUS_SG 				0x00000010			
#define DMA_STATUS_BUSY			0x00000004	
// dma int 
#define DMA_INT_EN				0x38			
#define DMA_INT_ENALL			0x80000000			
#define DMA_SND_EN				0x00000001			
#define DMA_RCV_EN				0x00000002		
#define DMA_SFSND_EN			0x00000008			
#define DMA_SFRCV_EN			0x00000010	
//dma int status
#define DMA_INT_STAT			0x34						
#define DMA_SND_INT				0x00000001			
#define DMA_RCV_INT				0x00000002	
#define DMA_SFSND_INT			0x00000008			
#define DMA_SFRCV_INT			0x00000010				
#define DMA_INT_ALL				0x0000001B
// recv list status reg
#define RECV_OTHER_COUNT		0x54	//0-15:the count of head on other side,16-31:the count of tail on other side.(RD)
#define RECV_OWN_HEAD			0x58	//0-15:the count of head on own side.(WR)
#define RECV_LOCAL_COUNT		0x5C
// FPGA config reg
#define FPGA_SOFT_VERISON		0x80
#define FPGA_SW_DESTROY           0x3C
#define FPGA_CH_MODE			0x44
#define CUR_MODE_SET			0x48
#define ECC_TX_RX_SET0			0x8c
#define ECC_TX_RX_SET1			0x90
#define FPGA_HARD_STATUS		0x88
#define FPGA_SW_RST			0x94

//list contrl 
#define DMA_RCV_LIST_RESET       0xFFFFFF7F
#define DMA_RCV_LIST_FLAG		0x00000080
#define SND_LIST_END                   0x01000000
#define SND_LIST_RESET			0xFEFFFFFF

//LS SRAM
#define LS_SRAM_STATUS_REG			0x80
#define NP_SRAM_STATUS_REG			0x84
//BAR2 write to LS
#define NP_2_LS_RAM					0x0000
#define LS_2_NP_RAM					0x1000;
//FPGA config 
#define FPGA_CFG_STATUS		0x48
#define FPGA_CFG_DATA			0x4c
#define FPGA_CFG_PROG			0x50


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

#define UINT8	unsigned char 
#define UINT		unsigned int
#define UINT64	unsigned long

struct FpgaPacket
{
    int length;
    char *buf;
};

#define EnterFunction()	\
    do{\
		    printk("Enter: %s\n",  __FUNCTION__);		\
    } while (0)
    
#define LeaveFunction()  \
    do {\
			printk("Leave: %s\n",     __FUNCTION__);	\
    } while (0)

#define DEBUG_HERE() printk("Arrive: %s, %s line %i\n",__FUNCTION__, __FILE__, __LINE__)		

int pcie56_fasync(int fd, struct file *filp, int mode);
int pcie56_release(struct inode *inode, struct file *filp);
int pcie56_open(struct inode *inode, struct file *filp);
ssize_t pcie56_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t pcie56_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
unsigned int  pcie56_poll(struct file *filp, poll_table *wait);
//#ifndef Server
long  pcie56_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
//#else
//int pcie56_ioctl(struct inode *in, struct file *filp, unsigned int cmd ,unsigned long arg);
//#endif
		
void pcie56_int_enable(void);
void pcie56_int_disable(void);
void write_BAR0(phys_addr_t   offset, unsigned int data);

struct file_operations pcie56Drv_fops =
{
	.owner = THIS_MODULE,
	.open = pcie56_open,
	.release = pcie56_release,
	.read = pcie56_read,
	.write = pcie56_write,
	.poll = pcie56_poll,
//	.fasync = pcie56_fasync,
//#ifndef Server
//	.compat_ioctl = pcie56_ioctl,
	.unlocked_ioctl = pcie56_ioctl,
//#else
	//.unlocked_
	//.ioctl = pcie56_ioctl,
//#endif
};


struct timeval start_write;
struct timeval end_write;

struct timeval start_sendDMA;
struct timeval end_sendDMA;


struct timeval start_recvDMA;
struct timeval end_recvDMA;


struct timeval start_read;
struct timeval end_read;


struct task_struct *recvtask;
struct task_struct *sendtask;
wait_queue_head_t sendinq; 			
wait_queue_head_t recvoutq;			
//struct task_struct *task;
struct class *pcie56_device;
struct pci_dev *pcie56=NULL;        //pci device struct
struct device		*ps56_dev; 
struct cdev pcie56_cdev;       		 //pci device struct
int pcie56_major = DEVICE_MAJOR;
static int pcie56_opens = 0;
dma_addr_t sflistPh,recvlistPh;
static unsigned int *pcie56_BAR0_Addr=NULL;     //base0

struct pcie56_dev *pcie56_devs;
int glrzch = 0;


DECLARE_WAIT_QUEUE_HEAD(SendQ_Full);
DECLARE_WAIT_QUEUE_HEAD(RecvQ_Empty);

UINT  			Recv_Ready = 0;
volatile UINT		RHead = 0;					
volatile UINT		RTail = 0;					
volatile UINT		SFHead = 0;					
volatile UINT		SFTail = 0;					
volatile UINT 		Slisthead = 0;				
volatile UINT   	Slisttail = 0;					
volatile UINT 		Rlisthead = 0;				
volatile UINT   	Rlisttail = 0;					
volatile UINT 		Renhead = 0;				
volatile UINT   	Rentail = 0;		
volatile UINT		Recv_count = 0;
volatile UINT		Send_count = 0;
volatile UINT         Own_head = 0;

volatile UINT 		Localtail = 0;

char  *dma_buffer_wr = NULL;          //global buffer for dma
char  *dma_buffer_rd = NULL; 
UINT   rcv_len = 0;	
UINT   Rcv_wait = 0;		
UINT   Next_Send_count =0;
UINT   DstID = 0;
UINT   LocalID = 0;

struct queues{
	unsigned char *Buffer;
	dma_addr_t  BufferPh;
	int     len;
} RcvQ[MAXRECVQL], RsfQ[MAXENCRYPT];

void Change_BELE(unsigned char *p)
{
#if 0
	UINT8 b;
	b = p[0];
	p[0] = p[3];
	p[3] = b;
	b = p[1];
	p[1] = p[2];
	p[2] = b;
#endif
}
void SetBuffer_BYTE_ChgBELE(const unsigned char *p_data, unsigned int len)
{
#if 0
	unsigned int i = 0;	
	if (len == 0)
		return;
	if (len % 4 != 0){
		len += 4 - len % 4;
	}
	for (i=0;i<len;i+=4){
		Change_BELE((char*)(p_data+i));
	}
#endif
}

void write_prm(void)
{

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 0);
		write_BAR0(FPGA_CFG_PROG, 0x00000000);
		write_BAR0(FPGA_CFG_PROG, 0);
		write_BAR0(FPGA_CFG_PROG, 0x00000000);
		write_BAR0(FPGA_CFG_PROG, 0x80000000);
		write_BAR0(FPGA_CFG_PROG, 0x80000000);
		write_BAR0(FPGA_CFG_PROG, 0x80000000);
		write_BAR0(FPGA_CFG_PROG, 0x80000000);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);


		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);


		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

}

wait_for_status(void)
{

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 1);
		write_BAR0(FPGA_CFG_PROG, 0x00000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);
		write_BAR0(FPGA_CFG_PROG, 0x80000001);

}
wait_for_done(void)
{
		
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		

		
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);


		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);

		
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0);
		write_BAR0(FPGA_CFG_DATA, 0x00000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);
		write_BAR0(FPGA_CFG_DATA, 0x80000000);

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
	}*recv_list,*sf_list;
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


struct pcie56_dev {
 	unsigned int  DeviceID;
	struct recv_descriptor *devicerecv;
	struct send_descriptor *devicesend;
	struct queues devicesendq[MAXSENDQL];
	struct queues devicerecvq[MAXRECVQL];
	UINT		pcie56_opens;
	 UINT		RHead ;					
	 UINT		RTail ;					
	 UINT		SHead ;					
	 UINT		STail ;					
	 UINT 		Slisthead ;				
	 UINT   		Slisttail ;	
	 UINT 		Recv_count;
	 UINT            Send_count;
	 UINT		Send_ktcount;
//	 UINT 		Rlisthead ;				
//	 UINT   		Rlisttail ;	
	dma_addr_t sendlistPh;
	spinlock_t readlock;         			 	/* mutual exclusion semaphore */
	spinlock_t writelock;
	wait_queue_head_t sendoutq; 			/*队列有空间可以输入*/	
	wait_queue_head_t recvinq;			/*队列有输入可以被读取*/
	struct cdev cdev;                  		/* Char device structure */
	
};
spinlock_t recvlock;
spinlock_t sflock;



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
	//reg = DMA_INT_ENALL | DMA_SND_EN | DMA_RCV_EN | DMA_SFSND_EN|DMA_SFRCV_EN;
	reg = DMA_INT_ENALL | DMA_SND_EN | DMA_RCV_EN;
	write_BAR0(DMA_INT_EN, reg);
//		PRINTK("<======<pcie56_int_enable> \n");
}  

/*******************************************************************
 read the reg of other side driver recv
********************************************************************/
int Other_Side_Recv(void)
{
#if 0
	int other_head,head_flag;
	int other_tail,tail_flag;
	int ret;
	ret = read_BAR0(RECV_OTHER_COUNT);
	head_flag = (ret>>15)&0x01;
	tail_flag = (ret>>31)&0x01;
	other_head = ret&0x00007fff;
	other_tail = (ret>>16)&0x7fff;
	if(other_head == other_tail)
	{
		if(head_flag == tail_flag)
			ret = MAXRECVQL;
		else
			ret = 0;
	}else{
		if(other_tail > other_head)
			ret = (other_head+MAXRECVQL -other_tail)&MAX_NUM;//%MAXRECVQL
		else
			ret= other_head -other_tail;
	}
	if(ret < (SENDMAX*2)){
			//PRINTK("<Other Side Space> is less than 16  is %d   \n",ret);
			ret = 0;
		}
	else
		ret = ret -(SENDMAX*2);

//	ret =128;
	return ret;
#else
	return 128;
#endif
}

int Local_Side_Recv(void)
{
#if 0
	int local_head,local_tail;
	int head_flag,tail_flag;
	int ret,flag;
	flag = read_BAR0(RECV_LOCAL_COUNT);
	tail_flag = (flag>>15)&0x01;
	local_tail = flag&0x00007fff;
	head_flag  =  (Localtail >> 15)&0x01;
	local_head = Localtail & 0x7fff;
	if(local_head == local_tail)
	{
		if(head_flag == tail_flag)
			ret = MAXRECVQL;
		else
			ret = 0;
	}else{
		if(local_tail > local_head)
			ret = (MAXRECVQL + local_head - local_tail)&MAX_NUM;
		else
			ret= local_head - local_tail;
	}
	if(ret < (SENDMAX*2)){
			//PRINTK("<Other Side Space> is less than 128   is %d,flag is %x   Local_tail is %x \n",ret,flag,Localtail);
			ret = 0;
		}
	else
		ret = ret -(SENDMAX*2);
//	ret =128;
	return ret; 
#else
	return 128;
#endif
}

UINT Sendnum(UINT id,UINT head,UINT tail,UINT max)
{
	int count,space;
	
	if( id < 2 ){
		space = Local_Side_Recv();
		count = MIN (MIN(space,max),((pcie56_devs[id].SHead+MAXSENDQL-pcie56_devs[id].STail)&MAX_NUM));
	}else{
		space = Other_Side_Recv();
		count = MIN (MIN(space,max),((pcie56_devs[id].SHead+MAXSENDQL-pcie56_devs[id].STail)&MAX_NUM));

	}
//	if(count == 0)
	//	PRINTK("<Sendnum>  count is %d,  space is %d  SHead is %d    Stail is %d\n",count,space,pcie56_devs[id].SHead,pcie56_devs[id].STail);
	return count;
}

/***********************************************************************
		
************************************************************************/
int start_dma0(int deviceID,int Stail)
{
	unsigned long u64addr;
	UINT ret,i;
	u64addr =(UINT64)(pcie56_devs[deviceID].sendlistPh+sizeof(struct send_descriptor)*(Stail));
	write_BAR0(DMA_SND_ADD32, (unsigned int)u64addr);	
	write_BAR0(DMA_SND_ADD64, (unsigned int)(u64addr>>32));
	ret = DMA_STATUS_START|DMA_STATUS_SG;
	write_BAR0(DMA_SND_CTRL,ret );
	for(i=0;i<20;i++);
	while((read_BAR0(DMA_SND_CTRL)&DMA_STATUS_BUSY) != 0)
	{
		for(i=0;i<10;i++);
						
	}

	return SUCCESS;
}

/***********************************************************************

************************************************************************/
void start_dma1(void)
{
	unsigned long u64addr;
	UINT ret;
	u64addr = (unsigned long)(recvlistPh+sizeof(struct recv_descriptor)*Rlisthead);
	write_BAR0(DMA_RCV_ADD32, (unsigned int)u64addr);
	write_BAR0(DMA_RCV_ADD64, (unsigned int)(u64addr>>32));
	ret = DMA_STATUS_START|DMA_STATUS_SG;
	write_BAR0(DMA_RCV_CTRL, ret);
	
}

/***********************************************************************

************************************************************************/

int start_sfsend_dma(int deviceID,int Stail)
{
	unsigned long u64addr;
	UINT ret,i;

	u64addr =(UINT64)(pcie56_devs[deviceID].sendlistPh+sizeof(struct send_descriptor)*(Stail));
	write_BAR0(DMA_SFSND_ADD32, (unsigned int)u64addr);	
	write_BAR0(DMA_SFSND_ADD64, (unsigned int)(u64addr>>32));
	ret = DMA_STATUS_START|DMA_STATUS_SG;
	write_BAR0(DMA_SFSND_CTRL,ret );
	//PRINTK("start sfdma!\n");
	for(i=0;i<10;i++);
	while((read_BAR0(DMA_SFSND_CTRL)&DMA_STATUS_BUSY) != 0)
	{
		for(i=0;i<10;i++);
						
	}
		//PRINTK(" sfdma is  working!\n");
	return SUCCESS;
}

/***********************************************************************
	
************************************************************************/
void start_sfrecv_dma(void)
{
	unsigned long u64addr;
	UINT ret;
	u64addr = (unsigned long)(sflistPh+sizeof(struct recv_descriptor)*SFHead);
	write_BAR0(DMA_SFRCV_ADD32, (unsigned int)u64addr);
	write_BAR0(DMA_SFRCV_ADD64, (unsigned int)(u64addr>>32));
	ret = DMA_STATUS_START|DMA_STATUS_SG;
	write_BAR0(DMA_SFRCV_CTRL, ret);
}


/**********************************************************************
 			send_thread
**********************************************************************/
void send_thread(void)
{
	int i,sendcount;
	int idx = 0;

	i = 0;
	UINT 		Last_send = 0;
	//UINT 		SFLast_send = 0;
	while(1){
		//PRINTK("send_thread loop \n");
		if( kthread_should_stop()) 
			break;
#if 0		
		if((read_BAR0(DMA_SFSND_CTRL)&DMA_STATUS_BUSY)==0){     //SF SEND DMA is not working
			
			for( i = SFLast_send+1; i <( SFLast_send + 3); i++ ){

				idx = i % 3;
				
				//PRINTK("can send  \n");
				spin_lock_bh(&pcie56_devs[idx].writelock);
				
				sendcount = Sendnum(idx,SFHead,SFTail,SENDMAX);

				if(sendcount > 0){
					do_gettimeofday(&start_sendDMA);
				
					pcie56_devs[idx].devicesend[(pcie56_devs[idx].STail+sendcount -1)&MAX_NUM].NextDesc_low |= SND_LIST_END;	//
			//		PRINTK("before sf dma send \n");
					start_sfsend_dma(idx,pcie56_devs[idx].STail);
					 //updata the sendlist header	
					pcie56_devs[idx].Slisttail = pcie56_devs[idx].STail = (pcie56_devs[idx].STail+sendcount)&MAX_NUM; 
					wake_up_interruptible(&pcie56_devs[idx].sendoutq);
					spin_unlock_bh(&pcie56_devs[idx].writelock);
					do_gettimeofday(&end_sendDMA);
					break;
					
				
				}else
				{
					spin_unlock_bh(&pcie56_devs[idx].writelock);
				}
			}
			SFLast_send = idx;
		}
#endif
		if((read_BAR0(DMA_SND_CTRL)&DMA_STATUS_BUSY)==0){     //SEND DMA is not working

			for( i = Last_send; i < Last_send + DEVICE_COUNT; i++ ){

				idx = i % DEVICE_COUNT;
				if(idx > 2){
							
					spin_lock_bh(&pcie56_devs[idx].writelock);
					
					sendcount = Sendnum(idx,RHead,RTail,SENDMAX);

					if(sendcount > 0){
				
						pcie56_devs[idx].devicesend[(pcie56_devs[idx].STail+sendcount -1)&MAX_NUM].NextDesc_low |= SND_LIST_END;	
						//	PRINTK("before sf dma send \n");
						start_dma0(idx,pcie56_devs[idx].STail);
						 //updata the sendlist header	
						pcie56_devs[idx].Slisttail = pcie56_devs[idx].STail = (pcie56_devs[idx].STail+sendcount)&MAX_NUM;
						wake_up_interruptible(&pcie56_devs[idx].sendoutq);
						spin_unlock_bh(&pcie56_devs[idx].writelock);	
						
						break;
						
					}else
						{
							spin_unlock_bh(&pcie56_devs[idx].writelock);	
					}
				}
			}
			Last_send = idx;
		}
		else{
			//nothing to do
		}
			//end if(DMA is not working)

	}
	//end while 1 
}
/**********************************************************************
 			recv_thread
**********************************************************************/
void recv_thread(void)
{
	int id,recvlen,recvlisttail;
	unsigned char* recvbuff;
	int recv_flag = 0;

	while(1){
		if( kthread_should_stop()) 
			break;
		smp_mb();
#if 0		
		if((sf_list[SFTail].status&DMA_RCV_LIST_FLAG)!=0){
			do_gettimeofday(&start_recvDMA);
			id = *(unsigned int *)(&RsfQ[SFTail].Buffer[12]) ;
			recvlen	=  sf_list[SFTail].status;
			Change_BELE((unsigned char *)&recvlen);
			Change_BELE((unsigned char *)&id);
			id &= 0xff; 
			recvlen = recvlen & 0x7fffffff;
		//	PRINTK("recvlist is %d\n",recvlisttail);
			spin_lock_bh(&pcie56_devs[id].readlock);
			if(((pcie56_devs[id].RHead+1)&MAX_NUM)/*%MAXRECVQL*/ != pcie56_devs[id].RTail){
				recvbuff = pcie56_devs[id].devicerecvq[pcie56_devs[id].RHead].Buffer;
				memcpy(recvbuff,RsfQ[SFTail].Buffer,recvlen);
				pcie56_devs[id].Recv_count++;
				sf_list[SFTail].status = 0;
				SFTail = (SFTail + 1);
				if(SFTail>MAX_NUM){	
					SFTail &= MAX_NUM;
					Localtail = (Localtail&0x8000)+SFTail +0x8000;
						
				}else{
					Localtail = (Localtail&0x8000)+SFTail;
				}
				pcie56_devs[id].devicerecvq[pcie56_devs[id].RHead].len = recvlen;
				pcie56_devs[id].RHead= (pcie56_devs[id].RHead + 1)&MAX_NUM;//%MAXRECVQL;
				spin_unlock_bh(&pcie56_devs[id].readlock);
				wake_up_interruptible(&pcie56_devs[id].recvinq);	
				do_gettimeofday(&end_recvDMA);
				
				
			}
			else{
				spin_unlock_bh(&pcie56_devs[id].readlock);
			}

		}
		else{
		
		}
#endif
		if((recv_list[Rlisttail].status&DMA_RCV_LIST_FLAG)!=0){
		//	PRINTK("recv_thread habe data \n");
			//id = *(unsigned int *)(&RcvQ[Rlisttail].Buffer[12]) ;
			id = 0;
			recvlen= recv_list[Rlisttail].status;
			Change_BELE((unsigned char *)&recvlen);
			Change_BELE((unsigned char *)&id);
			id &= 0xff; 
			recvlen = recvlen & 0x7fffffff;
		//	PRINTK("%d recv_thread habe data \n",id);
			spin_lock_bh(&pcie56_devs[id].readlock);
			if(((pcie56_devs[id].RHead+1)&MAX_NUM)/*%MAXRECVQL*/ != pcie56_devs[id].RTail){
				recvbuff = pcie56_devs[id].devicerecvq[pcie56_devs[id].RHead].Buffer;
				
				memcpy(recvbuff,RcvQ[Rlisttail].Buffer,recvlen);
				recv_list[Rlisttail].status = 0;
				Rlisttail = RTail = Rlisttail+1;
				if(Rlisttail>MAX_NUM){	
					Rlisttail &= MAX_NUM;
					//Own_head = (Own_head&0x8000)+Rlisttail +0x8000;
						
				}else{
					//Own_head = (Own_head&0x8000)+Rlisttail;
				}
			
				//write_BAR0(RECV_OWN_HEAD, (Own_head&0xffff));
				pcie56_devs[id].devicerecvq[pcie56_devs[id].RHead].len = recvlen;
				pcie56_devs[id].RHead= (pcie56_devs[id].RHead + 1)&MAX_NUM;//%MAXRECVQL;
				
				spin_unlock_bh(&pcie56_devs[id].readlock);
				wake_up_interruptible(&pcie56_devs[id].recvinq);	
			}
			else{
				spin_unlock_bh(&pcie56_devs[id].readlock);
			}
		}
		else{

		}
	}
}

/**********************************************************************
		
**********************************************************************/
/*
int pcie56_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &async_queue);
}
*/
/**********************************************************************
		
**********************************************************************/
irqreturn_t pcie56Drv_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{

	int IntStat= 0;

	pcie56_int_disable();
	IntStat = read_BAR0(DMA_INT_STAT);
	write_BAR0(DMA_INT_STAT, IntStat);
    	pcie56_int_enable();
	//PRINTK("IntStat is 0x%x",IntStat);
	if((IntStat & DMA_INT_ALL) == 0){
		return ERR_IRQ_NONE;
	}
	if((IntStat & DMA_SND_INT) || (IntStat & DMA_SFSND_INT)){
//	PRINTK("<pcie56_interrupt_send>:send complete interrupt!\n");
		//wake_up_interruptible(&sendinq);
	}
	if(IntStat & DMA_RCV_INT){ 
//		PRINTK("<pcie56_interrupt_recv>:recv complete interrupt!\n");
		//spin_lock_bh(&recvlock);
		while(recv_list[Rlisthead].status&DMA_RCV_LIST_FLAG){
			Rlisthead = (Rlisthead + 1)&MAX_NUM;
		}
		RHead = Rlisthead; 
		//wake_up_interruptible(&recvoutq);
		//spin_unlock_bh(&recvlock);
	}
	if(IntStat & DMA_SFRCV_INT){
		//spin_lock_bh(&sflock);
		//PRINTK("<pcie56_interrupt_sfrecv>:sfrecv complete interrupt!\n");
		while(sf_list[SFHead].status&DMA_RCV_LIST_FLAG){
			SFHead = (SFHead + 1)&MAX_NUM;
		}
		//wake_up_interruptible(&recvoutq);
		//spin_unlock_bh(&sflock);
	}
	return IRQ_HANDLED;
}
/***********************************************************************
	
************************************************************************/
//#ifdef Server
 long pcie56_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
//#else

//int pcie56_ioctl(struct inode *in, struct file *filp, unsigned int cmd ,unsigned long arg)
//#endif
{
	PRINTK("<pcie56_ioctld>:into  ioctl   !\n");
	int i;
	int j = 0;
	int ret;
	int nolen;
	unsigned int fpga_data;
	unsigned char fpga_p[4096];
	struct FpgaPacket *fpga; 
	switch(cmd){
	case READ_VERSION:
		*(unsigned int *)arg = read_BAR0(FPGA_SOFT_VERISON);
		ret= SUCCESS;
		break;
	case ECC_TRXSET0:
		//*(unsigned int *)arg = read_BAR0(FPGA_SOFT_VERISON);
		write_BAR0(ECC_TX_RX_SET0, (unsigned int) arg);
		PRINTK("<ioctl> ECC_reg0  is 0x%08x \n",arg);
		ret= SUCCESS;
		break;
	case ECC_TRXSET1:
		//*(unsigned int *)arg = read_BAR0(FPGA_SOFT_VERISON);
		write_BAR0(ECC_TX_RX_SET1, (unsigned int) arg);
		PRINTK("<ioctl> ECC_reg1  is 0x%08x \n",arg);
		ret= SUCCESS;
		break;
	case FPGA_RESET:
		write_BAR0(FPGA_SW_RST,0x5a5a5a5a);
		 PRINTK("<pcie56_ioctl>:FPGA RESET !\n");
		ret = SUCCESS;
		break;
	case FPGA_DESTROY:
		write_BAR0(FPGA_SW_DESTROY,0x5a5a5a5a);
		 PRINTK("<pcie56_ioctl>:FPGA DESTROY !\n");
		ret = SUCCESS;
		break;
	case FPGA_CONFIG:
			if((read_BAR0(FPGA_CFG_STATUS)&0xff)!=0xff)
			{
				PRINTK("<pcie56_ioctl>FPGA is not INIT,0x%08x\n",read_BAR0(FPGA_CFG_STATUS));
				return  -EAGAIN;
			}
			write_prm();

			while((read_BAR0(FPGA_CFG_STATUS)&0xff)!=0xff)
				{
					wait_for_status();
					j = j++;
				}
			//PRINTK("j is %d\n",j);
			if((read_BAR0(FPGA_CFG_STATUS)&0xff)!=0xff)
			{
				PRINTK("<pcie56_ioctl> 2th FPGA is not INIT,0x%08x\n",read_BAR0(FPGA_CFG_STATUS));
				return  -EAGAIN;
			}
			PRINTK("<pcie56_ioctl> INIT,0x%08x\n",read_BAR0(FPGA_CFG_STATUS));
			wait_for_status();
			write_BAR0(FPGA_CFG_PROG, 0x01);
			fpga = (struct FpgaPacket *)arg;
			PRINTK("<pcie56_ioctl>datalen is %d \n",fpga->length);
			nolen = 0;
			while(fpga->length -nolen >4096){
				ret = __copy_from_user(fpga_p,fpga->buf+nolen,4096);
				if(ret<0){
					PRINTK("<pcie56_ioctl>copy from user error\n");
					return -EIO;
				}
				for(i=0;i<4096;i++){
					fpga_data = 0x000000ff&((unsigned int)fpga_p[i]);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
					write_BAR0(FPGA_CFG_DATA,fpga_data);
				
					fpga_data = 0x80000000|((unsigned int)fpga_p[i]);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
				}
				nolen += 4096;
				memset(fpga_p,0,4096);
			}
			ret = __copy_from_user(fpga_p,fpga->buf+nolen,fpga->length-nolen);
			if(ret<0){
				PRINTK("<pcie56_ioctl>copy from user error\n");
				return -EIO;
			}
			for(i=0;i<(fpga->length-nolen);i++){
					fpga_data = 0x000000ff&((unsigned int)fpga_p[i]);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					fpga_data = 0x80000000|((unsigned int )fpga_p[i]);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
					write_BAR0(FPGA_CFG_DATA, fpga_data);
			}
			PRINTK("<pcie56_ioctl>after write the cfg_data to fpga\n");
			PRINTK("<pcie56_ioctl>begin read the reg ,look this step is OK or FAULT? \n");
			wait_for_done();
			wait_for_done();
			wait_for_done();
			wait_for_done();
			wait_for_done();
			wait_for_done();
			wait_for_done();
			wait_for_done();
			for(i=0;i<16536;i++)
				wait_for_done();
			
			for(i=0;i<4096;i++){
				if((read_BAR0(FPGA_CFG_STATUS)&0xff00)==0xFF00)
					{
						PRINTK("<pcie56_ioctl>write config FPGA is SUCCESS!!!! \n");
						break;
					}
			}
			if((read_BAR0(FPGA_CFG_STATUS)&0xff00)==0xFF00)
				{
					PRINTK("<pcie56_ioctl>write config FPGA is SUCCESS!!!! \n");
					ret = SUCCESS;
				}
			else{
				PRINTK("<pcie56_ioctl> 2th FPGA is not DONE,0x%08x\n",read_BAR0(FPGA_CFG_STATUS));
					ret = FAULT;
				}
			
			break;
	case CHANEL_CONFIG:
			ret = *(unsigned int *)(arg);
			if(ret==0){
				write_BAR0(FPGA_CH_MODE,0);						//duli mode
				PRINTK("<pcie56_ioctl>set FPGA_CH_MODE 0 ! \n");
			}else{
				write_BAR0(FPGA_CH_MODE,1);						//xunhuan mode
				PRINTK("<pcie56_ioctl>set FPGA_CH_MODE 1 ! \n");
			}
			break;
	default:
			ret = -ECHRNG;
			break;
	}
	return ret;
}
/***********************************************************************
		透传数据读取
************************************************************************/
ssize_t pcie56_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int len;					
	int ret;
	char* buff;
	struct pcie56_dev *dev = filp->private_data;
	//EnterFunction();
	
	if((count==0)||(buf==NULL)){
		return 0;
	}
	spin_lock_bh(&dev->readlock);
	if( dev->RHead == dev->RTail){
		spin_unlock_bh(&dev->readlock);
		return -EAGAIN;
	}

	len = dev->devicerecvq[dev->RTail].len;
	buff = dev->devicerecvq[dev->RTail].Buffer;
/*
	if(count<len){
		spin_unlock_bh(&dev->readlock);
		return -EINVAL;
	}
*/
	spin_unlock_bh(&dev->readlock);
#if 1
#if 0
	if(dev->DeviceID <5){					// sf data ,they have 32byte header

		SetBuffer_BYTE_ChgBELE(buff,32);	
		ret = __copy_to_user(buf,buff,len); 
		if(ret<0)
			return -EIO;	
	}else
#endif
	{//other data  ,they don't have header
		len = len-8;
		//PRINTK("%s:len is %d\n", __func__, len);
		ret = __copy_to_user(buf,buff+8,len); 
		
		if(ret<0)
			return -EIO;	
	}	
#endif
//	PRINTK("%s: id is %d  RTAil is %d  ret is %d\n", __func__,dev->DeviceID,dev->RTail, ret);
	spin_lock_bh(&dev->readlock);
	dev->RTail = (dev->RTail+1)&MAX_NUM;//%MAXRECVQL;
//	wake_up_interruptible(&recvoutq);			//wake the recv kernel_thread to recv data
	spin_unlock_bh(&dev->readlock);
	
	//LeaveFunction();
	return len;
}
int data_read( char  *buf, int  count, int chanel)
{
	int len;					
	int ret;
	char* buff;
	do_gettimeofday(&start_read);
	struct pcie56_dev *dev = pcie56_devs+chanel;
	if((count==0)||(buf==NULL)){
		return 0;
	}
	spin_lock_bh(&dev->readlock);
	if( dev->RHead == dev->RTail){
		spin_unlock_bh(&dev->readlock);
		return -EAGAIN;
	}

	len = dev->devicerecvq[dev->RTail].len;
	buff = dev->devicerecvq[dev->RTail].Buffer;
	spin_unlock_bh(&dev->readlock);
	//SetBuffer_BYTE_ChgBELE(buff,32);	
	ret=memcpy(buf,buff,len);
	if(ret<0)
		return -EIO;
	spin_lock_bh(&dev->readlock);
	dev->RTail = (dev->RTail+1)&MAX_NUM;		//%MAXRECVQL;
//	wake_up_interruptible(&recvoutq);			//wake the recv kernel_thread to recv data
	spin_unlock_bh(&dev->readlock);
	do_gettimeofday(&end_read);
	return len;
}

int data_write(char  * buf,int  count,int chanel)
{	
 	int  ret = 0;
	u32 slen;
	char* buffer;
	//EnterFunction();
	do_gettimeofday(&start_write);
	struct pcie56_dev *dev = pcie56_devs+chanel;
	if ((GLOBALMEM_SIZE-32) < count){
		PRINTK("%s:write packet is too long,len = %ld\n", __func__, count);
		return -EINVAL;
	}
	if((count==0)||(buf==NULL))
		return 0;
	
	//PRINTK("<data_write>    dev->DeviceID is %d\n",dev->DeviceID);
	
	spin_lock_bh(&dev->writelock);
	if( ((dev->SHead+1)&MAX_NUM) == dev->STail){
		spin_unlock_bh(&dev->writelock);
		return -EAGAIN;
	}
	buffer = dev->devicesendq[dev->SHead].Buffer;
	spin_unlock_bh(&dev->writelock);	
	memcpy(buffer,buf,count);
	if(ret<0)
		return -EIO;		
	if(dev->DeviceID<3)
		{
			 *(unsigned int *)(&buffer[0]) &= 0xffffff00;
	 		 *(unsigned int *)(&buffer[0]) |= LocalID+2;	
		}else
		{
			 *(unsigned int *)(&buffer[0]) &= 0xffffff00;
	 		 *(unsigned int *)(&buffer[0]) |= LocalID;	
		}
	 *(unsigned int *)(&buffer[12]) &= 0xffffff00;
	 *(unsigned int *)(&buffer[12]) |= dev->DeviceID&0xff;						//It mask which device has recevied this data	

	SetBuffer_BYTE_ChgBELE(buffer,32);
	slen = count;
	spin_lock_bh(&dev->writelock);
	dev->devicesendq[dev->SHead].len = slen;
	dev->devicesend[dev->Slisthead].length =( dev->devicesendq[dev->SHead].len+7)&0xfffffff8;
	Change_BELE((char *)&dev->devicesend[dev->Slisthead].length );
	dev->devicesend[dev->Slisthead].NextDesc_low &=  SND_LIST_RESET;			//just let one-packet style
	dev->Slisthead = dev->SHead = (dev->SHead + 1) &MAX_NUM;					//% MAXSENDQL;
	// PRINTK("<pcie56_write>:dev->SHead  is %d   \n",dev->SHead);
//	wake_up_interruptible(&sendinq);											//wake up send kernel-thread to send the data
	spin_unlock_bh(&dev->writelock);
	do_gettimeofday(&end_write);
	//LeaveFunction();
    	return count;
}
/**********************************************************************

************************************************************************/
	u32 buffCount=0;

ssize_t pcie56_write(struct file * filp,const char __user * buf,size_t count,loff_t * f_pos)
{	
 	int  ret = 0;
	u32 slen;
	char* buffer;
	//EnterFunction();
	struct pcie56_dev *dev = filp->private_data;
	if ((GLOBALMEM_SIZE-8) < count){
		PRINTK("%s:write packet is too long,len = %ld\n", __func__, count);
		return -EINVAL;
	}
	if((count==0)||(buf==NULL))
		return 0;
	
	//PRINTK("<pcie_write>    dev->DeviceID is %d\n",dev->DeviceID);
	
	spin_lock_bh(&dev->writelock);
	if( ((dev->SHead+1)&MAX_NUM) == dev->STail){
		spin_unlock_bh(&dev->writelock);
		return -EAGAIN;
	}
	buffer = dev->devicesendq[dev->SHead].Buffer;
	spin_unlock_bh(&dev->writelock);
	//
#if 0
	//   test sf dma 
		slen = count+32;
		*(unsigned int *)(&buffer[0]) = (0x0810<<16)+((DstID)<<8)+(LocalID);	
		*(unsigned int *)(&buffer[4]) = (slen<<16)+slen;
		*(unsigned int *)(&buffer[8]) = (0<<16)+(0<<8);
		// PRINTK("<pcie_write >  before copy from user\n");
		 ret = __copy_from_user(buffer+32, buf, count);
		 if(ret)
	    		return -EIO;	
		 *(unsigned int *)(&buffer[12]) &= 0xffffff00;
		 *(unsigned int *)(&buffer[12])  = (Send_count << 8)+(dev->DeviceID&0xff);					//it mask which device has recevied	 this data	
		 Send_count ++;
		 //PRINTK("<pcie_write >   write the data own ID is %d\n",*(unsigned int *)(&buffer[12]));
	//	  PRINTK(" write  data is :\n");
	/*	 for(i=0;i<128;i++)
		 	PRINTK(" 0x%x ",buffer[i]);
	*/
		 SetBuffer_BYTE_ChgBELE(buffer,32);
	//	  PRINTK("<pcie56_write>:Write to sf FPGA ! id  is %d   \n",dev->DeviceID);
#else

#if 0
		if(dev->DeviceID<5){
			ret = __copy_from_user(buffer, buf, count);
			 if(ret)
		    		return -EIO;		
			if(dev->DeviceID<3){
				 *(unsigned int *)(&buffer[0]) &= 0xffffff00;
		 		 *(unsigned int *)(&buffer[0]) |= (LocalID+2);	
			}else{
				 *(unsigned int *)(&buffer[0]) &= 0xffffff00;
		 		 *(unsigned int *)(&buffer[0]) |= LocalID;	
			}                                                                                                                                                                                                                   
			if((dev->DeviceID==3)||(dev->DeviceID==4))
			{
					*(unsigned int *)(&buffer[0]) &= 0xffff00ff;
					*(unsigned int *)(&buffer[0]) |= (LocalID+6)<<8;	
			}
			if(dev->DeviceID==3)
			{
				*(unsigned int *)(&buffer[8]) &= 0xffff00ff;
				*(unsigned int *)(&buffer[8]) |= glrzch<<8;
				glrzch = (glrzch+1)&GLRZ_MAX;

			}
			
				
			 *(unsigned int *)(&buffer[12]) &= 0xffffff00;
			 *(unsigned int *)(&buffer[12]) |= dev->DeviceID&0xff;						//It mask which device has recevied this data	
	//		 PRINTK("data is :\n");
	//		 for(i=0;i<64;i++)
	//		 	PRINTK(" 0x%x ",buffer[i]);
			SetBuffer_BYTE_ChgBELE(buffer,32);
			slen = count;
		}else
#endif
		{

		slen = count+8;
		
		*(unsigned int *)(&buffer[0]) = ((0xd6fa<<16)|((buffCount++)&0xffff));	
		*(unsigned int *)(&buffer[4]) = slen&0xffff;
		//*(unsigned int *)(&buffer[8]) = (0<<16)+(0<<8);

		// PRINTK("<pcie_write >  before copy from user\n");
		 ret = __copy_from_user(buffer+8, buf, count);
		 if(ret)
	    		return -EIO;	
//		 *(unsigned int *)(&buffer[12]) &= 0xffffff00;
//		 *(unsigned int *)(&buffer[12])  =  dev->DeviceID&0xff;					//it mask which device has recevied	 this data	
		// Send_count ++;
		 //PRINTK("<pcie_write >   write the data own ID is %d\n",*(unsigned int *)(&buffer[12]));
	//	  PRINTK(" write  data is :\n");
	/*	 for(i=0;i<128;i++)
		 	PRINTK(" 0x%x ",buffer[i]);
	*/
		 SetBuffer_BYTE_ChgBELE(buffer,32);
	//	  PRINTK("<pcie56_write>:Write to sf FPGA ! id  is %d   \n",dev->DeviceID);


		}
		
#endif
#if 	0	 
	else{
		//userdata don't have 32byte packet header
		slen = count+32;
		*(unsigned int *)(&buffer[0]) = (0x0810<<16)+(DstID<<8)+LocalID;	
		*(unsigned int *)(&buffer[4]) = (slen<<16)+slen;
		
		// PRINTK("<pcie_write >  before copy from user\n");
		 ret = __copy_from_user(buffer+32, buf, count);
		 if(ret)
	    		return -EIO;	
		 *(unsigned int *)(&buffer[12]) &= 0xffffff00;
		 *(unsigned int *)(&buffer[12])  = (Send_count << 8)+(dev->DeviceID&0xff);					//it mask which device has recevied	 this data	
		 Send_count ++;
		 // PRINTK("<pcie56_write>:Write to np FPGA ! id  is %d   \n",dev->DeviceID);
		 //PRINTK("<pcie_write >   write the data own ID is %d\n",*(unsigned int *)(&buffer[12]));
		 SetBuffer_BYTE_ChgBELE(buffer,32);	
	}	
	if(dev->Send_count != (*(unsigned int *)(buffer+32+8)))
	{
		PRINTK("<Write ERROR>pcie56_dev[%d].Send_count is %d     PKG_id is %d\n",dev->DeviceID,dev->Send_count,*(unsigned int *)(buffer+32+8));
	}
	dev->Send_count++;
#endif	
	spin_lock_bh(&dev->writelock);
	dev->devicesendq[dev->SHead].len = slen;
	dev->devicesend[dev->Slisthead].length =( dev->devicesendq[dev->SHead].len+7)&0xfffffff8;
	Change_BELE((char *)&dev->devicesend[dev->Slisthead].length );
	dev->devicesend[dev->Slisthead].NextDesc_low &=  SND_LIST_RESET;				//just let one-packet style
	dev->Slisthead = dev->SHead = (dev->SHead + 1) & MAX_NUM;					//% MAXSENDQL;
//	PRINTK("<pcie56_write>:dev->SHead  is %d   \n",dev->SHead);
//	wake_up_interruptible(&sendinq);	//wake up send kernel-thread to send the data
	spin_unlock_bh(&dev->writelock);
	//LeaveFunction();
    	return count;
	
}

/***********************************************************************


************************************************************************/
unsigned int pcie56_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
//	int ret;
	/*
	 * The buffer is circular; it is considered full
	 * if "wp" is right behind "rp" and empty if the
	 * two are equal.
	 */
	struct pcie56_dev *dev = filp->private_data;
//	the write mask
	poll_wait(filp, &dev->recvinq, wait);
	poll_wait(filp, &dev->sendoutq,  wait);

	spin_lock_bh(&dev->writelock);
	if(((dev->SHead+1)&MAX_NUM) != dev->STail)
		mask |=  POLLOUT|POLLWRNORM;
	spin_unlock_bh(&dev->writelock);
//	the read Mask
	spin_lock_bh(&dev->readlock);
	if(dev->RTail != dev->RHead)
		mask |= POLLIN|POLLRDNORM;
	spin_unlock_bh(&dev->readlock);
	return mask;
}
/***********************************************************************


************************************************************************/
int pcie56_open(struct inode *inode, struct file *filp)
{
  	
	struct pcie56_dev *dev; 
	dev = container_of(inode->i_cdev, struct pcie56_dev, cdev);
	filp->private_data = dev;
	if (pcie56 == NULL){
		//PRINTK("<pcie56_open>: pcie56 open error\n");
		return -1;
	}	
#if 0	
	if(dev->pcie56_opens == 0){
		dev->RHead = 0;
		dev->RTail = 0;
		dev->SHead = 0;
		dev->STail = 0;
		dev->Slisthead = 0;
		dev->Slisttail = 0;
		dev->Send_count = 0;
		dev->Recv_count = 0;
	}

	if(pcie56_opens == 0){
		/*使能中断*/
		pcie56_int_enable();
		/*首先启动DMA1，准备接受数据*/
		start_dma1(); 
		/*首先启动sf recv DMA，准备接受数据*/
		start_sfrecv_dma();
	}
#endif  
	filp->f_op = &pcie56Drv_fops;
	dev->pcie56_opens++; 
	pcie56_opens++;
	PRINTK("<pcie56_open>: devID is %d\n",dev->DeviceID);
	
	return SUCCESS;	
	
}

/***********************************************************************

************************************************************************/
int pcie56_release(struct inode *inode, struct file *filp)
{	
	
	struct pcie56_dev *dev = filp->private_data;
	dev->pcie56_opens--;
	if(dev->pcie56_opens == 0){
		filp->private_data = NULL;
		//pcie56_fasync(-1, filp, 0);		
/*		dev->RHead = 0;
		dev->RTail = 0;
		dev->SHead = 0;
		dev->STail = 0;
		dev->Slisthead = 0;
		dev->Slisttail = 0;
*/
	}
	pcie56_opens--;
#if 0	
	if(pcie56_opens==0)
	{
		pcie56_int_disable();
		write_BAR0(DMA_RCV_CTRL, DMA_STATUS_ABORT);
		write_BAR0(DMA_SFRCV_CTRL, DMA_STATUS_ABORT);
		write_BAR0(FPGA_SW_RST, 0x5a5a5a5a);
		RHead = 0;
		Rlisthead = 0;
		RTail = 0;
		Rlisttail = 0;
		SFHead = 0;
		SFTail = 0;
		for(i=0;i<MAXRECVQL;i++)
		{
			recv_list[i].status = 0;
			sf_list[i].status = 0;
		}
	}		
#endif	
	return 0;
}
/***********************************************************************

************************************************************************/
int pcie56Drv_setup_cdev(struct pcie56_dev *dev, int index)
{
	int err;
	int devno = MKDEV(pcie56_major, index);
	cdev_init(&dev->cdev, &pcie56Drv_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &pcie56Drv_fops;
	err = cdev_add (&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		PRINTK("Error %d adding pcie56Drv%d\n", err, index);
	return err;
}

/***********************************************************************
	设备驱动初始化

************************************************************************/
static int __init pcie56Drv_init(void)
{
	char *dma_buffer = NULL;          //global buffer for dma
	char *recvdma_buffer = NULL;          //global buffer for dma
	char	*Sendlist = NULL;
	char *Recvlist = NULL;
	char *Recv_buffer = NULL;
//	char *encrypt = NULL;
	int i,ret,devcount,count,result;
	phys_addr_t   base0start = 0;			
	unsigned int base0len = 0;
	phys_addr_t   endsrc0 = 0;
	dev_t dev;

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
		ret = -EIO;
	}
	pcie56_devs = kmalloc(DEVICE_COUNT*sizeof (struct pcie56_dev), GFP_KERNEL);
	if (!pcie56_devs) {
              PRINTK("Malloc pcie56_devs failed!\n");
		ret =  -ENOMEM;
          	goto OUT;
	}
	memset(pcie56_devs, 0, DEVICE_COUNT*sizeof (struct pcie56_dev));
	PRINTK("###################################################\n");
       PRINTK("<pcie56Drv_init> enter into drv Init!\n");
	/* base 0 */
	base0start=pci_resource_start(pcie56,0);
	endsrc0=pci_resource_end(pcie56,0);
	base0len=endsrc0-base0start;
	request_mem_region(base0start,base0len,"pcie56");
	pcie56_BAR0_Addr = ioremap(base0start,base0len);   
	PRINTK("<pcie56_init>: base0 addr Range 0x%08X - 0x%08X Length:0x%16X\n", (int)base0start, (int)(base0start + base0len), 
	base0len);
	ret = alloc_chrdev_region(&dev, 0, DEVICE_COUNT, "pcie56");
	if (ret <0){
		PRINTK(" register pcie56Drv device number error\n");	
	}
	pcie56_major = MAJOR(dev);
	 for(devcount = 0; devcount< DEVICE_COUNT; devcount++){
              result = pcie56Drv_setup_cdev(pcie56_devs + devcount, devcount);
              if(result){
                    PRINTK(KERN_NOTICE "Error %d adding pcie56_dev %d", ret, devcount);
                   goto setup_cdev_err;
              }
        }
	 PRINTK("<pcie56Drv_init>: pcie56_cdev major is %d. \n", pcie56_major);

//	DMA Configure    
	//pcie56_int_disable();
	result=pci_set_dma_mask(pcie56, DMA_BIT_MASK(64));
	 if(result!=0){
		PRINTK("<pcie56_init>: set mask Error return 0x%08X\n", ret);
	 }
  	result= pci_set_consistent_dma_mask(pcie56, DMA_BIT_MASK(64));
	 if(result!=0){
		PRINTK("<pcie56_init>: set mask 2 Error return 0x%08X\n", ret);
	 }
   	 pci_set_master(pcie56);

//	alloc DMA buffer for send Queue;
	for(count=0;count<DEVICE_COUNT;count++)
	{
		spin_lock_init(&pcie56_devs[count].writelock);
		spin_lock_init(&pcie56_devs[count].readlock);
		pcie56_devs[count].DeviceID = count;
		pcie56_devs[count].Recv_count = 0;
		pcie56_devs[count].Send_count = 0;
		pcie56_devs[count].Send_ktcount = 0;
		PRINTK("<pcie56Drv_init>: DEVICE[%d]  alloc memery  !\n",count);
		
		dma_buffer = (void *)dma_mem_alloc(DMA_FIFO_SIZE*MAXSENDQL);
		
		if(dma_buffer==NULL){
			PRINTK("<pcie56Drv_init>: Malloc Send buffer ERROR!\n" );
			goto fail_alloc_devsendbuffer;
		}
		memset(dma_buffer,0,DMA_FIFO_SIZE*MAXSENDQL);
		pcie56_devs[count].devicesendq[0].BufferPh = dma_map_single(&pcie56->dev,dma_buffer,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
		PRINTK("<pcie56Drv_init>: SEND Dma Buffer is Create %08x\n,SndQ[0].BufferPh is %08x \n",(int)dma_buffer,(int)pcie56_devs[count].devicesendq[0].BufferPh);
		for (i=0;i<MAXSENDQL;i++){
			pcie56_devs[count].devicesendq[i].Buffer = dma_buffer+i*FRAMELEN;
			pcie56_devs[count].devicesendq[i].BufferPh =pcie56_devs[count].devicesendq[0].BufferPh + i*FRAMELEN;
			pcie56_devs[count].devicesendq[i].len=0;
		}
	//	initialize send S/G descriptor list
		Sendlist = (void *)dma_mem_alloc(sizeof(struct send_descriptor)*MAXSENDQL);
	
		if(Sendlist== NULL){
			printk(KERN_ERR "<pcie56_init_init>: Malloc Recv list buffer ERROR!\n" );
			goto fail_alloc_devsendlist;
		}
		memset(Sendlist,0,sizeof(struct send_descriptor)*MAXSENDQL);
		pcie56_devs[count].sendlistPh=dma_map_single(&pcie56->dev,Sendlist,sizeof(struct send_descriptor)*MAXSENDQL,DMA_BIDIRECTIONAL);
		printk("<pcie56_init_init>: sendlistBufferPh = 0x%x\n",(int)pcie56_devs[count].sendlistPh);
		printk("<pcie56_init_init>: sendlist_Buffer = 0x%x\n",(int)Sendlist);
		pcie56_devs[count].devicesend= (struct send_descriptor *)Sendlist;
		for (i=0;i<MAXSENDQL-1;i++){
			pcie56_devs[count].devicesend[i].length = 0;
			pcie56_devs[count].devicesend[i].PhAddr_low = (UINT)pcie56_devs[count].devicesendq[i].BufferPh;
			pcie56_devs[count].devicesend[i].PhAddr_hig =((UINT64)pcie56_devs[count].devicesendq[i].BufferPh)>>32;
			pcie56_devs[count].devicesend[i].NextDesc_low = pcie56_devs[count].sendlistPh+ (i+1)*sizeof(struct send_descriptor);
			pcie56_devs[count].devicesend[i].NextDesc_hig = (pcie56_devs[count].sendlistPh+ (i+1)*sizeof(struct send_descriptor))>>32;
			pcie56_devs[count].devicesend[i].SendFlagtag = 0;
			pcie56_devs[count].devicesend[i].SendFlagtag1 = 0;
			pcie56_devs[count].devicesend[i].SendFlagtag2 = 0;
		}
		pcie56_devs[count].devicesend[i].length = 0;
		pcie56_devs[count].devicesend[i].PhAddr_low = (UINT)pcie56_devs[count].devicesendq[i].BufferPh;
		pcie56_devs[count].devicesend[i].PhAddr_hig = ((UINT64)pcie56_devs[count].devicesendq[i].BufferPh)>>32;
		pcie56_devs[count].devicesend[i].NextDesc_low = pcie56_devs[count].sendlistPh ;
		pcie56_devs[count].devicesend[i].NextDesc_hig = ((UINT64) pcie56_devs[count].sendlistPh)>>32;
		pcie56_devs[count].devicesend[i].SendFlagtag = 0;
		pcie56_devs[count].devicesend[i].SendFlagtag1 = 0;
		pcie56_devs[count].devicesend[i].SendFlagtag2 = 0;

	//	alloc DMA buffer for recv Queue
		
		recvdma_buffer = (void *)dma_mem_alloc(DMA_FIFO_SIZE*MAXRECVQL);
		//pcie56_devs[count].devicerecvq[0].BufferPh = dma_map_single(pcie56,dma_buffer,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
		//PRINTK("<pcie56Drv_init>: RECV Dma Buffer is Create %x\n,RcvQ[0].BufferPh is %0x \n",(int)dma_buffer,pcie56_devs[count].devicerecvq[0].BufferPh);
		if(recvdma_buffer == NULL){
			PRINTK(KERN_ERR "<pcie56Drv_init>: Malloc Recv buffer ERROR!\n" );
			goto fail_alloc_devrecvbuff;
		}
		memset(recvdma_buffer,0,DMA_FIFO_SIZE*MAXRECVQL);
		for (i=0;i<MAXRECVQL;i++){ 
		  	pcie56_devs[count].devicerecvq[i].Buffer = recvdma_buffer+i*FRAMELEN;
			pcie56_devs[count].devicerecvq[i].BufferPh = 0;// pcie56_devs[count].devicerecvq[0].BufferPh+i*FRAMELEN;
	//		RcvQ[i].BufferPh = virt_to_bus(RcvQ[i].Buffer);
			pcie56_devs[count].devicerecvq[i].len=0;
		}
		//alloc encrypt Buffer
		pcie56_devs[count].SHead = 0;
		pcie56_devs[count].STail = 0;
		pcie56_devs[count].RHead = 0;
		pcie56_devs[count].RTail = 0;
		pcie56_devs[count].Slisthead = 0;
		pcie56_devs[count].Slisttail = 0;
//		pcie56_devs[count].Rlisthead = 0;
//		pcie56_devs[count].Rlisttail = 0;
		init_waitqueue_head(&(pcie56_devs[count].sendoutq));
		init_waitqueue_head(&(pcie56_devs[count].recvinq));
		SetBuffer_BYTE_ChgBELE(pcie56_devs[count].devicesend, sizeof(struct send_descriptor)*MAXSENDQL);
		
	}
	PRINTK("\n\n <pcie56_init>    alloc   RECV memery!\n\n");
//	alloc DMA buffer for recv Queue
	Recv_buffer= (void *)dma_mem_alloc(DMA_FIFO_SIZE*MAXRECVQL);
	if(!Recv_buffer){
		PRINTK(KERN_ERR "<pcie56Drv_init>: Malloc Recv buffer ERROR!\n" );
		goto fail_alloc_recvbuf;
	}
	memset(Recv_buffer,0,DMA_FIFO_SIZE*MAXRECVQL);
	RcvQ[0].BufferPh = dma_map_single(&pcie56->dev,Recv_buffer,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
	PRINTK("<pcie56Drv_init>: RECV Dma Buffer is Create %x\n,RcvQ[0].BufferPh is %0x \n",(int)Recv_buffer,RcvQ[0].BufferPh);
	for (i=0;i<MAXRECVQL;i++){ 
	  	RcvQ[i].Buffer = Recv_buffer+i*FRAMELEN;
		RcvQ[i].BufferPh = RcvQ[0].BufferPh+i*FRAMELEN;
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
	recvlistPh=dma_map_single(&pcie56->dev,Recvlist,sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
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
	//alloc DMA buffer for sf recv Queue
	Recv_buffer= (void *)dma_mem_alloc(DMA_FIFO_SIZE*MAXRECVQL);
	memset(Recv_buffer,0,DMA_FIFO_SIZE*MAXRECVQL);
	RsfQ[0].BufferPh = dma_map_single(&pcie56->dev,Recv_buffer,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
	PRINTK("<pcie56Drv_init>: RECV Dma Buffer is Create %x\n,RcvQ[0].BufferPh is %0x \n",(int)Recv_buffer,RsfQ[0].BufferPh);
//	PRINTK("<pcie56Drv_init>: RsfQ[0].BufferPh is Create 0x%08x\n",(int)RsfQ[0].BufferPh);
	if(!Recv_buffer){
		PRINTK(KERN_ERR "<pcie56Drv_init>: Malloc SF Recv buffer ERROR!\n" );
		goto fail_alloc_sfrecvbuf;
	}
	for (i=0;i<MAXRECVQL;i++){ 
	  	RsfQ[i].Buffer = Recv_buffer+i*FRAMELEN;
		RsfQ[i].BufferPh = RsfQ[0].BufferPh+i*FRAMELEN;
		RsfQ[i].len=0;
	}
//	 initialize recv S/G descriptor list
	Recvlist = (void *)dma_mem_alloc(sizeof(struct recv_descriptor)*MAXRECVQL);
//	Recvlist = pci_alloc_consistent(pcie56, sizeof(struct recv_descriptor)*MAXRECVQL ,&recvlistPh);
	if(!Recvlist){
		printk(KERN_ERR "<pcie56_init_init>: Malloc Recv list buffer ERROR!\n" );
		goto fail_alloc_sflist;
		}
	memset(Recvlist,0,sizeof(struct recv_descriptor)*MAXRECVQL);
	sflistPh=dma_map_single(&pcie56->dev,Recvlist,sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
	printk("<NP1_init>: sflistBufferPh = 0x%x\n",(int)sflistPh);
	printk("<pcie56_init>: sflist_Buffer = 0x%x\n",(int)Recvlist);
	sf_list= (struct recv_descriptor *)Recvlist;
	for (i=0;i<MAXRECVQL-1;i++){
		sf_list[i].status = 0;
		sf_list[i].PhAddr_low = (UINT)RsfQ[i].BufferPh;
		sf_list[i].PhAddr_hig = ( (UINT64)RsfQ[i].BufferPh)>>32;
		sf_list[i].NextDesc_low =(sflistPh + ((i+1)*sizeof(struct recv_descriptor)));	
		sf_list[i].NextDesc_hig =  ((UINT64)(sflistPh + ((i+1)*sizeof(struct recv_descriptor))))>>32;	
		sf_list[i].RecvFlagtag = 0;
		sf_list[i].RecvFlagtag1 = 0;
		sf_list[i].RecvFlagtag2 = 0;
		}
	sf_list[i].PhAddr_low = (UINT)RsfQ[i].BufferPh;
	sf_list[i].PhAddr_hig = ((UINT64)RsfQ[i].BufferPh)>>32;
	sf_list[i].status =0;
	sf_list[i].NextDesc_low =sflistPh;
	sf_list[i].NextDesc_hig = ((UINT64) sflistPh)>>32;
	sf_list[i].RecvFlagtag = 0;
	sf_list[i].RecvFlagtag1 = 0;
	recv_list[i].RecvFlagtag2 = 0;

	SetBuffer_BYTE_ChgBELE(recv_list, sizeof(struct recv_descriptor)*MAXRECVQL);
	SetBuffer_BYTE_ChgBELE(sf_list, sizeof(struct recv_descriptor)*MAXRECVQL);
	//SetBuffer_BYTE_ChgBELE(send_list, sizeof(struct send_descriptor)*MAXSENDQL);
// 	Init wait queue
	init_waitqueue_head(&sendinq);
	init_waitqueue_head(&recvoutq);
//	Init Send Queue head and tail, Receive Queue head and tail
	spin_lock_init(&recvlock);
	spin_lock_init(&sflock);
	SFHead = 0;
	SFTail = 0;
	RHead = 0;
	RTail = 0;
	Slisthead = 0;
	Slisttail = 0;
	Rlisthead = 0;
	Rlisttail = 0;
	Renhead = 0;				
   	Rentail = 0;		
	Localtail = 0;
#if	PCIE_INT
//	interrupt register/disable/enable
	ret = request_irq(pcie56->irq,
			pcie56Drv_interrupt,
			IRQF_SHARED,
			"pcie56",
			pcie56);	  
	if(ret) {

		PRINTK("<pcie56Drv_init>: Could not register interrupt\n");

		goto fail_req_irq;
		
	} else {    
	
		PRINTK("<pcie56Drv_init>: Register interrupt successful,irq 0x%lx\n",(unsigned long )pcie56->irq);
	}
#endif               	
//	interrupt register/disable/enable
	recvtask = kthread_run(recv_thread, NULL, "recv_kthread");
	if(!recvtask){
	       ret = PTR_ERR(recvtask);
	       goto fail_run_readthread;
	} 
	 sendtask = kthread_run(send_thread, NULL, "send_kthread");
      if(!sendtask){
           ret = PTR_ERR(sendtask);
           goto fail_run_sendthread;
       } 
	ret = read_BAR0(FPGA_SOFT_VERISON);
	//PRINTK("FPGA SOFTWARE VERISON is %08x\n",ret);
	PRINTK("FPGA SOFTWARE VERISON is %02d%02d%02d%02d%02d%02d\n",((ret>>17)&0x3f),((ret>>23)&0xf),((ret>>27)&0x1f),((ret>>12)&0x1f),((ret>>6)&0x3f),((ret>>0)&0x3f));
	//ret = read_BAR0(FPGA_HARD_STATUS);
	//PRINTK("PCIE STATUS is %08x\n",ret);
#if	PCIE_INT
	pcie56_int_enable();
#endif
	//write_BAR0(ECC_TX_RX_SET0, 0x24924924);
	//write_BAR0(ECC_TX_RX_SET1,0x00924924);
	start_dma1(); 
	//start_sfrecv_dma();
	return SUCCESS;

fail_run_sendthread:
	 kthread_stop(recvtask);
fail_run_readthread:
#if	PCIE_INT
	free_irq(pcie56->irq, pcie56);
fail_req_irq:
#endif
	dma_unmap_single(&pcie56->dev, recvlistPh, sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
	 dma_mem_free(recv_list, sizeof(struct recv_descriptor)*MAXSENDQL);
	
fail_alloc_sflist:
	dma_unmap_single(&pcie56->dev, RsfQ[0].BufferPh,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
	 dma_mem_free(RsfQ[0].Buffer, FRAMELEN*MAXSENDQL);

fail_alloc_sfrecvbuf:
	dma_unmap_single(&pcie56->dev, recvlistPh, sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
	 dma_mem_free(recv_list, sizeof(struct recv_descriptor)*MAXSENDQL);
fail_alloc_recvlist:
	 dma_unmap_single(&pcie56->dev, RcvQ[0].BufferPh,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
	 dma_mem_free(RcvQ[0].Buffer, FRAMELEN*MAXSENDQL);
 fail_alloc_recvbuf:
 	i = count; 
 	 for(;i>=0;i--){
		dma_unmap_single(&pcie56->dev,pcie56_devs[i].sendlistPh,FRAMELEN*MAXSENDQL,DMA_BIDIRECTIONAL);
	 	dma_mem_free(pcie56_devs[i].devicesend, FRAMELEN*MAXSENDQL);

		dma_unmap_single(&pcie56->dev,pcie56_devs[i].devicesendq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicesendq[0].Buffer, FRAMELEN*MAXSENDQL);
		
		//dma_unmap_single(pcie56,pcie56_devs[i].devicerecvq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicerecvq[0].Buffer, FRAMELEN*MAXSENDQL);
	}
	count = -1;
fail_alloc_devrecvbuff:
	  i = count;
	 for(;i>=0;i--){
		if(i != count){
			dma_unmap_single(&pcie56->dev,pcie56_devs[i].sendlistPh,FRAMELEN*MAXSENDQL,DMA_BIDIRECTIONAL);
	 		dma_mem_free(pcie56_devs[i].devicesend, FRAMELEN*MAXSENDQL);
			dma_unmap_single(&pcie56->dev,pcie56_devs[i].devicesendq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 		dma_mem_free(pcie56_devs[i].devicesendq[0].Buffer, FRAMELEN*MAXSENDQL);
		}
		dma_unmap_single(&pcie56->dev,pcie56_devs[i].sendlistPh,FRAMELEN*MAXSENDQL,DMA_BIDIRECTIONAL);
	 	dma_mem_free(pcie56_devs[i].devicesend, FRAMELEN*MAXSENDQL);
		dma_unmap_single(&pcie56->dev,pcie56_devs[i].devicesendq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicesendq[0].Buffer, FRAMELEN*MAXSENDQL);
		//dma_unmap_single(pcie56,pcie56_devs[i].devicerecvq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicerecvq[0].Buffer, FRAMELEN*MAXSENDQL);	
	}
	 count = -1;
	// count = 0;
fail_alloc_devsendlist:
	  i = count;
	 for(;i>=0;i--){
		if(i != count){
			dma_unmap_single(&pcie56->dev,pcie56_devs[i].devicesendq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 		dma_mem_free(pcie56_devs[i].devicesendq[0].Buffer, FRAMELEN*MAXSENDQL);
		}
		dma_unmap_single(&pcie56->dev,pcie56_devs[i].sendlistPh,FRAMELEN*MAXSENDQL,DMA_BIDIRECTIONAL);
	 	dma_mem_free(pcie56_devs[i].devicesend, FRAMELEN*MAXSENDQL);
		dma_unmap_single(&pcie56->dev,pcie56_devs[i].devicesendq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicesendq[0].Buffer, FRAMELEN*MAXSENDQL);
		//dma_unmap_single(pcie56,pcie56_devs[i].devicerecvq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicerecvq[0].Buffer, FRAMELEN*MAXSENDQL);	
	}
	 count = -1;
fail_alloc_devsendbuffer:
	 i = count;
	 for(;i>=0;i--){
		dma_unmap_single(&pcie56->dev,pcie56_devs[i].sendlistPh,FRAMELEN*MAXSENDQL,DMA_BIDIRECTIONAL);
	 	dma_mem_free(pcie56_devs[i].devicesend, FRAMELEN*MAXSENDQL);

		dma_unmap_single(&pcie56->dev,pcie56_devs[i].devicesendq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicesendq[0].Buffer, FRAMELEN*MAXSENDQL);
		
		//dma_unmap_single(pcie56,pcie56_devs[i].devicerecvq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
	 	dma_mem_free(pcie56_devs[i].devicerecvq[0].Buffer, FRAMELEN*MAXSENDQL);
	}
setup_cdev_err:
	for(;devcount>0;devcount--)
	 cdev_del(&pcie56_devs[devcount-1].cdev);
	unregister_chrdev_region(dev,DEVICE_COUNT);
deregister_chrdev_region:
	  iounmap(pcie56_BAR0_Addr);
	  kfree(pcie56_devs); 
	  pci_dev_put(pcie56);
OUT:
	return ret;
}

/***********************************************************************
	设备注销接口
***********************************************************************/
static void __exit pcie56Drv_cleanup(void)
{
	
	int i,devcount;	
	pcie56_opens = 0;
	if(pcie56 != NULL){
		 kthread_stop(sendtask);
		 kthread_stop(recvtask);
		 //free sfrecvlist
		 dma_unmap_single(&pcie56->dev, recvlistPh, sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
	 	 dma_mem_free(recv_list, sizeof(struct recv_descriptor)*MAXSENDQL);
		 //free sfrecvbuf
		dma_unmap_single(&pcie56->dev, RsfQ[0].BufferPh,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
		 dma_mem_free(RsfQ[0].Buffer, FRAMELEN*MAXSENDQL);
		 //free recvlist  
		dma_unmap_single(&pcie56->dev, recvlistPh, sizeof(struct recv_descriptor)*MAXRECVQL,DMA_BIDIRECTIONAL);
	 	dma_mem_free(recv_list, sizeof(struct recv_descriptor)*MAXSENDQL);
		 //free recv buf
		 dma_unmap_single(&pcie56->dev, RcvQ[0].BufferPh,FRAMELEN*MAXRECVQL,DMA_FROM_DEVICE);
		 dma_mem_free(RcvQ[0].Buffer, FRAMELEN*MAXSENDQL);
		 for(i=DEVICE_COUNT-1;i>=0;i--){
			dma_unmap_single(&pcie56->dev,pcie56_devs[i].sendlistPh,FRAMELEN*MAXSENDQL,DMA_BIDIRECTIONAL);
		 	dma_mem_free(pcie56_devs[i].devicesend, FRAMELEN*MAXSENDQL);
			dma_unmap_single(&pcie56->dev,pcie56_devs[i].devicesendq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
		 	dma_mem_free(pcie56_devs[i].devicesendq[0].Buffer, FRAMELEN*MAXSENDQL);
			
			//dma_unmap_single(pcie56,pcie56_devs[i].devicerecvq[0].BufferPh,FRAMELEN*MAXSENDQL,DMA_TO_DEVICE);
		 	dma_mem_free(pcie56_devs[i].devicerecvq[0].Buffer, FRAMELEN*MAXSENDQL);
		}
	for(devcount = DEVICE_COUNT;devcount>0;devcount--)
		 cdev_del(&pcie56_devs[devcount-1].cdev);
	/*取消中断*/
#if	PCIE_INT
	pcie56_int_disable();
#endif
	write_BAR0(DMA_RCV_CTRL, DMA_STATUS_ABORT);
	write_BAR0(DMA_SFRCV_CTRL, DMA_STATUS_ABORT);
	write_BAR0(FPGA_SW_RST, 0x5a5a5a5a);
//	 free_irq(pcie56->irq, pcie56);
	unregister_chrdev_region(MKDEV (pcie56_major, 0),DEVICE_COUNT); 
	iounmap(pcie56_BAR0_Addr);
	/*注销PCIE设备*/
		 
	 pci_dev_put(pcie56);

	}
	PRINTK("<pcie56Drv_cleanup> over!\n");
	PRINTK("###################################################\n");
}
EXPORT_SYMBOL(data_read);
EXPORT_SYMBOL(data_write);
EXPORT_SYMBOL(start_write);
EXPORT_SYMBOL(end_write);
EXPORT_SYMBOL(start_sendDMA);
EXPORT_SYMBOL(end_sendDMA);
EXPORT_SYMBOL(start_recvDMA);
EXPORT_SYMBOL(end_recvDMA);
EXPORT_SYMBOL(start_read);
EXPORT_SYMBOL(end_read);
/*module init */
module_init(pcie56Drv_init);
/* module_exit*/
module_exit(pcie56Drv_cleanup);
