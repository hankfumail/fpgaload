#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>

/* Configurable parameteres: START */
#define CONFIG_SYS_SDRAM_SIZE		(256 * 1024 * 1024)
#define CONFIG_SYS_MEM_TOP_HIDE         ( 16 * 1024 * 1024)
#define CONFIG_BUF_GRANULARITY          (        16 * 1024)

#define CONFIG_SYS_FPGA_WAIT            1000000  /* 1000ms */
/* Configurable parameteres: END */

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
/* SLCR definitions */
#define BASE_ADDR_SLCR              0xF8000000
#define SIZE_SLCR                   0x1000   
#define OFFSET_SLCR_IO_PLL_CTRL     0x108
#define OFFSET_SLCR_IO_PLL_CFG      0x118
#define OFFSET_SLCR_PCAP_CLK_CTRL   0x168
#define OFFSET_SLCR_UNLOCK          0x008
#define OFFSET_SLCR_LOCK            0x004
#define OFFSET_SLCR_LVL_SHFTR_EN    0x900
#define OFFSET_SLCR_FPGA_RESET    	0x240

#define SLCR_UNLOCK_VAL             0xDF0D
#define SLCR_LOCK_VAL               0x767B

#define BASE_ADDR_GPIO              0xE000A000
#define SIZE_GPIO                   0x1000   
//volatile unsigned int *p_gpio_emio_input = (unsigned int *)0xe000a068;
#define OFFSET_GPIO_INPUT2            0x068
#define OFFSET_GPIO_INPUT3            0x06c
#define OFFSET_GPIO_INT_TYPE2         0x29C
#define OFFSET_GPIO_INT_TYPE3         0x2dc


/* DEVCFG definitions */
#define BASE_ADDR_DEVCFG            0xF8007000
#define SIZE_DEVCFG                 0x1000   

#define XDCFG_CTRL_OFFSET		0x00 /* Control Register */
#define XDCFG_LOCK_OFFSET		0x04 /* Lock Register */
#define XDCFG_CFG_OFFSET		0x08 /* Configuration Register */
#define XDCFG_INT_STS_OFFSET		0x0C /* Interrupt Status Register */
#define XDCFG_INT_MASK_OFFSET		0x10 /* Interrupt Mask Register */
#define XDCFG_STATUS_OFFSET		0x14 /* Status Register */
#define XDCFG_DMA_SRC_ADDR_OFFSET	0x18 /* DMA Source Address Register */
#define XDCFG_DMA_DEST_ADDR_OFFSET	0x1C /* DMA Destination Address Reg */
#define XDCFG_DMA_SRC_LEN_OFFSET	0x20 /* DMA Source Transfer Length */
#define XDCFG_DMA_DEST_LEN_OFFSET	0x24 /* DMA Destination Transfer */
#define XDCFG_ROM_SHADOW_OFFSET		0x28 /* DMA ROM Shadow Register */
#define XDCFG_MULTIBOOT_ADDR_OFFSET	0x2C /* Multi BootAddress Pointer */
#define XDCFG_SW_ID_OFFSET		0x30 /* Software ID Register */
#define XDCFG_UNLOCK_OFFSET		0x34 /* Unlock Register */
#define XDCFG_MCTRL_OFFSET		0x80 /* Miscellaneous Control Reg */

#define XDCFG_CTRL_PCFG_PROG_B_MASK	0x40000000 /* PROGRAM_B */
#define XDCFG_CTRL_PCAP_PR_MASK	  	0x08000000 /* Enable PCAP for PR */
#define XDCFG_CTRL_PCAP_MODE_MASK	0x04000000 /* Enable PCAP */
#define XDCFG_CTRL_PCAP_RATE_EN_MASK	0x02000000 /* Enable PCAP send data to FPGA every 4 PCAP cycles */
#define XDCFG_IXR_FATAL_ERROR_MASK	0x00740040
#define XDCFG_IXR_RX_FIFO_OV_MASK	0x00040000
#define XDCFG_IXR_DMA_DONE_MASK		0x00002000 /* DMA Command Done */
#define XDCFG_IXR_D_P_DONE_MASK		0x00001000 /* DMA and PCAP transfers Done */
#define XDCFG_IXR_PCFG_DONE_MASK	0x00000004 /* Done Signal Mask */

#define XDCFG_STATUS_DMA_CMD_Q_F_MASK   0x80000000 /* DMA command Queue full */
#define XDCFG_STATUS_DMA_CMD_Q_E_MASK	0x40000000 /* DMA command Queue empty */
#define XDCFG_STATUS_DMA_DONE_CNT_MASK	0x30000000 /* Nr of completed DMA transfers that have not been acknowledged by software*/
#define XDCFG_STATUS_PCFG_INIT_MASK	0x00000010 /* FPGA Init Status */

#define XDCFG_MCTRL_PCAP_LPBK_MASK	0x00000010 /* PCAP loopback mask */
#define XDCFG_MCTRL_RFIFO_FLUSH	        0x00000002
#define XDCFG_MCTRL_WFIFO_FLUSH	        0x00000001

#define XDCFG_DMA_INVALID_ADDRESS	0xFFFFFFFF /* Invalid DMA address */

unsigned int BaseAddrMembuf;
unsigned int SizeMembuf;

unsigned int RegRead(void* base, unsigned int offset) {
    return *((volatile unsigned int *)((unsigned int)base+offset));
}

void RegWrite(void* base, unsigned int offset, unsigned int val) {
    volatile unsigned int *p = (volatile unsigned int *)((unsigned int)base+offset);
    *p = val;
}

int fpga_reset_test( int fd_mem, void *mapped_base_slcr )
{
	static unsigned int u32_call_num=0;
	volatile unsigned int u32_loop=0;
	volatile unsigned int u32_int_type;
	volatile unsigned int u32_rst_num_raw;
	volatile unsigned int u32_rst_num_overflow=0;
	volatile unsigned int u32_rst_num;
	//volatile unsigned int *p_gpio_emio_input = (unsigned int *)0xe000a068;
	//volatile unsigned int *p_fpga_reset_output = (unsigned int *)0xF8000240;
	
    void *mapped_base_gpio;

	u32_call_num++;
	printf("%s runs No.%d time\n\r", __func__, u32_call_num );
	
    /* Map GPIO register space into user space */
    mapped_base_gpio = mmap(0, SIZE_GPIO, PROT_READ | PROT_WRITE, 
			    MAP_SHARED, fd_mem, BASE_ADDR_GPIO  & ~MAP_MASK);    
    if (mapped_base_gpio == (void *) -1) 
	{
        printf("Can't map GPIO registers to user space.\n");
        return  -200;
    }
    printf("Mapped GPIO at virtual address %p.\n", mapped_base_gpio);
	
	u32_int_type =  RegRead(mapped_base_gpio, OFFSET_GPIO_INT_TYPE2);
	if( 0xffffffff != u32_int_type  )
	{
		printf("u32_int_type 2:0x%08x,GPIO access has problem.\n\r", 
				u32_int_type );


		/* SLCR unlock */
		RegWrite(mapped_base_slcr, OFFSET_SLCR_UNLOCK, SLCR_UNLOCK_VAL );
		// Caution:
		// Set GPIO clock.
		RegWrite(mapped_base_slcr, 0x12c, 0x00e44405 );
		/* SLCR lock */
		RegWrite(mapped_base_slcr, OFFSET_SLCR_LOCK, SLCR_LOCK_VAL );
	}
	else
	{
		printf("u32_int_type 2:0x%08x,GPIO access works well.\n\r", 
				u32_int_type );
	}
	
	u32_int_type =  RegRead(mapped_base_gpio, OFFSET_GPIO_INT_TYPE3);
	if( 0xffffffff != u32_int_type  )
	{
		printf("u32_int_type 3:0x%08x,GPIO access has problem.\n\r", 
				u32_int_type );

		/* SLCR unlock */
		RegWrite(mapped_base_slcr, OFFSET_SLCR_UNLOCK, SLCR_UNLOCK_VAL );
		// Caution:
		// Set GPIO clock.
		RegWrite(mapped_base_slcr, 0x12c, 0x00e44405 );
		/* SLCR lock */
		RegWrite(mapped_base_slcr, OFFSET_SLCR_LOCK, SLCR_LOCK_VAL );
	}
	else
	{
		printf("u32_int_type 3:0x%08x,GPIO access works well.\n\r", 
				u32_int_type );
	}

	
	/* SLCR unlock */
	RegWrite(mapped_base_slcr, OFFSET_SLCR_UNLOCK, SLCR_UNLOCK_VAL );

	//for( u32_loop =0; u32_loop<10000000; u32_loop++ )
	//for( u32_loop =0; u32_loop<1000; u32_loop++ )
	{
		// 0xF8000240
		RegWrite(mapped_base_slcr, OFFSET_SLCR_FPGA_RESET, 0x0 );
		usleep(1);
		RegWrite(mapped_base_slcr, OFFSET_SLCR_FPGA_RESET, 0xf );
		usleep(1);
		RegWrite(mapped_base_slcr, OFFSET_SLCR_FPGA_RESET, 0x0 );
		usleep(1);
		u32_rst_num_raw =  RegRead(mapped_base_gpio, OFFSET_GPIO_INPUT3);
		u32_rst_num =  (u32_rst_num_raw&0xffff0000)>>16;
		if( 0 == (u32_loop%10000)  )
		{
			printf("Reset num raw:0x%08x, reset num:%d-%d at No.%d loop.\n\r", 
					u32_rst_num_raw, u32_rst_num, u32_rst_num_overflow*0x10000+u32_rst_num, u32_loop );
		}
		
		if( u32_rst_num >=65535 )
		{
			printf("Reset num raw:0x%08x, reset num:%d-%d at No.%d loop.\n\r", 
					u32_rst_num_raw, u32_rst_num, u32_rst_num_overflow*0x10000+u32_rst_num, u32_loop );
			u32_rst_num_overflow++;
		}
	}
	printf("Reset num raw:0x%08x, reset num:%d-%d at No.%d loop.\n\r", 
			u32_rst_num_raw, u32_rst_num, u32_rst_num_overflow*0x10000+u32_rst_num, u32_loop );
	
	/* SLCR lock */
	RegWrite(mapped_base_slcr, OFFSET_SLCR_LOCK, SLCR_LOCK_VAL );
	
    if (munmap(mapped_base_gpio, SIZE_GPIO) == -1) {
        printf("Can't unmap GPIO from user space.\n");
		return -200;
    }

    return 0;
}


int main(int argc, char **argv)
{
    int fd_mem;
    int retval=0;
    void *mapped_base_slcr;

	
	printf("fpga_reset compilation time:%s-%s\r\n",__DATE__,__TIME__);


    fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd_mem == -1) {
        printf("Can't open /dev/mem.\n");
        exit(-1);
    }
    printf("/dev/mem opened.\n");

    /* Map SLCR register space into user space */
    mapped_base_slcr = mmap(0, SIZE_SLCR, PROT_READ | PROT_WRITE, MAP_SHARED, 
				    fd_mem, BASE_ADDR_SLCR  & ~MAP_MASK);    
    if (mapped_base_slcr == (void *) -1) {
        printf("Can't map SLCR registers to user space.\n");
        retval=-100;
        goto fail_100;
        
    }
    printf("Mapped SLCR at virtual address %p.\n", mapped_base_slcr);


    /* TODO: Add your application specific initialization here. */
	fpga_reset_test( fd_mem, mapped_base_slcr );

    printf("App exit gracefully.\r\n");

    if (munmap(mapped_base_slcr, SIZE_SLCR) == -1) {
        printf("Can't unmap SLCR from user space.\n");
    }
fail_100:
	//printf("fail_100, File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
    close(fd_mem);
    return retval;
}
