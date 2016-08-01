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

unsigned int diff_tv(struct timeval start, struct timeval end) {
    return (end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);
}


void dbg_mem_word_dump( unsigned int *pu32_mem_base, unsigned int u32_byte_num )
{
    unsigned int u32_loop;
    
    for( u32_loop=0; u32_loop<(u32_byte_num/sizeof(unsigned int)); u32_loop++ )
    {
        if( 0 == (u32_loop%4) )
        {
            printf("\n\r0x%08x-%08x:", (unsigned int)(&pu32_mem_base[u32_loop]), u32_loop*4);
        }
        printf(" %08x", pu32_mem_base[u32_loop]);
    }
    printf("\n\r");
}

void mem_byte_swap( unsigned int *pu32_mem_base, unsigned int u32_byte_num )
{
    unsigned int u32_loop;
    unsigned int u32_data;
    unsigned int u32_temp0;
    unsigned int u32_temp1;
    unsigned int u32_temp2;
    unsigned int u32_temp3;

	if( 0 != (u32_byte_num%4) )
	{
		printf("mem_byte_swap: bytes number is not good.\n\r");
	}
    
    for( u32_loop=0; u32_loop<(u32_byte_num/sizeof(unsigned int)); u32_loop++ )
    {
		u32_data = pu32_mem_base[u32_loop];
		u32_temp0 = (u32_data&0xff)<<24;
		u32_temp1 = (u32_data&0xff00)<<8;
		u32_temp2 = (u32_data&0xff0000)>>8;
		u32_temp3 = (u32_data&0xff000000)>>24;
		pu32_mem_base[u32_loop] = u32_temp0|u32_temp1|u32_temp2|u32_temp3; 
#if 0
		{
			static int debug_flag=0;
			if(debug_flag<4) 
			{
				printf("\n\ru32_data: %08x at loop:%d.\n\r", u32_data, u32_loop);
				printf("u32_temp0: %02x.\n\r", u32_temp0);
				printf("u32_temp1: %02x.\n\r", u32_temp1);
				printf("u32_temp2: %02x.\n\r", u32_temp2);
				printf("u32_temp3: %02x.\n\r", u32_temp3);
				printf("pu32_mem_base[u32_loop]: %08x at loop:%d.\n\r", pu32_mem_base[u32_loop], u32_loop);
			}
			debug_flag++;
		}
#endif
    }
}

int fpga_reset_test( int fd_mem, void *mapped_base_slcr )
{
	static unsigned int u32_call_num=0;
	volatile unsigned int u32_loop;
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
	for( u32_loop =0; u32_loop<10; u32_loop++ )
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
    int fd_mem, fd_bin;
    off_t file_length, file_length_remaining, file_length_read;
    int retval=0;
    void *mapped_base_slcr, *mapped_base_devcfg, *mapped_base_membuf;
    unsigned int regPcapClkCtrl=0, regPcapClkCtrlBackuped=0;
    unsigned int IntrStsReg, CtrlReg, StatusReg, PartialCfg = 0;
    unsigned int i;
    struct timeval tvStart, tvEnd; 
	
	printf("fpga_load compilation time:%s-%s\r\n",__DATE__,__TIME__);

    if(argc!=4) {
        printf("Usage: fpga_load <buffer phyical address in hex> <buffer size in hex> <filename>\n");
        printf("     Please reserve the buffer in U-Boot and Linux.\n");
        exit(0);
    }
    BaseAddrMembuf = strtol(argv[1], NULL, 16);
    SizeMembuf     = strtol(argv[2], NULL, 16);
    
    /* sanity check */
    if( (BaseAddrMembuf<(CONFIG_SYS_SDRAM_SIZE-CONFIG_SYS_MEM_TOP_HIDE)) 
        || ((BaseAddrMembuf+SizeMembuf)>CONFIG_SYS_SDRAM_SIZE) ) {
        printf("Err: buffer should be in range [0x%x, 0x%x]\n", CONFIG_SYS_SDRAM_SIZE-CONFIG_SYS_MEM_TOP_HIDE,
                                                                CONFIG_SYS_SDRAM_SIZE-1 );
        exit(0);
    }
    if( BaseAddrMembuf & (CONFIG_BUF_GRANULARITY-1) ) {
        printf("Err: buffer base addrese should be multiple of 0x%x\n", CONFIG_BUF_GRANULARITY );
        exit(0);
    }    
    if( (SizeMembuf<CONFIG_BUF_GRANULARITY) || (SizeMembuf & (CONFIG_BUF_GRANULARITY-1)) ) {
        printf("Err: buffer size should be multiple of 0x%x\n", CONFIG_BUF_GRANULARITY );
        exit(0);
    }
    printf("Using buffer [0x%x, 0x%x]\n", BaseAddrMembuf, BaseAddrMembuf+SizeMembuf-1);

    fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd_mem == -1) {
        printf("Can't open /dev/mem.\n");
        exit(-1);
    }
    printf("/dev/mem opened.\n");

    /* Map SLCR register space into user space */
    mapped_base_slcr = mmap(0, SIZE_SLCR, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, BASE_ADDR_SLCR  & ~MAP_MASK);    
    if (mapped_base_slcr == (void *) -1) {
        printf("Can't map SLCR registers to user space.\n");
        retval=-100;
        goto fail_100;
        
    }
    printf("Mapped SLCR at virtual address %p.\n", mapped_base_slcr);

    /* make sure PCAP clock is enabled. */
    regPcapClkCtrl = RegRead(mapped_base_slcr, OFFSET_SLCR_PCAP_CLK_CTRL);
    if( regPcapClkCtrl & 0x1 ) {
        printf("PCAP clock has already been enabled.\n");
        regPcapClkCtrlBackuped = 0;
    }
    else {
        printf("PCAP clock is turned on.\n");
        RegWrite(mapped_base_slcr, OFFSET_SLCR_UNLOCK, SLCR_UNLOCK_VAL );
        RegWrite(mapped_base_slcr, OFFSET_SLCR_PCAP_CLK_CTRL, regPcapClkCtrl|0x1 );
        RegWrite(mapped_base_slcr, OFFSET_SLCR_LOCK, SLCR_LOCK_VAL );
        regPcapClkCtrlBackuped = 1;
    }

    /* Map DevCfg register space into user space */
    mapped_base_devcfg = mmap(0, SIZE_DEVCFG, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, BASE_ADDR_DEVCFG & ~MAP_MASK);    
    if (mapped_base_devcfg == (void *) -1) {
        printf("Can't map DevCfg registers to user space.\n");
        retval=-101;
        goto fail_101;
        
    }
    printf("Mapped DevCfg at virtual address %p.\n", mapped_base_devcfg);

    /* Map Mem Buffer into user space */
    mapped_base_membuf = mmap(0, SizeMembuf, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, BaseAddrMembuf & ~MAP_MASK);    
    if (mapped_base_membuf == (void *) -1) {
        printf("Can't map Mem Buffer to user space.\n");
        retval=-102;
        goto fail_102;
        
    }
    printf("Mapped MemBuf at virtual address %p.\n", mapped_base_membuf);

    /* Open the .bit.bin file */
    fd_bin = open(argv[3], O_RDONLY | O_SYNC);
    if (fd_bin == -1) {
        printf("Can't open bitstream file(%s).\n", argv[3]);
        retval=-103;
        goto fail_103;
    }
    file_length_remaining = file_length = lseek(fd_bin, 0, SEEK_END);
    printf("Stripped bitstream file( %s ) opened, file length is %d.\n", argv[3], (int)file_length);

    /* Detect whether it is a full configure or partial re-configure by reading IXR_PCFG_DONE in INT_STS */
    IntrStsReg = RegRead(mapped_base_devcfg, XDCFG_INT_STS_OFFSET);
#if 0
    if (IntrStsReg & XDCFG_IXR_PCFG_DONE_MASK) {
        printf("DevCfg: PCFG_DONE is detected.  Start partial re-configuration.\n");
	PartialCfg = 1;
    }
    else 
#endif		
	{
        printf("DevCfg: PCFG_DONE not detected. Start full programming.\n");
    }
    /* clear XDCFG_IXR_PCFG_DONE_MASK */
    RegWrite(mapped_base_devcfg, XDCFG_INT_STS_OFFSET, XDCFG_IXR_PCFG_DONE_MASK);

    /* Disable the level-shifters from PS to PL. */
    if (!PartialCfg) {
        RegWrite(mapped_base_slcr, OFFSET_SLCR_UNLOCK, SLCR_UNLOCK_VAL );
        RegWrite(mapped_base_slcr, OFFSET_SLCR_LVL_SHFTR_EN, 0x0 );
        RegWrite(mapped_base_slcr, OFFSET_SLCR_LOCK, SLCR_LOCK_VAL );
    }

    /* Mask all interrupts, as we cannot use interrupt in Linux user space and only polling is possible. */
    RegWrite(mapped_base_devcfg, XDCFG_INT_MASK_OFFSET, 0xFFFFFFFF );

    /* Select PCAP interface and enable it. */
    CtrlReg = RegRead(mapped_base_devcfg, XDCFG_CTRL_OFFSET);
    RegWrite(mapped_base_devcfg, XDCFG_CTRL_OFFSET, CtrlReg | XDCFG_CTRL_PCAP_MODE_MASK | XDCFG_CTRL_PCAP_PR_MASK );
    
    /* Clear internal PCAP loopback */
    CtrlReg = RegRead(mapped_base_devcfg, XDCFG_MCTRL_OFFSET);
    RegWrite(mapped_base_devcfg, XDCFG_MCTRL_OFFSET, (CtrlReg & ~(XDCFG_MCTRL_PCAP_LPBK_MASK)) );

    /* Clear QUARTER_PCAP_RATE_EN bit so that the PCAP data is transmitted every clock.
       This is for Non secure PCAP write. */
    CtrlReg = RegRead(mapped_base_devcfg, XDCFG_CTRL_OFFSET);
    RegWrite(mapped_base_devcfg, XDCFG_CTRL_OFFSET, (CtrlReg & ~XDCFG_CTRL_PCAP_RATE_EN_MASK) );

    /* Setting PCFG_PROG_B signal to high */
    CtrlReg = RegRead(mapped_base_devcfg, XDCFG_CTRL_OFFSET);
    RegWrite(mapped_base_devcfg, XDCFG_CTRL_OFFSET, (CtrlReg | XDCFG_CTRL_PCFG_PROG_B_MASK) );

    /* Setting PCFG_PROG_B signal to low */
    RegWrite(mapped_base_devcfg, XDCFG_CTRL_OFFSET, (CtrlReg & ~(XDCFG_CTRL_PCFG_PROG_B_MASK)) );

    /* Polling the PCAP_INIT status for Reset */
    gettimeofday(&tvStart, NULL);
    while (RegRead(mapped_base_devcfg, XDCFG_STATUS_OFFSET) & XDCFG_STATUS_PCFG_INIT_MASK) {
        gettimeofday(&tvEnd, NULL);
	if (diff_tv(tvStart, tvEnd) > CONFIG_SYS_FPGA_WAIT) {
	    printf("Err: Timeout wait for PCFG_INIT to clear\n");
            retval=-104;
	    goto fail_104;
	}
    }

    /* Setting PCFG_PROG_B signal to high */
    CtrlReg = RegRead(mapped_base_devcfg, XDCFG_CTRL_OFFSET);
    RegWrite(mapped_base_devcfg, XDCFG_CTRL_OFFSET, (CtrlReg | XDCFG_CTRL_PCFG_PROG_B_MASK) );

    /* Polling the PCAP_INIT status for Set. 
     * PL INIT signal, indicates when housecleaning is done and the PL is ready to receive PCAP data.*/
    gettimeofday(&tvStart, NULL);
    while ( !(RegRead(mapped_base_devcfg, XDCFG_STATUS_OFFSET) & XDCFG_STATUS_PCFG_INIT_MASK) ) {
        gettimeofday(&tvEnd, NULL);
	if (diff_tv(tvStart, tvEnd) > CONFIG_SYS_FPGA_WAIT) {
	    printf("Err: Timeout wait for PCFG_INIT to set\n");
            retval=-104;
	    goto fail_104;
	}
    }
	printf("File: %s, Func: %s, Line: %d, PCAP_INIT status is good.\n", __FILE__, __func__, __LINE__ );

    /* Clear all interrupt status bits. */
    RegWrite(mapped_base_devcfg, XDCFG_INT_STS_OFFSET, 0xFFFFFFFF );

    if (IntrStsReg & XDCFG_IXR_FATAL_ERROR_MASK) {
        printf("Err: Fatal errors in isr_status 0x%X\n", IntrStsReg);

		/* If RX FIFO overflow, need to flush RX FIFO first */
		if (IntrStsReg & XDCFG_IXR_RX_FIFO_OV_MASK) {
	            RegWrite(mapped_base_devcfg, XDCFG_MCTRL_OFFSET, XDCFG_MCTRL_RFIFO_FLUSH	 );
		}
        retval=-104;
	goto fail_104;
		
    }
	printf("File: %s, Func: %s, Line: %d, Flush RX FIFO\n", __FILE__, __func__, __LINE__ );


    /* Check if DMA command queue is full */
    StatusReg = RegRead(mapped_base_devcfg, XDCFG_STATUS_OFFSET);
    if (StatusReg & XDCFG_STATUS_DMA_CMD_Q_F_MASK) {
	printf("Err: device busy. DMA command queue is full\n");
        retval=-104;
	goto fail_104;
    }
	printf("File: %s, Func: %s, Line: %d, DMA is ready.\n", __FILE__, __func__, __LINE__ );

    if (!(StatusReg & XDCFG_STATUS_DMA_CMD_Q_E_MASK)) {
		if (!(RegRead(mapped_base_devcfg, XDCFG_INT_STS_OFFSET) & XDCFG_IXR_DMA_DONE_MASK)) {
	            /* Error state, transfer cannot occur */
		    printf("Err: DMA command queue is not empty and DMA_DONE not set\n");
	            retval=-104;
		    goto fail_104;
		} 
	        else {
		    /* Clear out the interrupt status */
	            RegWrite(mapped_base_devcfg, XDCFG_INT_STS_OFFSET, XDCFG_IXR_DMA_DONE_MASK );
		}
    }
	printf("File: %s, Func: %s, Line: %d, Clear all interrupt status\n", __FILE__, __func__, __LINE__ );

    if (StatusReg & XDCFG_STATUS_DMA_DONE_CNT_MASK) {
	/* Clear the count of completed DMA transfers */
        RegWrite(mapped_base_devcfg, XDCFG_STATUS_OFFSET, XDCFG_STATUS_DMA_DONE_CNT_MASK );
    }
	printf("File: %s, Func: %s, Line: %d, Clear all DMA status\n", __FILE__, __func__, __LINE__ );

    /* Download bitstream in non secure mode */

    /* Start Programmming PL */
    printf("Zynq Programming PL started.\n\n");
    for(i=0; i<(file_length+SizeMembuf-1)/SizeMembuf; i++) {
        /* read the file into mem buffer. */
        lseek(fd_bin, i*SizeMembuf, SEEK_SET);
        file_length_read = file_length_remaining;
        if (file_length_read>SizeMembuf)
            file_length_read = SizeMembuf;
        read(fd_bin, mapped_base_membuf, file_length_read);
        file_length_remaining -= file_length_read;

		{
			unsigned int ui_swap_flag=0;
			unsigned int *pui_data = (unsigned int *)mapped_base_membuf;
			
			/* Swap data
			     Unit: 32-bit word, swap byte in the word.
			     0x12345678-> 0x78563412		     
			     vivado GUI bin: 0x00-00-00-bb at 0x20, 0xaa-99-55-66 at 0x30 in hex editor
			     Zynq Arm processor value for vivado GUI bin: 0xbb000000 at 0x20, 0x665599aa at 0x30
			     SDK bootgen .bit.bin: 0xbb-00-00-00 at 0x20, 0x66-55-99-aa at 0x30 in hex editor
			     Zynq Arm processor value for vivado GUI bin: 0x000000bb at 0x20, 0xaa995566 at 0x30
			*/
			if( 0 == i )
			{
				dbg_mem_word_dump(pui_data, 128);

				if( ( 0xbb000000 == pui_data[0x20/sizeof(unsigned int)] ) || ( 0x665599aa == pui_data[0x30/sizeof(unsigned int)] ) )
				{
					//printf("File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
					printf("Vivado GUI bin file.\n" );
					ui_swap_flag =1;					
				}
				else if( ( 0x000000bb == pui_data[0x20/sizeof(unsigned int)] ) || ( 0xaa995566 == pui_data[0x30/sizeof(unsigned int)] ) )
				{
					//printf("File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
					printf("SDK bootgen .bit.bin file.\n" );
				}
				else
				{
					printf("Unstripped bitstream file or unknown file.\n" );
				}
			}
			if( 1 == ui_swap_flag )
			{
				mem_byte_swap( pui_data, file_length_read );
				if( 0 == i )
				{
					printf("\nData after swapping.\n" );
					dbg_mem_word_dump(pui_data, 128);
				}
			}
		}
		

        /* Setting SRC_ADDR[1:0] and DST_ADDR[1:0] to 2'b01 will cause the DMA engine to hold the DMA
         * DONE interrupt until both the AXI and PCAP interfaces are done with the data transfer.
         * Otherwise the interrupt will trigger as soon as the AXI interface is done.
         * SRC_ADDR[1:0] is set to to 2'b01 only for last chunk of data. 
         */
			printf("File: %s, Func: %s, Line: %d, Start DMA operation\n", __FILE__, __func__, __LINE__ );
        if( file_length_remaining==0 )
            RegWrite(mapped_base_devcfg, XDCFG_DMA_SRC_ADDR_OFFSET,  BaseAddrMembuf+1);
        else
            RegWrite(mapped_base_devcfg, XDCFG_DMA_SRC_ADDR_OFFSET,  BaseAddrMembuf);
        RegWrite(mapped_base_devcfg, XDCFG_DMA_DEST_ADDR_OFFSET, XDCFG_DMA_INVALID_ADDRESS);
        RegWrite(mapped_base_devcfg, XDCFG_DMA_SRC_LEN_OFFSET,   file_length_read/4);
        RegWrite(mapped_base_devcfg, XDCFG_DMA_DEST_LEN_OFFSET,  0);        

		/* Poll IXR_DMA_DONE */
		printf("File: %s, Func: %s, Line: %d, waiting for IXR_DMA_DONE\n", __FILE__, __func__, __LINE__ );
        gettimeofday(&tvStart, NULL);
        while ( !(RegRead(mapped_base_devcfg, XDCFG_INT_STS_OFFSET) & XDCFG_IXR_DMA_DONE_MASK) ) {
            gettimeofday(&tvEnd, NULL);
	    if (diff_tv(tvStart, tvEnd) > CONFIG_SYS_FPGA_WAIT) {
	        printf("Err: Timeout waiting for DMA_DONE at chunk %d\n", i);
                retval=-104;
	        goto fail_104;
	    }
        }
        /* clear the interrupt status */
        RegWrite(mapped_base_devcfg, XDCFG_INT_STS_OFFSET, XDCFG_IXR_DMA_DONE_MASK);
    }
	printf("File: %s, Func: %s, Line: %d, File is over.\n", __FILE__, __func__, __LINE__ );

    if (PartialCfg) {
		printf("File: %s, Func: %s, Line: %d, waiting for IXR_D_P_DONE\n", __FILE__, __func__, __LINE__ );
	/* Poll IXR_D_P_DONE */
	while ((IntrStsReg & XDCFG_IXR_D_P_DONE_MASK) != XDCFG_IXR_D_P_DONE_MASK) {
	    IntrStsReg = RegRead(mapped_base_devcfg, XDCFG_INT_STS_OFFSET);
	}
    } else {
		printf("File: %s, Func: %s, Line: %d, waiting for IXR_PCFG_DONE\n", __FILE__, __func__, __LINE__ );
        /* Poll IXR_PCFG_DONE.
         * IXR_PCFG_DONE signal from PL indicating that programming is complete and PL is in user mode.
         * It is left uncleared to indicate PR for later programming.
         */
	while ((IntrStsReg & XDCFG_IXR_PCFG_DONE_MASK) != XDCFG_IXR_PCFG_DONE_MASK) {
	    IntrStsReg = RegRead(mapped_base_devcfg, XDCFG_INT_STS_OFFSET);
	}
	/* Enable the level-shifters from PS to PL. */
	printf("Enable the level-shifters, File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
        RegWrite(mapped_base_slcr, OFFSET_SLCR_UNLOCK, SLCR_UNLOCK_VAL );
        RegWrite(mapped_base_slcr, OFFSET_SLCR_LVL_SHFTR_EN, 0xF );
#if 0
        //RegWrite(mapped_base_slcr, OFFSET_SLCR_FPGA_RESET, 0x0 );
        //RegWrite(mapped_base_slcr, OFFSET_SLCR_FPGA_RESET, 0xF );
        //RegWrite(mapped_base_slcr, OFFSET_SLCR_FPGA_RESET, 0x0 );
    	printf("Reset PL, Func: %s, Line: %d\n", __func__, __LINE__ );
#else
    	printf("Does not reset PL, Func: %s, Line: %d\n", __func__, __LINE__ );
#endif
        RegWrite(mapped_base_slcr, OFFSET_SLCR_LOCK, SLCR_LOCK_VAL );
    }

    sleep(1);
    printf("Zynq Programming PL finished.\n");

    /* TODO: Add your application specific initialization here. */
	//fpga_reset_test( fd_mem, mapped_base_slcr );

    printf("App exit gracefully.\r\n");

fail_104:
	//printf("fail_104, File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
    close(fd_bin);
fail_103:
	//printf("fail_103, File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
    if (munmap(mapped_base_membuf, SizeMembuf) == -1) {
        printf("Can't unmap MemBuf from user space.\n");
    }
fail_102:
	//printf("fail_102, File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
    if (munmap(mapped_base_devcfg, SIZE_DEVCFG) == -1) {
        printf("Can't unmap DevCfg from user space.\n");
    }
fail_101:
	//printf("fail_101, File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
    if (regPcapClkCtrlBackuped == 1) {
        RegWrite(mapped_base_slcr, OFFSET_SLCR_UNLOCK, SLCR_UNLOCK_VAL );
        RegWrite(mapped_base_slcr, OFFSET_SLCR_PCAP_CLK_CTRL, regPcapClkCtrl);
        RegWrite(mapped_base_slcr, OFFSET_SLCR_LOCK, SLCR_LOCK_VAL );
        regPcapClkCtrlBackuped = 0;
    }
    if (munmap(mapped_base_slcr, SIZE_SLCR) == -1) {
        printf("Can't unmap SLCR from user space.\n");
    }
fail_100:
	//printf("fail_100, File: %s, Func: %s, Line: %d\n", __FILE__, __func__, __LINE__ );
    close(fd_mem);
    return retval;
}
