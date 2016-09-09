/*
Program to ouput "hi world:0" and then itirate the last digit +1 every loop. 
Using DMA control
*/

#include <sys/mman.h> //for mmap
#include <unistd.h> //for NULL
#include <stdio.h> //for printf
#include <stdlib.h> //for exit
#include <fcntl.h> //for file opening
#include <stdint.h> //for uint32_t
#include <string.h> //for memset	
#include <assert.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

// Physical Memory Allocation, from raspberrypi/userland demo.
#include "mailbox.h"


//#define RPI_V3
#include "hw-addresses.h" // for DMA addresses, etc.

#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size

//physical addresses for the DMA peripherals, as found in the processor documentation:
//#define DMA_BASE 0x3F007000
//-------- Relative offsets for DMA registers
//DMA Channel register sets (format of these registers is found in DmaChannelHeader struct):
#define DMACH(n) (0x100*(n))
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used (affects control block's SOURCE, DEST and NEXTCONBK addresses).
#define DMAENABLE 0x00000ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ACTIVE (1<<0)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_DEST_INC (1<<4)
#define DMA_CB_TI_SRC_INC (1<<8)




// GPIO which we want to toggle in this example.
#define TOGGLE_GPIO 17

// Raspberry Pi 2 or 1 ? Since this is a simple example, we don't
// bother auto-detecting but have it a compile-time option.

#define PI_VERSION 2  //version 2 works for v3


#define BCM2709_PI2_PERI_BASE  0x3F000000

// --- General, Pi-specific setup.
#define PERI_BASE BCM2709_PI2_PERI_BASE


#define PAGE_SIZE 4096

// ---- GPIO specific defines
#define GPIO_REGISTER_BASE 0x200000
#define GPIO_SET_OFFSET 0x1C
#define GPIO_CLR_OFFSET 0x28
#define PHYSICAL_GPIO_BUS (0x7E000000 + GPIO_REGISTER_BASE)

// ---- Memory mappping defines
#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

// ---- Memory allocating defines
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MEM_FLAG_DIRECT           (1 << 2)
#define MEM_FLAG_COHERENT         (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)

// ---- DMA specific defines
#define DMA_CHANNEL       5   // That usually is free.
//#define DMA_BASE          0x007000
#define DMA_BASE          0x3F007000


// BCM2385 ARM Peripherals 4.2.1.2
#define DMA_CB_TI_NO_WIDE_BURSTS (1<<26)
#define DMA_CB_TI_SRC_INC        (1<<8)
#define DMA_CB_TI_DEST_INC       (1<<4)
#define DMA_CB_TI_TDMODE         (1<<1)

#define DMA_CS_RESET    (1<<31)
#define DMA_CS_ABORT    (1<<30)
#define DMA_CS_DISDEBUG (1<<28)
#define DMA_CS_END      (1<<1)
#define DMA_CS_ACTIVE   (1<<0)

#define DMA_CB_TXFR_LEN_YLENGTH(y) (((y-1)&0x4fff) << 16)
#define DMA_CB_TXFR_LEN_XLENGTH(x) ((x)&0xffff)
#define DMA_CB_STRIDE_D_STRIDE(x)  (((x)&0xffff) << 16)
#define DMA_CB_STRIDE_S_STRIDE(x)  ((x)&0xffff)

#define DMA_CS_PRIORITY(x) ((x)&0xf << 16)
#define DMA_CS_PANIC_PRIORITY(x) ((x)&0xf << 20)


struct DmaChannelHeader {
    uint32_t CS; //Control and Status
        //31    RESET; set to 1 to reset DMA
        //30    ABORT; set to 1 to abort current DMA control block (next one will be loaded & continue)
        //29    DISDEBUG; set to 1 and DMA won't be paused when debug signal is sent
        //28    WAIT_FOR_OUTSTANDING_WRITES; set to 1 and DMA will wait until peripheral says all writes have gone through before loading next CB
        //24-74 reserved
        //20-23 PANIC_PRIORITY; 0 is lowest priority
        //16-19 PRIORITY; bus scheduling priority. 0 is lowest
        //9-15  reserved
        //8     ERROR; read as 1 when error is encountered. error can be found in DEBUG register.
        //7     reserved
        //6     WAITING_FOR_OUTSTANDING_WRITES; read as 1 when waiting for outstanding writes
        //5     DREQ_STOPS_DMA; read as 1 if DREQ is currently preventing DMA
        //4     PAUSED; read as 1 if DMA is paused
        //3     DREQ; copy of the data request signal from the peripheral, if DREQ is enabled. reads as 1 if data is being requested, else 0
        //2     INT; set when current CB ends and its INTEN=1. Write a 1 to this register to clear it
        //1     END; set when the transfer defined by current CB is complete. Write 1 to clear.
        //0     ACTIVE; write 1 to activate DMA (load the CB before hand)
    uint32_t CONBLK_AD; //Control Block Address
    uint32_t TI; //transfer information; see DmaControlBlock.TI for description
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t DEBUG; //controls debug settings
};



struct DmaControlBlock {
    uint32_t TI; //transfer information
        //31:27 unused
        //26    NO_WIDE_BURSTS
        //21:25 WAITS; number of cycles to wait between each DMA read/write operation
        //16:20 PERMAP; peripheral number to be used for DREQ signal (pacing). set to 0 for unpaced DMA.
        //12:15 BURST_LENGTH
        //11    SRC_IGNORE; set to 1 to not perform reads. Used to manually fill caches
        //10    SRC_DREQ; set to 1 to have the DREQ from PERMAP gate requests.
        //9     SRC_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //8     SRC_INC;   set to 1 to automatically increment the source address after each read (you'll want this if you're copying a range of memory)
        //7     DEST_IGNORE; set to 1 to not perform writes.
        //6     DEST_DREG; set to 1 to have the DREQ from PERMAP gate *writes*
        //5     DEST_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //4     DEST_INC;   set to 1 to automatically increment the destination address after each read (Tyou'll want this if you're copying a range of memory)
        //3     WAIT_RESP; make DMA wait for a response from the peripheral during each write. Ensures multiple writes don't get stacked in the pipeline
        //2     unused (0)
        //1     TDMODE; set to 1 to enable 2D mode
        //0     INTEN;  set to 1 to generate an interrupt upon completion
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t _reserved[2];
};

//cat /sys/module/dma/parameters/dmachans gives a bitmask of DMA channels that are not used by GPU. Results: ch 1, 3, 6, 7 are reserved.
//dmesg | grep "DMA"; results: Ch 2 is used by SDHC host
//ch 0 is known to be used for graphics acceleration
//Thus, applications can use ch 4, 5, or the LITE channels @ 8 and beyond.
int dmaChNum = 5;
volatile uint32_t *dmaBaseMem;
int memfd;

//configure DMA:
//allocate 1 page for the source and 1 page for the destination:
void *virtSrcPage, *physSrcPage;
void *virtDestPage, *physDestPage;

//allocate 1 page for the control blocks
void *virtCbPage, *physCbPage;

struct DmaControlBlock *cb1;
volatile struct DmaChannelHeader *dmaHeader;
struct UncachedMemBlock src_memblock;
struct UncachedMemBlock des_memblock;	
struct UncachedMemBlock cb_memblock;

char *srcArray;
void printThings();
static int mbox_fd = -1;   // used internally by the UncachedMemBlock-functions.
	
// A memory block that represents memory that is allocated in physical
// memory and locked there so that it is not swapped out.
// It is not backed by any L1 or L2 cache, so writing to it will directly
// modify the physical memory (and it is slower of course to do so).
// This is memory needed with DMA applications so that we can write through
// with the CPU and have the DMA controller 'see' the data.
// The UncachedMemBlock_{alloc,free,to_physical}
// functions are meant to operate on these.
struct UncachedMemBlock {
  void *mem;                  // User visible value: the memory to use.
  //-- Internal representation.
  uint32_t bus_addr;
  uint32_t mem_handle;
  size_t size;
};


// Allocate a block of memory of the given size (which is rounded up to the next
// full page). The memory will be aligned on a page boundary and zeroed out.
static struct UncachedMemBlock UncachedMemBlock_alloc(size_t size) {
  if (mbox_fd < 0) {
    mbox_fd = mbox_open();
    assert(mbox_fd >= 0);  // Uh, /dev/vcio not there ?
  }
  // Round up to next full page.
  size = size % PAGE_SIZE == 0 ? size : (size + PAGE_SIZE) & ~(PAGE_SIZE - 1);

  struct UncachedMemBlock result;
  result.size = size;
  result.mem_handle = mem_alloc(mbox_fd, size, PAGE_SIZE,
                                MEM_FLAG_L1_NONALLOCATING);
  result.bus_addr = mem_lock(mbox_fd, result.mem_handle);
  result.mem = mapmem(BUS_TO_PHYS(result.bus_addr), size);
  fprintf(stderr, "Alloc: %6d bytes;  %p (bus=0x%08x, phys=0x%08x)\n",
          (int)size, result.mem, result.bus_addr, BUS_TO_PHYS(result.bus_addr));
  assert(result.bus_addr);  // otherwise: couldn't allocate contiguous block.
  memset(result.mem, 0x00, size);

  return result;
}

// Free block previously allocated with UncachedMemBlock_alloc()
static void UncachedMemBlock_free(struct UncachedMemBlock *block) {
  if (block->mem == NULL) return;
  assert(mbox_fd >= 0);  // someone should've initialized that on allocate.
  unmapmem(block->mem, block->size);
  mem_unlock(mbox_fd, block->mem_handle);
  mem_free(mbox_fd, block->mem_handle);
  block->mem = NULL;
}


// Given a pointer to memory that is in the allocated block, return the
// physical bus addresse needed by DMA operations.
static uintptr_t UncachedMemBlock_to_physical(const struct UncachedMemBlock *blk, void *p) {
    uint32_t offset = (uint8_t*)p - (uint8_t*)blk->mem;
    assert(offset < blk->size);   // pointer not within our block.
    return blk->bus_addr + offset;
}


//map a physical address into our virtual address space. memfd is the file descriptor for /dev/mem
volatile uint32_t* mapPeripheral(int memfd, int addr) {
    ///dev/mem behaves as a file. We need to map that file into memory:
    void *mapped = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, addr);
    //now, *mapped = memory at physical address of addr.
    if (mapped == MAP_FAILED) {
        printf("failed to map memory (did you remember to run as root?)\n");
        exit(1);
    } else {
        printf("mapped: %p\n", mapped);
    }
    return (volatile uint32_t*)mapped;
}

void initialize_gpio_for_output(volatile uint32_t *gpio_registerset, int bit) {
  *(gpio_registerset+(bit/10)) &= ~(7<<((bit%10)*3));  // prepare: set as input
  *(gpio_registerset+(bit/10)) |=  (1<<((bit%10)*3));  // set as output.
}

	
	
	
//set bits designated by (mask) at the address (dest) to (value), without affecting the other bits
//eg if x = 0b11001100
//  writeBitmasked(&x, 0b00000110, 0b11110011),
//  then x now = 0b11001110
void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value) {
    uint32_t cur = *dest;
    uint32_t new = (cur & (~mask)) | (value & mask);
    *dest = new;
    *dest = new; //added safety for when crossing memory barriers.
}



void DMA_Run() {

	printThings(1);
    
    dmaHeader->CS |= DMA_CS_ABORT; //make sure to disable dma first.
    usleep(100);
    dmaHeader->CS &= ~DMA_CS_ACTIVE;
    dmaHeader->CS |= DMA_CS_RESET;
    printThings(2);
    
    
    
    dmaHeader->CS |= DMA_CS_END;
    dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
    //printThings(3);
    
    dmaHeader->CONBLK_AD = UncachedMemBlock_to_physical(&cb_memblock, cb1); //we have to point it to the PHYSICAL address of the control block (cb1)
    printThings(4);
    dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG;
    dmaHeader->CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.
    printThings(5);

}


void main()
{
		
	int count;
	int y;
	y=48;
	
	//First, open the linux device, /dev/mem
    //dev/mem provides access to the physical memory of the entire processor+ram
    //This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
    memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        printf("Failed to open /dev/mem (did you remember to run as root?)\n");

    }
    else
    {
		
		
		
		//now map /dev/mem into memory, but only map specific peripheral sections:
	    dmaBaseMem = mapPeripheral(memfd, DMA_BASE);
		
	
		
		//enable DMA channel (it's probably already enabled, but we want to be sure):
	    writeBitmasked(dmaBaseMem + DMAENABLE, 1 << dmaChNum, 1 << dmaChNum);
		
		char DMAtest[11];
		DMAtest[0]  = 'h';
	    DMAtest[1]  = 'i';
	    DMAtest[2]  = ' ';
	    DMAtest[3]  = 'w';
	    DMAtest[4]  = 'o';
	    DMAtest[5]  = 'r';
	    DMAtest[6]  = 'l';
	    DMAtest[7]  = 'd';
	    DMAtest[8]  = ' ';
	    DMAtest[9]  = ':';
	    DMAtest[10] = '0';
	     
	    //configure DMA:
	    //***** These need to be changed ****//   
	    ////allocate 1 page for the source and 1 page for the destination:
	    //makeVirtPhysPage(&virtSrcPage, &physSrcPage);
	    //makeVirtPhysPage(&virtDestPage, &physDestPage);
	    
	    ////allocate 1 page for the control blocks
	    //makeVirtPhysPage(&virtCbPage, &physCbPage);
	    

		src_memblock = UncachedMemBlock_alloc(sizeof(DMAtest));
		des_memblock = UncachedMemBlock_alloc(sizeof(DMAtest));
		
		// Prepare DMA control block. This needs to be in uncached data, so that
		// when we set it up, it is immediately visible to the DMA controller.
		// Also, only UncachedMemBlock allows us to conveniently get the physical
		// address.
		cb_memblock = UncachedMemBlock_alloc(sizeof(struct DmaControlBlock));

	    
	    //dedicate the first 8 words of this page to holding the cb.
	    cb1 = (struct DmaControlBlock*)cb_memblock.mem;
	    printf("DMA Control Block address: %x\n", UncachedMemBlock_to_physical(&cb_memblock, cb1));
	    
	    //fill the control block:
	    cb1->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each byte copied, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
	    cb1->SOURCE_AD = UncachedMemBlock_to_physical(&src_memblock, src_memblock.mem); //set source and destination DMA address
	    cb1->DEST_AD = UncachedMemBlock_to_physical(&des_memblock, des_memblock.mem);
	    cb1->TXFR_LEN = 12; //transfer 12 bytes
	    cb1->STRIDE = 0; //no 2D stride
	    cb1->NEXTCONBK = 0;//(uint32_t)physCbPage; 
	 
	    printf("DMA Control Block: TI: %x   Source_AD: %x   Dest_AD:%x   TX_LEN:%x   STRIDE: %x   NEXTCONBK: %x\n\n",  cb1->TI, cb1->SOURCE_AD, cb1->DEST_AD, cb1->TXFR_LEN, cb1->STRIDE, cb1->NEXTCONBK);
	    
	    //configure the DMA header to point to our control block:
	    dmaHeader = (volatile struct DmaChannelHeader*)(dmaBaseMem + (DMACH(dmaChNum))/4); //dmaBaseMem is a uint32_t ptr, so divide by 4 before adding byte offset
		printThings(1);
	    
		while(1)
		{
			y++;
			if(y>57)
			{
				y=48;	
			}
			DMAtest[10] = (char) y;
			//write a few bytes to the source page:
			srcArray = (char*)src_memblock.mem;
			for (count =0; count < 11; count++)
			{
				printf("%c", srcArray[count]);
				srcArray[count]  = DMAtest[count];
				//printf("%c", srcArray[count]);
			}
			srcArray[11] =  0; //null terminator used for printf call.
			printf("\n");
			
			printf("destination was initially: '%s'\n", des_memblock.mem);
			
			
			DMA_Run();
			printf("destination reads: '%s'\n\n", des_memblock.mem);
			printThings(6);
	
		}
		////cleanup
		//freeVirtPhysPage(virtCbPage);
		//freeVirtPhysPage(virtDestPage);
		//freeVirtPhysPage(virtSrcPage);
		UncachedMemBlock_free(&src_memblock);
		UncachedMemBlock_free(&des_memblock);
		UncachedMemBlock_free(&cb_memblock);
	}
}

void printThings(int a){
	printf("%d\n", a);
	printf("DMA Control Block :                              TI: %03x    SourceAd: %08x    DestAd: %08x    TxLen: %02x    Stride: %x    NextConBk: %x\n",  cb1->TI, cb1->SOURCE_AD, cb1->DEST_AD, cb1->TXFR_LEN, cb1->STRIDE, cb1->NEXTCONBK);
	printf("DMA Channel Header: CS: %02x    ConBk: %08x    TI: %03x    SourceAd: %08x    DestAd: %08x    TxLen: %02x    Stride: %x    NextConBk: %x    Debug: %x\n\n", 
				dmaHeader->CS, dmaHeader->CONBLK_AD, dmaHeader->TI, dmaHeader->SOURCE_AD, dmaHeader->DEST_AD, dmaHeader->TXFR_LEN, dmaHeader->STRIDE, dmaHeader->NEXTCONBK, dmaHeader->DEBUG);
}






	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/*
Copyright (c) 2012, Broadcom Europe Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Original:
https://github.com/raspberrypi/userland/blob/master/host_applications/linux/apps/hello_pi/hello_fft/mailbox.c
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "mailbox.h"


void *mapmem(unsigned base, unsigned size)
{
   int mem_fd;
   unsigned offset = base % PAGE_SIZE;
   base = base - offset;
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem\nThis program should be run as root. Try prefixing command with: sudo\n");
      exit (-1);
   }
   void *mem = mmap(
      0,
      size,
      PROT_READ|PROT_WRITE,
      MAP_SHARED/*|MAP_FIXED*/,
      mem_fd,
      base);
#ifdef DEBUG
   printf("base=0x%x, mem=%p\n", base, mem);
#endif
   if (mem == MAP_FAILED) {
      printf("mmap error %d\n", (int)mem);
      exit (-1);
   }
   close(mem_fd);
   return (char *)mem + offset;
}

void unmapmem(void *addr, unsigned size)
{
   int s = munmap(addr, size);
   if (s != 0) {
      printf("munmap error %d\n", s);
      exit (-1);
   }
}

/*
 * use ioctl to send mbox property message
 */

static int mbox_property(int file_desc, void *buf)
{
   int ret_val = ioctl(file_desc, IOCTL_MBOX_PROPERTY, buf);

   if (ret_val < 0) {
      printf("ioctl_set_msg failed:%d\n", ret_val);
   }

#ifdef DEBUG
   unsigned *p = buf; int i; unsigned size = *(unsigned *)buf;
   for (i=0; i<size/4; i++)
      printf("%04x: 0x%08x\n", i*sizeof *p, p[i]);
#endif
   return ret_val;
}

unsigned mem_alloc(int file_desc, unsigned size, unsigned align, unsigned flags)
{
   int i=0;
   unsigned p[32];
   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000c; // (the tag id)
   p[i++] = 12; // (size of the buffer)
   p[i++] = 12; // (size of the data)
   p[i++] = size; // (num bytes? or pages?)
   p[i++] = align; // (alignment)
   p[i++] = flags; // (MEM_FLAG_L1_NONALLOCATING)

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned mem_free(int file_desc, unsigned handle)
{
   int i=0;
   unsigned p[32];
   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000f; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = handle;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned mem_lock(int file_desc, unsigned handle)
{
   int i=0;
   unsigned p[32];
   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000d; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = handle;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned mem_unlock(int file_desc, unsigned handle)
{
   int i=0;
   unsigned p[32];
   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000e; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = handle;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned execute_code(int file_desc, unsigned code, unsigned r0, unsigned r1, unsigned r2, unsigned r3, unsigned r4, unsigned r5)
{
   int i=0;
   unsigned p[32];
   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x30010; // (the tag id)
   p[i++] = 28; // (size of the buffer)
   p[i++] = 28; // (size of the data)
   p[i++] = code;
   p[i++] = r0;
   p[i++] = r1;
   p[i++] = r2;
   p[i++] = r3;
   p[i++] = r4;
   p[i++] = r5;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned qpu_enable(int file_desc, unsigned enable)
{
   int i=0;
   unsigned p[32];

   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x30012; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = enable;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned execute_qpu(int file_desc, unsigned num_qpus, unsigned control, unsigned noflush, unsigned timeout) {
   int i=0;
   unsigned p[32];

   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request
   p[i++] = 0x30011; // (the tag id)
   p[i++] = 16; // (size of the buffer)
   p[i++] = 16; // (size of the data)
   p[i++] = num_qpus;
   p[i++] = control;
   p[i++] = noflush;
   p[i++] = timeout; // ms

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

int mbox_open() {
   int file_desc;

   // open a char device file used for communicating with kernel mbox driver
   file_desc = open(DEVICE_FILE_NAME, 0);
   if (file_desc < 0) {
      printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
      printf("Try creating a device file with: sudo mknod %s c %d 0\n", DEVICE_FILE_NAME, MAJOR_NUM);
      exit(-1);
   }
   return file_desc;
}

void mbox_close(int file_desc) {
  close(file_desc);
}
	
	
