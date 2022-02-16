#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <sys/types.h>

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */



#define PRU1_ADDR 	0x4b200000
#define PRU2_ADDR 	0x4b280000
#define SHRAM_OFFSET	0x00010000

//        0:{"remoteproc":4, "subsystem":1, "device":0, "posShramOffset":8  , "clk_t": 17, "data_t":  5}, 
//        1:{"remoteproc":5, "subsystem":1, "device":1, "posShramOffset":40 , "clk_t": 18, "data_t":  9}, 
//        2:{"remoteproc":6, "subsystem":2, "device":0, "posShramOffset":8  , "clk_t":  4, "data_t":  3}, 
//        3:{"remoteproc":7, "subsystem":2, "device":1, "posShramOffset":40 , "clk_t": 10, "data_t": 11}



uint32_t reverseBits(uint32_t num)
{
    unsigned int count = sizeof(num) * 8 - 1;
    uint32_t reverse_num = num;
      
    num >>= 1; 
    while(num)
    {
       reverse_num <<= 1;       
       reverse_num |= num & 1;
       num >>= 1;
       count--;
    }
    reverse_num <<= count;
    return reverse_num;
}

uint8_t reverseBits8(uint8_t num)
{
    unsigned int count = sizeof(num) * 8 - 1;
    uint8_t reverse_num = num;

    num >>= 1; 
    while(num)
    {
       reverse_num <<= 1;       
       reverse_num |= num & 1;
       num >>= 1;
       count--;
    }
    reverse_num <<= count;
    return reverse_num;
}



int
main(void)
{
  int fd = open("/dev/mem",O_RDWR | O_SYNC);

  ulong* prudata1 =  (ulong*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU1_ADDR + SHRAM_OFFSET);
  ulong* prudata2 =  (ulong*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU2_ADDR + SHRAM_OFFSET);

  while(1)
  {
  uint64_t position0 = reverseBits((uint32_t)prudata1[2]) + (reverseBits8((uint8_t)prudata1[3] & 0xFF) << 29);
  uint64_t position1 = reverseBits((uint32_t)prudata1[10]) + (reverseBits8((uint8_t)prudata1[11] & 0xFF) << 29);
  uint64_t position2 = reverseBits((uint32_t)prudata2[2]) + (reverseBits8((uint8_t)prudata2[3] & 0xFF) << 29);
  uint64_t position3 = reverseBits((uint32_t)prudata2[10]) + (reverseBits8((uint8_t)prudata2[11] & 0xFF) << 29);

  printf(" %lld %lld %lld %lld \n", position0, position1, position2, position3);
  }
return 0;
}

