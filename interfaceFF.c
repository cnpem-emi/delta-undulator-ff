#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mqueue.h>
#include <poll.h>




#define PRU1_ADDR 	0x4b200000
#define PRU2_ADDR 	0x4b280000
#define SHRAM_OFFSET	0x00010000
#define BUFFER_SIZE	40000
#define ITERACTIONS	1000000
#define BAUD B3000000


// Auxiliar functions
uint32_t reverseBits(uint32_t num);
uint8_t reverseBits8(uint8_t num);
void adjustVector(uint8_t *data_vector, float current1, float current2, float current3, float current4);




int main(void)
{

  int fd;

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("No valid serial device connected");
        return -1;
    }
    struct termios tty;
    tcgetattr(fd, &tty);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8 | CREAD | CLOCAL;

    tty.c_lflag &= ~ISIG;
    tty.c_lflag &= ~ICANON;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, BAUD);
    cfsetospeed(&tty, BAUD);

    tcsetattr(fd, TCSANOW, &tty); // TODO: Handle errors



  int i;
  float current_up[BUFFER_SIZE];

  int fdm = open("/dev/mem",O_RDWR | O_SYNC);
  
  uint64_t position[] = {0, 0, 0, 0};
  uint64_t oldpos = 0;

  uint8_t ajuste4setpoints[22] = {0x01, 0x50, 0x00, 0x11, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8d};

  volatile ulong* prudata1 =  (ulong*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fdm, PRU1_ADDR + SHRAM_OFFSET);
  volatile ulong* prudata2 =  (ulong*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fdm, PRU2_ADDR + SHRAM_OFFSET);

  struct mq_attr attr;

    attr.mq_maxmsg = 1000;
    attr.mq_msgsize = 32;
    attr.mq_flags = 0;

  mqd_t cmd_ff = mq_open("/cmd_mq", (O_RDWR | O_CREAT), 0666, &attr);

  // SIMULATED VECTOR
  for(i=0; i< BUFFER_SIZE; i++)
  {
    current_up[i] = i * 0.0001;
  }

  //clock_t tic, toc;
  struct timeval  tv1, tv2;
  mqd_t reply_ff = mq_open("/reply_ff", (O_RDWR | O_CREAT), 0666, &attr);
  char reply_buf[32];
  gettimeofday(&tv1, NULL);

    position[0] = reverseBits((uint32_t)prudata1[2]) + (reverseBits8((uint8_t)prudata1[3] & 0xFF) << 29);
    position[1] = reverseBits((uint32_t)prudata1[10]) + (reverseBits8((uint8_t)prudata1[11] & 0xFF) << 29);
    position[2] = reverseBits((uint32_t)prudata2[2]) + (reverseBits8((uint8_t)prudata2[3] & 0xFF) << 29);
    position[3] = reverseBits((uint32_t)prudata2[10]) + (reverseBits8((uint8_t)prudata2[11] & 0xFF) << 29);
    oldpos = position[2];



  //for(int i = 0; i < ITERACTIONS; i++)
  while(1)
  {
    position[0] = reverseBits((uint32_t)prudata1[2]) + (reverseBits8((uint8_t)prudata1[3] & 0xFF) << 29);
    position[1] = reverseBits((uint32_t)prudata1[10]) + (reverseBits8((uint8_t)prudata1[11] & 0xFF) << 29);
    position[2] = reverseBits((uint32_t)prudata2[2]) + (reverseBits8((uint8_t)prudata2[3] & 0xFF) << 29);
    position[3] = reverseBits((uint32_t)prudata2[10]) + (reverseBits8((uint8_t)prudata2[11] & 0xFF) << 29);


    // Adjust new setpoints before sending them
    if(position[2] != oldpos)
    {
        adjustVector(ajuste4setpoints, current_up[position[2]%BUFFER_SIZE], current_up[position[2]%BUFFER_SIZE], current_up[position[2]%BUFFER_SIZE], current_up[position[2]%BUFFER_SIZE]);
        write(fd, ajuste4setpoints, 22);
//        mq_send(cmd_ff, ajuste4setpoints, 22, 1);
        oldpos = position[2];
//        printf("New pos!\n");
//        printf("New pos!\n");
    //mq_receive(reply_ff, reply_buf, sizeof(reply_buf), 0);
    }
    //nanosleep((const struct timespec[]){{0, 700000000L}}, NULL);
    //printf("%lld %f\n", position[3], current_up[position[3]%BUFFER_SIZE]);

  }
  gettimeofday(&tv2, NULL);
  printf ("Total time = %f millisecs\n",
         1000*((double) (tv2.tv_usec - tv1.tv_usec) / 1000000 +
         (double) (tv2.tv_sec - tv1.tv_sec)));
  printf ("Round time = %f microsecs\n",
         1000*1000*((double) (tv2.tv_usec - tv1.tv_usec) / 1000000 +
         (double) (tv2.tv_sec - tv1.tv_sec))/ITERACTIONS);


return 0;
}




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

void adjustVector(uint8_t *data_vector, float current1, float current2, float current3, float current4)
{
    int i;
    // CURRENT 1
    for(i=0; i<4; i++){
        data_vector[5+i] = (uint32_t)(*(uint32_t*)&current1) >> (i * 8);
    }
    // CURRENT 2
    for(i=0; i<4; i++){
        data_vector[9+i] = (uint32_t)(*(uint32_t*)&current1) >> (i * 8);
    }
    // CURRENT 3
    for(i=0; i<4; i++){
        data_vector[13+i] = (uint32_t)(*(uint32_t*)&current1) >> (i * 8);
    }
    // CURRENT 4FeedForward
    for(i=0; i<4; i++){
        data_vector[17+i] = (uint32_t)(*(uint32_t*)&current1) >> (i * 8);
    }
    // CHECKSUM
    data_vector[21] = 0;
    for(i=0; i<21; i++){
        data_vector[21] -= data_vector[i];
    }
}
