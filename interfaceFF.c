#define _GNU_SOURCE

#include <hiredis/adapters/libevent.h>
#include <mqueue.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#define PRU1_ADDR 0x4b200000
#define PRU2_ADDR 0x4b280000
#define SHRAM_OFFSET 0x00010000
#define BUFFER_SIZE 40000
#define ITERATIONS 10000
#define BAUD B3000000
#define REDIS_SERVER "10.0.6.70"
#define TABLE_SIZE 2000

// Since we've only got 5 header (config) bytes, packing is necessary.
typedef union {
  struct __attribute__((__packed__)) msg {
    uint8_t config[5];
    float currents[4];
    uint8_t checksum;
  } msg;

  uint8_t data_vector[22];
} adjust_t;

typedef struct {
  double cols[5][400];
  size_t colSize = 0;
  const char* name;

} correctionTable;

correctionTable tableEntry[5] = {{.name = "Array:Test-Mon", .colSize = 0},
                                 {.name = "Array2:Test-Mon", .colSize = 0},
                                 {.name = "Array3:Test-Mon", .colSize = 0},
                                 {.name = "Array4:Test-Mon", .colSize = 0},
                                 {.name = "Array5:Test-Mon", .colSize = 0}};

int fd;
pthread_mutex_t serial_mutex;
redisContext* sync_c;

void onTableChange(redisAsyncContext* c, void* reply, void* privdata) {
  redisReply* r = reply;
  if (reply == NULL)
    return;

  if (r->type == REDIS_REPLY_ARRAY) {
    for (int i = 0; i < sizeof(tableEntry) / sizeof(tableEntry[0]); i++) {
      if (r->element[2]->str == NULL || strcmp(tableEntry[i].name, r->element[2]->str) == 0) {
        printf("%s\n", tableEntry[i].name);

        redisReply* r = redisCommand(sync_c, "LRANGE %s 0 -1", tableEntry[i].name);

        if (r->elements % 5 != 0) {
          if (tableEntry[i].colSize == 0)
            exit(-5);
          continue;
        }

        size_t arrayDivs = r->elements / 5;
        tableEntry[i].colSize = arrayDivs;

        pthread_mutex_lock(&serial_mutex);
        for (int j = 0; j < 5; j++) {
          for (int k = j * arrayDivs; k < (j + 1) * arrayDivs; k++)
            tableEntry[i].cols[j][k - j * arrayDivs] = strtod(r->element[k]->str, NULL);
        }
        pthread_mutex_unlock(&serial_mutex);

        freeReplyObject(r);
      }
    }
  }
}

void* listenForCommands() {
  struct mq_attr attr;
  struct event_base* base = event_base_new();

  redisAsyncContext* c = redisAsyncConnect(REDIS_SERVER, 6379);
  sync_c = redisConnect(REDIS_SERVER, 6379);
  if (!sync_c->err && !c->err) {
    redisLibeventAttach(c, base);
    redisAsyncCommand(c, onTableChange, NULL, "SUBSCRIBE ArraySubscription");
    event_base_dispatch(base);
  } else {
    printf("Redis server not available!\n");
  }

  attr.mq_maxmsg = 1000;
  attr.mq_msgsize = 32;
  attr.mq_flags = 0;

  mqd_t cmd_mq = mq_open("/cmd_mq", (O_RDWR | O_CREAT), 0666, &attr);
  mqd_t reply_ioc = mq_open("/reply_ioc", (O_RDWR | O_CREAT), 0666, &attr);
  size_t write_size, reply_size;
  char cmd_buf[32];
  char reply_buf[32];

  unsigned int priority;

  struct pollfd pfds[1];
  pfds[0].fd = fd;
  pfds[0].events = POLLIN;

  for (;;) {
    write_size = mq_receive(cmd_mq, cmd_buf, sizeof(cmd_buf), &priority);
    pthread_mutex_lock(&serial_mutex);
    write(fd, cmd_buf, write_size);

    if (poll(pfds, 1, 1) < 1) {
      mq_send(reply_ioc, "\xee", 1, priority);
    } else {
      reply_size = read(fd, &reply_buf, sizeof(reply_buf));
      mq_send(reply_ioc, reply_buf, reply_size, priority);
    }

    pthread_mutex_unlock(&serial_mutex);
  }
}

/* Bit reversal functions use a divide and conquer algorithm
 * Bytes are split in two halves, then pairs are swapped
 * until we're left with a complete reversal.
 *
 * See https://archive.org/details/1983-04-dr-dobbs-journal/page/24/mode/2up
 */

uint32_t reverseBits(uint32_t num) {
  num = (num & 0xffff0000) >> 16 | (num & 0x0000ffff) << 16;
  num = (num & 0xff00ff00) >> 8 | (num & 0x00ff00ff) << 8;
  num = (num & 0xf0f0f0f0) >> 4 | (num & 0x0f0f0f0f) << 4;
  num = (num & 0xcccccccc) >> 2 | (num & 0x33333333) << 2;
  num = (num & 0xaaaaaaaa) >> 1 | (num & 0x55555555) << 1;

  return num;
}

uint8_t reverseBits8(uint8_t num) {
  num = (num & 0xf0) >> 4 | (num & 0x0f) << 4;
  num = (num & 0xcc) >> 2 | (num & 0x33) << 2;
  num = (num & 0xaa) >> 1 | (num & 0x55) << 1;

  return num;
}

void adjustVector(adjust_t* setpoints,
                  float current1,
                  float current2,
                  float current3,
                  float current4) {
  memcpy(setpoints->msg.currents, (const char[]){current1, current2, current3, current4}, 4);

  // CHECKSUM
  setpoints->msg.checksum = 0;
  for (int i = 0; i < 21; i++)
    setpoints->msg.checksum -= setpoints->data_vector[i];
}

int main(void) {
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
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

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, BAUD);
  cfsetospeed(&tty, BAUD);

  tcsetattr(fd, TCSANOW, &tty);  // TODO: Handle errors

  int i;
  float current_up[BUFFER_SIZE];

  int fdm = open("/dev/mem", O_RDWR | O_SYNC);

  uint64_t position[] = {0, 0, 0, 0};
  uint64_t oldpos = 0;

  adjust_t setpoints = {.msg.config = {0x01, 0x50, 0x00, 0x11, 0x11}, .msg.checksum = 0x8d};

  volatile ulong* prudata1 =
      (ulong*)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fdm, PRU1_ADDR + SHRAM_OFFSET);
  volatile ulong* prudata2 =
      (ulong*)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fdm, PRU2_ADDR + SHRAM_OFFSET);

  // SIMULATED VECTOR
  for (i = 0; i < BUFFER_SIZE; i++)
    current_up[i] = i * 0.0001;

  position[0] =
      reverseBits((uint32_t)prudata1[2]) + (reverseBits8((uint8_t)prudata1[3] & 0xFF) << 29);
  position[1] =
      reverseBits((uint32_t)prudata1[10]) + (reverseBits8((uint8_t)prudata1[11] & 0xFF) << 29);
  position[2] =
      reverseBits((uint32_t)prudata2[2]) + (reverseBits8((uint8_t)prudata2[3] & 0xFF) << 29);
  position[3] =
      reverseBits((uint32_t)prudata2[10]) + (reverseBits8((uint8_t)prudata2[11] & 0xFF) << 29);
  oldpos = position[2];

  pthread_t thisThread = pthread_self();
  pthread_t cmdThread;

  pthread_create(&cmdThread, NULL, listenForCommands, NULL);
  pthread_mutex_init(&serial_mutex, NULL);

  cpu_set_t mainCpuSet;
  cpu_set_t cmdCpuSet;

  CPU_ZERO(&mainCpuSet);
  CPU_ZERO(&cmdCpuSet);

  CPU_SET(1, &mainCpuSet);
  CPU_SET(0, &cmdCpuSet);

  struct sched_param params;
  params.sched_priority = sched_get_priority_max(SCHED_FIFO);

  pthread_setschedparam(thisThread, SCHED_FIFO, &params);
  // pthread_setschedparam(cmdThread, SCHED_FIFO, &params);

  pthread_setaffinity_np(thisThread, sizeof(cpu_set_t), &mainCpuSet);
  pthread_setaffinity_np(cmdThread, sizeof(cpu_set_t), &cmdCpuSet);

  for (;;) {
    position[0] =
        reverseBits((uint32_t)prudata1[2]) + (reverseBits8((uint8_t)prudata1[3] & 0xFF) << 29);
    position[1] =
        reverseBits((uint32_t)prudata1[10]) + (reverseBits8((uint8_t)prudata1[11] & 0xFF) << 29);
    position[2] =
        reverseBits((uint32_t)prudata2[2]) + (reverseBits8((uint8_t)prudata2[3] & 0xFF) << 29);
    position[3] =
        reverseBits((uint32_t)prudata2[10]) + (reverseBits8((uint8_t)prudata2[11] & 0xFF) << 29);

    if (position[2] != oldpos) {
      pthread_mutex_lock(&serial_mutex);
      adjustVector(&setpoints, current_up[position[2] % BUFFER_SIZE],
                   current_up[position[2] % BUFFER_SIZE], current_up[position[2] % BUFFER_SIZE],
                   current_up[position[2] % BUFFER_SIZE]);
      write(fd, setpoints.data_vector, 22);
      pthread_mutex_unlock(&serial_mutex);
      oldpos = position[2];
    }
  }

  return 0;
}
