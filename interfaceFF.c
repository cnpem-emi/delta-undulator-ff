#include <hiredis/adapters/libevent.h>
#include <mqueue.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <hiredis/hiredis.h>

//gcc redistest.c -o redistest -I /usr/local/include/hiredis -lhiredis -levent_core


// Since we've only got 5 header (config) bytes, packing is necessary.
typedef union {
  struct __attribute__((__packed__)) msg {
    uint8_t config[5];
    float currents[4];
    uint8_t checksum;
  } msg;

  uint8_t data_vector[22];
} adjust_t;


typedef struct correctionTable {
  double cols[5][400];
  size_t colSize;
  const char* name;

} correctionTable;

correctionTable tableEntry[8] = {{.name = "PosCSDMon", .colSize = 0},
                                 {.name = "CurCSDMon", .colSize = 0},
                                 {.name = "PosCSEMon", .colSize = 0},
                                 {.name = "CurCSEMon", .colSize = 0},
                                 {.name = "PosCIDMon", .colSize = 0},
                                 {.name = "CurCIDMon", .colSize = 0},
                                 {.name = "PosCIEMon", .colSize = 0},
                                 {.name = "CurCIEMon", .colSize = 0},
                                 };

pthread_mutex_t serial_mutex;
redisContext* sync_c;

redisReply* pol;

char str3[100];
char str4[100];

static double s_position[5] = {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},         //CSD
                               {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},          //CSE
                               {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},          //CID
                               {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}} * 5};    //CIE

                               //5 polarizacoes

static double s_current[5] = {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},           //CSD
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},           //CSE
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},           //CID
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}} * 5};     //CIE

                              //5 polarizacoes

void onTableChange(redisAsyncContext* c, void* reply, void* privdata) {
  redisReply* r = reply;
  if (reply == NULL)
    return;


  if (r->type == REDIS_REPLY_ARRAY) {
    for (int i = 0; i < sizeof(tableEntry) / sizeof(tableEntry[0]); i++) {
      if (1 ||r->element[2]->str == NULL || strcmp(tableEntry[i].name, r->element[2]->str) == 0) {

        redisReply* r = redisCommand(sync_c, "LRANGE %s 0 4", tableEntry[i].name);

        if (r->elements % 5 != 0) {
          if (tableEntry[i].colSize == 0){
            exit(-5);
            }
          continue;
        }

        size_t arrayDivs = r->elements / 5;
        tableEntry[i].colSize = arrayDivs;

        pthread_mutex_lock(&serial_mutex);

        //Aqui, deve ser implementado uma maneira de se descobrir a polarizacao do delta. Inicialmente a ideia será ler uma variavel redis.

        pol = redisCommand(sync_c, "GET DELTAPOL");

        for (int j = 0; j < 5; j++) {
          for (int k = j * arrayDivs; k < (j + 1) * arrayDivs; k++){
            tableEntry[i].cols[j][k - j * arrayDivs] = strtod(r->element[k]->str, NULL); // Comando duplicado "/ entretanto, ao escrever diretamente em s_cirrent e s_position, percebi que os dados não eram salvos de maneira correta.

            if(i % 2){
               s_current[pol][i][j] = tableEntry[i].cols[j][k - j * arrayDivs];
            }
            else{
               s_position[pol][i][j] = tableEntry[i].cols[j][k - j * arrayDivs];
               }
            //printf("%d %lf \n", i, tableEntry[i].cols[j][k - j * arrayDivs]);
            }
        }

        pthread_mutex_unlock(&serial_mutex);

        freeReplyObject(r);

      }
    }
  }
}

void* listenForCommands() {

  struct event_base* base = event_base_new();
  redisAsyncContext* c = redisAsyncConnect("127.0.0.1", 6381);
  sync_c = redisConnect("127.0.0.1", 6381);

  if (!sync_c->err && !c->err) {
    redisLibeventAttach(c, base);
    redisAsyncCommand(c, onTableChange, NULL, "SUBSCRIBE ArraySubscription PolSubscription");
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

void adjustVector(adjust_t* setpoints, int encoder, char * position) { //Essa funcao nao usa algoritmos de busca binaria, deve ser avaliado se compromete a eficiência do FF

    float result;

    // Esta interpolação está sendo feita ponto a ponto, mas, talvez, fique mais rápido gerar as funções lineares antes
    for(int pos = 0; pos < (sizeof(s_current[pol][encoder])/sizeof(s_current[pol][encoder][0]) - 1); pos++){

      if((s_position[pol][encoder][pos] <= atof(position)) && (atof(position) <= s_position[pol][encoder][pos+1])){
        float ang = (s_current[pol][encoder][pos+1] - s_current[pol][encoder][pos])/(s_position[pol][encoder][pos+1] - s_position[pol][encoder][pos]);
        float lin =  s_current[pol][encoder][pos] - ang * s_position[pol][encoder][pos];

        result = ang * atof(position) + lin;
        printf("%s -> %f\n", position, ang*atof(position) + lin);
        break;
    }
  }

  setpoints->data_vector[encoder] = encoded_data; //Ainda é preciso transformar data para o padrão BSMP

  // CHECKSUM
  setpoints->msg.checksum = 0;
  for (int i = 0; i < 21; i++)
    setpoints->msg.checksum -= setpoints->data_vector[i];

  if !setpoints->msg.checksum {
    memcpy(setpoints->msg.currents, (const char[]){encoded_data[0], encoded_data[1], encoded_data[2], encoded_data[3]}, 4);
  }

}


int main (int argc, char **argv) {

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
  uint64_t oldposition[] = {0, 0, 0, 0};

  adjust_t setpoints = {.msg.config = {0x01, 0x50, 0x00, 0x11, 0x11}, .msg.checksum = 0x8d};

  volatile ulong* prudata1 =
      (ulong*)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fdm, PRU1_ADDR + SHRAM_OFFSET);
  volatile ulong* prudata2 =
      (ulong*)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fdm, PRU2_ADDR + SHRAM_OFFSET);



  position[0] =
      reverseBits((uint32_t)prudata1[2]) + (reverseBits8((uint8_t)prudata1[3] & 0xFF) << 29);
  position[1] =
      reverseBits((uint32_t)prudata1[10]) + (reverseBits8((uint8_t)prudata1[11] & 0xFF) << 29);
  position[2] =
      reverseBits((uint32_t)prudata2[2]) + (reverseBits8((uint8_t)prudata2[3] & 0xFF) << 29);
  position[3] =
      reverseBits((uint32_t)prudata2[10]) + (reverseBits8((uint8_t)prudata2[11] & 0xFF) << 29);

  oldposition[0] = position[0];
  oldposition[1] = position[1];
  oldposition[2] = position[2];
  oldposition[3] = position[3];



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


  redisReply *reply;

  for (;;) {
    position[0] =
        reverseBits((uint32_t)prudata1[2]) + (reverseBits8((uint8_t)prudata1[3] & 0xFF) << 29);
    position[1] =
        reverseBits((uint32_t)prudata1[10]) + (reverseBits8((uint8_t)prudata1[11] & 0xFF) << 29);
    position[2] =
        reverseBits((uint32_t)prudata2[2]) + (reverseBits8((uint8_t)prudata2[3] & 0xFF) << 29);
    position[3] =
        reverseBits((uint32_t)prudata2[10]) + (reverseBits8((uint8_t)prudata2[11] & 0xFF) << 29);

    for(int encoder = 0; encoder < 4; encoder++){
      if ((position[encoder] != oldposition[encoder]) && enableFF) { // A variável enableFF controlará quando haverá a correção de orbita. O IOC do FF rápido deve alterar esse parametro.
        pthread_mutex_lock(&serial_mutex);

        adjustVector(&setpoints, encoder, position[enc]);

        if !setpoints->msg.checksum{
          write(fd, setpoints.data_vector, 22); // Aqui envia o valor "data vetor" pela rede 485, não há implementações em data vetor, logo deve-se escrever algo para que esteja no padrão de comunicação das fontes.
        }

        pthread_mutex_unlock(&serial_mutex);
        oldposition[encoder] = position[encoder];
        }
    }
  }
}
