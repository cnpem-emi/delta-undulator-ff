#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <hiredis/hiredis.h>
#include <hiredis/adapters/libevent.h>
#include <unistd.h>
#include <pthread.h>

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

//correctionTable tableEntry[5] = {{.name = "Array:Test-Mon", .colSize = 0},
//                                 {.name = "Array2:Test-Mon", .colSize = 0},
//                                 {.name = "Array3:Test-Mon", .colSize = 0},
//                                 {.name = "Array4:Test-Mon", .colSize = 0},
//                                 {.name = "Array5:Test-Mon", .colSize = 0}};

correctionTable tableEntry[2] = {{.name = "PositionMon", .colSize = 0},
                                 {.name = "CurrentMon", .colSize = 0}};

pthread_mutex_t serial_mutex;
redisContext* sync_c;

char str3[100];
char str4[100];

static double s_position[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static double s_current[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


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
        
        for (int j = 0; j < 5; j++) {
          for (int k = j * arrayDivs; k < (j + 1) * arrayDivs; k++){
            tableEntry[i].cols[j][k - j * arrayDivs] = strtod(r->element[k]->str, NULL);

            if(i){
               s_current[j] = tableEntry[i].cols[j][k - j * arrayDivs];
            }
            else{
               s_position[j] = tableEntry[i].cols[j][k - j * arrayDivs];
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
    redisAsyncCommand(c, onTableChange, NULL, "SUBSCRIBE ArraySubscription");
    event_base_dispatch(base);
  } else {
    printf("Redis server not available!\n");
  }

}

void adjustVector(char * position) {

    float result;

    // Esta interpolação está sendo feita ponto a ponto, mas, talvez, fique mais rápido gerar as funções lineares antes
    for(int pos = 0; pos < (sizeof(s_current)/sizeof(s_current[0]) - 1); pos++){
      printf("%f %f %f\n", atof(position), s_position[pos], s_position[pos+1]);
      if((s_position[pos] <= atof(position)) && (atof(position) <= s_position[pos+1])){
        float ang = (s_current[pos+1] - s_current[pos])/(s_position[pos+1] - s_position[pos]);
        float lin =  s_current[pos] - ang * s_position[pos];

        result = ang * atof(position) + lin;
        printf("%s -> %f\n", position, ang*atof(position) + lin);
        break;
    }
  }
}



int main (int argc, char **argv) {

  pthread_t thisThread = pthread_self();
  pthread_t cmdThread;

  pthread_create(&cmdThread, NULL, listenForCommands, NULL);
  pthread_mutex_init(&serial_mutex, NULL);

  struct sched_param params;
  params.sched_priority = sched_get_priority_max(SCHED_FIFO);

  pthread_setschedparam(thisThread, SCHED_FIFO, &params);

 // pthread_setaffinity_np(thisThread, sizeof(cpu_set_t), &mainCpuSet);
 // pthread_setaffinity_np(cmdThread, sizeof(cpu_set_t), &cmdCpuSet);


  redisReply *reply;

  /* PINGs */
  while(1){
     adjustVector("200");
     printf("Here\n");
     sleep(10);
  }
}
