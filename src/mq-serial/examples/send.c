#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <sys/stat.h>

int main() {
  struct mq_attr attr;

  attr.mq_maxmsg = 1000;
  attr.mq_msgsize = 32;
  attr.mq_flags = 0;

  mqd_t cmd_ff = mq_open("/cmd_mq", (O_RDWR | O_CREAT), 0666, &attr);
  mqd_t reply_ff = mq_open("/reply_ff", (O_RDWR | O_CREAT), 0666, &attr);

  char cmd_buf[32];
  char reply_buf[32];

  // SAMPLE
  memcpy(cmd_buf, "\x01\x10\x00\x01\x03\xeb", 6);

  for (;;) {
    mq_send(cmd_ff, cmd_buf, 6, 0);
    mq_receive(reply_ff, reply_buf, sizeof(reply_buf), 0);
    // Do something with reply.msg_text
  }

  mq_close(cmd_ff);
  mq_close(reply_ff);

  return 0;
}
