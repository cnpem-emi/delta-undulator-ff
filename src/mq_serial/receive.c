#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mqueue.h>

#define BAUD B115200


int main()
{
    int fd;

    struct mq_attr attr;

    attr.mq_maxmsg = 128;
    attr.mq_msgsize = 32;
    attr.mq_flags = 0;

    mqd_t cmd_ff = mq_open("/cmd_ff", (O_RDWR | O_CREAT), 0666, &attr);
    mqd_t reply_ff = mq_open("/reply_ff", (O_RDWR | O_CREAT), 0666, &attr);

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); // TODO: Handle errors
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

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, BAUD);
    cfsetospeed(&tty, BAUD);

    tcsetattr(fd, TCSANOW, &tty); // TODO: Handle errors

    size_t write_size, reply_size;
    char cmd_buf[32];
    char reply_buf[32];

    unsigned int priority;

    for(;;) {
        write_size = mq_receive(cmd_ff, cmd_buf, sizeof(cmd_buf), &priority);
        write(fd, cmd_buf, write_size);
        reply_size = read(fd, &reply_buf, sizeof(reply_buf));
        if(priority == 0)
            mq_send(reply_ff, reply_buf, reply_size, priority);
        else
            mq_send(reply_ff, reply_size < 0 ? "\xee" : "\xff", 1, priority);
    }

    //TODO: Close out elegantly
    mq_close(cmd_ff);
    mq_close(reply_ff);

    return 0;
}

