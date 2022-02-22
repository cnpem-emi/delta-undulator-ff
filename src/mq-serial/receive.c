#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mqueue.h>
#include <poll.h>
#include <stdio.h>

#define BAUD B3000000

int main()
{
    int fd;

    struct mq_attr attr;

    attr.mq_maxmsg = 1000;
    attr.mq_msgsize = 32;
    attr.mq_flags = 0;

    mqd_t cmd_mq = mq_open("/cmd_mq", (O_RDWR | O_CREAT), 0666, &attr);
    mqd_t reply_ioc = mq_open("/reply_ioc", (O_RDWR | O_CREAT), 0666, &attr);
    mqd_t reply_ff = mq_open("/reply_ff", (O_RDWR | O_CREAT), 0666, &attr);

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); // TODO: Handle errors
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

    size_t write_size, reply_size;
    char cmd_buf[32];
    char reply_buf[32];

    unsigned int priority;

    struct pollfd pfds[1];
    pfds[0].fd = fd;
    pfds[0].events = POLLIN;

    for (;;)
    {
        write_size = mq_receive(cmd_mq, cmd_buf, sizeof(cmd_buf), &priority);
        write(fd, cmd_buf, write_size);

        if (poll(pfds, 1, 1) < 1)
        {
            mq_send(reply_ff, "\xee", 1, priority);
            continue;
        }

        reply_size = read(fd, &reply_buf, sizeof(reply_buf));

        mq_send(priority == 0 ? reply_ioc : reply_ff, reply_buf, reply_size, priority);
    }

    //TODO: Close out elegantly
    mq_close(cmd_mq);
    mq_close(reply_ioc);
    mq_close(reply_ff);

    return 0;
}
