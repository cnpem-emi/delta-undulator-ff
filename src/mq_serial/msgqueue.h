#ifndef MSGQUEUE_H
#define MSGQUEUE_H

struct msg_buf {
    long msg_type;
    char msg_text[16];
};

#endif
