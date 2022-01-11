SHELL = /bin/sh
CFLAGS := -O3 -lrt -Wall
CC := gcc

COMPILE.c = $(CC) $(CFLAGS)

SRCS = $(wildcard *.c)
PROGS = $(patsubst %.c,%,$(SRCS))

OUT = bin

.PHONY: all directories clean

all: $(PROGS)

%: %.c
	$(COMPILE.c) $^ -o $@

clean:
	rm -f $(PROGS)