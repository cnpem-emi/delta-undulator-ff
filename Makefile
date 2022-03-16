SHELL = /bin/sh
CFLAGS := -O2 -march=native -Wall -pthread -lrt
CC := gcc

COMPILE.c = $(CC) $(CFLAGS)
SRCS = $(wildcard *.c)
PROGS = $(patsubst %.c,%,$(SRCS))

OUT = bin

build: directories $(PROGS)

directories: $(OUT)

$(OUT):
	mkdir -p $(OUT)

%: %.c
	$(CC) $< -o $@ $(CFLAGS)
