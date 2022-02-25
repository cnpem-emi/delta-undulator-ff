SHELL = /bin/sh
CFLAGS := -O3 -march=native -Wall -lpthread -lrt
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
	$(CC) $(CFLAGS) $< -o $@
