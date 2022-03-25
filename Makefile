SHELL = /bin/sh
CFLAGS := -O3 -march=native -Wall -pthread -lrt -lhiredis -levent
CC := gcc

COMPILE.c = $(CC) $(CFLAGS)
SRCS = $(wildcard *.c)
PROGS = $(patsubst %.c,%,$(SRCS))

OUT = bin

build: directories $(PROGS)

directories: $(OUT)

$(OUT):
	mkdir -p $(OUT)

redis:
	git clone https://github.com/redis/hiredis.git
	cd ./hiredis; make && make install
	grep -qxF 'include /usr/local/lib' /etc/ld.so.conf || \
	echo 'include /usr/local/lib' >> /etc/ld.so.conf
	ldconfig
	rm -rf hiredis

%: %.c
	$(CC) $< -o $@ $(CFLAGS)
