# build lv_lora executable when user executes "make"
CROSS_COMPILE ?= mips-openwrt-linux-
CC := $(CROSS_COMPILE)gcc
loraradiotest: main.o 
	$(CC) $(LDFLAGS) -lpthread -ldl *.o -o loraradiotest

main.o: main.c
	$(CC) $(CFLAGS) -c main.c
	

# remove object files and executable when user executes "make clean"
clean:
	rm *.o loraradiotest
