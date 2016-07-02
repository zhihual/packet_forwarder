#!/bin

rmmod example
insmod /bin/loradio
mknod /dev/loradio c 252 0
