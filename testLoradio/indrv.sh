#!/bin/sh


echo "remove exampel"
rmmod example

echo "remove loradio"
rmmod loradio
echo "load radio.ko"
insmod /bin/loradio.ko

echo "creat node"
mknod /dev/loradio c 252 0
