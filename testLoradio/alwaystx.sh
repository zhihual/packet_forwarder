#!/bin/sh


min=1
max=100
while [ $min -le $max ]
do
	echo $min
 	min=$((min+1))
 	./loraradiotest -s 200;
done;
