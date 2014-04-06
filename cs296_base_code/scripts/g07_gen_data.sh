#!/bin/bash
for (( i=1; i<=1500; i++ ))
do
	for ((j=1; j<=150; j++ ))
	do
		../mybins/cs296_07_exe $i > ../data/g07out-$i-$j.txt
	done
done
