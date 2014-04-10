#!/bin/bash
for (( i=1; i<=50; i++ ))
do
	for ((j=1; j<=50; j++ ))
	do
		../mybins/cs296_07_exe $i > ../data/g07out-$i-$j.txt
	done
done
