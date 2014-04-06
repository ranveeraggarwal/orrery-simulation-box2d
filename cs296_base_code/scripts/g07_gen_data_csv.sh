#!/bin/bash

rm -rf ../data/g07_lab05data_02.csv

for (( i=1; i<=1500; i++ ))
do
	for ((j=1; j<=150; j++ ))
	do
		../mybins/cs296_07_exe $i |\
		 awk 'BEGIN {printf '$i' ", " '$j' ", " } NF > 0 && NR > 1 { printf $(NF-1) ", " } END { printf"\n"}'  >> ../data/g07_lab05data_02.csv
	done
done
