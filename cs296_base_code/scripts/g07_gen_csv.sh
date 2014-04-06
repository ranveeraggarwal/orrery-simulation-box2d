#!/bin/bash

rm -rf ../data/g07_lab05data_01.csv

for i in {1..1500}
do
	for j in {1..150}
	do
		awk 'BEGIN {printf '$i' ", " '$j' ", " } NF > 0 && NR > 1 { printf $(NF-1) ", " } END { printf"\n"}' < ../data/g07out-$i\-$j.txt >> ../data/g07_lab05data_01.csv
	done
done
