#!/bin/bash

rm -rf ../data/g07_lab05data_random.csv

for i in {1..50}
do
	start=$((($i-1)*50+1))
	end=$((($start)+49))
	sed -n ''$start', '$end' p' ../data/g07_lab05data_01.csv | shuf -n 15 | sort  >>  ../data/g07_lab05data_random.csv
done
