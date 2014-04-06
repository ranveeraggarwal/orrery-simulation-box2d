#!/bin/bash

rm -rf ../data/g07_lab05data_random.csv

for i in {1..1500}
do
	start=$((($i-1)*150+1))
	end=$((($start)+149))
	sed -n ''$start', '$end' p' ../data/g07_lab05data_02.csv | shuf -n 15 | sort  >>  ../data/g07_lab05data_random.csv
done
