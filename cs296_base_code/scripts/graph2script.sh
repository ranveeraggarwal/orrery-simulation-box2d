#!/bin/bash
#iters=56

start=$((50*(26-1)+1))
end=$(($start+49))

sed -n ''$start', '$end' p' ../data/g07_lab05data_01.csv | awk '{ printf ($3 * 1) "\n" }' > gdata2.temp

