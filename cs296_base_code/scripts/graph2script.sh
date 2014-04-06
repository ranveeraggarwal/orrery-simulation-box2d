#!/bin/bash
#iters=56

start=$((150*(56-1)+1))
end=$(($start+149))

sed -n ''$start', '$end' p' ../data/g07_lab05data_02.csv | awk '{ printf ($3 * 1) "\n" }' > gdata2.temp

