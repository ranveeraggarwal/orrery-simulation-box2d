#!/bin/bash
#iters steptime colltime veltime postime looptime sum(col vel pos) dev

awk 'BEGIN {avg3=0; cur="1,"; curno=1; }
	cur != $1 {
		printf curno " " (avg3/15) "\n";
		avg3=0;
		cur=$1;
		curno++;
	}
	cur == $1 {
		avg3=avg3+$3;
	}
	END {
	printf curno " " (avg3/15) "\n";
        }' < ../data/g07_lab05data_random.csv > gdata3.temp

