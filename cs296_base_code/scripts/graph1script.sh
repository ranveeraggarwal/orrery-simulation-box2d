#!/bin/bash
#iters steptime colltime veltime postime looptime sum(col vel pos) dev

awk 'BEGIN {min=9999999; max=0; avg3=0; avg4=0; avg5=0; avg6=0; avg7=0; cur="1,"; curno=1; }
	cur != $1 {
		printf curno " " (avg3/150) " " (avg4/150) " " (avg5/150) " " (avg6/150) " " (avg7/150) " " ((avg4+avg5+avg6)/150) " " (1*max - 1*min) "\n";
		avg3=0;
		avg4=0;
		avg5=0;
		avg6=0;
		avg7=0;
		cur=$1;
		min=$3;
		max=$3;
		curno++;
	}
	cur == $1 {
		avg3=avg3+$3;
		avg4=avg4+$4;
		avg5=avg5+$5;
		avg6=avg6+$6;
		avg7=avg7+$7;
		if($3 < min) min = $3;
		if($3 > max) max = $3;
	}
	END {
	printf curno " " (avg3/150) " " (avg4/150) " " (avg5/150) " " (avg6/150) " " (avg7/150) " " ((avg4+avg5+avg6)/150) " " (1*max - 1*min)  "\n";
        }' < ../data/g07_lab05data_01.csv > gdata.temp

