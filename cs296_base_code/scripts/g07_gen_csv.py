#!/usr/bin/python3
import os, sys, subprocess, random

f=open("./data/g07_lab05data_02.csv",'w');
f1=open("./data/g07_lab05data_rand.csv",'w');
for i in range(1,51):
	selperm=random.sample(range(0,51),15);
	for j in range(1,51):
		print(i,j);
		op = subprocess.check_output("./mybins/cs296_07_exe "+str(i), shell=True);
		op = op.split();
		times = str(i)+','+str(j)+','
		for p in range(len(op)):
			if op[p] == b'ms':
				times=times+str(float(op[p-1]))+','
		print(times,file=f);
		if j in selperm:
			print(times,file=f1);
f.close();
f1.close();

