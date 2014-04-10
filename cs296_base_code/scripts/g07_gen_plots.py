##
import matplotlib.pyplot as plotterglob
from pylab import *
f = open('./data/g07_lab05data_02.csv')
h = open('./data/g07_lab05data_random.csv')
readh = h.read()
readat = f.read()
dat2 = readh.strip().split('\n')
dat = readat.strip().split('\n')
dat = [[y.strip() for y in x.split(',')] for x in [t.strip() for t in dat]]
dat2 = [[y.strip() for y in x.split(',')] for x in [t.strip() for t in dat2]]
h.close()
f.close()
#plot stuff
fig, plotter = plotterglob.subplots();
plot1 = []
avsttime= [0 for x in range(50)];
avlooptime= [0 for x in range(50)];
avvltime= [0 for x in range(50)];
avpuptime= [0 for x in range(50)];
avcoltime = [0 for x in range(50)];
avrantime = [0 for x in range(50)];
for x in dat:
	avsttime[int(float(x[0]))-1]=avsttime[int(float(x[0]))-1]+float(x[2]);
	avlooptime[int(float(x[0]))-1] = avlooptime[int(float(x[0]))-1]+float(x[-2]);
	avvltime[int(float(x[0]))-1] = avvltime[int(float(x[0]))-1]+float(x[4]);
	avpuptime[int(float(x[0]))-1] = avpuptime[int(float(x[0]))-1]+float(x[5]);
	avcoltime[int(float(x[0]))-1] = avcoltime[int(float(x[0]))-1]+float(x[3]);

for x in dat2:
	avrantime[int(float(x[0]))-1]=avrantime[int(float(x[0]))-1]+float(x[2]);

avsttime= [x/50.0 for x in avsttime];
avlooptime = [x/50.0 for x in avlooptime];
avvltime= [x/50.0 for x in avvltime];
avpuptime = [x/50.0 for x in avpuptime];
avcoltime= [x/50.0 for x in avcoltime];
avrantime= [x/50.0 for x in avrantime];
sumtime = [0 for x in range(50)];
for i in range(50):
	sumtime[i] = avvltime[i]+avcoltime[i]+avpuptime[i];

xtix = [x+1 for x in range(50)];

maxavloop = max(avlooptime);
mxi = avlooptime.index(maxavloop);
minavloop = min(avlooptime);

mni = avlooptime.index(minavloop);
plotter2 = plotter.twinx();
plotter2.plot( xtix, avlooptime, color='r', label=r'Avg. loop time')
plotter.bar( xtix, avsttime, label=r'Av. step time');
plotter.legend(loc='upper right');
plotter2.legend(loc='upper left');
plotter2.annotate('Max', (mxi,floor(maxavloop)), xytext=(mxi,floor(maxavloop)), xycoords='data',
         textcoords='data', arrowprops=None)
plotter2.annotate('Min', (mni,floor(minavloop)), xytext=(mni,floor(minavloop)), xycoords='data',
         textcoords='data', arrowprops=None)

plotter2.set_ylabel("Avg. loop time", color='r');
plotter2.set_ylim(0,20);
plotter.set_ylabel(("Avg. step time"), color='b');
plotterglob.xlabel("No. iters");
fig.savefig("./plots/g07_lab09_plot01.png")

#plot2 stuff
plotter.cla();
plotter2.cla();
plotterglob.plot( xtix, avsttime, color='r',label='av step time' );
plotterglob.plot( xtix, avvltime, color='b' , label = 'av vel up time');
plotterglob.plot( xtix, avpuptime, color='g' , label = ' av pos up time');
plotterglob.plot( xtix, avcoltime, color='y' , label = 'av coll time');
plotterglob.plot( xtix, sumtime , color='c', label = 'av tot time');
plotterglob.legend(loc='upper right');
plotterglob.savefig("./plots/g07_lab09_plot02.png")

#plot3 stuff
plotter.cla();
plotter2.cla();
maxarr = [x for x in avsttime];
minarr = [x for x in avsttime];
for x in dat:
	if maxarr[int(float(x[0]))-1] < float(x[2]):
		maxarr[int(float(x[0]))-1] = float(x[2])
	if minarr[int(float(x[0]))-1] > float(x[2]):
		minarr[int(float(x[0]))-1] = float(x[2])

errarr = list(range(50));
for x in range(50):
	errarr[x]=maxarr[x]-minarr[x];

plotterglob.errorbar( xtix, avsttime, xerr=0, yerr=errarr, label='Step time with error');
plotterglob.legend(loc='upper right');
plotterglob.savefig("./plots/g07_lab09_plot03.png");

#plot4 stuff
plotter.cla();
plotter2.cla();
rolliter = dat[(19*50):(19*50+50)];
freqstep = [0 for x in range(50)]
for x in rolliter:
	freqstep[int(float(x[1])) - 1] = float(x[2])

plotter.hist(freqstep,10, label='Freq plot')
plotter2.hist(freqstep,10,cumulative=True,histtype="step",color='r', label='cum freq')
plotter.legend(loc='upper right');
plotter2.legend(loc='upper left');
plotterglob.savefig("./plots/g07_lab09_plot04.png");

#plot5 stuff]

plotter.cla();
plotter2.cla();
plotter.scatter(xtix,avrantime, label='rand samp step time scatter')
m1,b1 = polyfit(xtix,avrantime,1)
avrantimey = [m1*x + b1 for x in xtix]
plotter.plot(xtix,avrantimey, label='rand samp step time line fit')
plotter2.scatter(xtix,avsttime,color='r', label = 'step time scatter')
m2,b2 = polyfit(xtix,avsttime,1)
avsttimey = [m2*x + b2 for x in xtix]
plotter2.plot(xtix,avsttimey,color='r', label='step time line fit')
plotter2.legend(loc='upper right');
plotter.legend(loc='upper left');
plotterglob.savefig("./plots/g07_lab09_plot05.png");



