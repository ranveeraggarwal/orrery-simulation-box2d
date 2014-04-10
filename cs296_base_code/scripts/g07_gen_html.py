import os, re, subprocess
texfile=open("./doc/tex/cs296_report_07.tex",'r');
htmfile=open("./doc/g07_prof_htm.html",'w');
lw=texfile.readlines();
lw=[x.replace("\\\\" , "<br>") for x in lw]
lw=[x.strip() for x in lw]

print("<!DOCTYPE html>", file=htmfile)
print("<html>", file=htmfile)
print("<head>", file=htmfile)
print("<title>", file=htmfile)

sect=0
for i in range(len(lw)):
	d=0
	k1=re.match(r'\\title{',lw[i])
	if(k1):
		d=1
		print(lw[i][k1.end():-1], file=htmfile)
		print("</title></head><body><div id='ttle'><h1>", file=htmfile)
		print(lw[i][k1.end():-1], file=htmfile)
		print("</h1>", file=htmfile)
	k1=re.match(r'\\author{',lw[i])
	if(k1):
		d=1
		print("</div><div id='auth'><h1>", file=htmfile)
		print(lw[i][k1.end():-1], file=htmfile)
		print("</h1>", file=htmfile)
	k1=re.match(r'\\section{', lw[i])
	if(k1):
		d=1
		print("</div><div id='sec'><h1>", file=htmfile)
		print(lw[i][k1.end():-1], file=htmfile)
		print("</h1>", file=htmfile)
	k1=re.match(r'\\subsection{', lw[i])
	if(k1):
		d=1
		print("</div><div id='subsec'><h1>", file=htmfile)
		print(lw[i][k1.end():-1], file=htmfile)
		print("</h1>", file=htmfile)
		
	k1=re.match(r'\\includegraphics\[.*\]{', lw[i]);
	if(k1):
		d=1
		print("</div><div id='subsec'><img src = './tex/", file=htmfile)
		print(lw[i][k1.end():-1], file=htmfile)
		print("'>", file=htmfile)
		
	k1=re.match(r'[a-zA-Z\<]',lw[i])
	if(k1):
		print(lw[i], file=htmfile);


print("</div><div id = 'sec'><h1>Matplotlib plots</h1>",file = htmfile);
print("</div><div><h1>Plot 1</h1><img src='tex/img/g07_lab09_plot01.png'></div>", file=htmfile)

print("<div><h1>Plot 2</h1><img src='tex/img/g07_lab09_plot02.png'></div>", file=htmfile)

print("<div><h1>Plot 3</h1><img src='tex/img/g07_lab09_plot03.png'></div>", file=htmfile)
print("<div><h1>Plot 4</h1><img src='tex/img/g07_lab09_plot04.png'></div>", file=htmfile)

print("<div><h1>Plot 5</h1><img src='tex/img/g07_lab09_plot05.png'></div>", file=htmfile)
print("<div><h3>References<h3></div>", file = htmfile);
f1 = open('./doc/tex/cs296_report_07.bib');
f1dat = f1.read();
fspl = re.findall(r'title = {(.*)}', f1dat);
for i in range(4):
	print("<div><p>", file = htmfile);
	print(fspl[i], file = htmfile)
	print("</p></div>", file = htmfile)
print("</body></html>", file=htmfile)
#close </div>, </body>, </html>
#Add images regex

		


		

		
