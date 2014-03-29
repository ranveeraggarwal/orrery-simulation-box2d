.PHONY: setup exe statlib

#Paths
ROOT=.
BOX2D_ROOT=$(ROOT)/external
BOX2D_LIB=$(BOX2D_ROOT)/lib
BOX2D_INCLUDE=$(BOX2D_ROOT)/include
BOX2D_SOURCE=$(BOX2D_ROOT)/src
BOX2D_ARCHIVE=$(BOX2D_SOURCE)/Box2D
BOX2D_BUILD=$(BOX2D_ARCHIVE)/build296
OBJ_DIR=$(ROOT)/myobjs
SRC_DIR=$(ROOT)/src
BIN_DIR=$(ROOT)/mybins
LIB_DIR=$(ROOT)/mylibs
SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
LIB_FILE=$(LIB_DIR)/*
LIB_OBJS = $(subst $(OBJ_DIR)/main.o, , $(OBJECTS))


# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing -fPIC
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

#Vars
SHARED_LIB = TRUE

#IO formatting
NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING=[ERRORS] See temp.log for first warning/error.
WARN_STRING=[WARNINGS] See temp.log for first warning/error.
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"

run: setup exe exelib

libgen: setup
	@if [ $(SHARED_LIB) = FALSE ] ;\
	then rm -f $(LIB_DIR)/libCS296test.so ;\
	make statlib ;\
	else rm -f $(LIB_DIR)/libCS296test.a ;\
	make dylib ;\
	fi;


setup:
	@rm -Rf temp.log temp.err
	@printf "Setting up installation..\n"
	@mkdir -p $(ROOT)/myobjs $(ROOT)/mybins $(ROOT)/mylibs
	@if [ ! -f $(BOX2D_LIB)/libBox2D.a ] || [ ! -d $(BOX2D_INCLUDE)/Box2D ] ;\
		then\
		if [ ! -d $(BOX2D_ARCHIVE) ];\
			printf "Extracting Box2D files..." ;\
			then tar xfz $(BOX2D_SOURCE)/Box2D.tgz --directory $(BOX2D_SOURCE) 2>>temp.log || touch temp.err;\
			if test -e temp.err; \
				then printf "$(ERR_STRING)\n"; \
			elif test -s temp.log; \
				then printf "$(WARN_STRING)\n"; \
			else printf "$(OK_STRING)\n"; \
			fi;\
		fi;\
		mkdir -p $(BOX2D_BUILD) ;\
		cd $(BOX2D_BUILD) ;\
		printf "Running cmake..." ;\
		cmake ../ 1> /dev/null 2>>temp.log || touch temp.err;\
		if test -e temp.err; \
			then printf "$(ERR_STRING)\n"  ; \
		elif test -s temp.log; \
			then printf "$(WARN_STRING)\n"  ; \
		else printf "$(OK_STRING)\n"; \
		fi;\
		printf "Running make..." ;\
		make 1> /dev/null 2>>temp.log || touch temp.err;\
		if test -e temp.err; \
			then printf "$(ERR_STRING)\n"  ; \
		elif test -s temp.log; \
			then printf "$(WARN_STRING)\n"  ; \
		else printf "$(OK_STRING)\n"; \
		fi;\
		printf "Running make install..." ;\
		make install 1> /dev/null 2>> temp.log || touch temp.err;\
		if test -e temp.err; \
			then printf "$(ERR_STRING)\n"  ; \
		elif test -s temp.log; \
			then printf "$(WARN_STRING)\n"  ; \
		else printf "$(OK_STRING)\n"; \
		fi;\
	fi;
	@printf "Box2D installation completed/verified \n"

$(OBJECTS) : $(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp 
	@printf "Generating $@ from $<..."
	@g++ $(CPPFLAGS) -c $< -o $@ -MD
	@printf "[OK]\n"


exe: setup  $(OBJECTS)
	@printf "Generating exe from objects..."
	@g++ -o $(ROOT)/mybins/cs296_07_exe $(LDFLAGS) $(OBJECTS) $(LIBS) 2>>temp.log || touch temp.err
	@if test -e temp.err; \
		then printf "$(ERR_STRING)\n"  ; \
	elif test -s temp.log; \
		then printf "$(WARN_STRING)\n"  ; \
	else printf "$(OK_STRING)\n"; \
	fi;\

statlib: $(LIB_OBJS)
	@printf "Generating static libraries..."
	@ar -cvq $(LIB_DIR)/libCS296test.a $(LIB_OBJS) 1> /dev/null 2>> temp.log || touch temp.err
	@if test -e temp.err; \
		then printf "$(ERR_STRING)\n"  ; \
	elif test -s temp.log; \
		then printf "$(WARN_STRING)\n"  ; \
	else printf "$(OK_STRING)\n"; \
	fi;\

dylib: $(LIB_OBJS)
	@printf "Generating dynamic shared library..."
	@g++ -shared -Wl,-soname,libCS296test.so -o $(LIB_DIR)/libCS296test.so  $(LIB_OBJS) 2>> temp.log || touch temp.err
	@if test -e temp.err; \
		then printf "$(ERR_STRING)\n"  ; \
	elif test -s temp.log; \
		then printf "$(WARN_STRING)\n"  ; \
	else printf "$(OK_STRING)\n"; \
	fi;\

exelib: libgen $(OBJECTS)
	@printf "Compiling exe from libraries..."
	@g++ -o $(ROOT)/mybins/cs296_07_exelib $(LDFLAGS) -L $(LIB_DIR)/ $(OBJ_DIR)/main.o  -lCS296test $(LIBS)  2>> temp.log || touch temp.err
	@if test -e temp.err; \
		then printf "$(ERR_STRING)\n"  ; \
	elif test -s temp.log; \
		then printf "$(WARN_STRING)\n"  ; \
	else printf "$(OK_STRING)\n"; \
	fi;\

clean:
	@printf "Cleaning up compiled files..."
	@rm -Rf my* 
	@printf "[OK]\n"
	@rm -Rf temp*
	@printf "Cleaning up documentation..."
	@rm -Rf ./doc/html
	@mv ./doc/tex/report.tex ./doc/tex/bakrep.tex 
	@mv ./doc/tex/report.bib ./doc/tex/bakrep.bib
	@rm -f ./doc/tex/report.*
	@mv ./doc/tex/bakrep.tex ./doc/tex/report.tex
	@mv ./doc/tex/bakrep.bib ./doc/tex/report.bib
	@printf "$(OK_STRING)\n"

distclean: clean
	@printf "Cleaning up Box2D install..."
	@rm -Rf $(BOX2D_LIB) $(BOX2D_INCLUDE) $(BOX2D_ARCHIVE)
	@mkdir $(BOX2D_LIB) $(BOX2D_INCLUDE)
	@printf "[OK]\n"


doc: ./doc/Doxyfile $(SOURCE_DIR)/*
	@printf "Building documentation..."
	@rm -Rf ./doc/html
	@doxygen ./doc/Doxyfile 1> /dev/null 2> temp.log
	@printf "$(OK_STRING)\n"

report: ./doc/tex/report.*
	@printf "Building pdf from tex..."
	@cd ./doc/tex; pdflatex -interaction=batchmode report.tex 1>/dev/null 2>temp.log;\
	bibtex report 1>/dev/null 2>temp.log;\
	pdflatex -interaction=batchmode report.tex 1>/dev/null 2>temp.log;\
	pdflatex -interaction=batchmode report.tex; 1>/dev/null 2>temp.log
	@printf "$(OK_STRING)\n"
