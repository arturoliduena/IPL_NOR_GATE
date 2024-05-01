all: nlsp

nlsp: nlsp.o
	$(CCC) $(CCFLAGS) nlsp.o -o nlsp $(CCLNFLAGS)
nlsp.o: nlsp.cpp
	$(CCC) -c $(CCFLAGS) nlsp.cpp -o nlsp.o

clean:
	rm --force nlsp nlsp.o

run_instances:
	@./run.sh

run_and_save:
	@./run_save_out.sh

################################

SYSTEM     = x86-64_osx
LIBFORMAT  = static_pic
CPLEXDIR      = /Applications/CPLEX_Studio_Community2211/cplex
CONCERTDIR    = /Applications/CPLEX_Studio_Community2211/concert

################

CCC = g++

# CCOPT = -std=c++11 -m64 -O3 -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD
CCOPT = -std=c++17 -arch x86_64 -m64 -fPIC -fexceptions -DNDEBUG -stdlib=libc++

CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -pthread -ldl

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) 