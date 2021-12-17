CC:=g++
FLAGS:=-Wall -Wextra -Wpedantic #-Werror
OUTPUTFILE:=$(abspath ./bin/$(shell basename $(CURDIR)))
CPPFILES:=$(wildcard ./*.cpp)
DEBUGFILE:=$(OUTPUTFILE)-debug
DEBUGGER=gdb

.PHONY: clean run debug clean-aux run-debug

$(OUTPUTFILE): $(CPPFILES)
	mkdir -p "bin"
	$(CC) $(FLAGS) $(CPPFILES) -o $(OUTPUTFILE)

run: rundir=ResultsFiles/run-$(shell date +"%Y-%m-%d-%T")
run: $(OUTPUTFILE)
	mkdir -p $(rundir)
	cd $(rundir) && $(OUTPUTFILE)

clean:
	rm -f $(OUTPUTFILE) $(DEBUGFILE)

debug: $(DEBUGFILE)

$(DEBUGFILE): $(CPPFILES)
	mkdir -p "bin"
	$(CC) -g $(FLAGS) $(CPPFILES) -o $(DEBUGFILE)

run-debug: rundir=ResultsFiles/run-debug-$(shell date +"%Y-%m-%d-%T")
run-debug: $(DEBUGFILE)
	mkdir -p $(rundir)
	cd $(rundir) && $(DEBUGGER) $(DEBUGFILE)

clean-aux:
	rm -f $(filter-out %Readme.txt,$(wildcard ./*.txt ./bin/*.txt))
	rm -rf $(wildcard ResultsFiles/run-*)