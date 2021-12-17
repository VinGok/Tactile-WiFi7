CC=g++
FLAGS=-Wall -Wextra -Wpedantic #-Werror
OUTPUTFILE=./bin/$(shell basename $(CURDIR))
DEBUGFILE=$(OUTPUTFILE)-debug
CPPFILES=$(wildcard ./*.cpp)

.PHONY: clean run debug clean-aux

$(OUTPUTFILE):
	mkdir -p "bin"
	$(CC) $(FLAGS) $(CPPFILES) -o $(OUTPUTFILE)

run: $(OUTPUTFILE)
	$(OUTPUTFILE)

clean:
	rm -f $(OUTPUTFILE) $(DEBUGFILE)

debug:
	mkdir -p "bin"
	$(CC) -g $(FLAGS) $(CPPFILES) -o $(DEBUGFILE)

clean-aux:
	rm -f $(filter-out %Readme.txt,$(wildcard ./*.txt ./bin/*.txt))