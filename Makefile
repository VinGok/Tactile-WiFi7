.EXPORT_ALL_VARIABLES:
COMPILER:=g++
FLAGS:=-Wall -Wextra -Wpedantic #-Werror
DEBUGGER=gdb

DIRS = $(patsubst %/Makefile,%,$(shell echo */Makefile))

all::	$(addprefix .all.,$(DIRS))
run::	$(addprefix .run.,$(DIRS))
clean::	$(addprefix .clean.,$(DIRS))
debug::	$(addprefix .debug.,$(DIRS))
clean-results:: $(addprefix .clean-results.,$(DIRS))

.all.%:
	make -C ./$*
	mkdir -p .make
	touch .make/$@

.debug.%:
	make -C ./$* debug
	mkdir -p .make
	touch .make/$@

.run.%:
	make -C ./$* run

.clean.%:
	make -C ./$* clean
	rm -rf .make

.clean-results.%:
	make -C ./$* clean-results