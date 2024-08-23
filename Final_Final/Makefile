# Makefile for compiling and linking the 10Hz.c, 1Hz.c, and 10HzAdditional.c programs

# Compiler and flags
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -pedantic
LDFLAGS = -lrt

# Source and object files
CFILES = 10Hz.c 1Hz.c 10HzAdditional.c
OBJS = ${CFILES:.c=.o}

# Default target: build all the executables
all: 10Hz 1Hz 10HzAdditional

# Rule to link the 10Hz executable
10Hz: 10Hz.o
	$(CC) $(CFLAGS) -o $@ 10Hz.o $(LDFLAGS)

# Rule to link the 1Hz executable
1Hz: 1Hz.o
	$(CC) $(CFLAGS) -o $@ 1Hz.o $(LDFLAGS)

# Rule to link the 10HzAdditional executable
10HzAdditional: 10HzAdditional.o
	$(CC) $(CFLAGS) -o $@ 10HzAdditional.o $(LDFLAGS)

# Rule to compile .c files to .o files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up the build directory by removing object files and the executables
clean:
	-rm -f *.o 10Hz 1Hz 10HzAdditional
	-rm -f frames10hz/*
	-rm -f frames1hz/*
	-rm -f frames10hzAdditional/*
	-rmdir frames10hz
	-rmdir frames1hz
	-rmdir frames10hzAdditional
	-rm -f 10Hz_capture.log 1Hz_capture.log 10HzAdditional_capture.log
	-rm -f *.txt

# Remove object files, executables, and additional generated files (useful for a fresh rebuild)
distclean: clean
	-rm -f output.webm
