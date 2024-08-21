# Makefile for compiling and linking the 10Hz.c and 1Hz.c programs

# Compiler and flags
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -pedantic
LDFLAGS = -lrt

# Source and object files
CFILES = 10Hz.c 1Hz.c
OBJS = ${CFILES:.c=.o}

# Default target: build both the 10Hz and 1Hz executables
all: 10Hz 1Hz

# Rule to link the 10Hz executable
10Hz: 10Hz.o
	$(CC) $(CFLAGS) -o $@ 10Hz.o $(LDFLAGS)

# Rule to link the 1Hz executable
1Hz: 1Hz.o
	$(CC) $(CFLAGS) -o $@ 1Hz.o $(LDFLAGS)

# Rule to compile .c files to .o files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up the build directory by removing object files and the executables
clean:
	-rm -f *.o 10Hz 1Hz
	-rm -f frames10hz/*
	-rm -f frames1hz/*
	-rm -f 10Hz_capture.log 1Hz_capture.log
	-rm -f *.txt

# Remove object files and executables (useful for a fresh rebuild)
distclean: clean
