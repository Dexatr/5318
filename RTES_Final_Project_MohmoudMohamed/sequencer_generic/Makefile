# Makefile for compiling and linking the sequencing programs

# Compiler and flags
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -pedantic $(INCLUDE_DIRS) $(CDEFS)
LDFLAGS = $(LIBS)

# Directories for includes and libraries
INCLUDE_DIRS = 
LIB_DIRS = 

# Libraries to link against
LIBS = -lpthread -lrt

# Source and object files
CFILES = seqgenex0.c seqgen.c seqgen2.c seqgen3.c seqv4l2.c capturelib.c
OBJS = ${CFILES:.c=.o}

# Default target: build all programs
all: seqgenex0 seqgen seqgen2 seqgen3 seqv4l2 clock_times capture

# Clean up the build directory by removing object files and executables
clean:
	-rm -f *.o *.d frames/*.pgm frames/*.ppm
	-rm -f seqgenex0 seqgen seqgen2 seqgen3 seqv4l2 clock_times capture

# Remove object files and dependencies (useful for a fresh rebuild)
distclean: clean

# Rules to link the programs
seqgenex0: seqgenex0.o
	$(CC) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

seqv4l2: seqv4l2.o capturelib.o
	$(CC) $(CFLAGS) -o $@ $@.o capturelib.o $(LDFLAGS)

seqgen3: seqgen3.o
	$(CC) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

seqgen2: seqgen2.o
	$(CC) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

seqgen: seqgen.o
	$(CC) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

clock_times: clock_times.o
	$(CC) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

capture: capture.o capturelib.o
	$(CC) $(CFLAGS) -o $@ $@.o capturelib.o $(LDFLAGS)

# Dependencies for the project
depend: .depend

.depend: $(CFILES)
	rm -f ./.depend
	$(CC) $(CFLAGS) -MM $^ > ./.depend

include .depend

# Generic rule to compile .c files to .o files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@
