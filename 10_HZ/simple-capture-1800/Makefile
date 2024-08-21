# Makefile for compiling and linking the capture.c program

# Compiler and flags
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -pedantic
LDFLAGS = -lrt

# Source and object files
CFILES = capture.c
OBJS = ${CFILES:.c=.o}

# Default target: build the capture executable
all: capture

# Rule to link the capture executable from object files
capture: ${OBJS}
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

# Rule to compile .c files to .o files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up the build directory by removing object files and the capture executable
clean:
	-rm -f *.o capture
	-rm -f frames/*
	-rm -f 10Hz_capture.log

# Remove object files and capture executable (useful for a fresh rebuild)
distclean: clean
