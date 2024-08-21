# Makefile for compiling and linking the capture.c program

# Compiler and flags
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -pedantic $(INCLUDE_DIRS) $(CDEFS)
LDFLAGS = $(LIBS)

# Directories for includes and libraries
INCLUDE_DIRS = 
LIB_DIRS = 

# Libraries to link against
LIBS = -lrt

# Source and object files
CFILES = capture.c
OBJS = ${CFILES:.c=.o}

# Default target: build the capture executable
all: capture

# Clean up the build directory by removing object files and the capture executable
clean:
	-rm -f *.o *.d capture
	-rm -f frames/*

# Remove object files and dependencies (useful for a fresh rebuild)
distclean: clean

# Rule to link the capture executable from object files
capture: ${OBJS}
	$(CC) $(CFLAGS) -o $@ $@.o $(LDFLAGS)

# Dependencies for the project
# depend: .depend

# .depend: $(CFILES)
# 	rm -f ./.depend
#	$(CC) $(CFLAGS) -MM $^ > ./.depend

# include .depend

# Rule to compile .c files to .o files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@
