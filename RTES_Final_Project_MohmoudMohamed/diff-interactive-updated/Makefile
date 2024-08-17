# Makefile for compiling and linking the capture.cpp program using OpenCV

# Compiler and flags
CC = g++
CFLAGS = -O2 -g -Wall -Wextra -pedantic $(INCLUDE_DIRS) $(CDEFS)
LDFLAGS = $(LIBS) $(CPPLIBS)

# Directories for includes and libraries
INCLUDE_DIRS = -I/usr/include/opencv4
LIB_DIRS = 

# Libraries to link against
LIBS = -lrt
CPPLIBS = `pkg-config --libs opencv4`

# Source and header files
CPPFILES = capture.cpp
CPPOBJS = ${CPPFILES:.cpp=.o}

# The main target: compile and link the capture program
all: capture

# Clean up the generated files
clean:
	-rm -f *.o *.d
	-rm -f capture

# Remove object files and dependencies (useful for a fresh rebuild)
distclean: clean

# Rule to link the capture program from its object file
capture: $(CPPOBJS)
	$(CC) $(CPPOBJS) -o $@ $(LDFLAGS)

# Dependencies for the project
depend: .depend

.depend: $(CPPFILES)
	rm -f ./.depend
	$(CC) $(CFLAGS) -MM $^ > ./.depend

include .depend

# Generic rule to compile .cpp files to .o files
.cpp.o:
	$(CC) $(CFLAGS) -c $< -o $@
