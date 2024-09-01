# Makefile for compiling and linking the 10Hz.c, 1Hz.c, 10HzAdditional.c, and 1HzAlternative.c programs

# Compiler and flags
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -pedantic
LDFLAGS = -lrt -lm  # Added -lm to link the math library

# Source files
CFILES_10HZ = 10Hz.c
CFILES_1HZ = 1Hz.c
CFILES_10HZ_ADDITIONAL = 10HzAdditional.c
CFILES_1HZ_ALTERNATIVE = 1HzAlternative.c  # New source file

# Object files
OBJS_10HZ = ${CFILES_10HZ:.c=.o}
OBJS_1HZ = ${CFILES_1HZ:.c=.o}
OBJS_10HZ_ADDITIONAL = ${CFILES_10HZ_ADDITIONAL:.c=.o}
OBJS_1HZ_ALTERNATIVE = ${CFILES_1HZ_ALTERNATIVE:.c=.o}  # New object file

# Default target: build all the executables
all: 10Hz 1Hz 10HzAdditional 1HzAlternative  # Added new target

# Rule to link the 10Hz executable
10Hz: $(OBJS_10HZ)
	$(CC) $(CFLAGS) -o $@ $(OBJS_10HZ) $(LDFLAGS)

# Rule to link the 1Hz executable
1Hz: $(OBJS_1HZ)
	$(CC) $(CFLAGS) -o $@ $(OBJS_1HZ) $(LDFLAGS)

# Rule to link the 10HzAdditional executable
10HzAdditional: $(OBJS_10HZ_ADDITIONAL)
	$(CC) $(CFLAGS) -o $@ $(OBJS_10HZ_ADDITIONAL) $(LDFLAGS)

# Rule to link the 1HzAlternative executable
1HzAlternative: $(OBJS_1HZ_ALTERNATIVE)  # New rule for 1HzAlternative
	$(CC) $(CFLAGS) -o $@ $(OBJS_1HZ_ALTERNATIVE) $(LDFLAGS)

# Rule to compile .c files to .o files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up the build directory by removing object files and the executables
clean: clean_10Hz clean_1Hz clean_10HzAdditional clean_1HzAlternative  # Added new clean target

# Individual clean rules
clean_10Hz:
	-rm -f $(OBJS_10HZ) 10Hz
	-rm -f frames10hz/*
	-rmdir frames10hz
	-rm -f 10Hz_capture.log
	-rm -f 10hz_syslog.txt

clean_1Hz:
	-rm -f $(OBJS_1HZ) 1Hz
	-rm -f frames1hz/*
	-rmdir frames1hz
	-rm -f 1Hz_capture.log
	-rm -f 1hz_syslog.txt

clean_10HzAdditional:
	-rm -f $(OBJS_10HZ_ADDITIONAL) 10HzAdditional
	-rm -f frames10hzAdditional/*
	-rmdir frames10hzAdditional
	-rm -f 10HzAdditional_capture.log
	-rm -f 10hz_additional_syslog.txt

clean_1HzAlternative:  # New clean rule for 1HzAlternative
	-rm -f $(OBJS_1HZ_ALTERNATIVE) 1HzAlternative
	-rm -f frames1hzAlternative/*
	-rmdir frames1hzAlternative
	-rm -f 1HzAlternative_capture.log
	-rm -f 1hz_alternative_syslog.txt

# Remove object files, executables, and additional generated files (useful for a fresh rebuild)
distclean: clean
	-rm -f output.webm

