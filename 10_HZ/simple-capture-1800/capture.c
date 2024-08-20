#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <time.h>
#include <limits.h>
#include <libgen.h>
#include <syslog.h>
#include <sys/utsname.h>

// Macros to clear memory, set resolution, and define frame capture limits
#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

// Frame capture configuration parameters
#define START_UP_FRAMES 8
#define LAST_FRAMES 1
#define CAPTURE_FRAMES (1800 + LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + START_UP_FRAMES + LAST_FRAMES)

#define FRAMES_PER_SEC 10   // Updated to 10 FPS for the assignment requirements

#define DUMP_FRAMES

// Struct to hold video format details
static struct v4l2_format fmt;

// Enum for different I/O methods (read, memory-mapped, user pointer)
enum io_method { IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR };

// Struct to represent a buffer in memory for frame data
struct buffer {
    void   *start;
    size_t  length;
};

// Global variables for device name, file descriptors, buffers, and frame counts
static char *dev_name;
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
static struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int force_format = 1;
static int frame_count = FRAMES_TO_ACQUIRE;

// Timing-related variables for frame processing
static double fnow = 0.0, fstart = 0.0, fstop = 0.0;
static struct timespec time_now, time_start, time_stop;
static int log_fd = -1; // File descriptor for the custom syslog file

// Function to handle errors and exit the program with an error message
static void errno_exit(const char *s) {
    syslog(LOG_ERR, "%s error %d, %s\n", s, errno, strerror(errno));
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

// Wrapper for the ioctl system call, which allows low-level control of the video device
static int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

// Function to log system information (uname -a) to syslog
void log_system_info(void) {
    struct utsname sysinfo;
    if (uname(&sysinfo) == 0) {
        syslog(LOG_INFO, "%s", sysinfo.nodename);
    } else {
        syslog(LOG_ERR, "Failed to retrieve system information");
    }
}

// Function to process each captured frame, including saving to file and converting formats
static void process_image(const void *p, int size) {
    struct timespec frame_time;
    clock_gettime(CLOCK_REALTIME, &frame_time);

    framecnt++;
    fnow = (double)frame_time.tv_sec + (double)frame_time.tv_nsec / 1000000000.0;
    double capture_time = fnow - fstart;

    syslog(LOG_INFO, "[Course #:4] [Final Project] [Frame Count: %d] [Image Capture Start Time: %.9lf seconds]", framecnt, capture_time);

    // Other frame processing code...
}

// Main function to parse command-line arguments, initialize and run the video capture
int main(int argc, char **argv) {
    // Open the custom log file
    log_fd = open("syslog.txt", O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (log_fd < 0) {
        perror("Failed to open syslog.txt");
        exit(EXIT_FAILURE);
    }

    // Redirect syslog output to the custom log file
    openlog("capture_app", LOG_PID | LOG_CONS, LOG_USER);
    dup2(log_fd, STDERR_FILENO);

    syslog(LOG_INFO, "Starting capture application\n");

    // Log system information as the first entry
    log_system_info();

    // Other initialization and capturing code...

    // Close syslog
    syslog(LOG_INFO, "Capture application finished\n");
    closelog();

    // Close the custom log file
    close(log_fd);

    return 0;
}
