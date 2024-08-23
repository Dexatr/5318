// make clean && make && sudo truncate -s 0 /var/log/syslog && ./10HzAdditional && uname -a > 10hz_syslog.txt && sudo grep -F "[10Hz]" /var/log/syslog >> 10hz_syslog.txt

// Linux raspberrypi 6.6.31+rpt-rpi-v8 #1 SMP PREEMPT Debian 1:6.6.31-1+rpt1 (2024-05-29) aarch64 GNU/Linux

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

#define FRAMES_PER_SEC 10   // Set to 10 FPS for the assignment requirements

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

// Frame counter and buffer for holding the frame data
int framecnt = -8;
unsigned char bigbuffer[(1280 * 960)];

// Function to handle errors and exit the program with an error message
static void errno_exit(const char *s) {
    syslog(LOG_ERR, "%s error %d, %s [10Hz]\n", s, errno, strerror(errno));
    fprintf(stderr, "%s error %d, %s [10Hz]\n", s, errno, strerror(errno));
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

// Headers and file name templates for saving frames as PPM or PGM files
char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[PATH_MAX];
char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[PATH_MAX];

// Function to create a directory for saving frame images
int create_directory(const char *path) {
    struct stat st = {0};

    if (stat(path, &st) == -1) {
        if (mkdir(path, 0700) != 0) {
            perror("Failed to create frames directory");
            return -1;
        }
    }

    return 0;
}

// Function to set the output directory for saving frame images
void set_output_directory(const char *dir) {
    snprintf(ppm_dumpname, PATH_MAX, "%s/test0000.ppm", dir);
    snprintf(pgm_dumpname, PATH_MAX, "%s/test0000.pgm", dir);
}

// Function to save a frame in PPM format (used for RGB images)
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&ppm_dumpname[strlen(ppm_dumpname) - 8], 9, "%04d.ppm", tag);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "Failed to open ppm file %s: %s [10Hz]\n", ppm_dumpname, strerror(errno));
        perror("Failed to open ppm file [10Hz]");
        return;
    }

    // Add the timestamp to the PPM header
    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));

    // Write the PPM header to the file
    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "Failed to write ppm header: %s [10Hz]\n", strerror(errno));
        perror("Failed to write ppm header [10Hz]");
        close(dumpfd);
        return;
    }

    // Write the frame data to the file
    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "Failed to write ppm data: %s [10Hz]\n", strerror(errno));
            perror("Failed to write ppm data [10Hz]");
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    // Log the time at which the frame was written
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "[Course #:4] [Final Project] [Frame Count: %d] [Image Capture Start Time: %lf seconds] PPM frame written to %s at %lf, %d bytes [10Hzgrep]\n", framecnt, fnow - fstart, ppm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

// Function to save a frame in PGM format (used for grayscale images)
static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&pgm_dumpname[strlen(pgm_dumpname) - 8], 9, "%04d.pgm", tag);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "Failed to open pgm file %s: %s [10Hz]\n", pgm_dumpname, strerror(errno));
        perror("Failed to open pgm file [10Hz]");
        return;
    }

    // Add the timestamp to the PGM header
    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));

    // Write the PGM header to the file
    written = write(dumpfd, pgm_header, sizeof(pgm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "Failed to write pgm header: %s [10Hz]\n", strerror(errno));
        perror("Failed to write pgm header [10Hz]");
        close(dumpfd);
        return;
    }

    // Write the frame data to the file
    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "Failed to write pgm data: %s [10Hz]\n", strerror(errno));
            perror("Failed to write pgm data [10Hz]");
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    // Log the time at which the frame was written
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "[Course #:4] [Final Project] [Frame Count: %d] [Image Capture Start Time: %lf seconds] PGM frame written to %s at %lf, %d bytes [10Hz]\n", framecnt, fnow - fstart, pgm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

// Function to convert YUV format to RGB (used for processing frames from the camera)
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b) {
   int r1, g1, b1;
   int c = y - 16, d = u - 128, e = v - 128;

   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   *r = (r1 > 255) ? 255 : (r1 < 0) ? 0 : r1;
   *g = (g1 > 255) ? 255 : (g1 < 0) ? 0 : g1;
   *b = (b1 > 255) ? 255 : (b1 < 0) ? 0 : b1;
}

// Function to process each captured frame, including saving to file and converting formats
static void process_image(const void *p, int size) {
    int i, newi;
    struct timespec frame_time;
    unsigned char *pptr = (unsigned char *)p;

    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    syslog(LOG_INFO, "Processing frame %d with size %d [10Hz]\n", framecnt, size);

    if (framecnt == 0) {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

#ifdef DUMP_FRAMES
    // Save frames based on their format (GRAY, YUYV, RGB)
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        dump_pgm(p, size, framecnt, &frame_time);
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        for (i = 0, newi = 0; i < size; i += 4, newi += 2) {
            bigbuffer[newi] = pptr[i];
            bigbuffer[newi + 1] = pptr[i + 2];
        }
        if (framecnt > -1) {
            dump_pgm(bigbuffer, (size / 2), framecnt, &frame_time);
        }
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
        dump_ppm(p, size, framecnt, &frame_time);
    } else {
        syslog(LOG_ERR, "ERROR - unknown dump format [10Hz]\n");
    }
#endif
}

// Function to read a single frame of video data and process it
static int read_frame(void) {
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io) {
        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                    default:
                        errno_exit("read");
                }
            }
            process_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                    default:
                        syslog(LOG_ERR, "mmap failure [10Hz]\n");
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < n_buffers);
            process_image(buffers[buf.index].start, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_#include <stdio.h>
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

#define FRAMES_PER_SEC 10   // Set to 10 FPS for the assignment requirements

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

// Frame counter and buffer for holding the frame data
int framecnt = -8;
unsigned char bigbuffer[(1280 * 960)];

// Function to handle errors and exit the program with an error message
static void errno_exit(const char *s) {
    syslog(LOG_ERR, "%s error %d, %s [10Hz]\n", s, errno, strerror(errno));
    fprintf(stderr, "%s error %d, %s [10Hz]\n", s, errno, strerror(errno));
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

// Headers and file name templates for saving frames as PPM or PGM files
char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[PATH_MAX];
char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[PATH_MAX];

// Function to create a directory for saving frame images
int create_directory(const char *path) {
    struct stat st = {0};

    if (stat(path, &st) == -1) {
        if (mkdir(path, 0700) != 0) {
            perror("Failed to create frames directory");
            return -1;
        }
    }

    return 0;
}

// Function to set the output directory for saving frame images
void set_output_directory(const char *dir) {
    snprintf(ppm_dumpname, PATH_MAX, "%s/test0000.ppm", dir);
    snprintf(pgm_dumpname, PATH_MAX, "%s/test0000.pgm", dir);
}

// Function to save a frame in PPM format (used for RGB images)
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&ppm_dumpname[strlen(ppm_dumpname) - 8], 9, "%04d.ppm", tag);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "Failed to open ppm file %s: %s [10Hz]\n", ppm_dumpname, strerror(errno));
        perror("Failed to open ppm file [10Hz]");
        return;
    }

    // Add the timestamp to the PPM header
    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));

    // Write the PPM header to the file
    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "Failed to write ppm header: %s [10Hz]\n", strerror(errno));
        perror("Failed to write ppm header [10Hz]");
        close(dumpfd);
        return;
    }

    // Write the frame data to the file
    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "Failed to write ppm data: %s [10Hz]\n", strerror(errno));
            perror("Failed to write ppm data [10Hz]");
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    // Log the time at which the frame was written
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "[Course #:4] [Final Project] [Frame Count: %d] [Image Capture Start Time: %lf seconds] PPM frame written to %s at %lf, %d bytes [10Hzgrep]\n", framecnt, fnow - fstart, ppm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

// Function to save a frame in PGM format (used for grayscale images)
static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&pgm_dumpname[strlen(pgm_dumpname) - 8], 9, "%04d.pgm", tag);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "Failed to open pgm file %s: %s [10Hz]\n", pgm_dumpname, strerror(errno));
        perror("Failed to open pgm file [10Hz]");
        return;
    }

    // Add the timestamp to the PGM header
    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));

    // Write the PGM header to the file
    written = write(dumpfd, pgm_header, sizeof(pgm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "Failed to write pgm header: %s [10Hz]\n", strerror(errno));
        perror("Failed to write pgm header [10Hz]");
        close(dumpfd);
        return;
    }

    // Write the frame data to the file
    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "Failed to write pgm data: %s [10Hz]\n", strerror(errno));
            perror("Failed to write pgm data [10Hz]");
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    // Log the time at which the frame was written
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "[Course #:4] [Final Project] [Frame Count: %d] [Image Capture Start Time: %lf seconds] PGM frame written to %s at %lf, %d bytes [10Hz]\n", framecnt, fnow - fstart, pgm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

// Function to convert YUV format to RGB (used for processing frames from the camera)
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b) {
   int r1, g1, b1;
   int c = y - 16, d = u - 128, e = v - 128;

   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   *r = (r1 > 255) ? 255 : (r1 < 0) ? 0 : r1;
   *g = (g1 > 255) ? 255 : (g1 < 0) ? 0 : g1;
   *b = (b1 > 255) ? 255 : (b1 < 0) ? 0 : b1;
}

// Function to process each captured frame, including saving to file and converting formats
static void process_image(const void *p, int size) {
    int i, newi;
    struct timespec frame_time;
    unsigned char *pptr = (unsigned char *)p;

    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    syslog(LOG_INFO, "Processing frame %d with size %d [10Hz]\n", framecnt, size);

    if (framecnt == 0) {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

#ifdef DUMP_FRAMES
    // Save frames based on their format (GRAY, YUYV, RGB)
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        dump_pgm(p, size, framecnt, &frame_time);
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        for (i = 0, newi = 0; i < size; i += 4, newi += 2) {
            bigbuffer[newi] = pptr[i];
            bigbuffer[newi + 1] = pptr[i + 2];
        }
        if (framecnt > -1) {
            dump_pgm(bigbuffer, (size / 2), framecnt, &frame_time);
        }
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
        dump_ppm(p, size, framecnt, &frame_time);
    } else {
        syslog(LOG_ERR, "ERROR - unknown dump format [10Hz]\n");
    }
#endif
}

// Function to read a single frame of video data and process it
static int read_frame(void) {
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io) {
        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                    default:
                        errno_exit("read");
                }
            }
            process_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                    default:
                        syslog(LOG_ERR, "mmap failure [10Hz]\n");
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < n_buffers);
            process_image(buffers[buf.index].start, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_#include <stdio.h>
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

#define FRAMES_PER_SEC 10   // Set to 10 FPS for the assignment requirements

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

// Frame counter and buffer for holding the frame data
int framecnt = -8;
unsigned char bigbuffer[(1280 * 960)];

// Function to handle errors and exit the program with an error message
static void errno_exit(const char *s) {
    syslog(LOG_ERR, "%s error %d, %s [10Hz]\n", s, errno, strerror(errno));
    fprintf(stderr, "%s error %d, %s [10Hz]\n", s, errno, strerror(errno));
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

// Headers and file name templates for saving frames as PPM or PGM files
char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[PATH_MAX];
char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[PATH_MAX];

// Function to create a directory for saving frame images
int create_directory(const char *path) {
    struct stat st = {0};

    if (stat(path, &st) == -1) {
        if (mkdir(path, 0700) != 0) {
            perror("Failed to create frames directory");
            return -1;
        }
    }

    return 0;
}

// Function to set the output directory for saving frame images
void set_output_directory(const char *dir) {
    snprintf(ppm_dumpname, PATH_MAX, "%s/test0000.ppm", dir);
    snprintf(pgm_dumpname, PATH_MAX, "%s/test0000.pgm", dir);
}

// Function to save a frame in PPM format (used for RGB images)
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&ppm_dumpname[strlen(ppm_dumpname) - 8], 9, "%04d.ppm", tag);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "Failed to open ppm file %s: %s [10Hz]\n", ppm_dumpname, strerror(errno));
        perror("Failed to open ppm file [10Hz]");
        return;
    }

    // Add the timestamp to the PPM header
    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));

    // Write the PPM header to the file
    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "Failed to write ppm header: %s [10Hz]\n", strerror(errno));
        perror("Failed to write ppm header [10Hz]");
        close(dumpfd);
        return;
    }

    // Write the frame data to the file
    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "Failed to write ppm data: %s [10Hz]\n", strerror(errno));
            perror("Failed to write ppm data [10Hz]");
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    // Log the time at which the frame was written
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "[Course #:4] [Final Project] [Frame Count: %d] [Image Capture Start Time: %lf seconds] PPM frame written to %s at %lf, %d bytes [10Hzgrep]\n", framecnt, fnow - fstart, ppm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

// Function to save a frame in PGM format (used for grayscale images)
static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&pgm_dumpname[strlen(pgm_dumpname) - 8], 9, "%04d.pgm", tag);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "Failed to open pgm file %s: %s [10Hz]\n", pgm_dumpname, strerror(errno));
        perror("Failed to open pgm file [10Hz]");
        return;
    }

    // Add the timestamp to the PGM header
    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));

    // Write the PGM header to the file
    written = write(dumpfd, pgm_header, sizeof(pgm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "Failed to write pgm header: %s [10Hz]\n", strerror(errno));
        perror("Failed to write pgm header [10Hz]");
        close(dumpfd);
        return;
    }

    // Write the frame data to the file
    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "Failed to write pgm data: %s [10Hz]\n", strerror(errno));
            perror("Failed to write pgm data [10Hz]");
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    // Log the time at which the frame was written
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "[Course #:4] [Final Project] [Frame Count: %d] [Image Capture Start Time: %lf seconds] PGM frame written to %s at %lf, %d bytes [10Hz]\n", framecnt, fnow - fstart, pgm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

// Function to convert YUV format to RGB (used for processing frames from the camera)
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b) {
   int r1, g1, b1;
   int c = y - 16, d = u - 128, e = v - 128;

   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   *r = (r1 > 255) ? 255 : (r1 < 0) ? 0 : r1;
   *g = (g1 > 255) ? 255 : (g1 < 0) ? 0 : g1;
   *b = (b1 > 255) ? 255 : (b1 < 0) ? 0 : b1;
}

// Function to process each captured frame, including saving to file and converting formats
static void pr