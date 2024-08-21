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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

#define START_UP_FRAMES 8
#define LAST_FRAMES 1
#define CAPTURE_FRAMES (1800 + LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + START_UP_FRAMES + LAST_FRAMES)

#define FRAMES_PER_SEC 10

#define DUMP_FRAMES

static struct v4l2_format fmt;

enum io_method { IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR };

struct buffer {
    void   *start;
    size_t  length;
};

static char *dev_name;
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
static struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int force_format = 1;
static int frame_count = FRAMES_TO_ACQUIRE;

static double fnow = 0.0, fstart = 0.0, fstop = 0.0;
static struct timespec time_now, time_start, time_stop;

char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[PATH_MAX];
char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[PATH_MAX];

FILE *log_file;

void errno_exit(const char *s) {
    syslog(LOG_ERR, "10Hz: %s error %d, %s\n", s, errno, strerror(errno));
    fprintf(log_file, "10Hz: %s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

int create_directory(const char *path) {
    struct stat st = {0};

    if (stat(path, &st) == -1) {
        if (mkdir(path, 0700) != 0) {
            perror("Failed to create frames directory");
            fprintf(log_file, "10Hz: Failed to create frames directory\n");
            return -1;
        }
    }

    return 0;
}

void set_output_directory(const char *dir) {
    snprintf(ppm_dumpname, PATH_MAX, "%s/test0000.ppm", dir);
    snprintf(pgm_dumpname, PATH_MAX, "%s/test0000.pgm", dir);
}

void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&ppm_dumpname[strlen(ppm_dumpname) - 8], 9, "%04d.ppm", tag);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "10Hz: Failed to open ppm file %s: %s\n", ppm_dumpname, strerror(errno));
        fprintf(log_file, "10Hz: Failed to open ppm file %s: %s\n", ppm_dumpname, strerror(errno));
        return;
    }

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "10Hz: Failed to write ppm header: %s\n", strerror(errno));
        fprintf(log_file, "10Hz: Failed to write ppm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "10Hz: Failed to write ppm data: %s\n", strerror(errno));
            fprintf(log_file, "10Hz: Failed to write ppm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "10Hz: PPM frame written to %s at %lf, %d bytes\n", ppm_dumpname, (fnow - fstart), total);
    fprintf(log_file, "10Hz: PPM frame written to %s at %lf, %d bytes\n", ppm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&pgm_dumpname[strlen(pgm_dumpname) - 8], 9, "%04d.pgm", tag);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "10Hz: Failed to open pgm file %s: %s\n", pgm_dumpname, strerror(errno));
        fprintf(log_file, "10Hz: Failed to open pgm file %s: %s\n", pgm_dumpname, strerror(errno));
        return;
    }

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    written = write(dumpfd, pgm_header, sizeof(pgm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "10Hz: Failed to write pgm header: %s\n", strerror(errno));
        fprintf(log_file, "10Hz: Failed to write pgm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "10Hz: Failed to write pgm data: %s\n", strerror(errno));
            fprintf(log_file, "10Hz: Failed to write pgm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "10Hz: PGM frame written to %s at %lf, %d bytes\n", pgm_dumpname, (fnow - fstart), total);
    fprintf(log_file, "10Hz: PGM frame written to %s at %lf, %d bytes\n", pgm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

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

int framecnt = -8;
unsigned char bigbuffer[(1280 * 960)];

void process_image(const void *p, int size) {
    int i, newi;
    struct timespec frame_time;
    unsigned char *pptr = (unsigned char *)p;

    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    syslog(LOG_INFO, "10Hz: Processing frame %d with size %d\n", framecnt, size);
    fprintf(log_file, "10Hz: Processing frame %d with size %d\n", framecnt, size);
    
    if (framecnt == 0) {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

#ifdef DUMP_FRAMES
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        syslog(LOG_INFO, "10Hz: Dumping GRAY frame as-is\n");
        fprintf(log_file, "10Hz: Dumping GRAY frame as-is\n");
        dump_pgm(p, size, framecnt, &frame_time);
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        syslog(LOG_INFO, "10Hz: Converting YUYV frame to YY and dumping\n");
        fprintf(log_file, "10Hz: Converting YUYV frame to YY and dumping\n");
        for (i = 0, newi = 0; i < size; i += 4, newi += 2) {
            bigbuffer[newi] = pptr[i];
            bigbuffer[newi + 1] = pptr[i + 2];
        }
        if (framecnt > -1) {
            dump_pgm(bigbuffer, (size / 2), framecnt, &frame_time);
        }
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
        syslog(LOG_INFO, "10Hz: Dumping RGB frame as-is\n");
        fprintf(log_file, "10Hz: Dumping RGB frame as-is\n");
        dump_ppm(p, size, framecnt, &frame_time);
    } else {
        syslog(LOG_ERR, "10Hz: ERROR - unknown dump format\n");
        fprintf(log_file, "10Hz: ERROR - unknown dump format\n");
    }
#endif
}

int read_frame(void) {
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
                        syslog(LOG_ERR, "10Hz: mmap failure\n");
                        fprintf(log_file, "10Hz: mmap failure\n");
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

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                if (buf.m.userptr == (unsigned long)buffers[i].start && buf.length == buffers[i].length)
                    break;

            assert(i < n_buffers);
            process_image((void *)buf.m.userptr, buf.bytesuse#include <stdio.h>
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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

#define START_UP_FRAMES 8
#define LAST_FRAMES 1
#define CAPTURE_FRAMES (1800 + LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + START_UP_FRAMES + LAST_FRAMES)

#define FRAMES_PER_SEC 10

#define DUMP_FRAMES

static struct v4l2_format fmt;

enum io_method { IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR };

struct buffer {
    void   *start;
    size_t  length;
};

static char *dev_name;
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
static struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int force_format = 1;
static int frame_count = FRAMES_TO_ACQUIRE;

static double fnow = 0.0, fstart = 0.0, fstop = 0.0;
static struct timespec time_now, time_start, time_stop;

char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[PATH_MAX];
char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[PATH_MAX];

FILE *log_file;

void errno_exit(const char *s) {
    syslog(LOG_ERR, "10Hz: %s error %d, %s\n", s, errno, strerror(errno));
    fprintf(log_file, "10Hz: %s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

int create_directory(const char *path) {
    struct stat st = {0};

    if (stat(path, &st) == -1) {
        if (mkdir(path, 0700) != 0) {
            perror("Failed to create frames directory");
            fprintf(log_file, "10Hz: Failed to create frames directory\n");
            return -1;
        }
    }

    return 0;
}

void set_output_directory(const char *dir) {
    snprintf(ppm_dumpname, PATH_MAX, "%s/test0000.ppm", dir);
    snprintf(pgm_dumpname, PATH_MAX, "%s/test0000.pgm", dir);
}

void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&ppm_dumpname[strlen(ppm_dumpname) - 8], 9, "%04d.ppm", tag);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "10Hz: Failed to open ppm file %s: %s\n", ppm_dumpname, strerror(errno));
        fprintf(log_file, "10Hz: Failed to open ppm file %s: %s\n", ppm_dumpname, strerror(errno));
        return;
    }

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "10Hz: Failed to write ppm header: %s\n", strerror(errno));
        fprintf(log_file, "10Hz: Failed to write ppm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "10Hz: Failed to write ppm data: %s\n", strerror(errno));
            fprintf(log_file, "10Hz: Failed to write ppm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "10Hz: PPM frame written to %s at %lf, %d bytes\n", ppm_dumpname, (fnow - fstart), total);
    fprintf(log_file, "10Hz: PPM frame written to %s at %lf, %d bytes\n", ppm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&pgm_dumpname[strlen(pgm_dumpname) - 8], 9, "%04d.pgm", tag);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "10Hz: Failed to open pgm file %s: %s\n", pgm_dumpname, strerror(errno));
        fprintf(log_file, "10Hz: Failed to open pgm file %s: %s\n", pgm_dumpname, strerror(errno));
        return;
    }

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    written = write(dumpfd, pgm_header, sizeof(pgm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "10Hz: Failed to write pgm header: %s\n", strerror(errno));
        fprintf(log_file, "10Hz: Failed to write pgm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "10Hz: Failed to write pgm data: %s\n", strerror(errno));
            fprintf(log_file, "10Hz: Failed to write pgm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "10Hz: PGM frame written to %s at %lf, %d bytes\n", pgm_dumpname, (fnow - fstart), total);
    fprintf(log_file, "10Hz: PGM frame written to %s at %lf, %d bytes\n", pgm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

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

int framecnt = -8;
unsigned char bigbuffer[(1280 * 960)];

void process_image(const void *p, int size) {
    int i, newi;
    struct timespec frame_time;
    unsigned char *pptr = (unsigned char *)p;

    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    syslog(LOG_INFO, "10Hz: Processing frame %d with size %d\n", framecnt, size);
    fprintf(log_file, "10Hz: Processing frame %d with size %d\n", framecnt, size);
    
    if (framecnt == 0) {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

#ifdef DUMP_FRAMES
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        syslog(LOG_INFO, "10Hz: Dumping GRAY frame as-is\n");
        fprintf(log_file, "10Hz: Dumping GRAY frame as-is\n");
        dump_pgm(p, size, framecnt, &frame_time);
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        syslog(LOG_INFO, "10Hz: Converting YUYV frame to YY and dumping\n");
        fprintf(log_file, "10Hz: Converting YUYV frame to YY and dumping\n");
        for (i = 0, newi = 0; i < size; i += 4, newi += 2) {
            bigbuffer[newi] = pptr[i];
            bigbuffer[newi + 1] = pptr[i + 2];
        }
        if (framecnt > -1) {
            dump_pgm(bigbuffer, (size / 2), framecnt, &frame_time);
        }
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
        syslog(LOG_INFO, "10Hz: Dumping RGB frame as-is\n");
        fprintf(log_file, "10Hz: Dumping RGB frame as-is\n");
        dump_ppm(p, size, framecnt, &frame_time);
    } else {
        syslog(LOG_ERR, "10Hz: ERROR - unknown dump format\n");
        fprintf(log_file, "10Hz: ERROR - unknown dump format\n");
    }
#endif
}

int read_frame(void) {
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
                        syslog(LOG_ERR, "10Hz: mmap failure\n");
                        fprintf(log_file, "10Hz: mmap failure\n");
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

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;
                    case EIO:
                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                if (buf.m.userptr == (unsigned long)buffers[i].start && buf.length == buffers[i].length)
                    break;

            assert(i < n_buffers);
            process_image((void *)buf.m.userptr, buf.bytesuse#include <stdio.h>
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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

#define START_UP_FRAMES 8
#define LAST_FRAMES 1
#define CAPTURE_FRAMES (1800 + LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + START_UP_FRAMES + LAST_FRAMES)

#define FRAMES_PER_SEC 10

#define DUMP_FRAMES

static struct v4l2_format fmt;

enum io_method { IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR };

struct buffer {
    void   *start;
    size_t  length;
};

static char *dev_name;
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
static struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int force_format = 1;
static int frame_count = FRAMES_TO_ACQUIRE;

static double fnow = 0.0, fstart = 0.0, fstop = 0.0;
static struct timespec time_now, time_start, time_stop;

char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[PATH_MAX];
char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[PATH_MAX];

FILE *log_file;

void errno_exit(const char *s) {
    syslog(LOG_ERR, "10Hz: %s error %d, %s\n", s, errno, strerror(errno));
    fprintf(log_file, "10Hz: %s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

int create_directory(const char *path) {
    struct stat st = {0};

    if (stat(path, &st) == -1) {
        if (mkdir(path, 0700) != 0) {
            perror("Failed to create frames directory");
            fprintf(log_file, "10Hz: Failed to create frames directory\n");
            return -1;
        }
    }

    return 0;
}

void set_output_directory(const char *dir) {
    snprintf(ppm_dumpname, PATH_MAX, "%s/test0000.ppm", dir);
    snprintf(pgm_dumpname, PATH_MAX, "%s/test0000.pgm", dir);
}

void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&ppm_dumpname[strlen(ppm_dumpname) - 8], 9, "%04d.ppm", tag);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "10Hz: Failed to open ppm file %s: %s\n", ppm_dumpname, strerror(errno));
        fprintf(log_file, "10Hz: Failed to open ppm file %s: %s\n", ppm_dumpname, strerror(errno));
        return;
    }

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "10Hz: Failed to write ppm header: %s\n", strerror(errno));
        fprintf(log_file, "10Hz: Failed to write ppm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "10Hz: Failed to write ppm data: %s\n", strerror(errno));
            fprintf(log_file, "10Hz: Failed to write ppm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "10Hz: PPM frame written to %s at %lf, %d bytes\n", ppm_dumpname, (fnow - fstart), total);
    fprintf(log_file, "10Hz: PPM frame written to %s at %lf, %d bytes\n", ppm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&pgm_dumpname[strlen(pgm_dumpname) - 8], 9, "%04d.pgm", tag);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        syslog(LOG_ERR, "10Hz: Failed to open pgm file %s: %s\n", pgm_dumpname, strerror(errno));
        fprintf(log_file, "10Hz: Failed to open pgm file %s: %s\n", pgm_dumpname, strerror(errno));
        return;
    }

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    written = write(dumpfd, pgm_header, sizeof(pgm_header) - 1);
    if (written == -1) {
        syslog(LOG_ERR, "10Hz: Failed to write pgm header: %s\n", strerror(errno));
        fprintf(log_file, "10Hz: Failed to write pgm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            syslog(LOG_ERR, "10Hz: Failed to write pgm data: %s\n", strerror(errno));
            fprintf(log_file, "10Hz: Failed to write pgm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    syslog(LOG_INFO, "10Hz: PGM frame written to %s at %lf, %d bytes\n", pgm_dumpname, (fnow - fstart), total);
    fprintf(log_file, "10Hz: PGM frame written to %s at %lf, %d bytes\n", pgm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

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

int framecnt = -8;
unsigned char bigbuffer[(1280 * 960)];

void process_image(const void *p, int size) {
    int i, newi;
    struct timespec frame_time;
    unsigned char *pptr = (unsigned char *)p;

    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    syslog(LOG_INFO, "10Hz: Processing frame %d with size %d\n", framecnt, size);
    fprintf(log_file, "10Hz: Processing frame %d with size %d\n", framecnt, size);
    
    if (framecnt == 0) {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

#ifdef DUMP_FRAMES
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        syslog(LOG_INFO, "10Hz: Dumping GRAY frame as-is\n");
        fprintf(log_file, "10Hz: Dumping GRAY frame as-is\n");
        dump_pgm(p, size, framecnt, &frame_time);
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        syslog(LOG_INFO, "10Hz: Converting YUYV frame to YY and dumping\n");
        fprintf(log_file, "10Hz: Converting YUYV frame