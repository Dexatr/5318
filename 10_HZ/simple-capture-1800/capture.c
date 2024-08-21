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
#include <stdarg.h> // Include for va_start and va_end

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

FILE *log_file = NULL; // Initialize to NULL to prevent unintended use

void log_message(const char *format, ...) {
    va_list args;
    va_start(args, format);

    // Log to syslog
    vsyslog(LOG_INFO, format, args);

    // Log to the file only if log_file is successfully opened
    if (log_file != NULL) {
        va_start(args, format);  // Re-initialize the argument list for the file log
        vfprintf(log_file, format, args);
        fflush(log_file);  // Ensure the log is written out
    }

    va_end(args);
}

void errno_exit(const char *s) {
    log_message("10Hz: %s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

int xioctl(int fh, unsigned long request, void *arg) {
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
            log_message("10Hz: Failed to create frames directory\n");
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
        log_message("10Hz: Failed to open ppm file %s: %s\n", ppm_dumpname, strerror(errno));
        return;
    }

    snprintf(&ppm_header[4], 11, "%010ld", time->tv_sec);
    snprintf(&ppm_header[14], 6, " sec ");
    snprintf(&ppm_header[20], 11, "%010ld", (time->tv_nsec) / 1000000);
    snprintf(&ppm_header[30], 20, " msec \n"HRES_STR" "VRES_STR"\n255\n");

    written = write(dumpfd, ppm_header, strlen(ppm_header));
    if (written == -1) {
        log_message("10Hz: Failed to write ppm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            log_message("10Hz: Failed to write ppm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    log_message("10Hz: PPM frame written to %s at %lf, %d bytes\n", ppm_dumpname, (fnow - fstart), total);

    close(dumpfd);
}

void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    snprintf(&pgm_dumpname[strlen(pgm_dumpname) - 8], 9, "%04d.pgm", tag);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 0644);

    if (dumpfd == -1) {
        log_message("10Hz: Failed to open pgm file %s: %s\n", pgm_dumpname, strerror(errno));
        return;
    }

    snprintf(&pgm_header[4], 11, "%010ld", time->tv_sec);
    snprintf(&pgm_header[14], 6, " sec ");
    snprintf(&pgm_header[20], 11, "%010ld", (time->tv_nsec) / 1000000);
    snprintf(&pgm_header[30], 20, " msec \n"HRES_STR" "VRES_STR"\n255\n");

    written = write(dumpfd, pgm_header, strlen(pgm_header));
    if (written == -1) {
        log_message("10Hz: Failed to write pgm header: %s\n", strerror(errno));
        close(dumpfd);
        return;
    }

    total = 0;
    do {
        written = write(dumpfd, p, size);
        if (written == -1) {
            log_message("10Hz: Failed to write pgm data: %s\n", strerror(errno));
            close(dumpfd);
            return;
        }
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    log_message("10Hz: PGM frame written to %s at %lf, %d bytes\n", pgm_dumpname, (fnow - fstart), total);

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
    log_message("10Hz: Processing frame %d with size %d\n", framecnt, size);
    
    if (framecnt == 0) {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

#ifdef DUMP_FRAMES
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        log_message("10Hz: Dumping GRAY frame as-is\n");
        dump_pgm(p, size, framecnt, &frame_time);
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        log_message("10Hz: Converting YUYV frame to YY and dumping\n");
        for (i = 0, newi = 0; i < size; i += 4, newi += 2) {
            bigbuffer[newi] = pptr[i];
            bigbuffer[newi + 1] = pptr[i + 2];
        }
        if (framecnt > -1) {
            dump_pgm(bigbuffer, (size / 2), framecnt, &frame_time);
        }
    } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
        log_message("10Hz: Dumping RGB frame as-is\n");
        dump_ppm(p, size, framecnt, &frame_time);
    } else {
        log_message("10Hz: ERROR - unknown dump format\n");
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
                        log_message("10Hz: mmap failure\n");
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
            process_image((void *)buf.m.userptr, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
            break;
    }

    return 1;
}

void mainloop(void) {
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;

    printf("Running at 10 frames/sec\n");
    read_delay.tv_sec = 0;
    read_delay.tv_nsec = 100000000; // 10Hz = 100ms delay

    count = frame_count;

    while (count > 0) {
        for (;;) {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r) {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r) {
                fprintf(stderr, "select timeout\n");
                log_message("10Hz: select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame()) {
                if (nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                else {
                    if (framecnt > 1) {
                        clock_gettime(CLOCK_MONOTONIC, &time_now);
                        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
                        log_message("10Hz: Frame %d read at %lf, @ %lf FPS\n", framecnt, (fnow - fstart), (double)(framecnt + 1) / (fnow - fstart));
                    } else {
                        log_message("10Hz: Initial frame read at %lf\n", fnow);
                    }
                }

                count--;
                break;
            }

            if (count <= 0) break;
        }

        if (count <= 0) break;
    }

    clock_gettime(CLOCK_MONOTONIC, &time_stop);
    fstop = (double)time_stop.tv_sec + (double)time_stop.tv_nsec / 1000000000.0;
    log_message("10Hz: Capture ended, total capture time=%lf, for %d frames, %lf FPS\n", (fstop - fstart), CAPTURE_FRAMES + 1, ((double)CAPTURE_FRAMES / (fstop - fstart)));
}

void stop_capturing(void) {
    enum v4l2_buf_type type;

    switch (io) {
        case IO_METHOD_READ:
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                errno_exit("VIDIOC_STREAMOFF");
            break;
    }
}

void start_capturing(void) {
    unsigned int i;
    enum v4l2_buf_type type;

    switch (io) {
        case IO_METHOD_READ:
            break;

        case IO_METHOD_MMAP:
            for (i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            }
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");
            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                buf.index = i;
                buf.m.userptr = (unsigned long)buffers[i].start;
                buf.length = buffers[i].length;

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            }
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");
            break;
    }
}

void uninit_device(void) {
    unsigned int i;

    switch (io) {
        case IO_METHOD_READ:
            free(buffers[0].start);
            break;

        case IO_METHOD_MMAP:
            for (i = 0; i < n_buffers; ++i)
                if (-1 == munmap(buffers[i].start, buffers[i].length))
                    errno_exit("munmap");
            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < n_buffers; ++i)
                free(buffers[i].start);
            break;
    }

    free(buffers);
}

void init_read(unsigned int buffer_size) {
    buffers = calloc(1, sizeof(*buffers));

    if (!buffers) {
        fprintf(stderr, "10Hz: Out of memory\n");
        log_message("10Hz: Out of memory\n");
        exit(EXIT_FAILURE);
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc(buffer_size);

    if (!buffers[0].start) {
        fprintf(stderr, "10Hz: Out of memory\n");
        log_message("10Hz: Out of memory\n");
        exit(EXIT_FAILURE);
    }
}

void init_mmap(void) {
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 6;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "10Hz: %s does not support memory mapping\n", dev_name);
            log_message("10Hz: %s does not support memory mapping\n");
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        fprintf(stderr, "10Hz: Insufficient buffer memory on %s\n", dev_name);
        log_message("10Hz: Insufficient buffer memory on %s\n");
        exit(EXIT_FAILURE);
    }

    buffers = calloc(req.count, sizeof(*buffers));

    if (!buffers) {
        fprintf(stderr, "10Hz: Out of memory\n");
        log_message("10Hz: Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            errno_exit("mmap");
    }
}

void init_userp(unsigned int buffer_size) {
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "10Hz: %s does not support user pointer I/O\n", dev_name);
            log_message("10Hz: %s does not support user pointer I/O\n");
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    buffers = calloc(4, sizeof(*buffers));

    if (!buffers) {
        fprintf(stderr, "10Hz: Out of memory\n");
        log_message("10Hz: Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = malloc(buffer_size);

        if (!buffers[n_buffers].start) {
            fprintf(stderr, "10Hz: Out of memory\n");
            log_message("10Hz: Out of memory\n");
            exit(EXIT_FAILURE);
        }
    }
}

void init_device(void) {
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf(stderr, "10Hz: %s is no V4L2 device\n", dev_name);
            log_message("10Hz: %s is no V4L2 device\n");
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "10Hz: %s is no video capture device\n", dev_name);
        log_message("10Hz: %s is no video capture device\n");
        exit(EXIT_FAILURE);
    }

    switch (io) {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                fprintf(stderr, "10Hz: %s does not support read I/O\n", dev_name);
                log_message("10Hz: %s does not support read I/O\n");
                exit(EXIT_FAILURE);
            }
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                fprintf(stderr, "10Hz: %s does not support streaming I/O\n", dev_name);
                log_message("10Hz: %s does not support streaming I/O\n");
                exit(EXIT_FAILURE);
            }
            break;
    }

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect;

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
                case EINVAL:
                    break;
                default:
                    break;
            }
        }
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format) {
        printf("FORCING FORMAT\n");
        log_message("10Hz: FORCING FORMAT\n");
        fmt.fmt.pix.width = HRES;
        fmt.fmt.pix.height = VRES;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            errno_exit("VIDIOC_S_FMT");
    } else {
        printf("ASSUMING FORMAT\n");
        log_message("10Hz: ASSUMING FORMAT\n");
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            errno_exit("VIDIOC_G_FMT");
    }

    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    switch (io) {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}

void close_device(void) {
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
}

void open_device(void) {
    struct stat st;

    if (-1 == stat(dev_name, &st)) {
        fprintf(stderr, "10Hz: Cannot identify '%s': %d, %s\n", dev_name, errno, strerror(errno));
        log_message("10Hz: Cannot identify '%s': %d, %s\n", dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
        fprintf(stderr, "10Hz: %s is no device\n", dev_name);
        log_message("10Hz: %s is no device\n");
        exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);

    if (-1 == fd) {
        fprintf(stderr, "10Hz: Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
        log_message("10Hz: Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

void usage(FILE *fp, int argc, char **argv) {
    fprintf(fp,
             "Usage: %s [options]\n\n"
             "Version 1.3\n"
             "Options:\n"
             "-d | --device name   Video device name [%s]\n"
             "-h | --help          Print this message\n"
             "-m | --mmap          Use memory-mapped buffers [default]\n"
             "-r | --read          Use read() calls\n"
             "-u | --userp         Use application-allocated buffers\n"
             "-o | --output        Outputs stream to stdout\n"
             "-f | --format        Force format to 640x480 GREY\n"
             "-c | --count         Number of frames to grab [%i]\n",
             argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";
static const struct option long_options[] = {
    { "device", required_argument, NULL, 'd' },
    { "help",   no_argument,       NULL, 'h' },
    { "mmap",   no_argument,       NULL, 'm' },
    { "read",   no_argument,       NULL, 'r' },
    { "userp",  no_argument,       NULL, 'u' },
    { "output", no_argument,       NULL, 'o' },
    { "format", no_argument,       NULL, 'f' },
    { "count",  required_argument, NULL, 'c' },
    { 0, 0, 0, 0 }
};

int main(int argc, char **argv) {
    char exec_path[PATH_MAX];
    char *exec_dir;
    char frames_dir[PATH_MAX];
    char log_file_path[PATH_MAX];

    openlog("10Hz_capture_app", LOG_PID | LOG_CONS, LOG_USER);
    log_message("10Hz: Starting capture application\n");

    if (readlink("/proc/self/exe", exec_path, sizeof(exec_path) - 1) != -1) {
        exec_dir = dirname(exec_path);
        snprintf(frames_dir, sizeof(frames_dir), "%s/frames", exec_dir);
        snprintf(log_file_path, sizeof(log_file_path), "%s/10Hz_capture.log", exec_dir);
        set_output_directory(frames_dir);
        log_message("10Hz: Output directory set to %s\n", frames_dir);
    } else {
        perror("Failed to determine executable path");
        log_message("10Hz: Failed to determine executable path\n");
        exit(EXIT_FAILURE);
    }

    log_file = fopen(log_file_path, "w");
    if (!log_file) {
        log_message("10Hz: Failed to open log file %s\n", log_file_path);
        perror("Failed to open log file");
        exit(EXIT_FAILURE);
    }

    if (create_directory(frames_dir) != 0) {
        log_message("10Hz: Failed to create directory %s\n", frames_dir);
        exit(EXIT_FAILURE);
    }

    if (argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video0";

    for (;;) {
        int idx;
        int c;

        c = getopt_long(argc, argv, short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c) {
            case 0:
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'r':
                io = IO_METHOD_READ;
                break;

            case 'u':
                io = IO_METHOD_USERPTR;
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                    errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    open_device();
    init_device();

    start_capturing();
    mainloop();

    stop_capturing();

    log_message("10Hz: Total capture time=%lf, for %d frames, %lf FPS\n", (fstop - fstart), CAPTURE_FRAMES + 1, ((double)CAPTURE_FRAMES / (fstop - fstart)));

    uninit_device();
    close_device();
    fprintf(stderr, "\n");

    log_message("10Hz: Capture application finished\n");

    if (log_file != NULL) {
        fclose(log_file);
    }
    closelog();

    return 0;
}
