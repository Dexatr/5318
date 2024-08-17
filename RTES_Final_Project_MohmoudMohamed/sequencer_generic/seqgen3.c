// Sam Siewert, December 2020
//
// Sequencer Generic Demonstration
//
// The purpose of this code is to provide an example for how to best
// sequence a set of periodic services in Linux user space without specialized hardware like
// an auxiliary programmable interval timer and/or real-time clock. This is useful for problems similar to and including
// the final project in real-time systems.
//
// AMP Configuration (check core status with "lscpu"):
//
// 1) Uses SCHED_FIFO - https://man7.org/linux/man-pages/man7/sched.7.html
// 2) Sequencer runs on core 1
// 3) EVEN thread indexes run on core 2
// 4) ODD thread indexes run on core 3
// 5) Linux kernel mostly runs on core 0, but does load balance non-RT workload over all cores
// 6) Check for irqbalance [https://linux.die.net/man/1/irqbalance] which also distributes IRQ handlers
//
// What we really want in addition to SCHED_FIFO with CPU core affinity is:
//
// 1) A reliable periodic source of interrupts (emulated by delay in a loop here)
// 2) An accurate (minimal drift) and precise timestamp
//    * e.g., accurate to 1 millisecond or less, ideally 1 microsecond, but not realistic on an RTOS even
//    * Overall, what we want is predictable response with some accuracy (minimal drift) and precision
//
// Linux user space presents a challenge because:
//
// 1) Accurate timestamps are either not available, or the ASM instructions to read system clocks can't
//    be issued in user space for security reasons (x86 and x64 TSC, ARM STC).
// 2) User space time with clock_gettime is recommended, but still requires the overhead of a system call.
// 3) Linux user space is inherently driven by the jiffy and tick as shown by:
//    * "getconf CLK_TCK" - normally 10 msec tick at 100 Hz
//    * cat /proc/timer_list
// 4) Linux kernel space certainly has more accurate timers that are high resolution, but we would have to
//    write our entire solution as a kernel module and/or use custom kernel modules for timekeeping and
//    selected services.
// 5) Linux kernel patches for best real-time performance include RT PREEMPT (http://www.frank-durr.de/?p=203)
// 6) MUTEX semaphores can cause unbounded priority inversion with SCHED_FIFO, so they should be avoided or
//    * Use kernel patches for RT semaphore support
//      [https://opensourceforu.com/2019/04/how-to-avoid-priority-inversion-and-enable-priority-inheritance-in-linux-kernel-programming/]
//    * Use the FUTEX instead of standard POSIX semaphores
//      [https://eli.thegreenplace.net/2018/basics-of-futexes/]
//    * POSIX semaphores do have inversion-safe features, but they do not work on un-patched Linux distros.
//
// However, for our class goals for soft real-time synchronization with a 1 Hz and a 10 Hz external
// clock (and physical process), the user space approach should provide sufficient accuracy required and
// precision which is limited by our camera frame rate to 30 Hz anyway (33.33 msec).
//
// Sequencer - 100 Hz 
//                   [gives semaphores to all other services]
// Service_1 - 50 Hz, every other Sequencer loop
// Service_2 - 20 Hz, every 5th Sequencer loop 
// Service_3 - 10 Hz ,every 10th Sequencer loop
// Service_4 -  5 Hz, every 20th Sequencer loop
// Service_5 -  2 Hz ,every 50th Sequencer loop
// Service_6 -  1 Hz, every 100th Sequencer loop
// Service_7 -  1 Hz, every 100th Sequencer loop
//
// With the above, priorities by RM policy would be:
//
// Sequencer = RT_MAX    @ 100 Hz
// Service_1 = RT_MAX-1  @ 50  Hz
// Service_2 = RT_MAX-2  @ 20  Hz
// Service_3 = RT_MAX-3  @ 10  Hz
// Service_4 = RT_MAX-4  @ 5   Hz
// Service_5 = RT_MAX-5  @ 2   Hz
// Service_6 = RT_MAX-6  @ 1   Hz
// Service_7 = RT_MIN    @ 1   Hz
//
/////////////////////////////////////////////////////////////////////////////
// JETSON SYSTEM NOTES:
/////////////////////////////////////////////////////////////////////////////
//
// Here are a few hardware/platform configuration settings on your Jetson
// that you should also check before running this code:
//
// 1) Check to ensure all your CPU cores on in an online state - USE "lscpu"
//
// 2) Check /sys/devices/system/cpu or do lscpu.
//
//    Tegra is normally configured to hot-plug CPU cores, so to make all
//    available, as root do:
//
//    echo 0 > /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable
//    echo 1 > /sys/devices/system/cpu/cpu1/online
//    echo 1 > /sys/devices/system/cpu/cpu2/online
//    echo 1 > /sys/devices/system/cpu/cpu3/online
//
// 3) The Jetson NANO requires a sysctl setting to allow for SCHED_FIFO to be used:
//
//    sysctl -w kernel.sched_rt_runtime_us=-1
//
//    See - https://forums.developer.nvidia.com/t/pthread-setschedparam-sched-fifo-fails/64394/3
//
// 4) Check for precision time resolution and support with cat /proc/timer_list
//
// 5) Ideally, all printf calls should be eliminated as they can interfere with
//    timing. They should be replaced with an in-memory event logger or at
//    least calls to syslog.
//
// 6) For determinism, you should use CPU affinity for AMP scheduling. Note that without specific affinity,
//    threads will be SMP by default, and will be migrated to the least busy core, so be careful.

// This is necessary for CPU affinity macros in Linux
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <errno.h>

#include <signal.h>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (4)         // Total number of CPU cores
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (7)           // Number of threads to create

// Of the available user space clocks, CLOCK_MONOTONIC_RAW is typically the most precise and not subject to 
// updates from external timer adjustments

//#define MY_CLOCK_TYPE CLOCK_REALTIME
//#define MY_CLOCK_TYPE CLOCK_MONOTONIC
#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW
//#define MY_CLOCK_TYPE CLOCK_REALTIME_COARSE
//#define MY_CLOCK_TYPE CLOCK_MONOTONIC_COARSE

// Flags to abort the test and each service
int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE, abortS5=FALSE, abortS6=FALSE, abortS7=FALSE;

// Semaphores for each service
sem_t semS1, semS2, semS3, semS4, semS5, semS6, semS7;

// Structure to hold the start time
struct timespec start_time_val;
double start_realtime;
unsigned long long sequencePeriods;  // Number of sequencing periods

static timer_t timer_1;    // Timer for sequencing
static struct itimerspec itime = {{1,0}, {1,0}};  // Timer intervals
static struct itimerspec last_itime;  // Last timer interval

static unsigned long long seqCnt=0;  // Sequence counter

// Structure to hold thread parameters
typedef struct
{
    int threadIdx;  // Thread index
} threadParams_t;

// Function prototypes
void Sequencer(int id);

void *Service_1(void *threadp);
void *Service_2(void *threadp);
void *Service_3(void *threadp);
void *Service_4(void *threadp);
void *Service_5(void *threadp);
void *Service_6(void *threadp);
void *Service_7(void *threadp);

double getTimeMsec(void);
double realtime(struct timespec *tsptr);
void print_scheduler(void);

// Main function
void main(void)
{
    struct timespec current_time_val, current_time_res;
    double current_realtime, current_realtime_res;

    int i, rc, scope, flags=0;

    cpu_set_t threadcpu;
    cpu_set_t allcpuset;

    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio, cpuidx;

    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;

    pthread_attr_t main_attr;
    pid_t mainpid;

    // Start the sequencer demo
    printf("Starting High Rate Sequencer Demo\n");
    clock_gettime(MY_CLOCK_TYPE, &start_time_val); start_realtime=realtime(&start_time_val);
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    clock_getres(MY_CLOCK_TYPE, &current_time_res); current_realtime_res=realtime(&current_time_res);
    printf("START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);
    syslog(LOG_CRIT, "START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);

    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

    // Initialize CPU set
    CPU_ZERO(&allcpuset);

    for(i=0; i < NUM_CPU_CORES; i++)
        CPU_SET(i, &allcpuset);

    printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));

    // Initialize the semaphores for the services
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
    if (sem_init (&semS4, 0, 0)) { printf ("Failed to initialize S4 semaphore\n"); exit (-1); }
    if (sem_init (&semS5, 0, 0)) { printf ("Failed to initialize S5 semaphore\n"); exit (-1); }
    if (sem_init (&semS6, 0, 0)) { printf ("Failed to initialize S6 semaphore\n"); exit (-1); }
    if (sem_init (&semS7, 0, 0)) { printf ("Failed to initialize S7 semaphore\n"); exit (-1); }

    mainpid=getpid();  // Get process ID

    // Set maximum and minimum priorities for SCHED_FIFO
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);  // Set scheduler to SCHED_FIFO
    if(rc < 0) perror("main_param");
    print_scheduler();  // Print the scheduler policy

    pthread_attr_getscope(&main_attr, &scope);  // Get the scope of the main thread

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    // Set up thread attributes and create service threads
    for(i=0; i < NUM_THREADS; i++)
    {
      // Run even indexed threads on core 2
      if(i % 2 == 0)
      {
          CPU_ZERO(&threadcpu);
          cpuidx=(2);
          CPU_SET(cpuidx, &threadcpu);
      }
      // Run odd indexed threads on core 3
      else
      {
          CPU_ZERO(&threadcpu);
          cpuidx=(3);
          CPU_SET(cpuidx, &threadcpu);
      }

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);  // Set scheduling policy to SCHED_FIFO
      rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);  // Set CPU affinity

      rt_param[i].sched_priority=rt_max_prio-i;  // Set priority
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    // Create service threads
    // Service_1 = RT_MAX-1 @ 50 Hz
    rt_param[0].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Service_1, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");

    // Service_2 = RT_MAX-2 @ 20 Hz
    rt_param[1].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1], &rt_sched_attr[1], Service_2, (void *)&(threadParams[1]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");

    // Service_3 = RT_MAX-3 @ 10 Hz
    rt_param[2].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_3, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        printf("pthread_create successful for service 3\n");

    // Service_4 = RT_MAX-4 @ 5 Hz
    rt_param[3].sched_priority=rt_max_prio-4;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], Service_4, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 4");
    else
        printf("pthread_create successful for service 4\n");

    // Service_5 = RT_MAX-5 @ 2 Hz
    rt_param[4].sched_priority=rt_max_prio-5;
    pthread_attr_setschedparam(&rt_sched_attr[4], &rt_param[4]);
    rc=pthread_create(&threads[4], &rt_sched_attr[4], Service_5, (void *)&(threadParams[4]));
    if(rc < 0)
        perror("pthread_create for service 5");
    else
        printf("pthread_create successful for service 5\n");

    // Service_6 = RT_MAX-6 @ 1 Hz
    rt_param[5].sched_priority=rt_max_prio-6;
    pthread_attr_setschedparam(&rt_sched_attr[5], &rt_param[5]);
    rc=pthread_create(&threads[5], &rt_sched_attr[5], Service_6, (void *)&(threadParams[5]));
    if(rc < 0)
        perror("pthread_create for service 6");
    else
        printf("pthread_create successful for service 6\n");

    // Service_7 = RT_MIN @ 1 Hz
    rt_param[6].sched_priority=rt_min_prio;
    pthread_attr_setschedparam(&rt_sched_attr[6], &rt_param[6]);
    rc=pthread_create(&threads[6], &rt_sched_attr[6], Service_7, (void *)&(threadParams[6]));
    if(rc < 0)
        perror("pthread_create for service 7");
    else
        printf("pthread_create successful for service 7\n");

    // Create Sequencer thread, which like a cyclic executive, is the highest priority
    printf("Start sequencer\n");
    sequencePeriods=2000;  // Number of sequencing periods

    // Set up the timer to signal SIGALRM if the timer expires
    timer_create(CLOCK_REALTIME, NULL, &timer_1);

    // Set the signal handler for SIGALRM to the Sequencer function
    signal(SIGALRM, (void(*)()) Sequencer);

    // Arm the interval timer for the sequencer
    itime.it_interval.tv_sec = 0;
    itime.it_interval.tv_nsec = 10000000;  // 100 Hz
    itime.it_value.tv_sec = 0;
    itime.it_value.tv_nsec = 10000000;
    timer_settime(timer_1, flags, &itime, &last_itime);

    // Wait for service threads to complete
    for(i=0;i<NUM_THREADS;i++)
    {
        if(rc=pthread_join(threads[i], NULL) < 0)
            perror("main pthread_join");
        else
            printf("joined thread %d\n", i);
    }

   printf("\nTEST COMPLETE\n");
}

// Sequencer function to release services based on the sequence count
void Sequencer(int id)
{
    struct timespec current_time_val;
    double current_realtime;
    int rc, flags=0;

    seqCnt++;  // Increment the sequence counter

    // Release each service at a sub-rate of the generic sequencer rate

    // Service_1 = RT_MAX-1 @ 50 Hz
    //if((seqCnt % 2) == 0) sem_post(&semS1);

    // Service_2 = RT_MAX-2 @ 20 Hz
    //if((seqCnt % 5) == 0) sem_post(&semS2);

    // Service_3 = RT_MAX-3 @ 10 Hz
    //if((seqCnt % 10) == 0) sem_post(&semS3);

    // Service_4 = RT_MAX-4 @ 5 Hz
    if((seqCnt % 20) == 0) sem_post(&semS4);

    // Service_5 = RT_MAX-5 @ 2 Hz
    //if((seqCnt % 50) == 0) sem_post(&semS5);

    // Service_6 = RT_MAX-6 @ 1 Hz
    //if((seqCnt % 100) == 0) sem_post(&semS6);

    // Service_7 = RT_MIN 1 Hz
    if((seqCnt % 100) == 0) sem_post(&semS7);

    // Check if the test should be aborted or if the sequence count has reached its limit
    if(abortTest || (seqCnt >= sequencePeriods))
    {
        // Disable the interval timer
        itime.it_interval.tv_sec = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec = 0;
        itime.it_value.tv_nsec = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);
        printf("Disabling sequencer interval timer with abort=%d and %llu of %lld\n", abortTest, seqCnt, sequencePeriods);

        // Shutdown all services
        sem_post(&semS1); sem_post(&semS2); sem_post(&semS3);
        sem_post(&semS4); sem_post(&semS5); sem_post(&semS6);
        sem_post(&semS7);

        abortS1=TRUE; abortS2=TRUE; abortS3=TRUE;
        abortS4=TRUE; abortS5=TRUE; abortS6=TRUE;
        abortS7=TRUE;
    }
}

// Service_1 function: runs at 50 Hz
void *Service_1(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S1 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S1 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS1) // Check for synchronous abort request
    {
        // Wait for service request from the sequencer, a signal handler, or ISR in the kernel
        sem_wait(&semS1);

        S1Cnt++;  // Increment the service counter

        // DO WORK

        // Record the time and log it
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S1 50 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S1Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}

// Service_2 function: runs at 20 Hz
void *Service_2(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S2Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S2 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S2 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS2)
    {
        sem_wait(&semS2);  // Wait for service request
        S2Cnt++;

        // Record the time and log it
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S2 20 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S2Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}

// Service_3 function: runs at 10 Hz
void *Service_3(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S3Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S3 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S3 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS3)
    {
        sem_wait(&semS3);  // Wait for service request
        S3Cnt++;

        // Record the time and log it
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S3 10 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S3Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}

// Service_4 function: runs at 5 Hz
void *Service_4(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S4Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S4 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S4 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS4)
    {
        sem_wait(&semS4);  // Wait for service request
        S4Cnt++;

        // Record the time and log it
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S4 5 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S4Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}

// Service_5 function: runs at 2 Hz
void *Service_5(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S5Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S5 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S5 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS5)
    {
        sem_wait(&semS5);  // Wait for service request
        S5Cnt++;

        // Record the time and log it
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S5 2 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S5Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}

// Service_6 function: runs at 1 Hz
void *Service_6(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S6Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S6 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S6 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS6)
    {
        sem_wait(&semS6);  // Wait for service request
        S6Cnt++;

        // Record the time and log it
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S6 1 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S6Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}

// Service_7 function: runs at 1 Hz
void *Service_7(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S7Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S7 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S7 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS7)
    {
        sem_wait(&semS7);  // Wait for service request
        S7Cnt++;

        // Record the time and log it
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S7 1 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S7Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}

// Function to get time in milliseconds
double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(MY_CLOCK_TYPE, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

// Function to convert timespec to real time in seconds
double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec)/1000000000.0));
}

// Function to print the current scheduler policy
void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
           break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       //case SCHED_DEADLINE:
       //    printf("Pthread Policy is SCHED_DEADLINE\n"); exit(-1);
       //    break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}
