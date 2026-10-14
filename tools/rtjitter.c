// rtjitter.c - real-time scheduling jitter benchmark for the control loop.
// Simulates a periodic control loop (sleep until next deadline + do work)
// and reports wake-up jitter statistics.
//
//   zig cc -target arm-linux-musleabihf -O3 -mcpu=cortex_a7+neon -static \
//       -o rtjitter rtjitter.c
//
// Usage: rtjitter [period_us] [duration_s] [work_us] [fifo0/1] [prio]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>

static double now_us(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

static void busy(double us)
{
    double t0 = now_us();
    volatile double x = 0;
    while (now_us() - t0 < us) { x += 1.0 * 1.000001; }
}

int main(int argc, char **argv)
{
    long period = (argc > 1) ? atol(argv[1]) : 1000;      // us
    int  dur    = (argc > 2) ? atoi(argv[2]) : 10;        // s
    double work = (argc > 3) ? atof(argv[3]) : 50;        // us of work per loop
    int  fifo   = (argc > 4) ? atoi(argv[4]) : 0;
    int  prio   = (argc > 5) ? atoi(argv[5]) : 80;

    if (fifo) {
        struct sched_param sp; memset(&sp, 0, sizeof(sp));
        sp.sched_priority = prio;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            perror("sched_setscheduler(SCHED_FIFO)");
            printf("(falling back to SCHED_OTHER)\n");
            fifo = 0;
        } else printf("scheduler = SCHED_FIFO prio %d\n", prio);
    } else printf("scheduler = SCHED_OTHER\n");

    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    long np = 0;
    double smin = 1e18, smax = -1e18, ssum = 0;
    long hist[7] = {0};   // <50 <100 <200 <500 <1000 <2000 >=2000 us
    long misses = 0, n = 0;
    double end = now_us() + dur * 1e6;

    while (now_us() < end) {
        next.tv_nsec += (period % 1000000) * 1000;
        next.tv_sec  += period / 1000000 + next.tv_nsec / 1000000000L;
        next.tv_nsec %= 1000000000L;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        double w = now_us();
        double tgt = next.tv_sec * 1e6 + next.tv_nsec / 1e3;
        double j = w - tgt;                    // positive = late
        if (j < 0) j = 0;                      // early wakeup: ignore sign
        if (j < smin) smin = j;
        if (j > smax) smax = j;
        ssum += j;
        if      (j <   50) hist[0]++;
        else if (j <  100) hist[1]++;
        else if (j <  200) hist[2]++;
        else if (j <  500) hist[3]++;
        else if (j < 1000) hist[4]++;
        else if (j < 2000) hist[5]++;
        else               hist[6]++;
        if (j > period) misses++;
        n++;
        busy(work);
    }
    printf("period=%ldus dur=%ds work=%.0fus loops=%ld\n", period, dur, work, n);
    printf("wake jitter (late): min %.1f  avg %.1f  max %.1f us\n", smin, ssum / n, smax);
    printf("hist: <50:%ld <100:%ld <200:%ld <500:%ld <1000:%ld <2000:%ld >=2000:%ld us\n",
           hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], hist[6]);
    printf("missed deadlines (>period): %ld / %ld  (%.3f%%)\n", misses, n, 100.0 * misses / n);
    return 0;
}
