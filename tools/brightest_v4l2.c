// brightest_v4l2.c - V4L2(NV12) capture + NEON brightest pixel, with a
// decomposed latency report (queue age / scan) and V4L2 timestamp info.
//
// zig cc -target arm-linux-musleabihf -O3 -mcpu=cortex_a7+neon -static \
//     -o brightest_v4l2 brightest_v4l2.c
//
// Usage: brightest_v4l2 [dev] [w] [h] [nframes] [nbuf]
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <arm_neon.h>

struct buf { void *start; size_t length; };

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static inline uint32_t scan_brightest(const uint8_t *p, int w, int h, int stride,
                                      uint8_t *out_max)
{
    uint8_t gmax = 0;
    for (int y = 0; y < h; y++) {
        const uint8_t *row = p + (size_t)y * stride;
        uint8x16_t v = vdupq_n_u8(0);
        int x = 0;
        for (; x + 64 <= w; x += 64) {
            uint8x16_t a = vld1q_u8(row + x), b = vld1q_u8(row + x + 16);
            uint8x16_t c = vld1q_u8(row + x + 32), d = vld1q_u8(row + x + 48);
            v = vmaxq_u8(v, vmaxq_u8(vmaxq_u8(a, b), vmaxq_u8(c, d)));
        }
        for (; x + 16 <= w; x += 16) v = vmaxq_u8(v, vld1q_u8(row + x));
        uint8x8_t m8 = vpmax_u8(vget_low_u8(v), vget_high_u8(v));
        m8 = vpmax_u8(m8, m8); m8 = vpmax_u8(m8, m8); m8 = vpmax_u8(m8, m8);
        uint8_t rmax = vget_lane_u8(m8, 0);
        for (; x < w; x++) if (row[x] > rmax) rmax = row[x];
        if (rmax > gmax) gmax = rmax;
    }
    *out_max = gmax;
    uint8x16_t vm = vdupq_n_u8(gmax);
    for (int y = 0; y < h; y++) {
        const uint8_t *row = p + (size_t)y * stride;
        int x = 0;
        for (; x + 16 <= w; x += 16) {
            uint8x16_t eq = vceqq_u8(vld1q_u8(row + x), vm);
            uint32x4_t nz = vreinterpretq_u32_u8(eq);
            if (vgetq_lane_u32(nz, 0) | vgetq_lane_u32(nz, 1) |
                vgetq_lane_u32(nz, 2) | vgetq_lane_u32(nz, 3)) {
                uint8_t lanes[16]; vst1q_u8(lanes, eq);
                for (int k = 0; k < 16; k++)
                    if (lanes[k]) return (uint32_t)y * w + (x + k);
            }
        }
        for (; x < w; x++) if (row[x] == gmax) return (uint32_t)y * w + x;
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *dev = (argc > 1) ? argv[1] : "/dev/video11";
    int w = (argc > 2) ? atoi(argv[2]) : 640;
    int h = (argc > 3) ? atoi(argv[3]) : 480;
    int nframes = (argc > 4) ? atoi(argv[4]) : 200;
    int nbuf = (argc > 5) ? atoi(argv[5]) : 2;

    int fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }

    struct v4l2_format fmt; memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = w; fmt.fmt.pix_mp.height = h;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.num_planes = 1;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); return 1; }
    w = fmt.fmt.pix_mp.width; h = fmt.fmt.pix_mp.height;
    int stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline; if (stride < w) stride = w;

    struct v4l2_requestbuffers req; memset(&req, 0, sizeof(req));
    req.count = nbuf; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) { perror("REQBUFS"); return 1; }
    if (req.count > 8) req.count = 8;
    struct buf bufs[8];
    for (unsigned i = 0; i < req.count; i++) {
        struct v4l2_buffer b; struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&b, 0, sizeof(b)); memset(planes, 0, sizeof(planes));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; b.memory = V4L2_MEMORY_MMAP;
        b.index = i; b.m.planes = planes; b.length = VIDEO_MAX_PLANES;
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { perror("QUERYBUF"); return 1; }
        bufs[i].length = planes[0].length;
        bufs[i].start = mmap(NULL, planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, planes[0].m.mem_offset);
        if (bufs[i].start == MAP_FAILED) { perror("mmap"); return 1; }
    }
    for (unsigned i = 0; i < req.count; i++) {
        struct v4l2_buffer b; struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&b, 0, sizeof(b)); memset(planes, 0, sizeof(planes));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; b.memory = V4L2_MEMORY_MMAP;
        b.index = i; b.m.planes = planes; b.length = VIDEO_MAX_PLANES;
        if (ioctl(fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF"); return 1; }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) { perror("STREAMON"); return 1; }

    printf("dev=%s %dx%d stride=%d nbuf=%u frames=%d\n", dev, w, h, stride, req.count, nframes);

    double q_min = 1e18, q_sum = 0;      // frame-ready -> DQBUF returned
    double s_min = 1e18, s_sum = 0;      // scan only
    double lat_min = 1e18, lat_sum = 0;  // frame-ready -> result
    double dt_min = 1e18, dt_sum = 0;    // frame-to-frame (buffertimestamp)
    double tb_prev = 0;
    int got = 0, warm = 10; uint8_t last = 0; uint32_t lastidx = 0;
    int flags_printed = 0;

    for (int i = 0; i < nframes + warm; i++) {
        fd_set fds; struct timeval tv = { 1, 0 };
        FD_ZERO(&fds); FD_SET(fd, &fds);
        if (select(fd + 1, &fds, NULL, NULL, &tv) <= 0) { fprintf(stderr, "timeout\n"); break; }
        struct v4l2_buffer b; struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&b, 0, sizeof(b)); memset(planes, 0, sizeof(planes));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; b.memory = V4L2_MEMORY_MMAP;
        b.m.planes = planes; b.length = VIDEO_MAX_PLANES;
        if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) { if (errno == EAGAIN) { i--; continue; } perror("DQBUF"); break; }
        double t_buf = b.timestamp.tv_sec * 1000.0 + b.timestamp.tv_usec / 1000.0;
        double t_dq = now_ms();

        if (!flags_printed) {
            unsigned tt = b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
            unsigned ss = b.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
            printf("v4l2 tstamp: type=0x%x src=0x%x (%s)\n", tt, ss,
                   (ss == 0x00010000u) ? "SOF" : "EOF/frame-done");
            flags_printed = 1;
        }

        uint8_t mx;
        uint32_t idx = scan_brightest(bufs[b.index].start, w, h, stride, &mx);
        double t1 = now_ms();
        ioctl(fd, VIDIOC_QBUF, &b);

        if (i >= warm) {
            double q = t_dq - t_buf, s = t1 - t_dq, lat = t1 - t_buf;
            if (q < q_min) q_min = q; q_sum += q;
            if (s < s_min) s_min = s; s_sum += s;
            if (lat < lat_min) lat_min = lat; lat_sum += lat;
            if (tb_prev > 0) { double dt = t_buf - tb_prev; if (dt < dt_min) dt_min = dt; dt_sum += dt; }
            tb_prev = t_buf;
            got++;
        }
        last = mx; lastidx = idx;
    }
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    if (got > 0) {
        printf("brightest luma = %u @ (%u,%u)\n", last, lastidx % w, lastidx / w);
        printf("frame-ready -> DQBUF return : min %.3f  avg %.3f ms\n", q_min, q_sum / got);
        printf("scan (DQBUF -> result)      : min %.3f  avg %.3f ms\n", s_min, s_sum / got);
        printf("frame-ready -> result       : min %.3f  avg %.3f ms\n", lat_min, lat_sum / got);
        printf("frame interval (buf tstamp) : avg %.3f ms -> %.1f fps\n", dt_sum / got, 1000.0 / (dt_sum / got));
    }
    close(fd);
    return 0;
}
