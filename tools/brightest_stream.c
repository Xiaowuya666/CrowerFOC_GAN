// brightest_stream.c - V4L2(NV12) capture -> NEON brightest pixel -> draw
// crosshair -> grayscale JPEG -> HTTP MJPEG stream (multipart/x-mixed-replace).
//
// Build (Windows, Zig):
//   zig cc -target arm-linux-musleabihf -O3 -mcpu=cortex_a7+neon -static \
//       -o brightest_stream brightest_stream.c
//
// Usage: brightest_stream [dev] [w] [h] [nbuf] [port] [quality]
//   then open  http://<board-ip>:8080/
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <linux/videodev2.h>
#include <arm_neon.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_NO_PNG
#define STBI_NO_BMP
#define STBI_NO_TGA
#define STBI_NO_HDR
#define STBI_NO_PIC
#define STBI_NO_PNM
#include "stb_image_write.h"

struct buf { void *start; size_t length; };

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

// biggest luma value + first index, NEON
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

static inline void putpx(uint8_t *im, int w, int h, int x, int y, uint8_t v)
{
    if ((unsigned)x < (unsigned)w && (unsigned)y < (unsigned)h) im[(size_t)y * w + x] = v;
}

static void draw_marker(uint8_t *im, int w, int h, int cx, int cy)
{
    const int len = 18, t = 2;
    for (int i = -len; i <= len; i++) {
        for (int k = -t; k <= t; k++) {
            putpx(im, w, h, cx + i, cy + k, 255);
            putpx(im, w, h, cx + k, cy + i, 255);
        }
    }
    for (int i = -len; i <= len; i++) {     // thin black cross for contrast
        putpx(im, w, h, cx + i, cy, 0);
        putpx(im, w, h, cx, cy + i, 0);
    }
}

// ---- JPEG into memory buffer ----
struct jmem { unsigned char *p; size_t len, cap; };
static void jmem_write(void *ctx, void *data, int size)
{
    struct jmem *j = ctx;
    if (j->len + size > j->cap) { j->cap = (j->len + size) * 2; j->p = realloc(j->p, j->cap); }
    memcpy(j->p + j->len, data, size);
    j->len += size;
}

static int send_all(int fd, const void *buf, size_t n)
{
    const char *p = buf;
    while (n) {
        ssize_t r = send(fd, p, n, MSG_NOSIGNAL);
        if (r <= 0) return -1;
        p += r; n -= r;
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *dev = (argc > 1) ? argv[1] : "/dev/video11";
    int w = (argc > 2) ? atoi(argv[2]) : 640;
    int h = (argc > 3) ? atoi(argv[3]) : 480;
    int nbuf = (argc > 4) ? atoi(argv[4]) : 4;
    int port = (argc > 5) ? atoi(argv[5]) : 8080;
    int quality = (argc > 6) ? atoi(argv[6]) : 75;

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

    uint8_t *gray = malloc((size_t)w * h);
    struct jmem jpg; jpg.p = malloc((size_t)w * h); jpg.len = 0; jpg.cap = (size_t)w * h;

    int lsock = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1; setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa; memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET; sa.sin_addr.s_addr = INADDR_ANY; sa.sin_port = htons(port);
    if (bind(lsock, (struct sockaddr *)&sa, sizeof(sa)) < 0) { perror("bind"); return 1; }
    listen(lsock, 1);
    fcntl(lsock, F_SETFL, O_NONBLOCK);
    printf("MJPEG: http://<board-ip>:%d/   (dev=%s %dx%d stride=%d nbuf=%u q=%d)\n",
           port, dev, w, h, stride, req.count, quality);
    fflush(stdout);

    int csock = -1;
    double t0 = now_ms(), fc = 0; uint8_t last = 0; uint32_t lastidx = 0;
    double tlast = now_ms();

    for (;;) {
        if (csock < 0) {
            csock = accept(lsock, NULL, NULL);
            if (csock >= 0) {
                int nd = 1; setsockopt(csock, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
                const char *hdr = "HTTP/1.0 200 OK\r\nConnection: close\r\n"
                    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
                if (send_all(csock, hdr, strlen(hdr)) < 0) { close(csock); csock = -1; }
            }
        }
        fd_set fds; struct timeval tv = { 1, 0 };
        FD_ZERO(&fds); FD_SET(fd, &fds);
        if (select(fd + 1, &fds, NULL, NULL, &tv) <= 0) continue;

        struct v4l2_buffer b; struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&b, 0, sizeof(b)); memset(planes, 0, sizeof(planes));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; b.memory = V4L2_MEMORY_MMAP;
        b.m.planes = planes; b.length = VIDEO_MAX_PLANES;
        if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) { if (errno == EAGAIN) continue; perror("DQBUF"); break; }

        uint8_t *y = bufs[b.index].start;
        uint8_t mx; uint32_t idx = scan_brightest(y, w, h, stride, &mx);
        int bx = idx % w, by = idx / w;

        for (int r = 0; r < h; r++) memcpy(gray + (size_t)r * w, y + (size_t)r * stride, w);
        draw_marker(gray, w, h, bx, by);

        ioctl(fd, VIDIOC_QBUF, &b);

        jpg.len = 0;
        stbi_write_jpg_to_func(jmem_write, &jpg, w, h, 1, gray, quality);

        if (csock >= 0) {
            char ph[128];
            int n = snprintf(ph, sizeof(ph),
                "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", jpg.len);
            if (send_all(csock, ph, n) < 0 || send_all(csock, jpg.p, jpg.len) < 0) {
                close(csock); csock = -1;
            }
        }

        fc++; last = mx; lastidx = idx;
        double tn = now_ms();
        if (tn - tlast >= 2000) {
            printf("fps=%.1f  brightest=%u @(%u,%u)  jpeg=%zuB\n",
                   fc * 1000.0 / (tn - tlast), last, lastidx % w, lastidx / w, jpg.len);
            fflush(stdout); fc = 0; tlast = tn;
        }
    }
    return 0;
}
