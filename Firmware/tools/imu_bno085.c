// Linux polling SH-2 acquisition. Sensor values stay in the raw sensor frame.
// No persistent tare/calibration writes; a reference pose is set by the consumer.
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

static int fd = -1;
static volatile sig_atomic_t stopping;
static unsigned counts[4], read_errors, resets;
static unsigned generation, tare_samples;
static int io_failed, invalid_sample, tared, sh2_is_open;
static int continuous_mode, commissioning_mode;
static unsigned long retain_lines;
static uint64_t output_lines, tare_rx_ns;
static uint64_t last_sample_ns, last_accel_ns, last_gyro_ns, stable_since_ns;
static float accel_norm, gyro_norm;
static double tare_sum[4], tare_ref[4];
static double norm(const float *v, unsigned n) {
    double sum=0; for (unsigned i=0; i<n; ++i) sum+=(double)v[i]*v[i]; return sqrt(sum);
}
static uint64_t now_ns(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ULL + t.tv_nsec;
}
static void emit_tare(uint64_t rx_ns, int checkpoint) {
    printf("{\"kind\":\"tare\",\"rx_ns\":%" PRIu64 ",\"generation\":%u,\"method\":\"host_stationary_game_rv\",\"q_ref_xyzw\":[%.9g,%.9g,%.9g,%.9g],\"mount_alignment_valid\":false%s}\n",
           rx_ns,generation,tare_ref[0],tare_ref[1],tare_ref[2],tare_ref[3],
           checkpoint ? ",\"checkpoint\":true" : "");
    ++output_lines;
}
static void maybe_rotate_trace(void) {
    if (!continuous_mode || retain_lines == 0 || output_lines < retain_lines) return;
    struct stat st;
    if (fflush(stdout) != 0 || fstat(STDOUT_FILENO, &st) != 0 || !S_ISREG(st.st_mode)) {
        fprintf(stderr,"IMU trace retention disabled: stdout is not a seekable regular file\n");
        retain_lines = 0;
        return;
    }
    if (ftruncate(STDOUT_FILENO, 0) != 0 || lseek(STDOUT_FILENO, 0, SEEK_SET) < 0) {
        fprintf(stderr,"IMU trace retention failed: %s\n",strerror(errno));
        retain_lines = 0;
        return;
    }
    clearerr(stdout);
    output_lines = 0;
    printf("{\"kind\":\"trace_reset\",\"rx_ns\":%" PRIu64 ",\"generation\":%u,\"reason\":\"retained_lines\",\"tare_invalidated\":false}\n",
           now_ns(),generation);
    ++output_lines;
    // This is a checkpoint of the still-live reference, not a new tare.
    // Preserve its original timestamp and mark the row explicitly.
    if (tared) emit_tare(tare_rx_ns, 1);
}
static void stop(int sig) { (void)sig; stopping = 1; }
static int hal_open(sh2_Hal_t *self) {
    (void)self;
    fd = open("/dev/i2c-1", O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("open i2c"); return -1; }
    if (ioctl(fd, I2C_SLAVE, 0x4a) < 0) { perror("I2C_SLAVE"); close(fd); fd = -1; return -1; }
    uint8_t reset[] = {5, 0, 1, 0, 1};
    for (int attempt = 0; attempt < (commissioning_mode ? 1 : 6); ++attempt) {
        // Match the upstream Adafruit SH-2 I2C reset settling interval. The
        // original lab's 20 ms delay races the sensor's reset/advertisements.
        if (write(fd, reset, sizeof reset) == sizeof reset) { usleep(300000); return 0; }
        usleep(300000);
    }
    perror("IMU soft reset"); close(fd); fd = -1; return -1;
}
static void hal_close(sh2_Hal_t *self) { (void)self; if (fd >= 0) close(fd); fd = -1; }
static int hal_read(sh2_Hal_t *self, uint8_t *out, unsigned len, uint32_t *time_us) {
    (void)self;
    uint8_t header[4];
    // This is polling time, not an interrupt edge timestamp. SH-2 subtracts its
    // report delays; consumers still must tolerate polling/transport latency.
    *time_us = (uint32_t)(now_ns() / 1000);
    ssize_t header_n = read(fd, header, 4);
    if (header_n != 4) {
        if (read_errors < 3) fprintf(stderr,"I2C header n=%zd errno=%d (%s)\n",header_n,errno,strerror(errno));
        ++read_errors; io_failed=1; usleep(1000); return 0;
    }
    unsigned size = (header[0] | (header[1] << 8)) & 0x7fff;
    if (size == 0) return 0;
    if (size < 4 || size > len) {
        if (read_errors < 3) fprintf(stderr,"I2C packet size=%u buffer=%u header=%02x%02x/%u/%u\n",size,len,header[0],header[1],header[2],header[3]);
        ++read_errors; io_failed=1; return 0;
    }
    // Linux accepts the entire bounded packet. Match the owner's earlier
    // ESP-IDF port: header peek, then one complete receive, no chunk joins.
    // Transfer sequence advances after the peek; do not require equality.
    ssize_t n=read(fd, out, size);
    if (n != (ssize_t)size || out[2] != header[2]) {
        fprintf(stderr,"I2C packet n=%zd expected=%u errno=%d\n",n,size,errno);
        ++read_errors; io_failed=1; return 0;
    }
    return (int)size;
}
static int hal_write(sh2_Hal_t *self, uint8_t *data, unsigned len) {
    (void)self;
    if (write(fd, data, len) == (ssize_t)len) return (int)len;
    io_failed=1; return 0;
}
static uint32_t hal_time(sh2_Hal_t *self) { (void)self; return (uint32_t)(now_ns() / 1000); }
static sh2_Hal_t hal = {.open=hal_open, .close=hal_close, .read=hal_read, .write=hal_write, .getTimeUs=hal_time};
static void event(void *cookie, sh2_AsyncEvent_t *ev) {
    (void)cookie;
    if (ev->eventId == SH2_RESET) ++resets;
    printf("{\"kind\":\"event\",\"rx_ns\":%" PRIu64 ",\"event_id\":%d}\n", now_ns(), ev->eventId);
    ++output_lines;
}
static void sensor(void *cookie, sh2_SensorEvent_t *ev) {
    (void)cookie;
    sh2_SensorValue_t v = {0};
    if (sh2_decodeSensorEvent(&v, ev) != SH2_OK) return;
    const char *name; float a[4]; int n = 3, index;
    switch (v.sensorId) {
    case SH2_ACCELEROMETER:
        name="accel"; index=0; a[0]=v.un.accelerometer.x; a[1]=v.un.accelerometer.y; a[2]=v.un.accelerometer.z; break;
    case SH2_GYROSCOPE_CALIBRATED:
        name="gyro"; index=1; a[0]=v.un.gyroscope.x; a[1]=v.un.gyroscope.y; a[2]=v.un.gyroscope.z; break;
    case SH2_ROTATION_VECTOR:
        name="rv"; index=2; n=4; a[0]=v.un.rotationVector.i; a[1]=v.un.rotationVector.j; a[2]=v.un.rotationVector.k; a[3]=v.un.rotationVector.real; break;
    case SH2_GAME_ROTATION_VECTOR:
        name="game_rv"; index=3; n=4; a[0]=v.un.gameRotationVector.i; a[1]=v.un.gameRotationVector.j; a[2]=v.un.gameRotationVector.k; a[3]=v.un.gameRotationVector.real; break;
    default: return;
    }
    for (int i=0; i<n; ++i) if (!isfinite(a[i])) { invalid_sample=1; return; }
    if (n==4 && (norm(a,4)<0.99 || norm(a,4)>1.01)) { invalid_sample=1; return; }
    maybe_rotate_trace();
    uint64_t rx = now_ns(), rx_us = rx / 1000;
    // Lift the SDK's host-derived 32-bit time onto CLOCK_MONOTONIC's full epoch.
    int64_t sample_us = (int64_t)rx_us + (int32_t)((uint32_t)v.timestamp - (uint32_t)rx_us);
    last_sample_ns=rx;
    if (index==0) { accel_norm=norm(a,3); last_accel_ns=rx; }
    if (index==1) { gyro_norm=norm(a,3); last_gyro_ns=rx; }
    if (index==3 && !tared) {
        double qn=norm(a,4);
        int stationary=rx-last_accel_ns<100000000ULL && rx-last_gyro_ns<100000000ULL &&
            accel_norm>8.5 && accel_norm<11.0 && gyro_norm<0.03 && (v.status&3)>=2 &&
            qn>0.99 && qn<1.01 && (int64_t)rx-sample_us*1000>=0 && (int64_t)rx-sample_us*1000<100000000;
        if (!stationary) { stable_since_ns=0; tare_samples=0; memset(tare_sum,0,sizeof tare_sum); }
        else {
            if (!stable_since_ns) stable_since_ns=rx;
            double dot=0; for (int i=0; i<4; ++i) dot+=tare_sum[i]*a[i];
            double sign=dot<0 ? -1 : 1;
            for (int i=0; i<4; ++i) tare_sum[i]+=sign*a[i]/qn;
            ++tare_samples;
            if (rx-stable_since_ns>=2000000000ULL && tare_samples>=80) {
                double sn=0; for (int i=0; i<4; ++i) sn+=tare_sum[i]*tare_sum[i]; sn=sqrt(sn);
                for (int i=0; i<4; ++i) tare_ref[i]=tare_sum[i]/sn;
                tared=1;
                tare_rx_ns=rx;
                emit_tare(rx, 0);
            }
        }
    }
    ++counts[index];
    printf("{\"kind\":\"sample\",\"sensor\":\"%s\",\"rx_ns\":%" PRIu64 ",\"sample_ns\":%" PRId64
           ",\"sh2_us\":%" PRIu64 ",\"generation\":%u,\"sequence\":%u,\"status\":%u,\"values\":[",
           name, rx, sample_us*1000, v.timestamp, generation, v.sequence, v.status & 3);
    for (int i=0; i<n; ++i) printf("%s%.9g", i ? "," : "", a[i]);
    printf("]");
    if (index==3 && tared) {
        // q_ref^-1 * q_current, relative to initial sensor axes; not base pose.
        double x=-tare_ref[0],y=-tare_ref[1],z=-tare_ref[2],w=tare_ref[3], qn=norm(a,4);
        printf(",\"relative_xyzw\":[%.9g,%.9g,%.9g,%.9g]",
            (w*a[0]+x*a[3]+y*a[2]-z*a[1])/qn, (w*a[1]-x*a[2]+y*a[3]+z*a[0])/qn,
            (w*a[2]+x*a[1]-y*a[0]+z*a[3])/qn, (w*a[3]-x*a[0]-y*a[1]-z*a[2])/qn);
    }
    puts("}");
    ++output_lines;
}
static int open_stream(void) {
    if (sh2_open(&hal, event, NULL) != SH2_OK) { fprintf(stderr,"sh2_open failed\n"); return 1; }
    sh2_is_open=1;
    sh2_ProductIds_t ids = {0};
    int identity_rc = sh2_getProdIds(&ids);
    if (identity_rc != SH2_OK) { fprintf(stderr,"product identity failed rc=%d read_errors=%u resets=%u\n",identity_rc,read_errors,resets); return 1; }
    for (int i=0; i<ids.numEntries; ++i) {
        sh2_ProductId_t *p = &ids.entry[i];
        printf("{\"kind\":\"product\",\"part\":%u,\"version\":\"%u.%u.%u\",\"build\":%u,\"reset_cause\":%u}\n",
               p->swPartNumber, p->swVersionMajor, p->swVersionMinor, p->swVersionPatch, p->swBuildNumber, p->resetCause);
        ++output_lines;
    }
    sh2_setSensorCallback(sensor, NULL);
    const sh2_SensorId_t sensors[] = {SH2_ACCELEROMETER, SH2_GYROSCOPE_CALIBRATED, SH2_ROTATION_VECTOR, SH2_GAME_ROTATION_VECTOR};
    sh2_SensorConfig_t cfg = {0}; cfg.reportInterval_us = 20000;
    for (unsigned i=0; i<4; ++i) {
        int rc=sh2_setSensorConfig(sensors[i], &cfg);
        printf("{\"kind\":\"config\",\"sensor_id\":%u,\"rc\":%d,\"requested_interval_us\":%u}\n", sensors[i], rc, cfg.reportInterval_us);
        ++output_lines;
        if (rc) return 1;
    }
    return 0;
}
int main(int argc, char **argv) {
    char *end = NULL;
    commissioning_mode = argc == 2 && strcmp(argv[1], "--commissioning") == 0;
    int continuous = commissioning_mode || (argc >= 2 && strcmp(argv[1], "--continuous") == 0);
    long seconds = continuous ? 0 : (argc >= 2 ? strtol(argv[1], &end, 10) : -1);
    if (continuous && argc == 4 && strcmp(argv[2], "--retain-lines") == 0) {
        char *retain_end = NULL;
        errno = 0;
        unsigned long parsed = strtoul(argv[3], &retain_end, 10);
        if (!errno && retain_end != argv[3] && *retain_end == '\0' && parsed >= 64 && parsed <= 1000000)
            retain_lines = parsed;
        else {
            fprintf(stderr, "--retain-lines must be 64..1000000\n"); return 2;
        }
    } else if ((continuous && argc != 2) || (!continuous && argc != 2)) {
        fprintf(stderr, "Usage: imu-bno085 SECONDS (0 or 1..120) | --continuous [--retain-lines N]; owns/resets BNO085 I2C1:0x4a\n"); return 2;
    }
    if (!continuous && (*end || seconds < 0 || seconds > 120)) {
        fprintf(stderr, "Usage: imu-bno085 SECONDS (0 or 1..120) | --continuous [--retain-lines N]; owns/resets BNO085 I2C1:0x4a\n"); return 2;
    }
    continuous_mode = continuous || seconds == 0;
    char lock_path[80]; snprintf(lock_path, sizeof lock_path, "/tmp/ota-imu-%u.lock", getuid());
    int lock = open(lock_path, O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB) < 0) { perror("IMU ownership"); return 2; }
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, stop); signal(SIGTERM, stop);
    uint64_t until=continuous_mode ? UINT64_MAX : now_ns()+(uint64_t)seconds*1000000000ULL;
    int failed=0; unsigned recoveries=0;
    for (;;) {
        io_failed=0; tared=0; stable_since_ns=0; tare_samples=0; memset(tare_sum,0,sizeof tare_sum);
        tare_rx_ns=0;
        last_accel_ns=last_gyro_ns=0;
        int opened=open_stream();
        unsigned initial_resets=resets;
        last_sample_ns=now_ns();
        while (!opened && !stopping && !invalid_sample && !io_failed && resets==initial_resets && now_ns()<until) {
            sh2_service();
            if (now_ns()-last_sample_ns>500000000ULL) { io_failed=1; fprintf(stderr,"IMU stream stale >500ms\n"); }
            usleep(1000);
        }
        failed=opened || invalid_sample || io_failed || resets!=initial_resets;
        if (sh2_is_open) { sh2_close(); sh2_is_open=0; }
        if (!failed || stopping || invalid_sample || commissioning_mode || recoveries>=1 || now_ns()>=until) break;
        // Recover at the session boundary, not recursively inside SH-2's read
        // callback. Consumers must discard pre-reset tare/continuity.
        printf("{\"kind\":\"gap\",\"rx_ns\":%" PRIu64 ",\"reason\":\"reset_recovery\",\"tare_invalidated\":true}\n",now_ns());
        ++output_lines;
        ++recoveries; ++generation;
    }
    printf("{\"kind\":\"summary\",\"counts\":[%u,%u,%u,%u],\"read_errors\":%u,\"recoveries\":%u,\"failed\":%s,\"tared\":%s}\n",
           counts[0],counts[1],counts[2],counts[3],read_errors,recoveries,failed ? "true":"false",tared ? "true":"false");
    ++output_lines;
    close(lock);
    return !failed && counts[0]>0 && counts[1]>0 && counts[2]>0 && counts[3]>0 ? 0 : 1;
}
