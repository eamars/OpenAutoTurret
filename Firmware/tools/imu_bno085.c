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
#include <time.h>
#include <unistd.h>
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

static int fd = -1;
static volatile sig_atomic_t stopping;
static unsigned counts[4], read_errors, resets;
static uint64_t now_ns(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ULL + t.tv_nsec;
}
static void stop(int sig) { (void)sig; stopping = 1; }
static int hal_open(sh2_Hal_t *self) {
    (void)self;
    fd = open("/dev/i2c-1", O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("open i2c"); return -1; }
    if (ioctl(fd, I2C_SLAVE, 0x4a) < 0) { perror("I2C_SLAVE"); close(fd); fd = -1; return -1; }
    uint8_t reset[] = {5, 0, 1, 0, 1};
    for (int attempt = 0; attempt < 6; ++attempt) {
        // Match the upstream Adafruit SH-2 I2C reset settling interval. The
        // original lab's 20 ms delay races the sensor's reset/advertisements.
        if (write(fd, reset, sizeof reset) == sizeof reset) { usleep(300000); return 0; }
        usleep(30000);
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
        ++read_errors; usleep(1000); return 0;
    }
    unsigned size = (header[0] | (header[1] << 8)) & 0x7fff;
    if (size == 0) return 0;
    if (size < 4 || size > len) {
        if (read_errors < 3) fprintf(stderr,"I2C packet size=%u buffer=%u header=%02x%02x/%u/%u\n",size,len,header[0],header[1],header[2],header[3]);
        ++read_errors; return 0;
    }
    unsigned got = 0;
    while (got < size) {
        uint8_t chunk[60];
        unsigned wanted = size - got + (got ? 4 : 0);
        if (wanted > sizeof chunk) wanted = sizeof chunk;
        ssize_t n = read(fd, chunk, wanted);
        // BNO085 advances the transfer sequence on each I2C transaction,
        // including the header peek. It is not constant across chunk reads.
        if (n != (ssize_t)wanted || n < 4 || chunk[2] != header[2]) {
            if (read_errors < 5) fprintf(stderr, "I2C packet read n=%zd wanted=%u got=%u header=%02x%02x/%u/%u chunk=%02x%02x/%u/%u\n",
                n,wanted,got,header[0],header[1],header[2],header[3],chunk[0],chunk[1],chunk[2],chunk[3]);
            ++read_errors; return 0;
        }
        unsigned skip = got ? 4 : 0;
        memcpy(out + got, chunk + skip, (unsigned)n - skip);
        got += (unsigned)n - skip;
    }
    return (int)got;
}
static int hal_write(sh2_Hal_t *self, uint8_t *data, unsigned len) {
    (void)self;
    return write(fd, data, len) == (ssize_t)len ? (int)len : 0;
}
static uint32_t hal_time(sh2_Hal_t *self) { (void)self; return (uint32_t)(now_ns() / 1000); }
static sh2_Hal_t hal = {.open=hal_open, .close=hal_close, .read=hal_read, .write=hal_write, .getTimeUs=hal_time};
static void event(void *cookie, sh2_AsyncEvent_t *ev) {
    (void)cookie;
    if (ev->eventId == SH2_RESET) ++resets;
    printf("{\"kind\":\"event\",\"rx_ns\":%" PRIu64 ",\"event_id\":%d}\n", now_ns(), ev->eventId);
}
static void sensor(void *cookie, sh2_SensorEvent_t *ev) {
    (void)cookie;
    sh2_SensorValue_t v;
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
    for (int i=0; i<n; ++i) if (!isfinite(a[i])) { stopping=1; return; }
    uint64_t rx = now_ns(), rx_us = rx / 1000;
    // Lift the SDK's host-derived 32-bit time onto CLOCK_MONOTONIC's full epoch.
    int64_t sample_us = (int64_t)rx_us + (int32_t)((uint32_t)v.timestamp - (uint32_t)rx_us);
    ++counts[index];
    printf("{\"kind\":\"sample\",\"sensor\":\"%s\",\"rx_ns\":%" PRIu64 ",\"sample_ns\":%" PRId64
           ",\"sh2_us\":%" PRIu64 ",\"delay_us\":%u,\"sequence\":%u,\"status\":%u,\"values\":[",
           name, rx, sample_us*1000, v.timestamp, v.delay, v.sequence, v.status & 3);
    for (int i=0; i<n; ++i) printf("%s%.9g", i ? "," : "", a[i]);
    puts("]}");
}
int main(int argc, char **argv) {
    char *end = NULL;
    long seconds = argc == 2 ? strtol(argv[1], &end, 10) : 0;
    if (argc != 2 || *end || seconds < 1 || seconds > 120) {
        fprintf(stderr, "Usage: imu-bno085 SECONDS (1..120); owns/resets BNO085 I2C1:0x4a\n"); return 2;
    }
    char lock_path[80]; snprintf(lock_path, sizeof lock_path, "/tmp/ota-imu-%u.lock", getuid());
    int lock = open(lock_path, O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB) < 0) { perror("IMU ownership"); return 2; }
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, stop); signal(SIGTERM, stop);
    if (sh2_open(&hal, event, NULL) != SH2_OK) { fprintf(stderr,"sh2_open failed\n"); return 1; }
    sh2_ProductIds_t ids = {0};
    int identity_rc = sh2_getProdIds(&ids);
    if (identity_rc != SH2_OK) { fprintf(stderr,"product identity failed rc=%d read_errors=%u resets=%u\n",identity_rc,read_errors,resets); sh2_close(); return 1; }
    for (int i=0; i<ids.numEntries; ++i) {
        sh2_ProductId_t *p = &ids.entry[i];
        printf("{\"kind\":\"product\",\"part\":%u,\"version\":\"%u.%u.%u\",\"build\":%u,\"reset_cause\":%u}\n",
               p->swPartNumber, p->swVersionMajor, p->swVersionMinor, p->swVersionPatch, p->swBuildNumber, p->resetCause);
    }
    sh2_setSensorCallback(sensor, NULL);
    const sh2_SensorId_t sensors[] = {SH2_ACCELEROMETER, SH2_GYROSCOPE_CALIBRATED, SH2_ROTATION_VECTOR, SH2_GAME_ROTATION_VECTOR};
    sh2_SensorConfig_t cfg = {0}; cfg.reportInterval_us = 20000;
    for (unsigned i=0; i<4; ++i) {
        sh2_SensorConfig_t actual = {0};
        int rc=sh2_setSensorConfig(sensors[i], &cfg);
        if (!rc) rc=sh2_getSensorConfig(sensors[i], &actual);
        printf("{\"kind\":\"config\",\"sensor_id\":%u,\"rc\":%d,\"interval_us\":%u}\n", sensors[i], rc, actual.reportInterval_us);
        // GetFeature can race activation and report the previous interval.
        // Actual samples and measured cadence are the viability check.
        if (rc) { sh2_close(); return 1; }
    }
    uint64_t until=now_ns()+(uint64_t)seconds*1000000000ULL;
    unsigned initial_resets=resets;
    while (!stopping && now_ns()<until && resets==initial_resets) { sh2_service(); usleep(1000); }
    sh2_close();
    printf("{\"kind\":\"summary\",\"counts\":[%u,%u,%u,%u],\"read_errors\":%u,\"unexpected_resets\":%u}\n",
           counts[0],counts[1],counts[2],counts[3],read_errors,resets-initial_resets);
    close(lock);
    return counts[0]>0 && counts[1]>0 && counts[2]>0 && counts[3]>0 && resets==initial_resets ? 0 : 1;
}
