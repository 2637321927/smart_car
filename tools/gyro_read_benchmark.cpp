#include "lq_i2c_mpu6050.hpp"

#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

namespace
{
long long elapsed_us(std::chrono::steady_clock::time_point start,
                     std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}
} // namespace

int main(int argc, char **argv)
{
    int samples = 100;
    int sleep_us_between_reads = 0;

    if (argc >= 2) {
        samples = std::atoi(argv[1]);
    }
    if (argc >= 3) {
        sleep_us_between_reads = std::atoi(argv[2]);
    }
    if (samples <= 0) {
        samples = 100;
    }
    if (sleep_us_between_reads < 0) {
        sleep_us_between_reads = 0;
    }

    lq_i2c_mpu6050 mpu6050;
    const uint8_t id = mpu6050.get_mpu6050_id();
    printf("[GYRO_BENCH] device id=0x%02x, samples=%d, sleep_us=%d\n",
           id, samples, sleep_us_between_reads);

    int ok_count = 0;
    int fail_count = 0;
    long long min_us = LLONG_MAX;
    long long max_us = 0;
    long long sum_us = 0;
    int16_t last_gx = 0;
    int16_t last_gy = 0;
    int16_t last_gz = 0;

    for (int i = 0; i < samples; ++i) {
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;

        const auto start = std::chrono::steady_clock::now();
        const bool ok = mpu6050.get_mpu6050_ang(&gx, &gy, &gz);
        const auto end = std::chrono::steady_clock::now();
        const long long used_us = elapsed_us(start, end);

        if (ok) {
            ++ok_count;
            sum_us += used_us;
            if (used_us < min_us) min_us = used_us;
            if (used_us > max_us) max_us = used_us;
            last_gx = gx;
            last_gy = gy;
            last_gz = gz;
        } else {
            ++fail_count;
        }

        printf("[GYRO_BENCH] %03d ok=%d used=%.3fms gx=%d gy=%d gz=%d\n",
               i + 1,
               ok ? 1 : 0,
               used_us / 1000.0,
               gx,
               gy,
               gz);

        if (sleep_us_between_reads > 0) {
            usleep((useconds_t)sleep_us_between_reads);
        }
    }

    if (ok_count > 0) {
        printf("[GYRO_BENCH] summary ok=%d fail=%d min=%.3fms avg=%.3fms max=%.3fms last=(%d,%d,%d)\n",
               ok_count,
               fail_count,
               min_us / 1000.0,
               (sum_us / (double)ok_count) / 1000.0,
               max_us / 1000.0,
               last_gx,
               last_gy,
               last_gz);
    } else {
        printf("[GYRO_BENCH] summary ok=0 fail=%d, no valid gyro sample\n", fail_count);
    }

    return fail_count == samples ? 1 : 0;
}
