#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <pigpio.h>
#include <wiringSerial.h>

#define SERVO_PAN  18
#define SERVO_TILT 17

volatile int running = 1;

// Shared servo positions
volatile int curr_pan = 500;
volatile int curr_tilt = 2000;

pthread_mutex_t pos_mutex = PTHREAD_MUTEX_INITIALIZER;

// Serial handle for TF-Luna
int lidar_serial;

// ----- Signal handler -----
void handle_sigint(int sig) {
    running = 0;
    gpioServo(SERVO_PAN, 0);
    gpioServo(SERVO_TILT, 0);
    gpioTerminate();
    serialClose(lidar_serial);
    printf("\nExiting cleanly.\n");
    exit(0);
}

// ----- Servo sweep thread -----
void* servo_sweep(void* arg) {
    int tilt_start = 2000, tilt_end = 2500, tilt_step = 50;
    int pan_min = 500, pan_max = 2000, pan_steps = 100;
    int delay_per_step_us = 100000;

    int pan_direction = 1; // 1=forward, -1=backward

    for (int tilt = tilt_start; tilt <= tilt_end && running; tilt += tilt_step) {
        // Move tilt
        gpioServo(SERVO_TILT, tilt);
        usleep(900000);

        pthread_mutex_lock(&pos_mutex);
        curr_tilt = tilt;
        pthread_mutex_unlock(&pos_mutex);

        // Sweep pan
        int pwm = (pan_direction == 1) ? pan_min : pan_max;
        int step_size = (pan_max - pan_min) / pan_steps;
        if (pan_direction == -1) step_size = -step_size;

        for (int i = 0; i <= pan_steps && running; i++) {
            gpioServo(SERVO_PAN, pwm);

            pthread_mutex_lock(&pos_mutex);
            curr_pan = pwm;
            pthread_mutex_unlock(&pos_mutex);

            usleep(delay_per_step_us);
            pwm += step_size;
        }

        pan_direction *= -1; // Reverse direction for next tilt
    }
    return NULL;
}

// ----- LIDAR reading thread -----
void* lidar_read(void* arg) {
    int data[9];

    while (running) {
        if (serialDataAvail(lidar_serial) >= 9) {
            // Read full 9-byte frame
            for (int i = 0; i < 9; i++) {
                data[i] = serialGetchar(lidar_serial);
            }

            if (data[0] == 0x59 && data[1] == 0x59) {
                int dist = data[2] + data[3] * 256;
                int strength = data[4] + data[5] * 256;

                if (strength > 50 && dist < 6000) {
                    int pan_snapshot, tilt_snapshot;

                    pthread_mutex_lock(&pos_mutex);
                    pan_snapshot = curr_pan;
                    tilt_snapshot = curr_tilt;
                    pthread_mutex_unlock(&pos_mutex);

                    // Send CSV over serial: pan,tilt,distance\n
                    printf("%d,%d,%d\n", pan_snapshot, tilt_snapshot, dist);
                    fflush(stdout);
                }
            }
        }
        usleep(5000);
    }
    return NULL;
}

// ----- MAIN -----
int main() {
    signal(SIGINT, handle_sigint);

    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio init failed\n");
        return 1;
    }

    // Open TF-Luna serial
    if ((lidar_serial = serialOpen("/dev/serial0", 115200)) < 0) {
        fprintf(stderr, "Unable to open TF-Luna serial\n");
        return 1;
    }

    pthread_t servo_thread, lidar_thread;
    pthread_create(&servo_thread, NULL, servo_sweep, NULL);
    pthread_create(&lidar_thread, NULL, lidar_read, NULL);

    pthread_join(servo_thread, NULL);
    pthread_join(lidar_thread, NULL);

    gpioServo(SERVO_PAN, 0);
    gpioServo(SERVO_TILT, 0);
    gpioTerminate();
    serialClose(lidar_serial);
    return 0;
}
