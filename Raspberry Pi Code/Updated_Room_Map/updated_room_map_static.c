#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <pigpio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <ncurses.h>
#include <pthread.h>
#include <math.h>

// ===== GPIO ASSIGNMENTS =====
#define SERVO_PAN  18   // GPIO 18 - PWM0
#define SERVO_TILT 17   // GPIO 17 - PWM1

volatile int running = 1;

// Servo positions shared between threads
volatile int curr_pan;
volatile int curr_tilt;

// Servo sweep parameters
volatile int global_pan = 500;   // PWM min
volatile int global_tilt = 2000;  // PWM min
volatile int width = 2000;       // PWM range (~0-180 deg)
int pan_dir = 3;

pthread_mutex_t pos_mutex = PTHREAD_MUTEX_INITIALIZER;

// ===== GLOBALS =====
int serial_port;      // TF-Luna serial handle

// ======== TF-LUNA READ FUNCTION ========
void read_tfluna_data(int serial_port, int *data) {
    for (int i = 0; i < 9; i++) {
        data[i] = serialGetchar(serial_port);
    }
}

int read_lidar_distance(int serial_port) {
    int data[9];
    if (serialDataAvail(serial_port) >= 9) {
        read_tfluna_data(serial_port, data);
        if (data[0] == 0x59 && data[1] == 0x59) { // valid frame
            int dist = data[2] + data[3]*256;
            int strength = data[4] + data[5]*256;
            if (strength > 50 && dist < 6000) return dist;
        }
    }
    return -1; // no valid reading
}

// ======== CLEAN EXIT HANDLER ========
void handle_sigint(int sig) {
    gpioServo(SERVO_PAN, 0);
    gpioServo(SERVO_TILT, 0);
    gpioTerminate();
    serialClose(serial_port);
    endwin();
    printf("\nExiting cleanly.\n");
    exit(0);
}

// ======== MODE 1: SERVO MOVEMENT ONLY ========
void servo_move_only() {
    clear();
    mvprintw(0,0,"MODE 1: Servo movement only (press 'q' to stop)");
    refresh();
    nodelay(stdscr, TRUE);

    int ch;
    int tilt_start = 2000, tilt_end = 2500, tilt_step = 100;
    int pan_start = 500, pan_end = 2500, pan_steps = 100;
    int delay_per_step_us = 100000; 

    for (int tilt = tilt_start; tilt <= tilt_end; tilt += tilt_step) {
        gpioServo(SERVO_TILT, tilt);
        usleep(500000); // allow tilt to move
        int pwm = pan_start;
        int step_size = (pan_end - pan_start) / pan_steps;

        for (int i = 0; i <= pan_steps; i++) {
            if ((ch = getch()) == 'q') {
                nodelay(stdscr, FALSE);
                return;
            }
            gpioServo(SERVO_PAN, pwm);
            usleep(delay_per_step_us);
            pwm += step_size;

            float pan_deg = (pwm - 500) * 180.0 / 2000.0;
            float tilt_deg = (tilt - 500) * 180.0 / 2000.0;
            mvprintw(1,0,"Pan: %.1f°  Tilt: %.1f°   ", pan_deg, tilt_deg);
            refresh();
        }
    }
    nodelay(stdscr, FALSE);
    mvprintw(3,0,"Servo sweep complete.");
    refresh();
    sleep(1);
}

// ======== MODE 2: LIDAR DISPLAY ONLY ========
void lidar_display_only() {
	printf("MODE 2: LiDAR only (press Ctrl+C to stop)\n");

    int data[9];
	while (1) {
        if (serialDataAvail(serial_port) >= 9) {
            read_tfluna_data(serial_port, data);

            int distance = data[2] + data[3] * 256;
            int strength = data[4] + data[5] * 256;
            float temp = data[6] + data[7] * 256;
            temp = (temp / 8.0f) - 256.0f;


            printf("Distance: %d cm | Strength: %d | Temp: %.2f °C\n",
                   distance, strength, temp);

            fflush(stdout);
        }
        delay(1); // Avoid 100% CPU
    }
	

}

// Thread to sweep servos
void* servo_sweep(void* arg) {
    int tilt_start = 2000, tilt_end = 2500, tilt_step = 50;
    int pan_min = 500, pan_max = 2000, pan_steps = 100;
    int delay_per_step_us = 100000;

    int tilt;
    int pan_direction = 1; // 1 = forward, -1 = backward
    int pwm;

    for (tilt = tilt_start; tilt <= tilt_end && running; tilt += tilt_step) {
        // Move tilt first
        gpioServo(SERVO_TILT, tilt);
        usleep(900000); // allow tilt to reach position

        // Update shared tilt for LiDAR thread
        pthread_mutex_lock(&pos_mutex);
        curr_tilt = tilt;
        pthread_mutex_unlock(&pos_mutex);

        // Determine pan sweep direction
        if (pan_direction == 1) {
            pwm = pan_min;
        } else {
            pwm = pan_max;
        }

        int step_size = (pan_max - pan_min) / pan_steps;
        if (pan_direction == -1) step_size = -step_size;

        // Sweep pan
        for (int i = 0; i <= pan_steps && running; i++) {
            gpioServo(SERVO_PAN, pwm);

            // Update shared pan for LiDAR thread
            pthread_mutex_lock(&pos_mutex);
            curr_pan = pwm;
            pthread_mutex_unlock(&pos_mutex);

            usleep(delay_per_step_us);
            pwm += step_size;
        }

        // Reverse direction for next tilt step
        pan_direction *= -1;
    }

    return NULL;
}

// Thread to read TF-Luna and log/print distances
void* lidar_thread(void* arg) {
    int data[9];
    FILE *fp = fopen("scan_results.txt", "w");
    if (!fp) {
        perror("Failed to open file");
        return NULL;
    }

    while (running) {
		int data[9];
		read_tfluna_data(serial_port, data);
		
		int dist = data[2] + data[3]*256;
		
		if (dist > 0) {
			pthread_mutex_lock(&pos_mutex);
			int pan_snapshot = curr_pan;
			int tilt_snapshot = curr_tilt;
			pthread_mutex_unlock(&pos_mutex);
			
			fprintf(fp, "%d\t%d\t%d\n", pan_snapshot, tilt_snapshot, dist);
			fflush(fp);

			printf("Distance: %d cm | Pan: %d | Tilt: %d\n", dist, pan_snapshot, tilt_snapshot);
			fflush(stdout);
		}
       
        usleep(5000);
    }

    fclose(fp);
    return NULL;
}

// Main function
void servo_and_lidar() {
    pthread_t servo_thread, lidar_thread_id;

    pthread_create(&servo_thread, NULL, servo_sweep, NULL);
    pthread_create(&lidar_thread_id, NULL, lidar_thread, NULL);

    // Wait for threads
    pthread_join(servo_thread, NULL);
    pthread_join(lidar_thread_id, NULL);

    gpioServo(SERVO_PAN, 0);
    gpioServo(SERVO_TILT, 0);
    gpioTerminate();
    printf("Servo + LiDAR scan stopped.\n");
}

// ======== MAIN ========
int main() {
    // --- Setup Serial ---
    if ((serial_port = serialOpen("/dev/serial0", 115200)) < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }

    // --- Setup WiringPi and pigpio ---
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Unable to start wiringPi\n");
        return 1;
    }
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio init failed\n");
        return 1;
    }

    // --- Signal handling ---
    signal(SIGINT, handle_sigint);

    int choice;

    while (1) {
        printf("\nTF-LUNA LiDAR Scanner Menu\n");
        printf("1 - Servo Movement Only\n");
        printf("2 - LiDAR Display Only\n");
        printf("3 - Servo + LiDAR Scan\n");
        printf("Ctrl+C - Quit\n");
        printf("Enter choice: ");
        fflush(stdout);

        if (scanf("%d", &choice) != 1) {
            // Clear invalid input
            int c; while ((c = getchar()) != '\n' && c != EOF);
            continue;
        }

        switch (choice) {
            case 1:
                servo_move_only();
                break;
            case 2:
                lidar_display_only();
                break;
            case 3:
                servo_and_lidar();
                break;
            default:
                printf("Invalid choice. Try again.\n");
        }
    }

    gpioTerminate();
    serialClose(serial_port);
    return 0;
}
