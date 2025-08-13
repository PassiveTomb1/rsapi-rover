#include "car.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cmath>
#include <string>
#include <chrono>
#include <fstream>
#include <gpiod.h> // Using the libgpiod library

// GPIO pins for ultrasonic sensor
const int TRIG_PIN = 27;  // GPIO 27
const int ECHO_PIN = 22;  // GPIO 22

// --- libgpiod specific variables ---
struct gpiod_chip *chip;
struct gpiod_line *trig_line;
struct gpiod_line *echo_line;
const char *chipname = "gpiochip0";


int openI2C(int addr) {
    const char *device = "/dev/i2c-1";
    int file = open(device, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C device\n";
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        std::cerr << "Failed to set I2C address\n";
        close(file);
        return -1;
    }
    return file;
}

void writeRegister(int file, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2) {
        std::cerr << "Failed to write to register " << (int)reg << "\n";
    }
}

void setPWM(int file, int channel, int on, int off) {
    int base = 0x06 + 4 * channel;
    writeRegister(file, static_cast<uint8_t>(base),     static_cast<uint8_t>(on & 0xFF));
    writeRegister(file, static_cast<uint8_t>(base + 1), static_cast<uint8_t>((on >> 8) & 0xFF));
    writeRegister(file, static_cast<uint8_t>(base + 2), static_cast<uint8_t>(off & 0xFF));
    writeRegister(file, static_cast<uint8_t>(base + 3), static_cast<uint8_t>((off >> 8) & 0xFF));
}

void initPCA9685(int file) {
    writeRegister(file, 0x00, 0x00); // MODE1 reset
    usleep(5000);

    int freq = 50; // 50 Hz for servos/motors
    int prescale = static_cast<int>(round(25000000.0 / (4096 * freq)) - 1);

    uint8_t oldmode;
    lseek(file, 0x00, SEEK_SET);
    if (read(file, &oldmode, 1) != 1) {
        std::cerr << "Failed to read MODE1 register\n";
        return;
    }

    writeRegister(file, 0x00, static_cast<uint8_t>((oldmode & 0x7F) | 0x10)); // sleep
    writeRegister(file, 0xFE, static_cast<uint8_t>(prescale));               // prescale
    writeRegister(file, 0x00, oldmode);                                      // wake
    usleep(5000);
    writeRegister(file, 0x00, static_cast<uint8_t>(oldmode | 0x80));         // restart
    usleep(5000);
}

void setMotorPWM(int file, int chA, int chB, int duty) {
    duty = std::min(4095, std::max(-4095, duty));
    if (duty > 0) {
        setPWM(file, chA, 0, 0);
        setPWM(file, chB, 0, duty);
    } else if (duty < 0) {
        setPWM(file, chB, 0, 0);
        setPWM(file, chA, 0, -duty);
    } else {
        setPWM(file, chA, 0, 4095);
        setPWM(file, chB, 0, 4095);
    }
}

void setMotorModel(int file, int d1, int d2, int d3, int d4) {
    setMotorPWM(file, 0, 1, d1); // Left upper
    setMotorPWM(file, 3, 2, d2); // Left lower
    setMotorPWM(file, 6, 7, d3); // Right upper
    setMotorPWM(file, 4, 5, d4); // Right lower
}

void moveForDuration(int file, int frontLeft, int backLeft, int frontRight, int backRight, double seconds, const std::string& action) {
    std::cout << action << " for " << seconds << " seconds\n";
    setMotorModel(file, -frontLeft, -backLeft, -frontRight, -backRight);
    usleep(static_cast<useconds_t>(seconds * 1000000));
    setMotorModel(file, 0, 0, 0, 0);
}


// Initialize ultrasonic sensor GPIO pins using libgpiod
bool initUltrasonic() {
    // Open the GPIO chip
    chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        std::cerr << "Failed to open GPIO chip\n";
        return false;
    }

    // Get the GPIO lines
    trig_line = gpiod_chip_get_line(chip, TRIG_PIN);
    echo_line = gpiod_chip_get_line(chip, ECHO_PIN);
    if (!trig_line || !echo_line) {
        std::cerr << "Failed to get GPIO lines\n";
        gpiod_chip_close(chip);
        return false;
    }

    // Request TRIG line as output
    if (gpiod_line_request_output(trig_line, "car_app", 0) < 0) {
        std::cerr << "Failed to request TRIG line as output\n";
        gpiod_chip_close(chip);
        return false;
    }

    // ***MODIFIED***: Request ECHO line for both rising and falling edge events
    if (gpiod_line_request_both_edges_events(echo_line, "car_app") < 0) {
        std::cerr << "Failed to request ECHO line events\n";
        gpiod_line_release(trig_line);
        gpiod_chip_close(chip);
        return false;
    }
    
    // Ensure trigger is low initially and wait for sensor to settle
    gpiod_line_set_value(trig_line, 0);
    usleep(30000);

    return true;
}

// Cleanup function to release GPIO resources
void cleanupGPIO() {
    gpiod_line_release(trig_line);
    gpiod_line_release(echo_line);
    gpiod_chip_close(chip);
}

// ***REWRITTEN***: Get distance using an event-driven approach for reliability
double getDistance() {
    // --- Clear any stale events from the buffer before we start ---
    struct timespec no_timeout = { 0, 0 };
    while (gpiod_line_event_wait(echo_line, &no_timeout) == 1) {
        struct gpiod_line_event old_event;
        gpiod_line_event_read(echo_line, &old_event);
    }

    // --- Send a 10 microsecond trigger pulse ---
    gpiod_line_set_value(trig_line, 1);
    usleep(10);
    gpiod_line_set_value(trig_line, 0);

    struct timespec timeout = { 0, 50000000 }; // 50 millisecond timeout
    struct gpiod_line_event rising_event, falling_event;

    // --- Wait for the rising edge (pulse start) ---
    int rv = gpiod_line_event_wait(echo_line, &timeout);
    if (rv <= 0) { // 0 is timeout, < 0 is error
        return -1.0;
    }
    if (gpiod_line_event_read(echo_line, &rising_event) != 0 || rising_event.event_type != GPIOD_LINE_EVENT_RISING_EDGE) {
        return -2.0; // Failed to read event or got an unexpected event
    }

    // --- Wait for the falling edge (pulse end) ---
    rv = gpiod_line_event_wait(echo_line, &timeout);
    if (rv <= 0) {
        return -1.0; // Timeout waiting for pulse to end
    }
    if (gpiod_line_event_read(echo_line, &falling_event) != 0 || falling_event.event_type != GPIOD_LINE_EVENT_FALLING_EDGE) {
        return -2.0; // Failed to read event or got an unexpected event
    }

    // --- Calculate the duration from the precise event timestamps ---
    long long start_ns = (long long)rising_event.ts.tv_sec * 1000000000 + rising_event.ts.tv_nsec;
    long long end_ns = (long long)falling_event.ts.tv_sec * 1000000000 + falling_event.ts.tv_nsec;
    long long duration_ns = end_ns - start_ns;
    
    // Convert nanoseconds to microseconds for the distance formula
    double duration_us = static_cast<double>(duration_ns) / 1000.0;
    
    // Speed of sound is ~343 m/s, which is 0.0343 cm/µs.
    // Distance (cm) = (duration * speed_of_sound) / 2 (for the round trip)
    double distance = (duration_us * 0.0343) / 2.0;

    return distance;
}