#include "drive.h"
#include "config.h"
#include "functions/math.h"
#include "main.h"
#include "pros/rtos.hpp"

// TODO: make units, remember d is in inches
void move(double d, double t) {
    set_drive_brake_mode(MOTOR_BRAKE_BRAKE);

    // Calculate the distance to move
    // rpm = distance/circumference * 60/time
    // double r = (d / WHEEL_CIRCUMFERENCE) * (60 / t);

    // drive_velocity_left(r);
    // drive_velocity_right(-r);

    // pros::delay(t);

    // drive_velocity_left(0);
    // drive_velocity_right(0);
    dist = d;

    pros::delay(t);

    dist = 0;
};

void turn(double a, double t) {
    drive_left(a);
    drive_right(-a);

    pros::delay(t);

    drive_left(0);
    drive_right(0);
}

void wait(double t) {
    pros::delay(t);
}

// Implement drive_left
void drive_left(double v) {
    // Move the motors on the left side with the specified voltage
    DRIVE_MOTOR_L1.move(v);
    DRIVE_MOTOR_L2.move(v);
    DRIVE_MOTOR_L3.move(v);
}

// implement drive_right
void drive_right(double v) {
    // Move the motors on the right side with the specified voltage
    DRIVE_MOTOR_R1.move(v);
    DRIVE_MOTOR_R2.move(v);
    DRIVE_MOTOR_R3.move(v);
}

// Implement drive_velocity_left
void drive_velocity_left(double v) {
    // Move the motors on the left side with the specified velocity
    DRIVE_MOTOR_L1.move_velocity(v);
    DRIVE_MOTOR_L2.move_velocity(v);
    DRIVE_MOTOR_L3.move_velocity(v);
}

// Implement drive_velocity_right
void drive_velocity_right(double v) {
    // Move the motors on the right side with the specified velocity
    DRIVE_MOTOR_R1.move_velocity(v);
    DRIVE_MOTOR_R2.move_velocity(v);
    DRIVE_MOTOR_R3.move_velocity(v);
}

// Implement set_drive_brake_mode
void set_drive_brake_mode(pros::motor_brake_mode_e_t m) {
    // Set the brake mode for each motor individually
    DRIVE_MOTOR_L1.set_brake_mode(m);
    DRIVE_MOTOR_L2.set_brake_mode(m);
    DRIVE_MOTOR_L3.set_brake_mode(m);

    DRIVE_MOTOR_R1.set_brake_mode(m);
    DRIVE_MOTOR_R2.set_brake_mode(m);
    DRIVE_MOTOR_R3.set_brake_mode(m);
}

// Implement get_expo_drive_voltage
std::pair<double, double> get_expo_drive_voltage(double f, double t) {
    // Exponential drive calculation
    f = (pow(fabs(f), POWER_FACTOR) / pow(127, POWER_FACTOR - 1) * sgn(f));
    t = (pow(fabs(t), POWER_FACTOR) / pow(127, POWER_FACTOR - 1) * sgn(t));
    return std::pair<double, double>(f, t);
}
