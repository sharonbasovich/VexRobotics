#include "pid.h"
#include "config.h"
#include "drive.h"
#include "functions/math.h"


// namespace PID {

// Implement the _controller class constructor and compute function
PID::_controller::_controller(double f) : k(f) {}

// Implement the control_loop class constructor and update function
PID::control_loop::control_loop(std::vector<_controller*> v,
                                std::pair<double, double> m)
    : control_set(v) {
    min = std::get<0>(m);
    max = std::get<1>(m);
}

double PID::control_loop::update(double t, double c) {
    double output = 0;
    for (_controller* i : control_set) {
        output += i->compute(t, c);
    }
    return output;
    // return clamp(output, min, max);
}

// Implement the proportional class constructor and compute function
PID::proportional::proportional(double f, std::pair<double, double> m)
    : _controller(f) {
    max = std::get<0>(m);
    min = std::get<1>(m);
}

double PID::proportional::compute(double target, double current) {
    double v = k * (target - current);
    return v;
    // return clamp(v, min, max);
}

// Implement the integral class constructor and compute function
PID::integral::integral(double f, std::pair<double, double> m)
    : _controller(f) {
    max = std::get<0>(m);
    min = std::get<1>(m);
}

double PID::integral::compute(double target, double current) {
    if ((int)target == int(current)) {
        total_err = 0;
        return 0;
    }
    total_err += target - current;
    double v = k * total_err;
    return v;
}

// // Implement the derivative class constructor and compute function
// derivative::derivative(double f, std::pair<double, double> m) :
// _controller(f) {
//     max = std::get<0>(m);
//     min = std::get<1>(m);
// }

// double derivative::compute(double target, double current) {
//     double v = k * (target - current - prev_err);
//     prev_err = target - current;
//     return v;
//     // return clamp(v, min, max);
// }

// }  // namespace PID

double adiff(double a, double b) {
    double d = b - a;
    while (d < -180)
        d += 360;
    while (d > 180)
        d -= 360;
    return d;
}

class foo {
    public:
        void bar(); // these are methods because `bar` is associated with the `foo` class
    private:
        void foobar();
};

void bar(double foo);  // these are functions because they are not associated with methods
void foobar ();

    void
    updatePID() {
    while (runAuton) {
        double left_heading = IMU_LEFT.get_heading() * 365.0 / 360.0;
        double right_heading = IMU_RIGHT.get_heading();

        heading = ((left_heading + right_heading) / 2);
        double updated = calcPID(heading);
        pros::lcd::print(0, "Heading: %f", heading);
        pros::lcd::print(1, "error: %f", error);
        pros::lcd::print(2, "drive: %f", updated);
        // double turn_power = error * 0.4;
        drive_left(dist + updated);
        drive_right(-dist + updated);

        pros::delay(10);
    }
}
double calcPID(double heading) {
    error = adiff(target, heading);
    pros::lcd::print(1, "error: %f", error);

    double p = Kp * error;
    total_err += error;
    double i = Ki * total_err;
    double d = Kd * (error - prev_err);
    prev_err = error;

    return p + i + d;
}