#include "main.h"
#include "config.h"
#include "drive.h"
#include "pid.h"

#include "functions/math.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();

    IMU_LEFT.reset();
    IMU_RIGHT.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void wings(bool state) {
    WINGS_SOLENOID.set_value(state);
}

void intake(bool state) {
    INTAKE_SOLENOID.set_value(state);
}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void closeSide() {
    set_drive_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    runAuton = true; 
    pros::Task pidLoop(updatePID);
    // drive to top left
    target = 20.0;
    wait(200);
    move(80, 900);
    wait(200);
    target = -40.0;
    wait(200);
    move(80, 410);
    // intake top left
    intake(true);
    move(-80, 200);
    target = 90.0;
    // deposit top left
    wait(200);
    move(80, 550);
    wait(200);
    intake(false);
    wait(200);
    move(-80, 300);
    // go back for top middle
    wait(200);
    target = 10.0;
    wait(300);
    move(80, 425);
    wait(200);
    // intake top middle
    intake(true);
    wait(200);
    target = 135.0;
    // deposit top middle, push all 3 in net
    wait(200);
    intake(false);
    wings(true);
    wait(200);
    move(100, 1000);
    wait(200);
    move(-80, 200);
    wait(200);
    wings(false);
    wait(300);
    target = -135.0;
    wait(200);
    move(80, 1200);
    wait(200);
    target = -45.0;
    wait(200);
    move(80, 605);
    wait(200);
    intake(true);
    wait(200);
    move(-80, 715);
    wait(200);
    target = 100.0;
    wait(400);
    wings(true);
    intake(false);
    move(100, 800);
    wait(200);
    target = 30.0;
    move(80, 800);
    target = 80.0;
    wings(false);
    move(-80, 800);
    wait(100);
    wings(true);
    wait(200);
    move(127, 700);
    wings(false);
    target = 25.0;
    move(127, 400);
    wait(300);
    target = 5.0;
    move(127, 400);
    target = -90.0;
}
// move(80, 620);

// prog skills
void progSkills() {
    // AUTON SKILLS
    set_drive_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    runAuton = true;
    pros::Task pidLoop(updatePID);
    // move to pole postition
    target = 45.0;
    wait(150);
    move(80, 300);
    target = -20.0;
    move(60, 300);
    wings(true);
    CATA_MOTORS.move(-127);
    pros::delay(1000);
    wings(false);
    CATA_MOTORS.move(0);
    move(-60, 300);
    wait(200);
    // move across feild
    target = -145.0;
    wait(500);
    move(80, 700);
    wait(200);
    target = -180.0;
    wait(100);
    move(90, 1550);
    target = 135.0;
    wait(250);
    //push from side
   //wings(true);
    move(127, 500);
    target = 90.0;
    move(127, 700);
    move(-80, 500);
    wait(200);
    move(127, 700);
    move(-80, 500);
    target = 120.0;
    wait(200);
    move(127, 700);
    //move to front
    //wings(false);
    target = 200.0;//front push
    move(-110,1000);
    target = 90.0;
    move(60,800);
    target = 180.0;
    wings(true);
    move(100,1200);
    move(-50,500);
    target = 135.0;
    move(100,1200);
    move(-80,700);
    
}

void farSide() {
    target = 0.0;
    set_drive_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    runAuton = true;
    pros::Task pidLoop(updatePID);
    move(80, 1050);
    wait(100);       
    target = -40.0; 
    wait(100);
    intake(true);
    wait(200);
    target = 120.0;
    wait(600);
    intake(false);
    wait(200);
    target = -65.0;
    wait(200);
    move(80, 800);
    wait(100);
    intake(true);
    wait(100);
    target = 95.0;
    wait(400);
    intake(false);
    wings(true);
    move(127, 600);
    wings(false);
}

void autonomous() {
    closeSide();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // set brake mode to coast, so it doesn't shred the gearbox
    set_drive_brake_mode(MOTOR_BRAKE_COAST);
    runAuton = false;
    bool wings = false;
    bool intake = false;
    double drivebreak = 1.0;

    while (true) {
        double _x = master.get_analog(ANALOG_LEFT_Y);  // joystick up and down
        double _y = -master.get_analog(ANALOG_RIGHT_X) *
                    TURN_FACTOR;  // joystick left and right        // if
                                  // the joystick input is too small to
                                  // result in any significant movement
        // `fabs()` is the global floating point function
        if (fabs(_y) < JOYSTICK_Y_MIN) {
            _y = 0;
        }
        if (fabs(_x) < JOYSTICK_X_MIN) {  // TODO: tune
            _x = 0;
        }

        // currently using exponential drive, could convert to linear drive
        std::pair<double, double> _v = get_expo_drive_voltage(_y, _x);

        double fwd_power = std::get<0>(_v);
        double turn_power = std::get<1>(_v);
        if (master.get_digital(DIGITAL_L2)){
            drivebreak = 0.5;
        }
        else{
            drivebreak = 1.0;
        }
        drive_left((fwd_power + turn_power)*drivebreak);
        drive_right((fwd_power - turn_power)*drivebreak);

        // buttons
        if (master.get_digital(
                DIGITAL_R1)) {  // we flip the value of the bool every time the
                                // button is pressed
            WINGS_SOLENOID.set_value(true);
        } else {
            WINGS_SOLENOID.set_value(false);
        }
        if (master.get_digital(DIGITAL_R2)) {
            INTAKE_SOLENOID.set_value(true);
        } else {
            INTAKE_SOLENOID.set_value(false);
        }

        bool cata = master.get_digital(DIGITAL_L1);
        CATA_MOTORS.move(cata ? -127 : 0);

        double left_heading = IMU_LEFT.get_heading() * 365.0 / 360.0;
        double right_heading = IMU_RIGHT.get_heading();

        double heading = ((left_heading + right_heading) / 2);

        pros::lcd::print(0, "Heading: %f", heading);

        // odom test

        pros::delay(20);
    }
}
