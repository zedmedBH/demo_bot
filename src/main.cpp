#include "main.h"

ASSET(path_txt);//means path.txt

// --- Motors ---
// Standard PROS motor groups. (Negative ports reverse the motor)
MotorGroup leftMotors({1, 9});
MotorGroup rightMotors({-2, -10});

MotorGroup intakeMotors({8,-3});
Motor intakeLiftMotor(-5);
Motor trayLiftMotor(7);

// --- Drivetrain Configuration ---
Drivetrain drivetrain(&leftMotors, // left motor group
                      &rightMotors, // right motor group
                      12.0, // track width (12 inches)
                      Omniwheel::NEW_4, // using new 4" omnis
                      200, // drivetrain rpm
                      2); // horizontal drift (2 is standard for normal drive)

// --- Odometry Sensors ---
Imu imu(17); 
OdomSensors sensors(nullptr, // vertical tracking wheel 1
                    nullptr, // vertical tracking wheel 2
                    nullptr, // horizontal tracking wheel 1
                    nullptr, // horizontal tracking wheel 2
                    &imu); // inertial sensor

// --- PID Controllers ---
// Placeholder values - you will need to tune these!
//ControllerSettings(kP,kI,kD, windup, smallError, smallErrorTime, largeError, largeErrorTime, slew);
ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
ControllerSettings angular_controller(2, 0, 10, 3, 1, 100, 3, 500, 0);

// --- Initialize Chassis ---
Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);


/**
 * A callback function for LLEMU's center button.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        lcd::set_text(2, "I was pressed!");
    } else {
        lcd::clear_line(2);
    }
}

/**
 * Runs initialization code.
 */
void initialize() {
    lcd::initialize();
    lcd::set_text(1, "Hello BH Student!");
    lcd::register_btn1_cb(on_center_button);

    //Encoder Limits
    intakeLiftMotor.tare_position();
    trayLiftMotor.tare_position();

    //Set Brakes
    intakeLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    trayLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Calibrate the chassis (calibrates the IMU and starts odometry)
    chassis.calibrate(); 
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void follow_path(){
    // 1. Set Start Position
    // The X, Y, and Heading must exactly match the start of your JerryIO path!
    chassis.setPose(0, 0, 0); 

    // 2. Follow the Path
    // Parameters: (Asset Name, Lookahead Distance, Timeout in milliseconds)
    chassis.follow(path_txt, 15, 4000); 

    // 3. Wait for Completion
    // This prevents the robot from skipping to the next line of code before it finishes driving.
    chassis.waitUntilDone();
}

/**
 * Runs the operator control code.
 */
void opcontrol() {
    // Thanks to PROS_USE_SIMPLE_NAMES, we can just say CONTROLLER_MASTER
    Controller master(CONTROLLER_MASTER);

    //Speed Limits
    const int INTAKE_SPEED = 127;
    const int ARM_UP_SPEED = 100;
    const int ARM_DOWN_SPEED = 70;
    const int TRAY_OUT_SPEED = 50;
    const int TRAY_IN_SPEED = 70;


    //Encoders Limits (Hard Stops)
    const double LIFT_MAX_POS = 3000.0;
    const double TRAY_MAX_POS = 1750.0;

    while (true) {

        // --- Manual Drive Logic ---
        int throttle = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        chassis.curvature(throttle, turn);

        // --- Follow Predefined Path ---
        if (master.get_digital_new_press(DIGITAL_X)) {
            //follow_path();
            chassis.moveToPoint(0,24,2000);
        }

        //Intake Control
        if (master.get_digital(DIGITAL_R2)){
            intakeMotors.move(INTAKE_SPEED);
        }else if (master.get_digital(DIGITAL_L2)){
            intakeMotors.move(-INTAKE_SPEED);
        }else{
            intakeMotors.brake();
        }

        //Get Motor Positions
        double currentLiftPos = intakeLiftMotor.get_position();
        double currentTrayPos = trayLiftMotor.get_position();

        //Lift Arm
        if (master.get_digital(DIGITAL_R1) && currentLiftPos < LIFT_MAX_POS){
            intakeLiftMotor.move(ARM_UP_SPEED);
        }else if (master.get_digital(DIGITAL_L1) && currentLiftPos > 0){
            intakeLiftMotor.move(-ARM_DOWN_SPEED);
        }else{
            intakeLiftMotor.brake();
        }

        //Tray Motor
        if (master.get_digital(DIGITAL_UP) && currentTrayPos < TRAY_MAX_POS){
            trayLiftMotor.move(TRAY_OUT_SPEED);
        }else if (master.get_digital(DIGITAL_DOWN) && currentTrayPos > 0){
            trayLiftMotor.move(-TRAY_IN_SPEED);
        }else{
            trayLiftMotor.brake();
        }

        delay(10); 
    }
}