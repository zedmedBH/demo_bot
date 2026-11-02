#include "main.h"

// --- Motors ---
// Standard PROS motor groups. (Negative ports reverse the motor)
MotorGroup leftMotors({1, 9});
MotorGroup rightMotors({-2, -10});

MotorGroup intakeMotors({8,-3});
Motor intakeLiftMotor(-5);
Motor trayLiftMotor(7);

AIVision ai_vision(20);

AIVision::Color color_purple = {
    .id = 1,
    .red = 50,
    .green = 17,
    .blue = 98,
    .hue_range = 22.0,
    .saturation_range = 0.67
};

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

    ai_vision.set_color(color_purple);

    ai_vision.enable_detection_types(pros::AivisionModeType::colors);

}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

/**
 * Runs the operator control code.
 */
void opcontrol() {
    // Thanks to PROS_USE_SIMPLE_NAMES, we can just say CONTROLLER_MASTER
    Controller master(CONTROLLER_MASTER);

    const double kP = 0.5;
    const int CENTER_FOV_X = 160;

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

         auto objects = ai_vision.get_all_objects();
         bool found_purple = false;
         double target_x = 0;

         for (auto &obj : objects){

            if(obj.id == 1){
                target_x = obj.object.color.xoffset + (obj.object.color.width/2.0);

                pros::lcd::print(1," ID:%d CenterX:%.1f",
                    obj.id,
                    target_x
                );
                found_purple = true;
                break;
            }
         }

         if (!found_purple){
            pros::lcd::set_text(1,"No purple cube detected!");
         }


         //Turn robot to center on the purple object
         if (master.get_digital(DIGITAL_LEFT) && found_purple){
            double error = CENTER_FOV_X - target_x;
            int turn_speed = 0;

            //get center but within 15 pixels
            if(std::abs(error) > 15 ){
                turn_speed = error * kP;
            }

            chassis.arcade(0,turn_speed);
         } else{
            int throttle = master.get_analog(ANALOG_LEFT_Y);
            int turn = master.get_analog(ANALOG_RIGHT_X);

            chassis.arcade(throttle,turn);
         }


        // --- Manual Drive Logic ---
 /*       int throttle = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        chassis.curvature(throttle, turn);

        // --- Metric Movement Test (~30cm) ---
        if (master.get_digital_new_press(DIGITAL_X)) {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 11.81, 2000);
            chassis.waitUntilDone();
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
        }*/

        delay(100); 
    }
}