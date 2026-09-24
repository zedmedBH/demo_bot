#include "main.h"

ASSET(path_txt);

// --- Motors & Sensors ---
MotorGroup leftMotors({1, 9});
MotorGroup rightMotors({-2, -10});
MotorGroup intakeMotors({8,-3});
Motor intakeLiftMotor(-5);
Motor trayLiftMotor(7);
pros::AIVision ai_sensor(21); 

// --- Drivetrain Configuration ---
Drivetrain drivetrain(&leftMotors, &rightMotors, 12.0, Omniwheel::NEW_4, 200, 2); 
Imu imu(17);  
OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu); 
ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
ControllerSettings angular_controller(2, 0, 10, 3, 1, 100, 3, 500, 0);
Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// =========================================================
// ACTION FUNCTIONS (STUDENTS: EDIT THE CODE INSIDE THESE)
// =========================================================

// Called when in AprilTag Mode
void process_apriltag(int tag_id, double pixel_error, double distance_inches) {
    if (tag_id == 1) { // Example: Tag 1 is the Red Station
        if (distance_inches > 15.0) {
            pros::lcd::print(5, "Tag 1: Driving to Red Station ");
            
            // PROPORTIONAL STEERING EXAMPLE:
            // Multiply the pixel error by a small constant to get a turn speed.
            // If the error is 50 pixels right, turn speed = 50 * 0.3 = 15
            double turn_speed = pixel_error * 0.3; 
            int forward_speed = 50; 
            
            // Drive forward while adjusting the turn to stay centered
            chassis.arcade(forward_speed, turn_speed); 
            
        } else {
            pros::lcd::print(5, "Tag 1: Arrived at Red Station!");
            leftMotors.brake();
            rightMotors.brake();
        }
    } else {
        pros::lcd::print(5, "Unknown Tag: %d               ", tag_id);
    }
}

// Called when in AI Object Mode
void process_ai_object(int class_id, double pixel_error) {
    // 1 = Red Ring, 2 = Blue Ring, 3 = Mobile Goal
    if (class_id == 1) {
        pros::lcd::print(5, "AI: Chasing RED Ring!         ");
    } else if (class_id == 2) {
        pros::lcd::print(5, "AI: Avoiding BLUE Ring!       ");
    } else if (class_id == 3) {
        pros::lcd::print(5, "AI: Aligning to Mobile Goal!  ");
    } else {
        pros::lcd::print(5, "AI: Unknown Object ID: %d     ", class_id);
    }
}

// =========================================================
// STANDARD PROS FUNCTIONS
// =========================================================

void on_center_button() { /* Keep your original logic here */ }

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello BH Student!");
    
    intakeLiftMotor.tare_position();
    trayLiftMotor.tare_position();
    intakeLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    trayLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    chassis.calibrate();
    
    pros::delay(500); 

    printf("--- AI Vision Loaded Classes ---\n");
    
    // Loop through the first 5 potential IDs to see what the sensor is trained on
    for (int i = 1; i <= 5; i++) {
        auto name = ai_sensor.get_class_name(i);
        if (name.has_value()) {
            printf("Class ID %d is: %s\n", i, name.value().c_str());
        }
    }
}

void disabled() {}
void competition_initialize() {}
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.follow(path_txt, 15, 4000);
    chassis.waitUntilDone();
}

// =========================================================
// OPCONTROL LOOP
// =========================================================

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    // Limits and Speeds
    const int INTAKE_SPEED = 127;
    const int ARM_UP_SPEED = 100;
    const int ARM_DOWN_SPEED = 70;
    const int TRAY_OUT_SPEED = 50;
    const int TRAY_IN_SPEED = 70;
    const double LIFT_MAX_POS = 3000.0;
    const double TRAY_MAX_POS = 1750.0;

    // Vision State Machine
    enum VisionMode { MODE_OFF, MODE_APRILTAG, MODE_AI_OBJECT };
    VisionMode current_mode = MODE_OFF;
    
    const double SCREEN_CENTER_X = 160.0;
    const double DISTANCE_CONSTANT = 1200.0; 

    // Start with vision disabled
    ai_sensor.disable_detection_types(pros::AivisionModeType::all);

    while (true) {
        // --- 1. VISION MODE TOGGLES ---
        // Using LEFT, RIGHT, and A buttons so we don't conflict with your intake/lift controls
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            current_mode = MODE_APRILTAG;
            ai_sensor.disable_detection_types(pros::AivisionModeType::all);
            ai_sensor.enable_detection_types(pros::AivisionModeType::tags);
            ai_sensor.set_tag_family(pros::AivisionTagFamily::tag_21H7);
            pros::lcd::print(2, "Mode: APRILTAG NAVIGATION ");
            
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            current_mode = MODE_AI_OBJECT;
            ai_sensor.disable_detection_types(pros::AivisionModeType::all);
            ai_sensor.enable_detection_types(pros::AivisionModeType::objects);
            pros::lcd::print(2, "Mode: AI ELEMENT CHASING  ");
            
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            current_mode = MODE_OFF;
            ai_sensor.disable_detection_types(pros::AivisionModeType::all);
            pros::lcd::print(2, "Mode: VISION OFF          ");
            pros::lcd::print(5, "                                 "); 
        }

        // --- 2. VISION PROCESSING ---
        auto objects = ai_sensor.get_all_objects();
        
        if (objects.size() > 0 && current_mode != MODE_OFF) {
            auto primary_obj = objects[0];
            double tag_center_x = primary_obj.object.element.xoffset + (primary_obj.object.element.width / 2.0);
            double pixel_error = tag_center_x - SCREEN_CENTER_X;
            
            switch (current_mode) {
                case MODE_APRILTAG:
                    if (pros::AIVision::is_type(primary_obj, pros::AivisionDetectType::tag)) {
                        double actual_distance = DISTANCE_CONSTANT / primary_obj.object.element.width;
                        process_apriltag(primary_obj.id, pixel_error, actual_distance);
                    }
                    break;
                    
                case MODE_AI_OBJECT:
                    if (pros::AIVision::is_type(primary_obj, pros::AivisionDetectType::object)) {
                        process_ai_object(primary_obj.id, pixel_error);
                    }
                    break;
                    
                case MODE_OFF: break;
            }
        } else {
            if (current_mode != MODE_OFF) {
               pros::lcd::print(5, "Action: Target lost...           "); 
            }
        }

        // --- 3. MANUAL DRIVE OVERRIDE ---
        // If vision is OFF, allow normal joystick driving.
        if (current_mode == MODE_OFF) {
            int throttle = master.get_analog(ANALOG_LEFT_Y);
            int turn = master.get_analog(ANALOG_RIGHT_X);
            chassis.curvature(throttle, turn);
        }

        // --- 4. YOUR EXISTING INTAKE/LIFT CONTROLS ---
        if (master.get_digital_new_press(DIGITAL_X)) {
            chassis.moveToPoint(0,24,2000);
        }

        if (master.get_digital(DIGITAL_R2)){
            intakeMotors.move(INTAKE_SPEED);
        } else if (master.get_digital(DIGITAL_L2)){
            intakeMotors.move(-INTAKE_SPEED);
        } else {
            intakeMotors.brake();
        }

        double currentLiftPos = intakeLiftMotor.get_position();
        double currentTrayPos = trayLiftMotor.get_position();

        if (master.get_digital(DIGITAL_R1) && currentLiftPos < LIFT_MAX_POS){
            intakeLiftMotor.move(ARM_UP_SPEED);
        } else if (master.get_digital(DIGITAL_L1) && currentLiftPos > 0){
            intakeLiftMotor.move(-ARM_DOWN_SPEED);
        } else {
            intakeLiftMotor.brake();
        }

        if (master.get_digital(DIGITAL_UP) && currentTrayPos < TRAY_MAX_POS){
            trayLiftMotor.move(TRAY_OUT_SPEED);
        } else if (master.get_digital(DIGITAL_DOWN) && currentTrayPos > 0){
            trayLiftMotor.move(-TRAY_IN_SPEED);
        } else {
            trayLiftMotor.brake();
        }

        pros::delay(20); 
    }
}