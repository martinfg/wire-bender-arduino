#ifndef PARAMETERS_H
#define PARAMETERS_H

// Serial communication
#define SERIAL_BAUD 115200  // Baudrate

// Motors
// Pins
#define STEPPER_DRIVER 1
#define PIN_FEEDER_STEP 5
#define PIN_FEEDER_DIR 6
#define PIN_ZAXIS_STEP 7
#define PIN_ZAXIS_DIR 8
#define PIN_BENDER_STEP 9
#define PIN_BENDER_DIR 10
#define SERVO_PIN 2
#define INVERT_ANGLES true

// constants
#define BEND_PIN_DOWN_POS 75
#define BEND_PIN_UP_POS 0
#define BENDING_SPEED 1000
#define FEEDING_SPEED 500
#define ZROTATION_SPEED 500
#define STEPPER_MAX_SPEED 2000
#define DELAY_BENDING_STEPS 250

// (default) operational constants
#define FEEDING_CONSTANT 48
#define Z_ANGLE_CONSTANT 13 
#define OFFSET_FOR_NEG_BEND 50
#define BEND_ANGLE_CONSTANT 13 
#define NEG_BEND_ANGLE_CONSTANT 10 

// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG false

#endif
