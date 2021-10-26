#include <AccelStepper.h>
#include <Servo.h>
#include <microTuple.h>

#include "parameters.h"
#include "order.h"
#include "serial_comm.h"

void setup( void );
void loop( void );
static inline int8_t sgn( int val );
void initMotors();
void moveStepper();
void feedWire( int dist );
void moveStepper( AccelStepper stepper, int amount, int dir, int vel );

AccelStepper feederStepper(STEPPER_DRIVER, PIN_FEEDER_STEP, PIN_FEEDER_DIR);
AccelStepper zAxisStepper(STEPPER_DRIVER, PIN_ZAXIS_STEP, PIN_ZAXIS_DIR);
AccelStepper benderStepper(STEPPER_DRIVER, PIN_BENDER_STEP, PIN_BENDER_DIR);
Servo bendPin;

bool pinIsUp = false;
bool homed = false;
bool isConnected = false;
int serialValue;

int feedingConstant = FEEDING_CONSTANT;
int zAngleConstant = Z_ANGLE_CONSTANT;
int offsetForNegBend = OFFSET_FOR_NEG_BEND;
int bendAngleConstant = BEND_ANGLE_CONSTANT;
int negBendAngleConstant = NEG_BEND_ANGLE_CONSTANT;

void setup() {
  Serial.begin(SERIAL_BAUD);
  initMotors();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void loop() {
  get_messages_from_serial();
}

void initMotors() {
  bendPin.attach(SERVO_PIN);
  bendPin.write(BEND_PIN_DOWN_POS);
  feederStepper.setMaxSpeed(STEPPER_MAX_SPEED);
  zAxisStepper.setMaxSpeed(STEPPER_MAX_SPEED);
  benderStepper.setMaxSpeed(STEPPER_MAX_SPEED);
  feederStepper.setCurrentPosition(0);
  zAxisStepper.setCurrentPosition(0);
  benderStepper.setCurrentPosition(0);
}

void moveStepper(AccelStepper stepper, int amount, int vel) {
  int dir = sgn(amount);
  int currentPos = stepper.currentPosition();
  int targetPos = currentPos + amount;
  while (stepper.currentPosition() != targetPos) {
    stepper.setSpeed(vel * dir);
    stepper.run();
  }
  stepper.setCurrentPosition(0);
  delay(DELAY_BENDING_STEPS);
}

void bendPinUp() {
  bendPin.write(BEND_PIN_UP_POS);
  delay(500);
}

void bendPinDown() {
  bendPin.write(BEND_PIN_DOWN_POS);
  delay(500);
}

void feedWire(int dist) {
  dist = dist * feedingConstant;
  int velocity = FEEDING_SPEED;
  moveStepper(feederStepper, dist, velocity);
}

void rotateBender(int angle) {
  if (INVERT_ANGLES) angle = angle * -1;
  angle = angle * bendAngleConstant;
  int velocity = BENDING_SPEED;
  moveStepper(benderStepper, angle, velocity);
}

void rotateBender(int angle, int angleConstant) {
  if (INVERT_ANGLES) angle = angle * -1;
  angle = angle * angleConstant;
  int velocity = BENDING_SPEED;
  moveStepper(benderStepper, angle, velocity);
}

void rotateZAxis(int angle) {
  if (INVERT_ANGLES) angle = angle * -1;
  angle = angle * zAngleConstant;
  int velocity = ZROTATION_SPEED;
  moveStepper(zAxisStepper, angle, velocity);
}

void bendWire(int angle) {
  if (angle > 0) {
    bendPinUp();
    rotateBender(angle, bendAngleConstant);
    rotateBender(-angle, bendAngleConstant);
    bendPinDown();
  } else if (angle < 0) {
    bendPinDown();
    rotateBender(offsetForNegBend);
    bendPinUp();
    rotateBender(angle, negBendAngleConstant);
    rotateBender(-angle, negBendAngleConstant);
    bendPinDown();
    rotateBender(-offsetForNegBend);
  }
}

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

void get_messages_from_serial()
{
  if (Serial.available() > 0)
  {
    Order order_received = read_order();
    write_order(RECEIVED);
    if (order_received == HELLO) {
      if (!isConnected) {
        isConnected = true;
        write_order(HELLO);
      } else {
        write_order(ALREADY_CONNECTED);
      }
    } else if (order_received == ALREADY_CONNECTED) {
      isConnected = true;
    } else if (order_received == ISALIVE) {
      write_order(ISALIVE);
    } else {
      switch (order_received) {

        case FEEDER:
          serialValue = read_i16();
          feedWire(serialValue);
          if (DEBUG) {
            write_order(FEEDER);
            write_i16(serialValue);
          }
          write_order(CMD_EXECUTED);
          break;

        case BENDER:
          serialValue = read_i16();
          rotateBender(serialValue);
          if (DEBUG) {
            write_order(BENDER);
            write_i16(serialValue);
          }
          write_order(CMD_EXECUTED);
          break;

        case BEND:
          serialValue = read_i16();
          if (!homed) {
            write_order(ERR_NOTHOMED);
          } else {
            bendWire(serialValue);
          }
          write_order(CMD_EXECUTED);
          break;

        case ZAXIS:
          serialValue = read_i16();
          rotateZAxis(serialValue);
          if (DEBUG) {
            write_order(ZAXIS);
            write_i16(serialValue);
          }
          write_order(CMD_EXECUTED);
          break;

        case PIN:
          if (pinIsUp) {
            bendPinDown();
            pinIsUp = false;
          } else {
            bendPinUp();
            pinIsUp = true;
          }
          write_order(CMD_EXECUTED);
          break;

        case SETHOMED:
          benderStepper.setCurrentPosition(0);
          homed = true;
          write_order(ISHOMED);
          break;

        case DELHOMED:
          homed = false;
          write_order(ERR_NOTHOMED);
          break;

        case SET_FEEDING_CONSTANT:
          serialValue = read_i16();
          feedingConstant = serialValue;
          break;

        case SET_Z_ANGLE_CONSTANT:
          serialValue = read_i16();
          zAngleConstant = serialValue;
          break;

        case SET_OFFSET_FOR_NEG_BEND:
          serialValue = read_i16();
          offsetForNegBend = serialValue;
          break;

        case SET_BEND_ANGLE_CONSTANT:
          serialValue = read_i16();
          bendAngleConstant = serialValue;
          break;

        case SET_NEG_BEND_ANGLE_CONSTANT:
          serialValue = read_i16();
          negBendAngleConstant = serialValue;
          break;

        default:
          write_order(ERR);
          write_i16(404);
          return;
      }
    }
  }
}

Order read_order()
{
  return (Order) Serial.read();
}

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
  unsigned long startTime = millis();
  //Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)) {}
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t* buffer, size_t n)
{
  size_t i = 0;
  int c;
  while (i < n)
  {
    c = Serial.read();
    if (c < 0) break;
    *buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
    i++;
  }
}

int16_t read_i16()
{
  int8_t buffer[2];
  wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
  read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int32_t read_i32()
{
  int8_t buffer[4];
  wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
  read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

void write_order(enum Order myOrder)
{
  uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
}

void write_i16(int16_t num)
{
  int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2 * sizeof(int8_t));
}

//MicroTuple<int, int> parseSerialIn(String data) {
//  MicroTuple<int, int> instruction;
//  int command = ERR; // init instruction with error constant
//  int value = 0;     // and value 0
//  switch (data.charAt(0)) {
//
//    case 'f':
//      command = CMD_FEED;
//      if (data.charAt(1) == '-') {
//        value = data.substring(2, data.length()).toInt();
//      } else {
//        value = data.substring(1, data.length()).toInt();
//      }
//      break;
//
//     case 'x':
//      command = CMD_ROTATE_X;
//      if (data.charAt(1) == '-') {
//}
