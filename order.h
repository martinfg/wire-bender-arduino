#ifndef ORDER_H
#define ORDER_H

// Define the orders that can be sent and received
enum Order {
  HELLO =               0,
  ALREADY_CONNECTED =   1,
  RECEIVED =            2,
  ERR =                 3,
  ISALIVE =             4,
  STOP =                5,

  FEEDER =              10,
  BENDER =              11,
  ZAXIS =               12,
  PIN =                 13,

  SETHOMED =            20,
  ISHOMED =             21,
  DELHOMED =            22,
  ERR_NOTHOMED =        23,
  BEND =                25,
  CMD_EXECUTED =        29, 

  OVERWRITE_SETTINGS =          40,
  SET_FEEDING_CONSTANT =        41,
  SET_Z_ANGLE_CONSTANT =        42,
  SET_OFFSET_FOR_NEG_BEND =     43,
  SET_BEND_ANGLE_CONSTANT =     44,
  SET_NEG_BEND_ANGLE_CONSTANT = 45,
  SUCC_SETTINGS =               49
};

typedef enum Order Order;
#endif
