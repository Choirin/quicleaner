#!/usr/bin/env python
import ctypes


"""
typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAB
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  TWIST    twist;               //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_TWIST_COMMAND;
"""
class PacketTwistCommand(ctypes.Structure):
    _fields_ = (
        ('marker'     , ctypes.c_uint8 * 2),
        ('size'       , ctypes.c_uint8),
        ('reserved'   , ctypes.c_uint8),
        ('tw_linear'  , ctypes.c_float),
        ('tw_angular' , ctypes.c_float),
        ('sum'        , ctypes.c_uint8),
        ('reserved2'  , ctypes.c_uint8 * 3),
    )

"""
typedef struct{
    uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAC
    uint8_t  size;                //  1 bytes
    uint8_t  reserved;            //  1 bytes
    float    speed[2];            //  8 bytes
    uint8_t  sum;                 //  1 bytes
    uint8_t  reserved2[3];        //  3 bytes
}PACKET_SPEED;
"""
class PacketSpeed(ctypes.Structure):
    _fields_ = (
        ('marker'     , ctypes.c_uint8 * 2),
        ('size'       , ctypes.c_uint8),
        ('reserved'   , ctypes.c_uint8),
        ('speed'      , ctypes.c_float * 2),
        ('sum'        , ctypes.c_uint8),
        ('reserved2'  , ctypes.c_uint8 * 3),
    )

"""
typedef struct{
    uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAD
    uint8_t  size;                //  1 bytes
    uint8_t  reserved;            //  1 bytes
    uint16_t value[6];            // 12 bytes
    uint8_t  sum;                 //  1 bytes
    uint8_t  reserved2[3];        //  3 bytes
}PACKET_SENSOR;
"""
class PacketSensor(ctypes.Structure):
    _fields_ = (
        ('marker'     , ctypes.c_uint8 * 2),
        ('size'       , ctypes.c_uint8),
        ('reserved'   , ctypes.c_uint8),
        ('value'      , ctypes.c_uint16 * 6),
        ('sum'        , ctypes.c_uint8),
        ('reserved2'  , ctypes.c_uint8 * 3),
    )

"""
typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAE
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  TWIST    twist;               //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_TWIST;
"""
class PacketTwist(ctypes.Structure):
    _fields_ = (
        ('marker'     , ctypes.c_uint8 * 2),
        ('size'       , ctypes.c_uint8),
        ('reserved'   , ctypes.c_uint8),
        ('tw_linear'  , ctypes.c_float),
        ('tw_angular' , ctypes.c_float),
        ('sum'        , ctypes.c_uint8),
        ('reserved2'  , ctypes.c_uint8 * 3),
    )