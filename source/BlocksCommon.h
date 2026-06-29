#ifndef BLOCKS_COMMON_H
#define BLOCKS_COMMON_H

#include "PxtShim.h"

#if MICROBIT_CODAL
#define BLOCKS_USE_SERIAL 1 // 1 for use USB serial
#else // MICROBIT_CODAL
// v1 has not enough memory space
#define BLOCKS_USE_SERIAL 0 // 1 for use USB serial
#endif // MICROBIT_CODAL

#define BLOCKS_DATA_RECEIVED 8000

/**
 * Data type of content.
 */
enum BlocksDataContentType
{
  //% block="number"
  BLOCKS_DATA_NUMBER = 1,
  //% block="text"
  BLOCKS_DATA_TEXT = 2,
};

#define BLOCKS_CH_BUFFER_SIZE_COMMAND 20
#define BLOCKS_CH_BUFFER_SIZE_NOTIFY 20
#define BLOCKS_CH_BUFFER_SIZE_STATE 7
#define BLOCKS_CH_BUFFER_SIZE_MOTION 18
// Consolidated analog characteristic: P0..P3 packed as uint16 LE at offsets
// 0/2/4/6 in a single read.
#define BLOCKS_CH_BUFFER_SIZE_ANALOG_IN 8

// Guard window (ms) after a touch pad is (re)armed during which button events
// from that pad are suppressed. A freshly-armed capacitive TouchButton
// calibrates/settles over ~0.5-2s (longer with several pads) and can emit a
// phantom DOWN/UP with no real touch; dropping those prevents ghost touches.
#define TOUCH_ARM_GUARD_MS 4000

// Guard window (ms) after each BLE (re)connect during which ALL button/touch
// events are suppressed. On-device the connect sequence emits a phantom click
// burst across Button A AND the touch pads; this covers sources like Button A
// that don't calibrate and so aren't caught by the per-pad arm guard.
#define CONNECT_GUARD_MS 6000

enum BlocksCommand // 3 bits (0x00..0x07)
{
  CMD_CONFIG = 0x00,
  CMD_PIN = 0x01,
  CMD_DISPLAY = 0x02,
  CMD_AUDIO = 0x03,
  CMD_DATA = 0x04,
  CMD_RGB = 0x05,
  CMD_MOTOR = 0x06,
};

enum BlocksMotorCommand
{
  SET_M0 = 0x01,
  SET_M1 = 0x02,
  SET_M0_M1 = 0x03,
  SET_MOTIONKIT_LEFT = 0x04,
  SET_MOTIONKIT_RIGHT = 0x05,
  SET_MOTIONKIT_BOTH = 0x06,
};

enum BlocksPinCommand
{
  SET_OUTPUT = 0x01,
  SET_PWM = 0x02,
  SET_SERVO = 0x03,
  SET_PULL = 0x04,
  SET_EVENT = 0x05,
};

enum BlocksDisplayCommand
{
  CLEAR = 0x00,
  TEXT = 0x01,
  PIXELS_0 = 0x02,
  PIXELS_1 = 0x03,
};

/**
 * @brief Enum for write mode of display pixels.
 */
enum BlocksDisplayWriteMode
{
  LAYER = 0,
  OVER_WRITE = 1
};

enum BlocksPullMode
{
  None = 0,
  Down = 1,
  Up = 2,
};

enum BlocksDataFormat
{
  CONFIG = 0x10, // not used at this version
  PIN_EVENT = 0x11,
  ACTION_EVENT = 0x12,
  DATA_NUMBER = 0x13,
  DATA_TEXT = 0x14
};

enum BlocksActionEvent
{
  BUTTON = 0x01,
  GESTURE = 0x02
};

enum BlocksButtonEvent
{
  DOWN = 1,
  UP = 2,
  CLICK = 3,
  LONG_CLICK = 4,
  HOLD = 5,
  DOUBLE_CLICK = 6
};

enum BlocksGestureEvent
{
  TILT_UP = 1,
  TILT_DOWN = 2,
  TILT_LEFT = 3,
  TILT_RIGHT = 4,
  FACE_UP = 5,
  FACE_DOWN = 6,
  FREEFALL = 7,
  G3 = 8,
  G6 = 9,
  G8 = 10,
  SHAKE = 11
};

enum BlocksPinEventType
{
  NONE = 0,
  ON_EDGE = 1,
  ON_PULSE = 2,
  ON_TOUCH = 3
};

enum BlocksPinEvent
{
  RISE = 2,
  FALL = 3,
  PULSE_HIGH = 4,
  PULSE_LOW = 5
};

/**
 * @brief Enum for sub-command about configurations.
 * 
 */
enum BlocksConfig
{
  MICPIN = 0x01, // microphone
  TOUCH = 0x02,
  RESET = 0x03 // soft-reset the blocks session (stop outputs, disarm touch, show name pattern)
};

/**
 * @brief Enum for sub-commands about audio.
 * 
 */
enum BlocksAudioCommand
{
  STOP_TONE = 0x00,
  PLAY_TONE = 0x01,
};

#endif // BLOCKS_COMMON_H
