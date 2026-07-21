#ifndef BLOCKS_COMMON_H
#define BLOCKS_COMMON_H

#include "PxtShim.h"

// USB live-Blocks transport selection. There are two USB transports; build at
// most one (BLE always coexists). Each is overridable via a -D build define.
//
//   BLOCKS_USE_DAP    — CMSIS-DAP RAM-mailbox (the host pokes a mailbox in our
//                       RAM over the debug port; no UART). codal/mini-3 only:
//                       fast + reliable (USB CRC+retransmit), no 115200 limit.
//   BLOCKS_USE_SERIAL — legacy UART CDC serial transport. Slow/lossy; kept
//                       behind the flag for A/B but built OFF on codal.
//
// On codal (mini 3) DAP replaces serial. On v1/DAL neither is used (BLE-only;
// the J-Link interface can't do CMSIS-DAP, and there isn't memory for serial).
#ifndef BLOCKS_USE_DAP
#if MICROBIT_CODAL
#define BLOCKS_USE_DAP 1
#else // MICROBIT_CODAL
#define BLOCKS_USE_DAP 0
#endif // MICROBIT_CODAL
#endif // BLOCKS_USE_DAP

#ifndef BLOCKS_USE_SERIAL
#define BLOCKS_USE_SERIAL 0
#endif // BLOCKS_USE_SERIAL

// BLOCKS_SERIAL_PROBE — minimal UART detection responder ("is Blocks on this
// mini, and which version?"). Answers ONLY the widget's REQ_READ 0x0100
// handshake with the regular RES_READ frame (hardware/protocol/route/runtime
// bytes); it is NOT a comms transport: no broadcaster, no command handling,
// and it never latches serialConnected, so BLE/DAP comms behave exactly as
// without it. Works over the mini 3's DAPLink CDC and the mini 2's J-Link
// UART bridge — one reliable, transport-uniform detection path for the host.
// Enabled on codal (mini 3); on DAL only for the 32KB (mini 2) build — the
// 16KB mini 1 has no RAM to spare and stays BLE-only.
#ifndef BLOCKS_SERIAL_PROBE
#if MICROBIT_CODAL
#define BLOCKS_SERIAL_PROBE 1
#elif defined(YOTTA_CFG_MICROBIT_DAL_SRAM_END) && (YOTTA_CFG_MICROBIT_DAL_SRAM_END >= 0x20008000)
#define BLOCKS_SERIAL_PROBE 1
#else
#define BLOCKS_SERIAL_PROBE 0
#endif
#endif // BLOCKS_SERIAL_PROBE

// The full serial transport already answers the probe handshake itself — never
// run both RX consumers on one UART.
#if BLOCKS_USE_SERIAL && BLOCKS_SERIAL_PROBE
#undef BLOCKS_SERIAL_PROBE
#define BLOCKS_SERIAL_PROBE 0
#endif // BLOCKS_USE_SERIAL && BLOCKS_SERIAL_PROBE

// Start-of-frame delimiter for the framed USB transports (serial + DAP mailbox).
#define BLOCKS_SFD 0xff

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


// Guard window (ms) after a pin is (re)armed for edge/pulse events during which
// events from that pin are suppressed. Arming flips the pull-up and arms the
// nRF SENSE latch, which can register a spurious RISE/FALL with no real input.
// Re-arming on green-flag / reconnect (the config-state reconcile) would
// otherwise surface that phantom as a ghost pin event. Short (a digital edge
// settles fast, unlike capacitive touch) so a real press right after start is
// never dropped. The pin-event analogue of TOUCH_ARM_GUARD_MS.
#define PIN_EVENT_ARM_GUARD_MS 300

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
  // Whole 5x5 on/off image in ONE frame: data[1..4] = 25-bit column-major-by-row
  // bitmap (bit (row*5+col); 1 = LED full-on, 0 = off). Renders immediately, so
  // it is atomic — no PIXELS_0/PIXELS_1 split, hence no torn "half image" — and
  // one BLE round-trip instead of two. Used by the editor for the standard
  // on/off display block; the brightness path still uses PIXELS_0/PIXELS_1.
  // Fits the 20-byte MTU (5 bytes: id + 4 bitmap). Added in runtime v2.
  PIXELS_PACKED = 0x04,
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
