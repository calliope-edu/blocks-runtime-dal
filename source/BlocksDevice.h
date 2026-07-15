#ifndef BLOCKS_DEVICE_H
#define BLOCKS_DEVICE_H

#include "PxtShim.h"

#include "MicroBit.h"
#include "MicroBitConfig.h"

#include "BlocksCommon.h"

#if BLOCKS_USE_SERIAL
#include "BlocksSerial.h"
class BlocksSerial;
#endif // BLOCKS_USE_SERIAL

#if BLOCKS_USE_DAP
#include "BlocksDap.h"
class BlocksDap;
#endif // BLOCKS_USE_DAP

#if MICROBIT_CODAL
#include "BlocksService.h"
class BlocksService;
#else // MICROBIT_CODAL
#include "BlocksServiceDAL.h"
class BlocksServiceDAL;
using BlocksService = BlocksServiceDAL;
#endif // NOT MICROBIT_CODAL

#if MICROBIT_CODAL
#define LIGHT_LEVEL_SAMPLES_SIZE 11
#define ANALOG_IN_SAMPLES_SIZE 5
#else // NOT MICROBIT_CODAL
#define LIGHT_LEVEL_SAMPLES_SIZE 5
#define ANALOG_IN_SAMPLES_SIZE 5
#endif // NOT MICROBIT_CODAL

#if MICROBIT_CODAL
#define BLOCKS_WAITING_DATA_LABELS_LENGTH 16
#define BLOCKS_WAITING_DATA_LABEL_NOT_FOUND 0xff
#define BLOCKS_DATA_LABEL_SIZE 8
#define BLOCKS_DATA_CONTENT_SIZE 11
#endif // MICROBIT_CODAL

/**
 * @brief Button ID in Blocks
 * This number is used to memory offset in state data.
 */
enum BlocksButtonStateIndex
{
  // GPIO array using [0..20]
  P0 = 24,
  P1 = 25,
  P2 = 26,
  P3 = 27,
  A = 28,
  B = 29,
  LOGO = 30  
};

/**
 * @brief Version of this micro:bit
 * 
 */
enum BlocksHardwareVersion
{
  MICROBIT_V1 = 1,
  MICROBIT_V2 = 2,
};

/**
 * @brief Version of protocol to use
 *
 */
enum BlocksProtocol
{
  BLOCKS_V2 = 2,
};

/**
 * @brief Monotonic blocks-runtime (hex) version.
 *
 * Reported on connect in the COMMAND (0x0100) response at byte [3] over both
 * BLE and serial. Bump +1 on any editor-visible runtime change so the editor
 * can detect an outdated hex on the device and offer a re-flash. Keep in
 * lockstep with `blocks-version.json` beside the bundled hex in scratch-gui.
 * An old hex with no version byte reads as 0; 0 is reserved for "old/unknown"
 * and always treated as outdated.
 *
 * v2 (2026-07): single-frame packed on/off display (CMD_DISPLAY PIXELS_PACKED),
 * torn-frame generation tag on the 2-frame brightness path, and STATE/MOTION
 * push via NOTIFY. The editor feature-gates all three on runtimeVersion >= 2.
 */
#define BLOCKS_RUNTIME_VERSION 2

/**
 * Class definition for main logics of Micribit More Service except bluetooth connectivity.
 *
 */
class BlocksDevice {
private:
  /**
   * Constructor.
   * Create a representation of default extension for the blocks editor.
   * @param _uBit The instance of a MicroBit runtime.
   */
  BlocksDevice(MicroBit &_uBit);

  /**
   * @brief Destroy the BlocksDevice object
   *
   */
  ~BlocksDevice();

public:
  // setup the class as singleton
  BlocksDevice(const BlocksDevice &) = delete;
  BlocksDevice &operator=(const BlocksDevice &) = delete;
  BlocksDevice(BlocksDevice &&) = delete;
  BlocksDevice &operator=(BlocksDevice &&) = delete;

  /**
   * @brief Get the Instance object as singleton
   * 
   * @return BlocksDevice& 
   */
  static BlocksDevice &getInstance() {
    static BlocksDevice instance(pxt::uBit);
    return instance;
  }

  /**
   * @brief Microbit runtime
   * 
   */
  MicroBit &uBit;

  /**
   * @brief BLE service for basic micro:bit extension.
   *
   */
  BlocksService *basicService = nullptr;

  /**
   * @brief BLE service for Microbit More extension.
   *
   */
  BlocksService *moreService = nullptr;

#if BLOCKS_USE_SERIAL
  /**
   * @brief Microbit More serial port connector.
   *
   */
  BlocksSerial *serialService;
#endif // BLOCKS_USE_SERIAL

#if BLOCKS_USE_DAP
  /**
   * @brief CMSIS-DAP RAM-mailbox connector (codal USB transport).
   *
   */
  BlocksDap *dapService;
#endif // BLOCKS_USE_DAP

  // ---------------------

  /**
   * @brief Whether the serial port communication is started.
   *
   */
  bool serialConnected = false;

  /**
   * @brief Whether the CMSIS-DAP mailbox communication is started (host attached).
   *
   */
  bool dapConnected = false;

  /**
   * @brief Index of controllabel GPIO pins.
   * 
   */
#if MICROBIT_CODAL
  int gpioPin[11] = {0, 1, 2, 3, 8, 12, 13, 14, 15, 16, 17};
#else // NOT MICROBIT_CODAL
  // On nRF51/DAL `uBit.io.pin[17]` aliases P19 (an internal I2C/SCL line), not a
  // P17 edge pad — the mini 1/2 MicroBitIO has no P17/P18 member. Drop 17 so the
  // per-tick digital scan and pin commands never read/drive P19. Restores the
  // pre-rewrite MbitMore gpioPin set for mini 1/2.
  int gpioPin[10] = {0, 1, 2, 3, 8, 12, 13, 14, 15, 16};
#endif // NOT MICROBIT_CODAL

  /**
   * @brief Pins which is pull-up at connected.
   * 
   */
  int initialPullUp[4] = {0, 1, 2, 3};

  bool touchMode[4] = {false};

  // Sensing mode each touch pin is currently armed in: -1 = not armed,
  // 0 = Capacitative, 1 = Resistive. Lets CMD_CONFIG TOUCH be idempotent — the
  // editor re-sends the touch-enable on every program (re)start and after each
  // pinMode reset, and re-running isTouched(mode) re-attaches/re-calibrates the
  // codal TouchButton and keeps the shared cap-touch sampler churning (with >1
  // armed pad that hammers the SoftDevice radio IRQ). We only (re)arm on a
  // genuine off->on transition or an actual mode change.
  int8_t touchArmedMode[4] = {-1, -1, -1, -1};

  // Wall-clock (ms, system_timer_current_time) when each touch pad was last
  // (re)armed. Drives the TOUCH_ARM_GUARD_MS window in onButtonChanged that
  // suppresses post-arm calibration/settle phantom events (ghost touches).
  uint32_t touchArmTime[4] = {0, 0, 0, 0};

  // Per-pin armed edge/pulse event type (BlocksPinEventType: 0=NONE, 1=ON_EDGE,
  // 2=ON_PULSE), indexed by pin number. Retained so updateVersionData() can
  // report which pins are armed for events (COMMAND data[5..7] bitmap), letting
  // the editor reconcile + re-send a dropped CMD_PIN SET_EVENT — the event-pin
  // analogue of touchMode[]'s data[4] touch reconcile. Sized past the max GPIO
  // index (17); writes are isGpio-guarded in listenPinEventOn(). Live RAM, so
  // naturally 0 (all disarmed) after boot.
  int8_t pinEventMode[24] = {0};

  // Wall-clock (ms) when each pin was last (re)armed for edge/pulse events.
  // Drives the PIN_EVENT_ARM_GUARD_MS window in onPinEvent() that drops the
  // phantom edge produced by arming (pull-up flip / SENSE latch). Same size +
  // indexing as pinEventMode.
  uint32_t pinEventArmTime[24] = {0};

  // Wall-clock (ms) of the last BLE connect, for the CONNECT_GUARD_MS window in
  // onButtonChanged that drops the connect-time phantom click burst.
  uint32_t bleConnectTime = 0;

  /**
   * @brief Shadow screen to display on the LED.
   *
   */
  uint8_t shadowPixcels[5][5] = {{0}};

  // Generation tag of the last CMD_DISPLAY PIXELS_0 (top rows) received, or -1
  // if none / not tagged. The editor stamps a matching generation into PIXELS_0
  // and PIXELS_1 of the same frame; displayShadowPixels() runs on PIXELS_1 only
  // when its tag matches, so a dropped/reordered half is never rendered as a
  // torn image (new-bottom over old-top). Legacy (untagged) frames render
  // unconditionally. See onCommandReceived CMD_DISPLAY.
  int pendingDisplayGen = -1;

  /**
   * Samples of Light Level.
   */
  int lightLevelSamples[LIGHT_LEVEL_SAMPLES_SIZE] = {0};

  /**
   * @brief Last index of the Light Level Samples.
   *
   */
  size_t lightLevelSamplesLast = 0;

#if MICROBIT_CODAL
  /**
   * @brief Structure of received data in Blocks.
   * 
   */
  typedef struct {
    char label[BLOCKS_DATA_LABEL_SIZE];            /** label of the data */
    BlocksDataContentType type;                     /** type of the content */
    uint8_t content[BLOCKS_DATA_CONTENT_SIZE + 1]; /** content of the data */
  } BlocksLabeledData;

  /**
   * @brief Store of received data from the blocks editor.
   * 
   */
  BlocksLabeledData receivedData[BLOCKS_WAITING_DATA_LABELS_LENGTH] = {{{0}}};
#endif // MICROBIT_CODAL

  /**
   * Samples of Light Level.
   */
  // One median-filter sample row per analog pin P0..P3. MUST be 4 rows: the
  // ANALOG_IN round-robin samples index 0..3 (updateAnalogIn(buf, 3) indexes
  // analogInSamples[3]); sizing this [3] was an out-of-bounds write + in-place
  // median sort that clobbered the following members (blocksProtocol/pullMode).
  int analogInSamples[4][ANALOG_IN_SAMPLES_SIZE] = {{0}};

#if MICROBIT_CODAL
  /**
   * @brief On-board microphone is in use or not.
   * 
   */
  bool micInUse = false;

  /**
   * @brief Laudness on the microphone.
   * 
   */
  float soundLevel = 0.0;
#endif // MICROBIT_CODAL

  /**
   * Protocol of microbit more.
   */
  int blocksProtocol;

  /**
   * Current mode of all pins.
   */
  BlocksPullMode pullMode[sizeof(gpioPin) / sizeof(gpioPin[0])];

  /**
   * @brief Set pin configuration for initial.
   *
   */
  void initializeConfig();

  /**
   * @brief Soft-reset the blocks session: stop outputs, disarm touch, show the
   * name pattern, and be ready for fresh commands/events. No reboot.
   */
  void resetBlocksState();

  /**
   * @brief Show the device's name histogram pattern (static; identifies the mini).
   */
  void displayNamePattern();

  /**
   * @brief Update version data on the charactaristic.
   * 
   */
  void updateVersionData();

  /**
   * @brief Invoked when BLE connected.
   * 
   * @param _e event which has connection data
   */
  void onBLEConnected(MicroBitEvent _e);

  /**
   * @brief Invoked when BLE disconnected.
   * 
   * @param _e event which has disconnection data
   */
  void onBLEDisconnected(MicroBitEvent _e);

  /**
   * @brief Invoke when serial port connects.
   *
   */
  void onSerialConnected();

  /**
   * @brief Invoke when the CMSIS-DAP mailbox host attaches (first COMMAND read).
   *
   */
  void onDapConnected();

  /**
   * @brief Call when a command was received.
   *
   * @param data
   * @param length
   */
  void onCommandReceived(uint8_t *data, size_t length);

  /**
   * @brief Set the pattern on the line of the shadow pixels.
   *
   * @param line Index of the lines to set.
   * @param pattern Array of brightness(0..255) according columns.
   */
  void setPixelsShadowLine(int line, uint8_t *pattern);

  /**
   * @brief Display the shadow pixels on the LED.
   *
   */
  void displayShadowPixels();

  /**
   * @brief Display text on LED.
   *
   * @param text Contents to display with null termination.
   * @param delay The time to delay between characters, in milliseconds.
   */
  void displayText(char *text, int delay);

  /**
   * @brief Update GPIO and sensors state.
   *
   * @param data Buffer for BLE characteristics.
   */
  void updateState(uint8_t *data);

  /**
   * @brief Update data of motion.
   *
   * @param data Buffer for BLE characteristics.
   */
  void updateMotion(uint8_t *data);

  /**
   * @brief Get data of analog input of the pin.
   *
   * @param data Buffer for BLE characteristics.
   * @param pinIndex Index of the pin [0, 1, 2, 3].
   */
  void updateAnalogIn(uint8_t *data, size_t pinIndex);

  /**
   * @brief Sample current light level and return filtered value.
   *
   * @return int Filtered light level.
   */
  int sampleLightLevel();

  /**
   * @brief Set PMW signal to the speaker pin for play tone.
   * 
   * @param period  PWM period (1000000 / frequency)[us]
   * @param volume laudness of the sound [0..255]
   */
  void playTone(int period, int volume);

  /**
   * @brief Stop playing tone.
   * 
   */
  void stopTone();

#if MICROBIT_CODAL
  /**
   * @brief Return index for the label
   * 
   * @param dataLabel label to find
   * @param dataType type of the data
   * @return int index of the label
   */
  int findWaitingDataLabelIndex(const char *dataLabel, BlocksDataContentType dataType);

  /**
   * @brief Register data label and retrun ID for the label.
   *
   * @param dataLabel label to register
   * @param dataType type of the data
   * @return int ID for the label
   */
  int registerWaitingDataLabel(ManagedString dataLabel, BlocksDataContentType dataType);

  /**
   * @brief Get type of content for the labeled data
   *
   * @param labelID ID of the label in received data
   * @return content type
   */
  BlocksDataContentType dataType(int labelID);

  /**
   * @brief Return content of the data as number
   *
   * @param labelID ID of the label in received data
   * @return content of the data
   */
  float dataContentAsNumber(int labelID);

  /**
   * @brief Return content of the data as text
   *
   * @param labelID ID of the label in received data
   * @return content of the data
   */
  ManagedString dataContentAsText(int labelID);

  /**
   * @brief Send number with label.
   * 
   * @param dataLabel 
   * @param dataContent 
   */
  void sendNumberWithLabel(ManagedString dataLabel, float dataContent);

  /**
   * @brief Send text with label.
   * 
   * @param dataLabel 
   * @param dataContent 
   */
  void sendTextWithLabel(ManagedString dataLabel, ManagedString dataContent);

#endif // MICROBIT_CODAL

  /**
   * Callback. Invoked when a pin event sent.
   */
  void onPinEvent(MicroBitEvent evt);

  /**
   * @brief Display friendly name of the micro:bit.
   * 
   */
  void displayFriendlyName();

  /**
   * @brief Display software version of Microbit More.
   * 
   */
  void displayVersion();

private:
  /**
   * @brief Listen pin events on the pin.
   * Make it listen events of the event type on the pin.
   * Remove listener if the event type is MICROBIT_PIN_EVENT_NONE.
   * 
   * @param pinIndex index in edge pins
   * @param eventType type of events
   */
  void listenPinEventOn(int pinIndex, int eventType);

  /**
   * @brief Set pull-mode.
   * 
   * @param pinIndex index to set
   * @param pull pull-mode to set
   */
  void setPullMode(int pinIndex, BlocksPullMode pull);

  /**
   * @brief Set the value on the pin as digital output.
   * 
   * @param pinIndex index in edge pins
   * @param value digital value [0 | 1]
   */
  void setDigitalValue(int pinIndex, int value);

  /**
   * @brief Set the value on the pin as analog output (PWM).
   * 
   * @param pinIndex index in edge pins
   * @param value analog value (0..1024)
   */
  void setAnalogValue(int pinIndex, int value);

  /**
   * @brief Set the value on the pin as servo driver.
   * 
   * @param pinIndex index in edge pins
   * @param angle the level to set on the output pin, in the range 0 - 180.
   * @param range which gives the span of possible values the i.e. the lower and upper bounds (center +/- range/2). Defaults to DEVICE_PIN_DEFAULT_SERVO_RANGE.
   * @param center the center point from which to calculate the lower and upper bounds. Defaults to DEVICE_PIN_DEFAULT_SERVO_CENTER
   */
  void setServoValue(int pinIndex, int angle, int range, int center);

  /**
   * @brief Invoked when button state changed.
   * 
   * @param evt event which has button states
   */
  void onButtonChanged(MicroBitEvent evt);

  /**
   * @brief Invoked when gesture state changed.
   * 
   * @param evt event which has gesture states.
   */
  void onGestureChanged(MicroBitEvent evt);

  /**
   * @brief Normalize angle when upside down.
   * 
   * @param heading value of the compass heading
   * @return normalizes angle relative to north [degree]
   */
  int normalizeCompassHeading(int heading);

  /**
   * @brief Whether the pin is a GPIO of not.
   * 
   * @param pinIndex index in edge pins
   * @return true the pin is a GPIO
   * @return false the pin is not a GPIO
   */
  bool isGpio(int pinIndex);
};

#endif // BLOCKS_DEVICE_H
