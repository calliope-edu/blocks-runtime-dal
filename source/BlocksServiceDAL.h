#include "PxtShim.h"

#include "MicroBit.h"
#include "MicroBitConfig.h"

#if !MICROBIT_CODAL

#ifndef BLOCKS_SERVICE_DAL_H
#define BLOCKS_SERVICE_DAL_H

#include "BlocksCommon.h"
#include "BlocksDevice.h"

// Forward declaration
class BlocksDevice;

/**
 * Class definition for the Blocks editor BLE service on micro:bit DAL (nRF51 /
 * Calliope mini 1/2). DAL/mbed-BLE counterpart of BlocksService (codal / mini
 * 3): exposes the SAME consolidated 5-characteristic Blocks protocol over the
 * legacy GattCharacteristic API, so scratch-vm talks to mini 1/2 and mini 3
 * identically.
 *
 * Consolidated 5-characteristic protocol (was 9 on the pre-rewrite MbitMore
 * DAL service): the pin- and action-event channels collapse into one notify
 * characteristic (SENSOR_EVENT) demuxed editor-side by data[19], and the four
 * per-pin analog characteristics collapse into one read characteristic sampled
 * round-robin across P0..P3.
 */
class BlocksServiceDAL {
public:
  /**
   * Constructor. Registers the GATT service + characteristics and wires the
   * BlocksDevice singleton.
   */
  BlocksServiceDAL();

  /**
   * Invoked (via the message bus) when BLE connects.
   */
  void onBLEConnected(MicroBitEvent _e);

  void notify();

  /**
   * @brief Notify a button/gesture (action) event on SENSOR_EVENT.
   */
  void notifyActionEvent();

  /**
   * @brief Notify a pin event on SENSOR_EVENT.
   */
  void notifyPinEvent();

  /**
   * @brief Notify a data message on SENSOR_EVENT.
   */
  void notifyData();

  /**
   * Callback. Invoked when the unified ANALOG_IN characteristic is read via BLE.
   * Samples one pin per read, round-robin across P0..P3.
   */
  void onReadAnalogIn(GattReadAuthCallbackParams *authParams);

  /**
   * Callback. Invoked when the COMMAND characteristic is read via BLE. Re-stamps
   * the live version/handshake bytes (updateVersionData) and returns the buffer,
   * so the editor's periodic version re-read always sees the current runtime
   * version. On DAL the SoftDevice keeps its own copy of the value, so a plain
   * READ would otherwise return the last COMMAND *write* (a command, byte[3]≈0)
   * and the editor would report the device as "outdated v0".
   */
  void onReadCommand(GattReadAuthCallbackParams *authParams);

  /**
   * Callback. Invoked when any of our attributes are written via BLE.
   */
  void onDataWritten(const GattWriteCallbackParams *params);

  void update();

  /**
   * True while a BLE central is connected.
   */
  bool isBleConnected();

  // Buffer of characteristic for receiving commands (and serving the version
  // handshake read; see BlocksDevice::updateVersionData).
  uint8_t commandChBuffer[BLOCKS_CH_BUFFER_SIZE_COMMAND] = {0};

  // Buffer of characteristic for sending data of GPIO and sensors state.
  uint8_t stateChBuffer[BLOCKS_CH_BUFFER_SIZE_STATE] = {0};

  // Buffer of characteristic for sending data about motion.
  uint8_t motionChBuffer[BLOCKS_CH_BUFFER_SIZE_MOTION] = {0};

  // Staging buffers for the unified SENSOR_EVENT notify channel. Pin events,
  // button/gesture (action) events and data messages are each staged in their
  // own buffer by BlocksDevice but notified on the one SENSOR_EVENT
  // characteristic; the editor demuxes by data[19].
  uint8_t pinEventChBuffer[BLOCKS_CH_BUFFER_SIZE_NOTIFY] = {0};
  uint8_t actionEventChBuffer[BLOCKS_CH_BUFFER_SIZE_NOTIFY] = {0};
  uint8_t dataChBuffer[BLOCKS_CH_BUFFER_SIZE_NOTIFY] = {0};

  // Buffer of the unified analog-input characteristic: P0..P3 as uint16 LE at
  // offsets 0/2/4/6.
  uint8_t analogInChBuffer[BLOCKS_CH_BUFFER_SIZE_ANALOG_IN] = {0};

  // Round-robin cursor for on-demand analog sampling (P0..P3): one pin sampled
  // per ANALOG_IN read.
  uint8_t analogReadIndex = 0;

private:
  /**
   * @brief micro:bit runtime object.
   */
  MicroBit &uBit;

  /**
   * @brief Blocks device logic object.
   */
  BlocksDevice *blocks;

  GattCharacteristic *commandCh;
  GattCharacteristic *stateCh;
  GattCharacteristic *motionCh;
  GattCharacteristic *sensorEventCh;
  GattCharacteristic *analogInCh;
};

#endif // BLOCKS_SERVICE_DAL_H
#endif // !MICROBIT_CODAL
