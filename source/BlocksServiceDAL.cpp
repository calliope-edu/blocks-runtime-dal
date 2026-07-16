#include "PxtShim.h"

#include "MicroBit.h"
#include "MicroBitConfig.h"

#if !MICROBIT_CODAL

#include "MicroBitButton.h"

#include "BlocksServiceDAL.h"

/**
 * @brief Service ID of the Blocks editor runtime.
 *        0b50f3e4-607f-4151-9091-7d008d6ffc5c
 */
const uint8_t BLOCKS_SERVICE[] = {0x0b, 0x50, 0xf3, 0xe4, 0x60, 0x7f, 0x41, 0x51, 0x90, 0x91, 0x7d, 0x00, 0x8d, 0x6f, 0xfc, 0x5c};

/**
 * @brief Characteristics of the Blocks editor service (consolidated 5-char
 *        protocol). The middle two bytes carry the 16-bit characteristic id;
 *        these match the codal BlocksService::charUUID exactly so scratch-vm
 *        talks to mini 1/2 and mini 3 over the identical GATT layout.
 */
const uint8_t BLOCKS_CH_COMMAND[]      = {0x0b, 0x50, 0x01, 0x00, 0x60, 0x7f, 0x41, 0x51, 0x90, 0x91, 0x7d, 0x00, 0x8d, 0x6f, 0xfc, 0x5c};
const uint8_t BLOCKS_CH_STATE[]        = {0x0b, 0x50, 0x01, 0x01, 0x60, 0x7f, 0x41, 0x51, 0x90, 0x91, 0x7d, 0x00, 0x8d, 0x6f, 0xfc, 0x5c};
const uint8_t BLOCKS_CH_MOTION[]       = {0x0b, 0x50, 0x01, 0x02, 0x60, 0x7f, 0x41, 0x51, 0x90, 0x91, 0x7d, 0x00, 0x8d, 0x6f, 0xfc, 0x5c};
// Unified notify channel: pin events + action events (+ data), demuxed by data[19].
const uint8_t BLOCKS_CH_SENSOR_EVENT[] = {0x0b, 0x50, 0x01, 0x10, 0x60, 0x7f, 0x41, 0x51, 0x90, 0x91, 0x7d, 0x00, 0x8d, 0x6f, 0xfc, 0x5c};
// Unified analog read channel: P0..P3 packed, sampled round-robin per read.
const uint8_t BLOCKS_CH_ANALOG_IN[]    = {0x0b, 0x50, 0x01, 0x20, 0x60, 0x7f, 0x41, 0x51, 0x90, 0x91, 0x7d, 0x00, 0x8d, 0x6f, 0xfc, 0x5c};

/**
 * Constructor.
 * Create a representation of the Blocks editor BLE service.
 */
BlocksServiceDAL::BlocksServiceDAL() : uBit(pxt::uBit) {
  blocks = &BlocksDevice::getInstance();
  blocks->moreService = this;

  commandCh = new GattCharacteristic(
      BLOCKS_CH_COMMAND, commandChBuffer, BLOCKS_CH_BUFFER_SIZE_COMMAND, BLOCKS_CH_BUFFER_SIZE_COMMAND,
      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
          GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE |
          GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ);
  commandCh->setReadAuthorizationCallback(this, &BlocksServiceDAL::onReadCommand);
  commandCh->requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);

  // STATE + MOTION are READ + NOTIFY (runtime v2): the device pushes them ~every
  // 57ms from update() so a subscribed editor need not poll them with 2 GATT
  // reads per tick. READ is kept for older editors. Mirrors codal BlocksService.
  // (mini 2 = 32KB nRF51, so the extra CCCDs are no longer a RAM concern.)
  stateCh = new GattCharacteristic(
      BLOCKS_CH_STATE, (uint8_t *)&stateChBuffer,
      BLOCKS_CH_BUFFER_SIZE_STATE, BLOCKS_CH_BUFFER_SIZE_STATE,
      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
          GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
  stateCh->requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);

  motionCh = new GattCharacteristic(
      BLOCKS_CH_MOTION, (uint8_t *)&motionChBuffer,
      BLOCKS_CH_BUFFER_SIZE_MOTION, BLOCKS_CH_BUFFER_SIZE_MOTION,
      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
          GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
  motionCh->requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);

  // Unified notify channel: pin events, button/gesture (action) events and
  // data messages are all notified here and demuxed editor-side by data[19].
  // Staged from pinEventChBuffer on registration; each notify() overrides the
  // value with the relevant staging buffer.
  sensorEventCh = new GattCharacteristic(
      BLOCKS_CH_SENSOR_EVENT, (uint8_t *)&pinEventChBuffer,
      BLOCKS_CH_BUFFER_SIZE_NOTIFY, BLOCKS_CH_BUFFER_SIZE_NOTIFY,
      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
          GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
  sensorEventCh->requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);

  // Unified analog read channel: P0..P3 sampled on demand (round-robin) into
  // one 8-byte payload via the read-authorization callback.
  analogInCh = new GattCharacteristic(
      BLOCKS_CH_ANALOG_IN, (uint8_t *)&analogInChBuffer,
      BLOCKS_CH_BUFFER_SIZE_ANALOG_IN, BLOCKS_CH_BUFFER_SIZE_ANALOG_IN,
      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ);
  analogInCh->setReadAuthorizationCallback(this, &BlocksServiceDAL::onReadAnalogIn);
  analogInCh->requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);

  GattCharacteristic *blocksChs[] = {
      commandCh,
      stateCh,
      motionCh,
      sensorEventCh,
      analogInCh,
  };

  uBit.messageBus.listen(
      MICROBIT_ID_BLE,
      MICROBIT_BLE_EVT_CONNECTED,
      this,
      &BlocksServiceDAL::onBLEConnected,
      MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);

  GattService blocksService(BLOCKS_SERVICE, blocksChs,
                            sizeof(blocksChs) / sizeof(GattCharacteristic *));
  uBit.ble->addService(blocksService);

  // Setup callbacks for events.
  uBit.ble->onDataWritten(this, &BlocksServiceDAL::onDataWritten);
}

/**
 * Invoked when BLE connected.
 */
void BlocksServiceDAL::onBLEConnected(MicroBitEvent _e) {
  blocks->updateVersionData();
  uBit.ble->gattServer().write(commandCh->getValueHandle(), commandChBuffer,
                               BLOCKS_CH_BUFFER_SIZE_COMMAND);
}

/**
 * Callback. Invoked when the unified ANALOG_IN characteristic is read via BLE.
 * Samples ONE pin per read, round-robin across P0..P3, into its slot of the
 * 8-byte payload (uint16 LE at 0/2/4/6). One pin per read keeps each read
 * cheap; the editor reads repeatedly, so all four slots refresh within a few
 * reads. Mirrors the codal BlocksService::onDataRead path.
 */
void BlocksServiceDAL::onReadAnalogIn(GattReadAuthCallbackParams *authParams) {
  if (authParams->handle == analogInCh->getValueHandle()) {
    blocks->updateAnalogIn(analogInChBuffer, analogReadIndex);
    analogReadIndex++;
    if (analogReadIndex > 3) analogReadIndex = 0;
    authParams->data = (uint8_t *)&analogInChBuffer;
    authParams->offset = 0;
    authParams->len = BLOCKS_CH_BUFFER_SIZE_ANALOG_IN;
    authParams->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
  }
}

/**
 * Callback. Invoked when the COMMAND characteristic is read via BLE. Re-stamps
 * the live version/handshake bytes and returns the buffer so the editor's
 * periodic version re-read reports the current runtime version (see header).
 */
void BlocksServiceDAL::onReadCommand(GattReadAuthCallbackParams *authParams) {
  if (authParams->handle == commandCh->getValueHandle()) {
    blocks->updateVersionData();
    authParams->data = (uint8_t *)&commandChBuffer;
    authParams->offset = 0;
    authParams->len = BLOCKS_CH_BUFFER_SIZE_COMMAND;
    authParams->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
  }
}

/**
 * Callback. Invoked when any of our attributes are written via BLE.
 */
void BlocksServiceDAL::onDataWritten(const GattWriteCallbackParams *params) {
  if (params->handle == commandCh->getValueHandle() && params->len > 0) {
    blocks->onCommandReceived((uint8_t *)params->data, params->len);
  }
}

/**
 * @brief Notify a button/gesture (action) event on the unified SENSOR_EVENT
 *        characteristic.
 */
void BlocksServiceDAL::notifyActionEvent() {
  if (!isBleConnected())
    return;
  uBit.ble->gattServer().notify(sensorEventCh->getValueHandle(),
                                actionEventChBuffer, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
}

/**
 * @brief Notify a pin event on the unified SENSOR_EVENT characteristic.
 */
void BlocksServiceDAL::notifyPinEvent() {
  if (!isBleConnected())
    return;
  uBit.ble->gattServer().notify(sensorEventCh->getValueHandle(),
                                pinEventChBuffer, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
}

/**
 * @brief Notify a data message on the unified SENSOR_EVENT characteristic.
 */
void BlocksServiceDAL::notifyData() {
  if (!isBleConnected())
    return;
  uBit.ble->gattServer().notify(sensorEventCh->getValueHandle(),
                                dataChBuffer, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
}

/**
 * Notify data to the blocks editor.
 */
void BlocksServiceDAL::notify() {}

/**
 * True while a BLE central is connected.
 */
bool BlocksServiceDAL::isBleConnected() {
  return uBit.ble->gap().getState().connected;
}

/**
 * Update all GPIO and sensors state. Pushed to the stack each broadcaster tick
 * so a central read always sees fresh values. When disconnected this is a
 * no-op: the device shows the static name pattern (set by resetBlocksState),
 * matching the codal BlocksService idle behaviour — no display takeover.
 */
void BlocksServiceDAL::update() {
  if (!isBleConnected())
    return;
  blocks->updateState(stateChBuffer);
  blocks->updateMotion(motionChBuffer);
  // Refresh the stored GATT value every tick (localOnly=true = update value, no
  // notification) so editors that READ (poll) see fresh data; NOTIFY is driven
  // on the v2 cadence below, not as a per-write side effect.
  uBit.ble->gattServer().write(stateCh->getValueHandle(), stateChBuffer,
                               BLOCKS_CH_BUFFER_SIZE_STATE, true);
  uBit.ble->gattServer().write(motionCh->getValueHandle(), motionChBuffer,
                               BLOCKS_CH_BUFFER_SIZE_MOTION, true);
  // Push STATE + MOTION via NOTIFY (runtime v2). update() runs ~every 19ms, so
  // notify every 3rd call (~57ms). gattServer().notify() is a harmless error
  // return when the central hasn't subscribed. Mirrors codal BlocksService.
  static uint8_t notifyDivider = 0;
  if (++notifyDivider >= 3) {
    notifyDivider = 0;
    uBit.ble->gattServer().notify(stateCh->getValueHandle(), stateChBuffer,
                                  BLOCKS_CH_BUFFER_SIZE_STATE);
    uBit.ble->gattServer().notify(motionCh->getValueHandle(), motionChBuffer,
                                  BLOCKS_CH_BUFFER_SIZE_MOTION);
  }
}

#endif // !MICROBIT_CODAL
