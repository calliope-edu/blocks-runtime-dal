#include "BlocksCommon.h"
#if BLOCKS_USE_SERIAL

#ifndef BLOCKS_SERIAL_H
#define BLOCKS_SERIAL_H

#include "BlocksDevice.h"

// BLOCKS_SFD now lives in BlocksCommon.h (shared by the serial + DAP transports).
#define BLOCKS_RX_BUFFER_SIZE 254
#define BLOCKS_TX_BUFFER_SIZE 254
// Max retries when CODAL Serial::send reports the TX lock is held by a
// concurrent send. 1ms per retry → ~100ms worst case before giving up (vs the
// old behaviour of silently dropping the frame on the first collision).
#define BLOCKS_TX_SEND_RETRY 100

// // Forward declaration
class BlocksDevice;

/**
 * Class definition for main logics of Microbit More Service except bluetooth connectivity.
 *
 */
class BlocksSerial {
private:
  /**
   * @brief Communication route between the blocks editor and the device
   * 
   */
  enum BlocksCommunicationRoute
  {
    BLE = 0,
    SERIAL = 1,
  };

  /**
   * @brief Request type from the blocks editor
   * 
   */
  enum ChRequest
  {
    REQ_READ = 0x01,
    REQ_WRITE = 0x10,
    REQ_WRITE_RESPONSE = 0x11,
    REQ_NOTIFY_STOP = 0x20,
    REQ_NOTIFY_START = 0x21,
  };

  /**
   * @brief Response type to the blocks editor
   * 
   */
  enum ChResponse
  {
    RES_READ = 0x01,
    RES_WRITE = 0x11,
    RES_NOTIFY = 0x21,
  };

public:
  /**
   * @brief Microbit More object.
   *
   */
  BlocksDevice &blocks;

  /**
   * @brief Construct a new Microbit More Serial service
   * 
   * @param _blocks An instance of Microbit More device controller
   */
  BlocksSerial(BlocksDevice &_blocks);

  /**
   * @brief Send a response for read request.
   * 
   * @param ch Characteristeic of the request
   * @param dataBuffer Buffer to send
   * @param len Length of the buffer to send
   */
  void readResponseOnSerial(uint16_t ch, uint8_t *dataBuffer, size_t len);

  /**
   * @brief Send a response for write request.
   * 
   * @param ch Characteristic of the request
   * @param response Response for the request
   */
  void writeResponseOnSerial(uint16_t ch, bool response);

  /**
   * @brief Notify data of the characteristic
   * 
   * @param ch Characteristic to notify
   * @param dataBuffer Buffer to notify
   * @param len Length of the buffer to notify
   */
  void notifyOnSerial(uint16_t ch, uint8_t *dataBuffer, size_t len);

  /**
   * @brief Start continuous receiving process from serial port.
   * 
   */
  void startSerialReceiving();

  /**
   * @brief Start continuous updating process to serial port.
   *
   */
  void startSerialUpdating();

  /**
   * @brief Send one fully-built frame over serial, waiting for TX-buffer room
   * and retrying briefly if a concurrent send holds the CODAL serial TX lock.
   *
   * CODAL Serial::send() returns DEVICE_SERIAL_IN_USE and transmits NOTHING when
   * another fiber/context is mid-send. All three serial outputs (read response,
   * write response, notify) plus the sensor-broadcaster fiber and message-bus
   * event handlers funnel through here so a collision retries instead of
   * silently dropping the frame.
   *
   * @param frame Pointer to the complete frame (header + payload + checksum).
   * @param frameSize Total frame length in bytes.
   */
  void sendFrameOnSerial(uint8_t *frame, size_t frameSize);
};
#endif // BLOCKS_SERIAL_H
#endif // BLOCKS_USE_SERIAL