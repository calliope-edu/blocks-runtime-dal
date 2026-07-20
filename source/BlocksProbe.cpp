#include "BlocksCommon.h"
#if BLOCKS_SERIAL_PROBE

#include "BlocksProbe.h"

static BlocksProbe *probe; // static pointer so create_fiber can reach the instance

static void startBlocksProbeResponding() {
  probe->startProbeResponding();
}

/**
 * @brief Sum-mod-0xFF checksum — same algorithm as the serial/DAP frame codecs.
 */
static uint8_t probeChksum8(const uint8_t *buff, size_t len) {
  unsigned int sum = 0;
  for (; len != 0; len--) {
    sum += *(buff++);
  }
  return (uint8_t)(sum % 0xFF);
}

/**
 * @brief Read one byte: ASYNC fast-path when buffered, event-driven wait
 * otherwise (parks only this fiber — same pattern as BlocksSerial::readSync).
 */
static uint8_t probeReadSync() {
  if (uBit.serial.rxBufferedSize() > 0) {
    return uBit.serial.read(ASYNC);
  }
  return uBit.serial.read(SYNC_SLEEP);
}

BlocksProbe::BlocksProbe(BlocksDevice &_blocks) : blocks(_blocks) {
  probe = this;
  // 115200 — what the host opens the DAPLink CDC / J-Link CDC bridge at.
#if MICROBIT_CODAL
  uBit.serial.setBaud(115200);
#else // NOT MICROBIT_CODAL
  uBit.serial.baud(115200);
#endif // NOT MICROBIT_CODAL
  create_fiber(startBlocksProbeResponding);
}

void BlocksProbe::respond() {
  // Fresh version payload. The route tag goes into a LOCAL copy — the shared
  // COMMAND buffer keeps its per-transport route byte for BLE/DAP readers.
  blocks.updateVersionData();
  uint8_t payload[BLOCKS_CH_BUFFER_SIZE_COMMAND];
  memcpy(payload, blocks.moreService->commandChBuffer, BLOCKS_CH_BUFFER_SIZE_COMMAND);
  payload[2] = 1; // communication-route tag: SERIAL (this reply's transport)

  uint8_t frame[6 + BLOCKS_CH_BUFFER_SIZE_COMMAND] = {0};
  frame[0] = BLOCKS_SFD;
  frame[1] = 0x01; // RES_READ
  frame[2] = 0x01; // COMMAND channel 0x0100, high byte
  frame[3] = 0x00; // low byte
  frame[4] = BLOCKS_CH_BUFFER_SIZE_COMMAND;
  memcpy(&frame[5], payload, BLOCKS_CH_BUFFER_SIZE_COMMAND);
  frame[sizeof(frame) - 1] = probeChksum8(frame, sizeof(frame) - 1);

  // Wait (bounded) until the TX ring has room for the WHOLE frame — an ASYNC
  // send with insufficient ring space copies only what fits and silently
  // truncates the response (same hazard BlocksSerial::sendFrameOnSerial
  // guards against). Give up rather than parking forever against a dead host.
  for (int waited = 0; (64 - uBit.serial.txBufferedSize()) < (int)sizeof(frame); waited++) {
    if (waited >= 100) return;
    fiber_sleep(1);
  }
  // Bounded ASYNC send: retry briefly while another consumer holds the TX
  // lock, give up rather than parking this fiber forever against a dead host.
  for (int attempt = 0; attempt < 100; attempt++) {
#if MICROBIT_CODAL
    if (uBit.serial.send(frame, (int)sizeof(frame), ASYNC) != DEVICE_SERIAL_IN_USE) {
      return;
    }
#else // NOT MICROBIT_CODAL — microbit-dal spells the busy code differently
    if (uBit.serial.send(frame, (int)sizeof(frame), ASYNC) != MICROBIT_SERIAL_IN_USE) {
      return;
    }
#endif // NOT MICROBIT_CODAL
    fiber_sleep(1);
  }
}

void BlocksProbe::startProbeResponding() {
  // Room for one full response frame in the TX ring (DAL's default ring is a
  // tiny 20 bytes); RX only ever holds the 4-byte handshake.
  uBit.serial.setTxBufferSize(64);
  uBit.serial.setRxBufferSize(32);

  // The probe handshake the widget sends: SFD + REQ_READ on channel 0x0100.
  const uint8_t pattern[4] = {BLOCKS_SFD, 0x01, 0x01, 0x00};
  size_t matched = 0;
  while (true) {
    uint8_t b = probeReadSync();
    if (b == pattern[matched]) {
      matched++;
      if (matched == sizeof(pattern)) {
        matched = 0;
        respond();
      }
    } else {
      // Restart the match; the mismatching byte may itself be a frame start.
      matched = (b == pattern[0]) ? 1 : 0;
    }
  }
}

#endif // BLOCKS_SERIAL_PROBE
