#include "BlocksCommon.h"
#if BLOCKS_SERIAL_PROBE

#ifndef BLOCKS_PROBE_H
#define BLOCKS_PROBE_H

#include "BlocksDevice.h"

class BlocksDevice;

/**
 * Minimal UART detection responder: "is Blocks on this mini, and which
 * version?"
 *
 * Answers exactly ONE request — the host's probe handshake
 * `[0xFF (SFD), 0x01 (REQ_READ), 0x01, 0x00]` (a read of the COMMAND channel
 * 0x0100) — with the regular `RES_READ 0x0100` frame carrying the COMMAND
 * payload (hardware / protocol / route / runtime-version bytes, see
 * BlocksDevice::updateVersionData). Every other byte on the wire is ignored.
 *
 * Deliberately NOT a comms transport (that is BLOCKS_USE_SERIAL, built off):
 *  - no STATE/MOTION broadcaster, no command/write handling, no notifies;
 *  - never latches BlocksDevice::serialConnected, so event routing and the
 *    BLE / DAP transports behave exactly as without it;
 *  - never resets blocks state.
 *
 * Works over the mini 3's DAPLink CDC and the mini 2's J-Link UART bridge,
 * giving the connection widget one reliable, transport-uniform way to detect
 * the runtime (program-type.ts probeUsb / probeJlinkSerial).
 */
class BlocksProbe {
public:
  /**
   * @brief The blocks device controller (source of the version payload).
   *
   */
  BlocksDevice &blocks;

  BlocksProbe(BlocksDevice &_blocks);

  /**
   * @brief RX loop: scan the UART for the probe handshake, answer, repeat.
   * Runs as its own fiber (started by the constructor).
   */
  void startProbeResponding();

private:
  /** Build + send the RES_READ 0x0100 response frame. */
  void respond();
};

#endif // BLOCKS_PROBE_H
#endif // BLOCKS_SERIAL_PROBE
