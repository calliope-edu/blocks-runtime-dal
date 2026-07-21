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

  /**
   * @brief TX announcer: broadcast the RES_READ 0x0100 payload (version)
   * on the UART once per second, forever, EXCEPT while a DAP or BLE session
   * is active (those transports carry their own detection + comms, so a
   * serial announce would just be noise). Runs as its own fiber, started by
   * the constructor, and never exits.
   *
   * Rationale — this is the fix for the "not recognized until reset" reports
   * on mini 2. The DAL UART's RECEIVE path goes deaf after a while under
   * SoftDevice/event load, so the host's REQ_READ handshake is no longer
   * heard (reset briefly restored it). The SEND path stays alive, so an
   * unconditional low-rate announce lets a host detect the runtime + version
   * on connect/reload/reconnect WITHOUT relying on the device hearing
   * anything — no reset needed. The announcer must NOT stop permanently on
   * first contact (that would reintroduce the reload-not-detected bug); it
   * only pauses while an active non-serial session makes it redundant.
   */
  void startProbeAnnouncing();

private:
  /** Build + send the RES_READ 0x0100 response frame. */
  void respond();

  /** True while a transport other than this UART probe is carrying comms
   *  (DAP mailbox or BLE) — the announce pauses so it doesn't add noise. */
  bool otherTransportActive();
};

#endif // BLOCKS_PROBE_H
#endif // BLOCKS_SERIAL_PROBE
