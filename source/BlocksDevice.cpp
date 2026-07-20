
#include "PxtShim.h"

#include "MicroBit.h"
#include "MicroBitConfig.h"

#if MICROBIT_CODAL
#define BUFFER_TYPE uint8_t*
#else
#define BUFFER_TYPE char*
#endif

#if MICROBIT_CODAL
// microphone sound level
#include "LevelDetector.h"
#include "LevelDetectorSPL.h"

#define MICROPHONE_MIN 52.0f
#define MICROPHONE_MAX 120.0f

namespace pxt {
  codal::LevelDetectorSPL *getMicrophoneLevel();
} // namespace pxt

int getMicLevel() {
  auto level = pxt::getMicrophoneLevel();
  if (NULL == level)
    return 0;
  const int micValue = level->getValue();
  const int scaled = max(MICROPHONE_MIN, min(micValue, MICROPHONE_MAX)) - MICROPHONE_MIN;
  return min(0xff, scaled * 0xff / (MICROPHONE_MAX - MICROPHONE_MIN));
}
#endif // MICROBIT_CODAL

/**
 * @brief Compute average value for the int array.
 *
 * @param data Array to compute average.
 * @param dataSize Length of the array.
 * @return average value.
 */
int average(int *data, int dataSize) {
  int sum = 0;
  int i;
  for (i = 0; i < dataSize - 1; i++) {
    sum += data[i];
  }
  return sum / dataSize;
}

/**
 * @brief Compute median value for the array.
 *
 * @param data Array to compute median.
 * @param dataSize Length of the array.
 * @return Median value.
 */
int median(int *data, int dataSize) {
  int temp;
  int i, j;
  // the following two loops sort the array x in ascending order
  for (i = 0; i < dataSize - 1; i++) {
    for (j = i + 1; j < dataSize; j++) {
      if (data[j] < data[i]) {
        // swap elements
        temp = data[i];
        data[i] = data[j];
        data[j] = temp;
      }
    }
  }
  return data[dataSize / 2];
}

/**
 * @brief Copy ManagedString to char array with max size.
 * 
 * @param dst char array as destination
 * @param mstr string as source
 * @param maxLength max size to copy
 */
void copyManagedString(char *dst, ManagedString mstr, size_t maxLength) {
  memcpy(dst, mstr.toCharArray(), ((size_t)mstr.length() < maxLength ? mstr.length() : maxLength));
}

/**
 * Position of data format in a value holder.
 */
#define BLOCKS_DATA_FORMAT_INDEX 19

#include "BlocksDevice.h"

/**
 * Constructor.
 * Create a representation of the device for Microbit More service.
 * @param _uBit The instance of a MicroBit runtime.
 */
BlocksDevice::BlocksDevice(MicroBit &_uBit) : uBit(_uBit) {
  // Reset compass

  if (!uBit.compass.isCalibrated()) {
    // Install an identity calibration so the device skips the interactive
    // tilt-to-calibrate ritual on boot. scale MUST be the 1024 unity scale, NOT
    // 1: CALIBRATED_SAMPLE multiplies the raw axis by scale then >>10, so
    // scale=1 collapses every heading/tilt reading to ~0 (garbage compass).
    CompassCalibration dummyCalibration;
    dummyCalibration.centre = Sample3D(0, 0, 0);
    dummyCalibration.scale = Sample3D(1024, 1024, 1024);
    dummyCalibration.radius = 100;
    uBit.compass.setCalibration(dummyCalibration);
  }
  
  if (uBit.buttonA.isPressed()) {
#if MICROBIT_CODAL
  // On microbit-v2, re-calibration destruct compass heading.
#else // NOT MICROBIT_CODAL
    uBit.compass.clearCalibration();
#endif // NOT MICROBIT_CODAL
    uBit.compass.calibrate();
  }

    // Compass must be calibrated before starting bluetooth service.
  if (!uBit.compass.isCalibrated()) {
    uBit.compass.calibrate();
  }

  // to detect 8G gesture event
  uBit.accelerometer.setRange(8);

  // displayVersion();

  uBit.messageBus.listen(
      MICROBIT_ID_BUTTON_A, MICROBIT_EVT_ANY,
      this,
      &BlocksDevice::onButtonChanged,
      MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
  uBit.messageBus.listen( 
      MICROBIT_ID_BUTTON_B,
      MICROBIT_EVT_ANY,
      this,
      &BlocksDevice::onButtonChanged,
      MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);

#if MICROBIT_CODAL
  uBit.messageBus.listen(
      MICROBIT_ID_LOGO,
      MICROBIT_EVT_ANY,
      this,
      &BlocksDevice::onButtonChanged,
      MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
#endif // MICROBIT_CODAL

  uBit.messageBus.listen(
      MICROBIT_ID_GESTURE,
      MICROBIT_EVT_ANY,
      this,
      &BlocksDevice::onGestureChanged,
      MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);

  uBit.messageBus.listen(
      MICROBIT_ID_BLE,
      MICROBIT_BLE_EVT_CONNECTED,
      this,
      &BlocksDevice::onBLEConnected,
      MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
  uBit.messageBus.listen(
      MICROBIT_ID_BLE,
      MICROBIT_BLE_EVT_DISCONNECTED,
      this,
      &BlocksDevice::onBLEDisconnected,
      MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
#if BLOCKS_USE_SERIAL
  serialService = new BlocksSerial(*this);
#endif // BLOCKS_USE_SERIAL
#if BLOCKS_USE_DAP
  dapService = new BlocksDap(*this);
#endif // BLOCKS_USE_DAP
#if BLOCKS_SERIAL_PROBE
  probeService = new BlocksProbe(*this);
#endif // BLOCKS_SERIAL_PROBE
}

BlocksDevice::~BlocksDevice() {
  uBit.messageBus.ignore(MICROBIT_ID_BUTTON_A, MICROBIT_EVT_ANY, this,
                         &BlocksDevice::onButtonChanged);
  uBit.messageBus.ignore(MICROBIT_ID_BUTTON_B, MICROBIT_EVT_ANY, this,
                         &BlocksDevice::onButtonChanged);
  uBit.messageBus.ignore(MICROBIT_ID_GESTURE, MICROBIT_EVT_ANY, this,
                         &BlocksDevice::onGestureChanged);
  uBit.messageBus.ignore(MICROBIT_ID_ANY, MICROBIT_EVT_ANY, this,
                         &BlocksDevice::onPinEvent);
  delete basicService;
}

/**
 * @brief Set pin configuration for initial.
 *
 */
void BlocksDevice::initializeConfig() {
  // P0..P3 rest pull-DOWN, matching the editor's digital-input default (its
  // "is Pn high" block forces pull-down) and the legacy MbitMore behaviour.
  // Booting pull-UP made the first "is Pn high" read return a stale HIGH before
  // the editor's pull-down propagated one STATE cycle later (the first-true-
  // then-false glitch). Edge detection still works on a pad-to-GND touch because
  // listenPinEventOn() arms pull-UP explicitly when an ON_EDGE event is set.
  for (size_t i = 0; i < (sizeof(initialPullUp) / sizeof(initialPullUp[0])); i++) {
    int pin = initialPullUp[i];
    // Skip a pin currently armed for touch. initializeConfig runs on EVERY
    // (re)connect; setPullMode + getDigitalValue on a touch pin tears down its
    // live codal TouchButton (a digital read disconnects it), leaving touchMode
    // set but no sensor — which is why touch went DEAD after a BLE reconnect.
    // Leave armed touch pads owned by their TouchButton so touch survives.
    if (pin >= 0 && pin <= 3 && touchMode[pin]) continue;
    setPullMode(pin, BlocksPullMode::Down);
    uBit.io.pin[pin].getDigitalValue(); // set the pin to input-mode
  }
}

/**
 * @brief Show the device's name histogram pattern (static, identifies the mini).
 *        Reimplements codal's private MicroBitBLEManager::showNameHistogram: the
 *        device ID rendered as five base-5 columns. Static — never blocks.
 */
void BlocksDevice::displayNamePattern() {
  uint32_t n = NRF_FICR->DEVICEID[1];
  int ld = 1;
  int d = 5;
  uBit.display.stopAnimation();
  uBit.display.clear();
  for (int i = 0; i < 5; i++) {
    int h = (n % d) / ld;
    n -= h;
    d *= 5;
    ld *= 5;
    for (int j = 0; j < h + 1; j++) {
      uBit.display.image.setPixelValue(5 - i - 1, 5 - j - 1, 255);
    }
  }
}

/**
 * @brief Soft-reset the blocks session: stop outputs, disarm touch, show the
 *        name pattern, ready for fresh commands/events. No reboot, so a parallel
 *        transport stays alive. Called on every BLE/USB connect+disconnect and
 *        on the editor/campus RESET command. The editor reconciles via the
 *        touch-armed bitmask in COMMAND data[4] and re-arms what it expects.
 */
void BlocksDevice::resetBlocksState() {
  // Disarm every touch pad: drop its listener and tear down the codal
  // TouchButton (a digital read disconnects it) so the next CONFIG TOUCH
  // re-arms + recalibrates clean.
  for (int p = 0; p <= 3; p++) {
    if (touchMode[p]) {
      uBit.messageBus.ignore(p + 100, MICROBIT_EVT_ANY, this,
                             &BlocksDevice::onButtonChanged);
      uBit.io.pin[p].getDigitalValue();
    }
    touchMode[p] = false;
    touchArmedMode[p] = -1;
  }
  // Stop actuators so a previous program's output never persists into the next.
#if MICROBIT_CODAL
  // Turn the on-board RGB neopixels off (3 LEDs x GRB = 9 zero bytes), matching
  // the CMD_RGB path which drives them via neopixel_send_buffer(uBit.io.RGB,...).
  uint8_t rgbOff[9] = {0};
  neopixel_send_buffer(uBit.io.RGB, rgbOff, sizeof(rgbOff));
#endif // MICROBIT_CODAL
  stopTone();
  // Re-apply default pin pulls (P0..P3 are all un-armed now) and (re)start the
  // connect/reset ghost-guard window (see onButtonChanged).
  initializeConfig();
  bleConnectTime = (uint32_t)system_timer_current_time();
  // Idle/ready indicator: the device name pattern (static).
  displayNamePattern();
}

/**
 * @brief Update version data on the characteristic.
 *
 */
void BlocksDevice::updateVersionData() {
  uint8_t *data = moreService->commandChBuffer;
#if MICROBIT_CODAL
  data[0] = BlocksHardwareVersion::MICROBIT_V2;
#else // NOT MICROBIT_CODAL
  data[0] = BlocksHardwareVersion::MICROBIT_V1;
#endif // NOT MICROBIT_CODAL
  data[1] = BlocksProtocol::BLOCKS_V2;
  // data[2] is the communication-route tag (0=BLE / 1=SERIAL), written by the
  // serial read path after this call and left 0 (=BLE) on the BLE path — do not
  // touch it here. data[3] carries the monotonic runtime version so the editor
  // can detect an outdated hex; it is (re)written on every connect/REQ_READ, so
  // a value left stale by a prior command WRITE into this shared buffer is
  // always refreshed before the response is sent.
  data[3] = BLOCKS_RUNTIME_VERSION;
  // data[4] = touch-armed bitmask (bit n = Pn currently armed for touch). The
  // editor reconciles against this: a pin it believes armed but that reads 0
  // here means the device was reset, so it re-sends CONFIG TOUCH. Live RAM state
  // (touchMode[]) — no persistence; naturally 0 after a reset/boot.
  data[4] = (uint8_t)((touchMode[0] ? 0x01 : 0) | (touchMode[1] ? 0x02 : 0) |
                      (touchMode[2] ? 0x04 : 0) | (touchMode[3] ? 0x08 : 0));
  // data[5..7] = pin-event-armed bitmap (bit p = Pn armed for an edge/pulse
  // event via CMD_PIN SET_EVENT). The editor reconciles against this just like
  // the touch mask: a pin it wants armed but reads 0 here was dropped/reset, so
  // it re-sends SET_EVENT; a pin armed here that it does NOT want, it disarms.
  // Live RAM (pinEventMode[]) — naturally 0 after boot. Written fresh here every
  // read since commandChBuffer is shared with command WRITEs.
  uint32_t pinEventMask = 0;
  for (int p = 0; p < (int)(sizeof(pinEventMode) / sizeof(pinEventMode[0])); p++) {
    if (pinEventMode[p] != BlocksPinEventType::NONE) pinEventMask |= (1u << p);
  }
  data[5] = (uint8_t)(pinEventMask & 0xff);
  data[6] = (uint8_t)((pinEventMask >> 8) & 0xff);
  data[7] = (uint8_t)((pinEventMask >> 16) & 0xff);
}

/**
 * @brief Invoked when BLE connected.
 * 
 * @param _e event which has connection data
 */
void BlocksDevice::onBLEConnected(MicroBitEvent _e) {
#if MICROBIT_CODAL
  fiber_sleep(100); // to change pull-mode in micro:bit v2
#endif // MICROBIT_CODAL
  resetBlocksState();
  // Visual connect confirmation: draw a one-time "C" (overwrites the idle name
  // pattern; stays until the editor's first display command). Lets the user
  // see the BLE link is up before the editor sends anything.
  uBit.display.print('C');
}

/**
 * @brief Invoked when BLE disconnected.
 *
 * @param _e event which has disconnection data
 */
void BlocksDevice::onBLEDisconnected(MicroBitEvent _e) {
  // Soft-reset on disconnect: stop outputs, disarm touch, show the name pattern
  // — ready for the next session. Soft (no reboot), so a parallel USB session
  // stays alive.
  resetBlocksState();
}

void BlocksDevice::onSerialConnected() {
  resetBlocksState();
  serialConnected = true;
}

void BlocksDevice::onDapConnected() {
  resetBlocksState();
  dapConnected = true;
}

/**
 * @brief Call when a command was received.
 *
 * @param data
 * @param length
 */
void BlocksDevice::onCommandReceived(uint8_t *data, size_t length) {
  const int command = (data[0] >> 5);
  if(command == BlocksCommand::CMD_MOTOR) {
    const int motorCommand = data[0] & 0b11111;
    // data[1] = DIR: 0 | 1
    // data[2] = Motor Speed: 0 - 100
    if(motorCommand == BlocksMotorCommand::SET_M0 || motorCommand == BlocksMotorCommand::SET_M1 || motorCommand == BlocksMotorCommand::SET_M0_M1){ 
#if MICROBIT_CODAL
        const int direction = data[1];
        const int speed = static_cast<int>((static_cast<double>(data[2]) / 100) * 1023); //Map 0-100 to 0-1023
        uBit.io.M_MODE.setDigitalValue(1);
        if(motorCommand == BlocksMotorCommand::SET_M0 || motorCommand == BlocksMotorCommand::SET_M0_M1){ 
            uBit.io.M_A_IN1.setDigitalValue(direction);
            uBit.io.M_A_IN2.setAnalogValue(speed);
        }
        if(motorCommand == BlocksMotorCommand::SET_M1 || motorCommand == BlocksMotorCommand::SET_M0_M1){ 
            uBit.io.M_B_IN1.setDigitalValue(direction);
            uBit.io.M_B_IN2.setAnalogValue(speed);
        }
#else
        const int speed = data[2];
        if(motorCommand == BlocksMotorCommand::SET_M0 || motorCommand == BlocksMotorCommand::SET_M0_M1){
            if (speed <= 0) uBit.soundmotor.motorAOff();
            else uBit.soundmotor.motorAOn(speed);
        }
        if(motorCommand == BlocksMotorCommand::SET_M1 || motorCommand == BlocksMotorCommand::SET_M0_M1){
            if (speed <= 0) uBit.soundmotor.motorBOff();
            else uBit.soundmotor.motorBOn(speed);
        }
#endif
    } else {
      // MOTIONKIT
      const int direction = data[1];
      const int speed = static_cast<int>((static_cast<double>(data[2]) / 100) * 255); //Map 0-100 to 0-255
      int buf[3];  // Define the buffer array
      buf[1] = direction;
      buf[2] = speed;
      if (motorCommand == BlocksMotorCommand::SET_MOTIONKIT_LEFT || motorCommand == BlocksMotorCommand::SET_MOTIONKIT_BOTH) {
          buf[0] = 0x00;
          uBit.i2c.write(0x10 << 1, reinterpret_cast<BUFFER_TYPE>(buf), sizeof(buf), false); // Send the data
      }
      if (motorCommand == BlocksMotorCommand::SET_MOTIONKIT_RIGHT || motorCommand == BlocksMotorCommand::SET_MOTIONKIT_BOTH) {
          buf[0] = 0x02;
          uBit.i2c.write(0x10 << 1, reinterpret_cast<BUFFER_TYPE>(buf), sizeof(buf), false); // Send the data
      }
    }


  } else if(command == BlocksCommand::CMD_RGB) {
#if MICROBIT_CODAL
        uint8_t rgbBuffer[9] = {0};
        // Neopixel awaits GRB instead of RGB, so its swtiched here.
        rgbBuffer[0] = data[2] * 20 / 100; // Green
        rgbBuffer[1] = data[1] * 20 / 100; // Red
        rgbBuffer[2] = data[3] * 20 / 100; // Blue
        rgbBuffer[3] = data[5] * 20 / 100; // G
        rgbBuffer[4] = data[4] * 20 / 100; // R
        rgbBuffer[5] = data[6] * 20 / 100; // B
        rgbBuffer[6] = data[8] * 20 / 100; // G
        rgbBuffer[7] = data[7] * 20 / 100; // R
        rgbBuffer[8] = data[9] * 20 / 100; // B
        neopixel_send_buffer(uBit.io.RGB, rgbBuffer, sizeof(rgbBuffer));
#else
        uBit.rgb.setColour(data[1], data[2], data[3], 0);
#endif
  } else if (command == BlocksCommand::CMD_DISPLAY) {
    const int displayCommand = data[0] & 0b11111;
    if (displayCommand == BlocksDisplayCommand::TEXT) {
      char text[length - 1] = {0};
      memcpy(text, &(data[2]), length - 2);
      displayText(text, (data[1] * 10));
    } else if (displayCommand == BlocksDisplayCommand::PIXELS_PACKED) {
      // Whole 5x5 on/off image in ONE atomic frame: data[1..4] = 25-bit bitmap,
      // bit (row*5+col); 1 = LED full-on. No PIXELS_0/PIXELS_1 split, so it can
      // never render torn, and it's a single BLE round-trip. The editor sends
      // this for the standard on/off display block (brightness still uses the
      // 2-frame path below).
      uint32_t bits = (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                      ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
      for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 5; col++) {
          shadowPixcels[row][col] = ((bits >> (row * 5 + col)) & 1u) ? 255 : 0;
        }
      }
      pendingDisplayGen = -1; // single frame: no half-frame state to carry
      displayShadowPixels();
    } else if (displayCommand == BlocksDisplayCommand::PIXELS_0) {
      setPixelsShadowLine(0, &data[1]);
      setPixelsShadowLine(1, &data[6]);
      setPixelsShadowLine(2, &data[11]);
      // Optional trailing generation byte (data[16]) — the editor stamps the
      // same value into this frame's PIXELS_0 and PIXELS_1 so PIXELS_1 can
      // detect a dropped/reordered top half. -1 when the editor sends none.
      pendingDisplayGen = (length > 16) ? (int)data[16] : -1;
    } else if (displayCommand == BlocksDisplayCommand::PIXELS_1) {
      const int gen = (length > 11) ? (int)data[11] : -1;
      // Torn-frame guard: if this bottom half is tagged with a generation that
      // does NOT match the last top half (PIXELS_0 dropped/reordered), skip the
      // render and keep the last complete image rather than showing new-bottom
      // over old-top (the BLE "half image"). Untagged frames render as before.
      if (gen >= 0 && pendingDisplayGen >= 0 && gen != pendingDisplayGen) {
        // torn frame — drop the render; the editor re-sends the full frame.
      } else {
        setPixelsShadowLine(3, &data[1]);
        setPixelsShadowLine(4, &data[6]);
        displayShadowPixels();
      }
      pendingDisplayGen = -1;
    }
  } else if (command == BlocksCommand::CMD_PIN) {
    const int pinCommand = data[0] & 0b11111;
    int pinIndex = (int)data[1];
    if (pinCommand == BlocksPinCommand::SET_PULL) {
      uBit.io.pin[pinIndex].getDigitalValue(); // set the pin to input mode
      setPullMode(pinIndex, (BlocksPullMode)data[2]);
    } else if (pinCommand == BlocksPinCommand::SET_OUTPUT) {
#if MICROBIT_CODAL
      // workaround to set d-out from touch-mode in microbit-codal-v2
      if (touchMode[pinIndex]) {
        uBit.io.pin[pinIndex].setAnalogValue(0);
      }
#endif // MICROBIT_CODAL
      setDigitalValue(pinIndex, data[2]);
    } else if (pinCommand == BlocksPinCommand::SET_PWM) {
      // value is read as uint16_t little-endian.
      uint16_t value;
      memcpy(&value, &(data[2]), 2);
      setAnalogValue(pinIndex, value);
    } else if (pinCommand == BlocksPinCommand::SET_SERVO) {
      // angle is read as uint16_t little-endian.
      uint16_t angle;
      memcpy(&angle, &(data[2]), 2);
      // range is read as uint16_t little-endian.
      uint16_t range;
      memcpy(&range, &(data[4]), 2);
      // center is read as uint16_t little-endian.
      uint16_t center;
      memcpy(&center, &(data[6]), 2);
      if (range == 0) {
        uBit.io.pin[pinIndex].setServoValue(angle);
      } else if (center == 0) {
        uBit.io.pin[pinIndex].setServoValue(angle, range);
      } else {
        uBit.io.pin[pinIndex].setServoValue(angle, range, center);
      }
    } else if (pinCommand == BlocksPinCommand::SET_EVENT) {
      listenPinEventOn(pinIndex, (int)data[2]);
    }
    // Repurposing a pin clears any touch arming on it. Guard the index: pin
    // commands address any GPIO (P0..P17) but touchMode/touchArmedMode are
    // sized for the 4 touch-capable pads only — an unguarded write here for
    // e.g. pin 13 was an out-of-bounds store.
    if (pinIndex >= 0 && pinIndex <= 3) {
      touchMode[pinIndex] = false;
      touchArmedMode[pinIndex] = -1;
    }
  } else if (command == BlocksCommand::CMD_AUDIO) {
    int audioCommand = data[0] & 0b11111;
    if (audioCommand == BlocksAudioCommand::PLAY_TONE) {
      uint32_t period;
      memcpy(&period, &(data[1]), 4);
      playTone(period, data[5]);
    } else if (audioCommand == BlocksAudioCommand::STOP_TONE) {
      stopTone();
    }
#if MICROBIT_CODAL
  } else if (command == BlocksCommand::CMD_DATA) {
    BlocksDataContentType dataType = (BlocksDataContentType)(data[0] & 0b11111);
    int index = findWaitingDataLabelIndex((char *)(&data[1]), dataType);
    if (index != BLOCKS_WAITING_DATA_LABEL_NOT_FOUND) {
      int contentStart = 1 + BLOCKS_DATA_LABEL_SIZE;
      memset(receivedData[index].content, 0, BLOCKS_DATA_CONTENT_SIZE);
      memcpy(receivedData[index].content, &data[contentStart], length - contentStart);
      MicroBitEvent evt(BLOCKS_DATA_RECEIVED, index + 1);
    }
#endif // MICROBIT_CODAL
  } else if (command == BlocksCommand::CMD_CONFIG) {
    const int config = data[0] & 0b11111;
    if (config == BlocksConfig::MICPIN) {
#if MICROBIT_CODAL
      micInUse = ((data[1] == 1) ? true : false);
#endif // MICROBIT_CODAL
    } else if (config == BlocksConfig::TOUCH) {
      int pinIndex = data[1];
      if (pinIndex > 3)
        return;
      int componentID = pinIndex + 100;
      if (data[2] == 1) {
        // Requested sensing mode: 1 = Resistive, else Capacitative.
        int8_t reqMode = (data[3] == 1) ? 1 : 0;
        // Idempotent arm. The editor re-sends TOUCH-enable on every program
        // (re)start and after each pinMode reset; re-running isTouched(mode)
        // re-attaches/re-calibrates the codal TouchButton and keeps the shared
        // cap-touch sampler churning, which (with >1 armed pad) repeatedly
        // masks the SoftDevice radio IRQ. Only (re)arm on an off->on transition
        // or an actual mode change.
        if (!(touchMode[pinIndex] && touchArmedMode[pinIndex] == reqMode)) {
          uBit.messageBus.listen(
              componentID,
              MICROBIT_EVT_ANY,
              this,
              &BlocksDevice::onButtonChanged,
              MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
#if MICROBIT_CODAL
          if (reqMode == 1) {
            uBit.io.pin[pinIndex].isTouched(codal::TouchMode::Resistive);
          } else {
            uBit.io.pin[pinIndex].isTouched(codal::TouchMode::Capacitative);
          }
#else // NOT MICROBIT_CODAL
          uBit.io.pin[pinIndex].isTouched();
#endif // NOT MICROBIT_CODAL
          touchMode[pinIndex] = true;
          touchArmedMode[pinIndex] = reqMode;
          // Start the post-arm guard window (see onButtonChanged): the freshly
          // (re)armed TouchButton calibrates/settles over the next ~0.5-2s and
          // can emit a phantom DOWN/UP with no real touch.
          touchArmTime[pinIndex] = (uint32_t)system_timer_current_time();
        }
      } else {
        uBit.messageBus.ignore(
            componentID,
            MICROBIT_EVT_ANY,
            this,
            &BlocksDevice::onButtonChanged);
        touchMode[pinIndex] = false;
        touchArmedMode[pinIndex] = -1;
      }
    } else if (config == BlocksConfig::RESET) {
      // Campus sends this on a program/editor switch (BLE stays connected, so
      // the device sees no transition). Soft-reset the blocks session.
      resetBlocksState();
    }
  }

  // Re-stamp the version/handshake bytes after handling any command. COMMAND is
  // one shared characteristic used for BOTH command writes and the version read,
  // and it is VLOC_USER, so a BLE write lands the command bytes directly in
  // commandChBuffer — clobbering data[3]. A later plain GATT read of COMMAND
  // would then return the command's byte 3 (0) instead of BLOCKS_RUNTIME_VERSION,
  // making the editor raise a false "outdated firmware" banner. The serial path
  // already re-stamps on every REQ_READ; this covers the BLE read path.
  updateVersionData();
}

/**
 * @brief Set the pattern on the line of the shadow pixels.
 *
 * @param line Index of the lines to set.
 * @param pattern Array of brightness(0..255) according columns.
 */
void BlocksDevice::setPixelsShadowLine(int line, uint8_t *pattern) {
  for (size_t col = 0; col < 5; col++) {
    shadowPixcels[line][col] = pattern[col];
  }
}

/**
 * @brief Display the shadow pixels on the LED.
 *
 */
void BlocksDevice::displayShadowPixels() {
  uBit.display.stopAnimation();
  for (size_t y = 0; y < 5; y++) {
    for (size_t x = 0; x < 5; x++) {
      uBit.display.image.setPixelValue(x, y, shadowPixcels[y][x]);
    }
  }
}

/**
 * @brief Display text on LED.
 *
 * @param text Contents to display with null termination.
 * @param delay The time to delay between characters, in milliseconds.
 */
void BlocksDevice::displayText(char *text, int delay) {
  ManagedString mstr(text);
  if (mstr.length() < 1) {
    return;
  }
  uBit.display.stopAnimation();
  if (delay <= 0) {
    uBit.display.printCharAsync(mstr.charAt(0), delay);
    return;
  }
  uBit.display.scrollAsync(mstr, delay);
}

/**
 * @brief Update GPIO and sensors state.
 *
 * @param data Buffer for BLE characteristics.
 */
void BlocksDevice::updateState(uint8_t *data) {
  uint32_t digitalLevels = 0;
  for (size_t i = 0; i < sizeof(gpioPin) / sizeof(gpioPin[0]); i++) {
    int p = gpioPin[i];
    // Skip pins armed for touch. On codal v0.3.5 a touch pin reports
    // IO_STATUS_DIGITAL_IN, so isDigital()/isInput() are true here; calling
    // getDigitalValue() then misses its fast-path (obj is the TouchButton) and
    // takes the mode-change path, which disconnect()s the TouchButton — every
    // update — so the pad never accumulates a capacitive reading and never
    // senses (boolean stays false, no touch events fire). The logo touch works
    // precisely because it is NOT in gpioPin[]. Touch pins report their state
    // via the touch bits (P0..P3) below instead.
    if (p >= 0 && p <= 3 && touchMode[p]) {
      continue;
    }
    if (uBit.io.pin[p].isDigital()) {
      if (uBit.io.pin[p].isInput()) {
        digitalLevels = digitalLevels | (uBit.io.pin[p].getDigitalValue() << p);
      }
    }
  }
  if (touchMode[0]) {
    digitalLevels = digitalLevels | (uBit.io.pin[0].isTouched() << BlocksButtonStateIndex::P0);
  }
  if (touchMode[1]) {
    digitalLevels = digitalLevels | (uBit.io.pin[1].isTouched() << BlocksButtonStateIndex::P1);
  }
  if (touchMode[2]) {
    digitalLevels = digitalLevels | (uBit.io.pin[2].isTouched() << BlocksButtonStateIndex::P2);
  }
  if (touchMode[3]) {
    digitalLevels = digitalLevels | (uBit.io.pin[3].isTouched() << BlocksButtonStateIndex::P3);
  }
  digitalLevels = digitalLevels | (uBit.buttonA.isPressed() << BlocksButtonStateIndex::A);
  digitalLevels = digitalLevels | (uBit.buttonB.isPressed() << BlocksButtonStateIndex::B);
#if MICROBIT_CODAL
  digitalLevels = digitalLevels | (uBit.logo.isPressed() << BlocksButtonStateIndex::LOGO);
#endif // MICROBIT_CODAL
  memcpy(data, (uint8_t *)&digitalLevels, 4);
  data[4] = sampleLightLevel();
  data[5] = (uint8_t)(uBit.thermometer.getTemperature() + 128);
#if MICROBIT_CODAL
  if (micInUse) {
    data[6] = getMicLevel();
  }
#endif // MICROBIT_CODAL
}

/**
 * @brief Update data of motion.
 *
 * @param data Buffer for BLE characteristics.
 */
void BlocksDevice::updateMotion(uint8_t *data) {
  // Accelerometer
  int16_t rot;
  // Pitch (radians / 1000) is sent as int16_t little-endian [0..1].
  rot = (int16_t)(uBit.accelerometer.getPitchRadians() * 1000);
  memcpy(&(data[0]), &rot, 2);
  // Roll (radians / 1000) is sent as int16_t little-endian [2..3].
  rot = (int16_t)(uBit.accelerometer.getRollRadians() * 1000);
  memcpy(&(data[2]), &rot, 2);

  int16_t acc;
  // Acceleration X [milli-g] is sent as int16_t little-endian [4..5].
  acc = (int16_t)-uBit.accelerometer.getX(); // Face side is positive in Z-axis.
  memcpy(&(data[4]), &acc, 2);
  // Acceleration Y [milli-g] is sent as int16_t little-endian [6..7].
  acc = (int16_t)uBit.accelerometer.getY();
  memcpy(&(data[6]), &acc, 2);
  // Acceleration Z [milli-g] is sent as int16_t little-endian [8..9].
  acc = (int16_t)-uBit.accelerometer.getZ(); // Face side is positive in Z-axis.
  memcpy(&(data[8]), &acc, 2);

  // Magnetometer
  // Compass Heading is sent as uint16_t little-endian [10..11]
  uint16_t heading = (uint16_t)normalizeCompassHeading(uBit.compass.heading());
  memcpy(&(data[10]), &heading, 2);

  int16_t force;
  // Magnetic force X (micro-teslas) is sent as int16_t little-endian[12..13].
  force = (int16_t)(uBit.compass.getX() / 1000);
  memcpy(&(data[12]), &force, 2);
  // Magnetic force Y (micro-teslas) is sent as int16_t little-endian[14..15].
  force = (int16_t)(uBit.compass.getY() / 1000);
  memcpy(&(data[14]), &force, 2);
  // Magnetic force Z (micro-teslas) is sent as int16_t little-endian[16..17].
  force = (int16_t)(uBit.compass.getZ() / 1000);
  memcpy(&(data[16]), &force, 2);
}

/**
 * @brief Get data of analog input of the pin.
 *
 * @param data Buffer for BLE characteristics.
 * @param pinIndex Index of the pin [0, 1, 2, 3].
 */
void BlocksDevice::updateAnalogIn(uint8_t *data, size_t pinIndex) {
  if (uBit.io.pin[pinIndex].isInput()) {
#if MICROBIT_CODAL
    uBit.io.pin[pinIndex].setPull(PullMode::None);
#else // NOT MICROBIT_CODAL
    uBit.io.pin[pinIndex].setPull(PinMode::PullNone);
#endif // NOT MICROBIT_CODAL

    // filter
    for (size_t i = 0; i < ANALOG_IN_SAMPLES_SIZE; i++) {
      analogInSamples[pinIndex][i] = uBit.io.pin[pinIndex].getAnalogValue();
    }
    uint16_t value = median(analogInSamples[pinIndex], ANALOG_IN_SAMPLES_SIZE);

    // analog value (0 to 1023) is sent as uint16_t little-endian, packed into
    // this pin's slot of the unified analog payload (P0..P2 at offsets 0/2/4).
    memcpy(&(data[pinIndex * 2]), &value, 2);
    setPullMode(pinIndex, pullMode[pinIndex]);
  }
}

/**
 * @brief Sample current light level and return filtered value.
 *
 * @return int Filtered light level.
 */
int BlocksDevice::sampleLightLevel() {
  lightLevelSamplesLast++;
  if (lightLevelSamplesLast == LIGHT_LEVEL_SAMPLES_SIZE) {
    lightLevelSamplesLast = 0;
  }
  lightLevelSamples[lightLevelSamplesLast] = uBit.display.readLightLevel();
  return average(lightLevelSamples, LIGHT_LEVEL_SAMPLES_SIZE);
}

/**
 * @brief Set PMW signal to the pin for play tone.
 * 
 * @param period  PWM period (1000000 / frequency)[us]
 * @param volume laudness of the sound [0..255]
 */
void BlocksDevice::playTone(int period, int volume) {
#if MICROBIT_CODAL
  MicroBitPin speakerPin = uBit.io.speaker;
#else // NOT MICROBIT_CODAL
  MicroBitPin speakerPin = uBit.io.pin[0];
#endif // NOT MICROBIT_CODAL
  if (period <= 0 || volume == 0) {
    speakerPin.setAnalogValue(0);
  } else {
    int v = 1 << (volume >> 5); // [2..14]
    speakerPin.setAnalogValue(v);
    speakerPin.setAnalogPeriodUs(period);
  }
}

/**
 * @brief Stop playing tone.
 * 
 */
void BlocksDevice::stopTone() {
#if MICROBIT_CODAL
  MicroBitPin speakerPin = uBit.io.speaker;
#else // NOT MICROBIT_CODAL
  MicroBitPin speakerPin = uBit.io.pin[0];
#endif // NOT MICROBIT_CODAL
  speakerPin.setAnalogValue(0);
}

#if MICROBIT_CODAL
/**
 * @brief Return index for the label
 * 
 * @param dataLabel label to find
 * @param dataType type of the data
 * @return int index of the label
 */
int BlocksDevice::findWaitingDataLabelIndex(const char *dataLabel, BlocksDataContentType dataType) {
  for (int i = 0; i < BLOCKS_WAITING_DATA_LABELS_LENGTH; i++) {
    if (receivedData[i].label[0] == 0)
      continue;
    if (receivedData[i].type == dataType) {
      if (0 == strncmp(receivedData[i].label,
                       dataLabel,
                       BLOCKS_DATA_LABEL_SIZE)) {
        return i;
      }
    }
  }
  return BLOCKS_WAITING_DATA_LABEL_NOT_FOUND;
}

/**
 * @brief Register data label and retrun ID for the label.
 *
 * @param dataLabel label to register
 * @param dataType type of the data
 * @return int ID for the label
 */
int BlocksDevice::registerWaitingDataLabel(ManagedString dataLabel, BlocksDataContentType dataType) {
  int index = findWaitingDataLabelIndex(dataLabel.toCharArray(), dataType);
  if (index == BLOCKS_WAITING_DATA_LABEL_NOT_FOUND) {
    // find blank index and resister it
    for (int i = 0; i < BLOCKS_WAITING_DATA_LABELS_LENGTH; i++) {
      if (receivedData[i].label[0] == 0) {
        index = i;
        receivedData[index].type = dataType;
        strncpy(
            receivedData[index].label,
            dataLabel.toCharArray(),
            BLOCKS_DATA_LABEL_SIZE);
        return index + 1; // It is used for event value and must not be 0 (0 to accept any events).
      }
    }
  }
  return 0;
}

/**
 * @brief Get type of content for the labeled data
 *
 * @param labelID ID of the label in received data
 * @return content type
 */
BlocksDataContentType BlocksDevice::dataType(int labelID) {
  return receivedData[labelID - 1].type;
}

/**
 * @brief Return content of the data as number
 *
 * @param labelID ID of the label in received data
 * @return content of the data
 */
float BlocksDevice::dataContentAsNumber(int labelID) {
  float content;
  memcpy(&content, receivedData[labelID - 1].content, 4);
  return content;
}

/**
 * @brief Return content of the data as text
 *
 * @param labelID ID of the label in received data
 * @return content of the data
 */
ManagedString BlocksDevice::dataContentAsText(int labelID) {
  return ManagedString((char *)(receivedData[labelID - 1].content));
}

/**
 * @brief Send number with label.
 * 
 * @param dataLabel 
 * @param dataContent 
 */
void BlocksDevice::sendNumberWithLabel(ManagedString dataLabel, float dataContent) {
  uint8_t *data = moreService->dataChBuffer;
  memset(data, 0, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
  copyManagedString((char *)(&data[0]), dataLabel, BLOCKS_DATA_LABEL_SIZE);
  memcpy(&data[BLOCKS_DATA_LABEL_SIZE], &dataContent, 4);
  data[BLOCKS_DATA_FORMAT_INDEX] = BlocksDataFormat::DATA_NUMBER;
#if BLOCKS_USE_DAP
  // Prefer BLE whenever a BLE central is connected; otherwise route the event to
  // the CMSIS-DAP mailbox (the codal USB transport — it replaces serial here).
  if (dapConnected && !moreService->isBleConnected()) {
    dapService->notifyOnDap(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_DAP
#if BLOCKS_USE_SERIAL
  // Prefer BLE whenever a BLE central is connected; only fall back to serial
  // when BLE is NOT connected. `serialConnected` latches true on the first USB
  // connect and is never cleared, so without the BLE check every event would
  // route to (a possibly-dead) serial forever after USB was ever plugged —
  // which is why button/touch/data events died after a USB→BLE switch.
  if (serialConnected && !moreService->isBleConnected()) {
    serialService->notifyOnSerial(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_SERIAL
  moreService->notifyData();
}

/**
 * @brief Send text with label.
 * 
 * @param dataLabel 
 * @param dataContent 
 */
void BlocksDevice::sendTextWithLabel(ManagedString dataLabel, ManagedString dataContent) {
  uint8_t *data = moreService->dataChBuffer;
  memset(data, 0, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
  copyManagedString(
      (char *)(&data[0]),
      dataLabel,
      BLOCKS_DATA_LABEL_SIZE);
  copyManagedString(
      (char *)(&data[BLOCKS_DATA_LABEL_SIZE]),
      dataContent,
      BLOCKS_DATA_CONTENT_SIZE);
  data[BLOCKS_DATA_FORMAT_INDEX] = BlocksDataFormat::DATA_TEXT;
#if BLOCKS_USE_DAP
  // Prefer BLE whenever a BLE central is connected; otherwise route the event to
  // the CMSIS-DAP mailbox (the codal USB transport — it replaces serial here).
  if (dapConnected && !moreService->isBleConnected()) {
    dapService->notifyOnDap(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_DAP
#if BLOCKS_USE_SERIAL
  // Prefer BLE whenever a BLE central is connected; only fall back to serial
  // when BLE is NOT connected. `serialConnected` latches true on the first USB
  // connect and is never cleared, so without the BLE check every event would
  // route to (a possibly-dead) serial forever after USB was ever plugged —
  // which is why button/touch/data events died after a USB→BLE switch.
  if (serialConnected && !moreService->isBleConnected()) {
    serialService->notifyOnSerial(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_SERIAL
  moreService->notifyData();
}

#endif // MICROBIT_CODAL

/**
 * @brief Listen pin events on the pin.
 * Make it listen events of the event type on the pin.
 * Remove listener if the event type is MICROBIT_PIN_EVENT_NONE.
 * 
 * @param pinIndex index in edge pins
 * @param eventType type of events
 */
void BlocksDevice::listenPinEventOn(int pinIndex, int eventType) {
  if (!isGpio(pinIndex)) {
    return;
  }
  // conventional scheme to convert from pin index to componentID in v1 and v2.
  int componentID = pinIndex + 100;
  uBit.messageBus.ignore(
      componentID,
      MICROBIT_PIN_EVT_RISE,
      this,
      &BlocksDevice::onPinEvent);
  uBit.messageBus.ignore(
      componentID,
      MICROBIT_PIN_EVT_FALL,
      this,
      &BlocksDevice::onPinEvent);
  uBit.messageBus.ignore(
      componentID,
      MICROBIT_PIN_EVT_PULSE_HI,
      this,
      &BlocksDevice::onPinEvent);
  uBit.messageBus.ignore(
      componentID,
      MICROBIT_PIN_EVT_PULSE_LO,
      this,
      &BlocksDevice::onPinEvent);

  if (eventType == BlocksPinEventType::ON_EDGE) {
    uBit.messageBus.listen(
        componentID,
        MICROBIT_PIN_EVT_RISE,
        this,
        &BlocksDevice::onPinEvent,
        MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
    uBit.messageBus.listen(
        componentID,
        MICROBIT_PIN_EVT_FALL,
        this,
        &BlocksDevice::onPinEvent,
        MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
    // Edge detection arms pull-UP so a pad-to-GND finger touch produces a clean
    // HIGH->LOW edge. The device's resting pull defaults to DOWN (for the
    // "is Pn high" digital-input path), so edge events must flip it UP here.
    // Establish the resting level with getDigitalValue() BEFORE eventOn() so the
    // nRF52 SENSE latch arms against the right level/transition.
    setPullMode(pinIndex, BlocksPullMode::Up);
    uBit.io.pin[pinIndex].getDigitalValue();
    uBit.io.pin[pinIndex].eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
  } else if (eventType == BlocksPinEventType::ON_PULSE) {
    uBit.messageBus.listen(
        componentID,
        MICROBIT_PIN_EVT_PULSE_HI,
        this,
        &BlocksDevice::onPinEvent,
        MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
    uBit.messageBus.listen(
        componentID,
        MICROBIT_PIN_EVT_PULSE_LO,
        this,
        &BlocksDevice::onPinEvent,
        MESSAGE_BUS_LISTENER_QUEUE_IF_BUSY);
#if MICROBIT_CODAL
    // ?? Freeze BLE when onEvent(PULSE) first time. ??
    uBit.io.pin[pinIndex].eventOn(MICROBIT_PIN_EVENT_NONE); // workaround to prevent to freeze BLE
    uBit.io.pin[pinIndex].eventOn(MICROBIT_PIN_EVENT_ON_PULSE);
    // ?? Pull-mode is released and will not be reset in this thread. ??
    setPullMode(pinIndex, pullMode[pinIndex]); // does not work?
#else // NOT MICROBIT_CODAL
    uBit.io.pin[pinIndex].eventOn(MICROBIT_PIN_EVENT_ON_PULSE);
#endif // NOT MICROBIT_CODAL
  } else if (eventType == BlocksPinEventType::NONE) {
    uBit.io.pin[pinIndex].eventOn(MICROBIT_PIN_EVENT_NONE);
#if MICROBIT_CODAL
    // ?? Pull-mode is released and will not be reset in this thread. ??
    setPullMode(pinIndex, pullMode[pinIndex]); // does not work?
#endif // MICROBIT_CODAL
  }
  // Record the armed event type so updateVersionData() reports it in the
  // pin-event bitmask (the editor reconciles + re-arms a dropped SET_EVENT).
  // isGpio(pinIndex) was already guaranteed at the top of this function.
  if (pinIndex >= 0 &&
      pinIndex < (int)(sizeof(pinEventMode) / sizeof(pinEventMode[0]))) {
    pinEventMode[pinIndex] = (int8_t)eventType;
    // Start the post-arm guard window so onPinEvent drops the phantom edge that
    // arming produces. Only on a real arm (NONE is a disarm — nothing to guard).
    if (eventType != BlocksPinEventType::NONE) {
      pinEventArmTime[pinIndex] = (uint32_t)system_timer_current_time();
    }
  }
}

/**
 * Callback. Invoked when a pin event sent.
 */
void BlocksDevice::onPinEvent(MicroBitEvent evt) {
  // conventional scheme to convert from componentID to pin index in v1 and v2.
  int pinIndex = evt.source - 100;
  // Drop the phantom edge a pin emits right after it is (re)armed (pull-up flip /
  // SENSE latch), so re-arming on green-flag / reconnect doesn't surface a ghost
  // event. Mirrors the touch pad arm guard. A real press settles well after this
  // short window, so genuine input is never dropped.
  if (pinIndex >= 0 &&
      pinIndex < (int)(sizeof(pinEventArmTime) / sizeof(pinEventArmTime[0]))) {
    uint32_t now = (uint32_t)system_timer_current_time();
    if (now - pinEventArmTime[pinIndex] < PIN_EVENT_ARM_GUARD_MS) return;
  }

  uint8_t *data = moreService->pinEventChBuffer;

  // pinIndex is sent as uint8_t.
  data[0] = pinIndex;
  // event ID is sent as uint8_t.
  data[1] = (uint8_t)evt.value;

  // event timestamp is sent as uint32_t little-endian
  // downcast from uint64_t value.
  uint32_t timestamp = (uint32_t)evt.timestamp;
  memcpy(&(data[2]), &timestamp, 4);
  data[BLOCKS_DATA_FORMAT_INDEX] = BlocksDataFormat::PIN_EVENT;
#if BLOCKS_USE_DAP
  // Prefer BLE whenever a BLE central is connected; otherwise route the event to
  // the CMSIS-DAP mailbox (the codal USB transport — it replaces serial here).
  if (dapConnected && !moreService->isBleConnected()) {
    dapService->notifyOnDap(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_DAP
#if BLOCKS_USE_SERIAL
  // Prefer BLE whenever a BLE central is connected; only fall back to serial
  // when BLE is NOT connected. `serialConnected` latches true on the first USB
  // connect and is never cleared, so without the BLE check every event would
  // route to (a possibly-dead) serial forever after USB was ever plugged —
  // which is why button/touch/data events died after a USB→BLE switch.
  if (serialConnected && !moreService->isBleConnected()) {
    serialService->notifyOnSerial(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_SERIAL
  moreService->notifyPinEvent();
}

/**
 * @brief Invoked when button state changed.
 * 
 * @param evt event which has button states
 */
void BlocksDevice::onButtonChanged(MicroBitEvent evt) {
  uint32_t now = (uint32_t)system_timer_current_time();
  // Connect-time ghost guard: drop ALL button/touch events during the window
  // right after a BLE (re)connect. On-device the connect sequence emits a
  // phantom DOWN/UP/CLICK burst across Button A AND the freshly-armed touch pads
  // (P0..P3) with no real interaction. This covers Button A (source 1), which
  // does not calibrate and so is not caught by the per-pad arm guard below.
  if ((now - bleConnectTime) < CONNECT_GUARD_MS) {
    return;
  }
  // Per-pad guard: also drop a touch pad's events during its post-arm
  // calibration/settle window (covers pads armed later in the session).
  // evt.source for a touch pad is MICROBIT_ID_IO_P0..P3 (100..103).
  int touchPin = (int)evt.source - 100;
  if (touchPin >= 0 && touchPin <= 3 && touchMode[touchPin] &&
      (now - touchArmTime[touchPin]) < TOUCH_ARM_GUARD_MS) {
    return;
  }
  uint8_t *data = moreService->actionEventChBuffer;
  data[0] = BlocksActionEvent::BUTTON;
  // source is a component ID that generated the event as uint16_t little-endian.
  // MICROBIT_ID_BUTTON_A, MICROBIT_ID_IO_P0, MICROBIT_ID_LOGO, etc.
  memcpy(&(data[1]), &evt.source, 2);
  // Event ID send as uint16_t little-endian.
  // MICROBIT_BUTTON_EVT_DOWN, MICROBIT_BUTTON_EVT_CLICK, etc.
  data[3] = (uint8_t)evt.value;
  // Timestamp of the event send as uint32_t little-endian.
  // downcast from uint64_t value.
  uint32_t timestamp = (uint32_t)evt.timestamp;
  memcpy(&(data[4]), &timestamp, 4);
  data[BLOCKS_DATA_FORMAT_INDEX] = BlocksDataFormat::ACTION_EVENT;
#if BLOCKS_USE_DAP
  // Prefer BLE whenever a BLE central is connected; otherwise route the event to
  // the CMSIS-DAP mailbox (the codal USB transport — it replaces serial here).
  if (dapConnected && !moreService->isBleConnected()) {
    dapService->notifyOnDap(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_DAP
#if BLOCKS_USE_SERIAL
  // Prefer BLE whenever a BLE central is connected; only fall back to serial
  // when BLE is NOT connected. `serialConnected` latches true on the first USB
  // connect and is never cleared, so without the BLE check every event would
  // route to (a possibly-dead) serial forever after USB was ever plugged —
  // which is why button/touch/data events died after a USB→BLE switch.
  if (serialConnected && !moreService->isBleConnected()) {
    serialService->notifyOnSerial(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_SERIAL
  moreService->notifyActionEvent();
}

/**
 * @brief Invoked when gesture state changed.
 * 
 * @param evt event which has gesture states.
 */
void BlocksDevice::onGestureChanged(MicroBitEvent evt) {
  uint8_t *data = moreService->actionEventChBuffer;
  data[0] = BlocksActionEvent::GESTURE;
  // Event ID send as uint8_t.
  // MICROBIT_ACCELEROMETER_EVT_TILT_UP, MICROBIT_ACCELEROMETER_EVT_FACE_UP, etc.
  data[1] = (uint8_t)evt.value;
  // Timestamp of the event send as uint32_t little-endian.
  // downcast from uint64_t value.
  uint32_t timestamp = (uint32_t)evt.timestamp;
  memcpy(&(data[2]), &timestamp, 4);
  data[BLOCKS_DATA_FORMAT_INDEX] = BlocksDataFormat::ACTION_EVENT;
#if BLOCKS_USE_DAP
  // Prefer BLE whenever a BLE central is connected; otherwise route the event to
  // the CMSIS-DAP mailbox (the codal USB transport — it replaces serial here).
  if (dapConnected && !moreService->isBleConnected()) {
    dapService->notifyOnDap(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_DAP
#if BLOCKS_USE_SERIAL
  // Prefer BLE whenever a BLE central is connected; only fall back to serial
  // when BLE is NOT connected. `serialConnected` latches true on the first USB
  // connect and is never cleared, so without the BLE check every event would
  // route to (a possibly-dead) serial forever after USB was ever plugged —
  // which is why button/touch/data events died after a USB→BLE switch.
  if (serialConnected && !moreService->isBleConnected()) {
    serialService->notifyOnSerial(0x0110, data, BLOCKS_CH_BUFFER_SIZE_NOTIFY);
    return;
  }
#endif // BLOCKS_USE_SERIAL
  moreService->notifyActionEvent();
}

/**
 * @brief Normalize angle when upside down.
 * 
 * @param heading value of the compass heading
 * @return normalizes angle relative to north [degree]
 */
int BlocksDevice::normalizeCompassHeading(int heading) {
  if (uBit.accelerometer.getZ() > 0) {
    if (heading <= 180) {
      heading = 180 - heading;
    } else {
      heading = 360 - (heading - 180);
    }
  }
  return heading;
}

/**
 * @brief Set pull-mode.
 * 
 * @param pinIndex index to set
 * @param pull pull-mode to set
 */
void BlocksDevice::setPullMode(int pinIndex, BlocksPullMode pull) {
  pullMode[pinIndex] = pull;
#if MICROBIT_CODAL
  switch (pull) {
  case BlocksPullMode::None:
    uBit.io.pin[pinIndex].setPull(PullMode::None);
    break;
  case BlocksPullMode::Up:
    uBit.io.pin[pinIndex].setPull(PullMode::Up);
    break;
  case BlocksPullMode::Down:
    uBit.io.pin[pinIndex].setPull(PullMode::Down);
    break;

  default:
    break;
  }
#else // NOT MICROBIT_CODAL
  switch (pull) {
  case BlocksPullMode::None:
    uBit.io.pin[pinIndex].setPull(PinMode::PullNone);
    break;
  case BlocksPullMode::Up:
    uBit.io.pin[pinIndex].setPull(PinMode::PullUp);
    break;
  case BlocksPullMode::Down:
    uBit.io.pin[pinIndex].setPull(PinMode::PullDown);
    break;

  default:
    break;
  }
#endif // NOT MICROBIT_CODAL
}

/**
 * @brief Set the value on the pin as digital output.
 * 
 * @param pinIndex index in edge pins
 * @param value digital value [0 | 1]
 */
void BlocksDevice::setDigitalValue(int pinIndex, int value) {
  uBit.io.pin[pinIndex].setDigitalValue(value);
}

/**
 * @brief Set the value on the pin as analog output (PWM).
 * 
 * @param pinIndex index in edge pins
 * @param value analog value (0..1024)
 */
void BlocksDevice::setAnalogValue(int pinIndex, int value) {
#if MICROBIT_CODAL
  // stable level is 0 .. 1022 in micro:bit v2,
  int validValue = value > 1022 ? 1022 : value;
#else // NOT MICROBIT_CODAL
  // stable level is 0 .. 1021 in micro:bit v1.5,
  int validValue = value > 1021 ? 1021 : value;
#endif // NOT MICROBIT_CODAL
  uBit.io.pin[pinIndex].setAnalogValue(validValue);
}

/**
 * @brief Set the value on the pin as servo driver.
 * 
 * @param pinIndex index in edge pins
 * @param angle the level to set on the output pin, in the range 0 - 180.
 * @param range which gives the span of possible values the i.e. the lower and upper bounds (center +/- range/2). Defaults to DEVICE_PIN_DEFAULT_SERVO_RANGE.
 * @param center the center point from which to calculate the lower and upper bounds. Defaults to DEVICE_PIN_DEFAULT_SERVO_CENTER
 */
void BlocksDevice::setServoValue(int pinIndex, int angle, int range,
                                   int center) {
  uBit.io.pin[pinIndex].setServoValue(angle, range, center);
}

/**
 * @brief Display friendly name of the micro:bit.
 * 
 */
void BlocksDevice::displayFriendlyName() {
  if (serialConnected)
    return;
  uBit.display.scrollAsync(ManagedString(microbit_friendly_name()), 120);
}

/**
 * @brief Display software version of Microbit More.
 * 
 */
void BlocksDevice::displayVersion() {
  uBit.display.scrollAsync(ManagedString(" -M 0.2.5- "), 120);
}

/**
 * @brief Whether the pin is a GPIO of not.
 * 
 * @param pinIndex index in edge pins
 * @return true the pin is a GPIO
 * @return false the pin is not a GPIO
 */
bool BlocksDevice::isGpio(int pinIndex) {
  for (size_t i = 0; i < (sizeof(gpioPin) / sizeof(gpioPin[0])); i++) {
    if (pinIndex == gpioPin[i])
      return true;
  }
  return false;
}