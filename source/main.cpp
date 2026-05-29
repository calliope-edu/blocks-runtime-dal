/**
 * blocks-runtime-dal — entry point for the Calliope Blocks editor runtime on
 * mini 1/2 (nRF51 / microbit-dal). DAL equivalent of FIRMWARE/blocks-runtime
 * (codal / mini 3).
 *
 * No MakeCode / TypeScript: the program logic runs in the browser (scratch-vm),
 * and this device binary is a fixed sensor/actuator proxy exposing the MbitMore
 * binary protocol over BLE (service 0b50f3e4-...) and over USB-CDC serial.
 *
 * Boot: init the DAL MicroBit, construct MbitMoreServiceDAL (registers the
 * GATT service + grabs the MbitMoreDevice singleton), start MbitMoreSerial
 * (USB-CDC, spawns its own RX fiber), spawn a broadcaster fiber that fills the
 * STATE characteristic every ~19 ms (matches the codal runtime), then hand to
 * the DAL scheduler.
 */
#include "MicroBit.h"
#include "MbitMoreServiceDAL.h"
#include "MbitMoreDevice.h"
// NB: no MbitMoreSerial here. MbitMoreCommon.h sets MBIT_MORE_USE_SERIAL=0 for
// V1/DAL ("v1 has not enough memory space"), so the mini 1/2 blocks-runtime is
// BLE-only — scratch-vm drives it over the MbitMore GATT service. (The codal /
// mini 3 runtime adds USB-CDC; V1 deliberately omits it.)

namespace pxt {
    /** The global MicroBit DAL instance the MbitMore sources reach via pxt.h. */
    MicroBit uBit;
}
using pxt::uBit;

// Broadcaster: keeps the MbitMore STATE characteristic fresh so scratch-vm sees
// live sensor data and the campus widget can detect the runtime over BLE.
static const int UPDATE_PERIOD_MS = 19;
static MbitMoreServiceDAL *s_service = NULL;

// Mirrors pxt-Scratch-more's pattern: the MbitMore GATT service is created
// FROM A FIBER, after the DAL scheduler is running and the BLE stack has
// settled. When called synchronously from main() right after uBit.init(),
// uBit.ble->addService() silently fails to make the service visible in GATT
// discovery (DFU + partial-flashing services that the DAL registers from
// inside bleManager.init() DO appear; user services added post-init don't).
static void blocksRuntimeFiber() {
    uBit.sleep(200);  // let BLE stack finish initialising

    // Registers the MbitMore GATT service + characteristics and wires the
    // MbitMoreDevice singleton (created with pxt::uBit).
    s_service = new MbitMoreServiceDAL();

    // microbit-dal's MicroBit::init() only starts advertising via
    // pairingMode() if A+B are held at boot. For the blocks runtime we want
    // BLE to be discoverable all the time so the Scratch/Blocks editor can
    // find the mini without a button gesture. Call advertise() explicitly
    // AFTER the GATT service is registered. With MICROBIT_BLE_OPEN=1 the link
    // is open (no bonding) so no whitelist gate prevents the editor connecting.
    uBit.bleManager.advertise();

    while (s_service != NULL) {
        s_service->update();
        uBit.sleep(UPDATE_PERIOD_MS);
    }
}

int main() {
    uBit.init();

    create_fiber(blocksRuntimeFiber);

    // Boot heart: visual confirmation DAL init succeeded; scratch-vm overwrites
    // the matrix on its first displayMatrix command.
    uBit.display.print(MicroBitImage("0,255,0,255,0\n255,255,255,255,255\n255,255,255,255,255\n0,255,255,255,0\n0,0,255,0,0\n"));

    release_fiber();
    return 0;
}
