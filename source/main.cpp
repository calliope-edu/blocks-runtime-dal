/**
 * blocks-runtime-dal — entry point for the Calliope Blocks editor runtime on
 * mini 1/2 (nRF51 / microbit-dal). DAL equivalent of FIRMWARE/blocks-runtime
 * (codal / mini 3); shares its BlocksDevice / BlocksCommon / BlocksSerial
 * sources (selected via the `#if !MICROBIT_CODAL` branches) and supplies a
 * DAL-flavoured BlocksServiceDAL for the legacy mbed BLE stack.
 *
 * No MakeCode / TypeScript: the program logic runs in the browser (scratch-vm),
 * and this device binary is a fixed sensor/actuator proxy exposing the
 * consolidated 5-characteristic Blocks protocol over BLE (service 0b50f3e4-...).
 *
 * Boot: init the DAL MicroBit, construct BlocksServiceDAL (registers the GATT
 * service + grabs the BlocksDevice singleton), draw the boot name pattern once,
 * start advertising, then spawn a broadcaster fiber that fills the STATE/MOTION
 * characteristics every ~19 ms and hands to the DAL scheduler.
 *
 * RAM: nRF51 (mini 1/2) has only ~8 KB app RAM above the S110 SoftDevice, which
 * is extremely tight for this BLE runtime. Fitting it (and surviving a live
 * connection) required several RAM cuts — see config.json (dfu_service off,
 * pairing_mode off), BlocksServiceDAL (STATE/MOTION are READ-only, no NOTIFY),
 * and the build (MICROBIT_STACK_SIZE trimmed to 1024 to grow the heap). Without
 * these the runtime panics 020 (out of memory) at service creation / on connect.
 * device_info_service is kept ON — the connection widget requires 0x180A.
 */
#include "MicroBit.h"
#include "BlocksServiceDAL.h"
#include "BlocksDevice.h"
// NB: no BlocksSerial here. BlocksCommon.h sets BLOCKS_USE_SERIAL=0 for V1/DAL
// ("v1 has not enough memory space"), so the mini 1/2 blocks-runtime is
// BLE-only — scratch-vm drives it over the Blocks GATT service. (The codal /
// mini 3 runtime adds USB-CDC; V1 deliberately omits it.)

namespace pxt {
    /** The global MicroBit DAL instance the shared Blocks sources reach via
     *  PxtShim.h. */
    MicroBit uBit;
}
using pxt::uBit;

// Broadcaster: keeps the STATE/MOTION characteristics fresh so scratch-vm sees
// live sensor data and the campus widget can detect the runtime over BLE.
static const int UPDATE_PERIOD_MS = 19;
static BlocksServiceDAL *s_service = NULL;

// Mirrors pxt-Scratch-more's pattern: the GATT service is created FROM A FIBER,
// after the DAL scheduler is running and the BLE stack has settled. When called
// synchronously from main() right after uBit.init(), uBit.ble->addService()
// silently fails to make the service visible in GATT discovery (DFU +
// partial-flashing services that the DAL registers from inside
// bleManager.init() DO appear; user services added post-init don't).
static void blocksRuntimeFiber() {
    uBit.sleep(200);  // let BLE stack finish initialising

    // Registers the Blocks GATT service + characteristics and wires the
    // BlocksDevice singleton (created with pxt::uBit). Constructing the device
    // HERE (not on the main fiber) is deliberate and load-bearing: the
    // BlocksDevice ctor registers the button/gesture message-bus listeners, and
    // BlocksServiceDAL's ctor sets blocks->moreService in the same straight-line
    // call (no scheduler yield in between). If the device were instead built on
    // the main fiber before this point, a gesture event (e.g. FACE_UP on a flat
    // board fires ~100 ms after boot) could reach onGestureChanged while
    // moreService is still null -> null-deref HardFault. Mirrors the codal
    // build, where the service is likewise constructed before the device is used.
    s_service = new BlocksServiceDAL();

    // Boot/idle indicator: the device name histogram pattern (static, one-time,
    // non-blocking) so the user can identify the mini. scratch-vm overwrites the
    // matrix on its first displayMatrix command after connecting.
    BlocksDevice::getInstance().displayNamePattern();

    // microbit-dal's MicroBit::init() only starts advertising via pairingMode()
    // if A+B are held at boot. For the blocks runtime we want BLE discoverable
    // all the time so the Scratch/Blocks editor can find the mini without a
    // button gesture. Call advertise() explicitly AFTER the GATT service is
    // registered. With MICROBIT_BLE_OPEN=1 the link is open (no bonding) so no
    // whitelist gate prevents the editor connecting.
    uBit.bleManager.advertise();

    while (s_service != NULL) {
        s_service->update();
        uBit.sleep(UPDATE_PERIOD_MS);
    }
}

int main() {
    uBit.init();

    create_fiber(blocksRuntimeFiber);

    // NB: the boot/idle name-pattern is shown from blocksRuntimeFiber AFTER the
    // service (and thus moreService) exists — deliberately NOT here, to avoid
    // constructing BlocksDevice while moreService is still null (boot-window
    // null-deref; see blocksRuntimeFiber).

    release_fiber();
    return 0;
}
