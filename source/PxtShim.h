/**
 * Minimal pxt.h shim for the DAL blocks-runtime (Calliope mini 1/2, nRF51).
 *
 * The shared Blocks sources (BlocksDevice / BlocksCommon / BlocksSerial) were
 * lifted from pxt-Scratch-more, where they #include "pxt.h" and use the
 * pxt-namespace global MicroBit instance. The full MakeCode pxt.h pulls in the
 * TypeScript runtime; we only need `pxt::uBit`.
 *
 * Crucially we do NOT define MICROBIT_CODAL: that selects the `#if
 * !MICROBIT_CODAL` (DAL / micro:bit V1) branches in the shared Blocks sources,
 * which is what we build for the nRF51 Calliope mini 1/2. (The microphone path
 * codal::LevelDetectorSPL, pxt::getMicrophoneLevel(), the on-board neopixel RGB
 * via uBit.io.RGB, the data-messaging store, etc. are all `#if MICROBIT_CODAL`
 * and excluded here.) The codal / mini 3 runtime ships its own PxtShim.h that
 * force-defines MICROBIT_CODAL=1 instead.
 */
#ifndef BLOCKS_RUNTIME_DAL_PXT_SHIM_H
#define BLOCKS_RUNTIME_DAL_PXT_SHIM_H

#include "MicroBit.h"

namespace pxt {
    /** The global MicroBit DAL instance, defined in main.cpp. */
    extern MicroBit uBit;
}

// The shared Blocks*.cpp use unqualified `uBit` (the full pxt.h does a
// translation-unit `using namespace pxt;`). Mirror just that symbol.
using pxt::uBit;

#endif // BLOCKS_RUNTIME_DAL_PXT_SHIM_H
