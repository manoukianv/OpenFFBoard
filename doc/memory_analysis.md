# OpenFFBoard Memory Analysis (STM32F407VG & STM32F411RE)

## Overview
This document provides a systematic analysis of the memory footprint of core OpenFFBoard classes on `feat/anticogging`, assuming a maximum configuration of **2 axes**. It reflects the migration from the legacy LUT-based anti-cogging (`data_cogging` / `CoggingCalibData`) to the **Continuous DFT-128** harmonic pipeline (`Harmonic[20]` + `CoggingDftBuffers`), and details the 78 KB FreeRTOS heap strategy (`heap_4.c`) to prevent heap exhaustion, fragmentation, and thread stack overflows.

## Target Architecture Context

Both the **STM32F407VG** and **STM32F411RE** share the same primary memory constraint: **128 KB of general SRAM**.
To maintain architectural consistency across targets, the FreeRTOS heap (`ucHeap`) is centrally managed within this 128 KB limit (`configTOTAL_HEAP_SIZE = 78 * 1024`), without relying on the F407's isolated CCMRAM.

## Memory Consumption Profile (Verified 2-Axis Setup)

| Component / Thread | Permanent Memory (Heap) | Thread Stack Allocation | Temporary Spikes (Heap) | Overflow Risks & Observations |
| :--- | :--- | :--- | :--- | :--- |
| **TMC4671** (×2 axes) | **~0.5 KB** (`2 × 240 B` for `cogging_harmonics[20]`) | **8.0 KB** (`2 × 1024`-word `TMC` threads) + **0.3 KB** (`80`-word `TMCENC` if external encoder) | **3.0 KB** (`CoggingDftBuffers`, `6 × 128` floats) | Replaces the legacy ~12.6 KB permanent LUT (`data_cogging`) and 17.3 KB spike (`CoggingCalibData`). DFT buffers are allocated via `pvPortMalloc` only during calibration and freed unconditionally in `cleanup:`. |
| **Axes & HidFFB** | **~13.0 KB** (Effects Array) | **1.25 KB** (`312`-word `FFBMAIN` thread) | **~5.1 KB** (Biquad Filters) | Array of 40 `FFB_Effect` is ~12 KB. Dynamic Biquad allocation is used for render quality. |
| **Command Interfaces** | ~1.0 KB | **3.9 KB** (`600`-word `CMD_MAIN` + `384`-word `CDCCMD`) | **~5.8 KB** (via `help`) | **[FIXED]** `help` command consumes ~5.8 KB peak. Stack rebalancing on `CDCCMD` and `CMD_MAIN` keeps command handling safe. |
| **BiSS-C Encoder** | < 1.0 KB | **0.5 KB** (`128`-word `BISSENC` thread) | - | **[CRITICAL FIX]** Original 64-word stack had only 60 bytes of headroom; increased to 128 words. |
| **Error Printer** | < 0.2 KB | **2.0 KB** (`512`-word `errprint` thread) | - | **[CRITICAL FIX]** Original 256-word (1 KB) stack overflowed in Debug (`-O0`) on USB connect (`1,152 B` peak via `broadcastCommandReplyAsync` → `CDC_CommandInterface::sendReplies`), corrupting adjacent `ucHeap` (`AxesManager::axes[0]`). Increased to 512 words. |

**Total Estimated Permanent Heap Usage (Base + Stacks):** ~36.0 KB *(includes +1.0 KB for `errprint` stack fix)*.
**Total Available Heap:** **78.0 KB** (`configTOTAL_HEAP_SIZE`, `heap_4.c`).
**Idle Heap Free:** **~42.0 KB**.

### Peak Stress Scenarios
1. **Help Command:** Consumes ~5.8 KB. Remaining heap: **~37.2 KB**. *(Status: **STABLE**)*
2. **Cogging Calibration (DFT-128):** Consumes **3.0 KB** (`CoggingDftBuffers`). Remaining heap: **~40.0 KB**. *(Status: **SAFE** — down from 17.3 KB peak / 18.7 KB remaining in the legacy LUT implementation)*
3. **Concurrent Help + Cogging Calibration:** Combined temporary spike ~8.8 KB. Remaining heap: **~34.2 KB**. *(Status: **SAFE**)*

## Thread Stack Configuration

Current verified stack allocations in `Firmware/FFBoard/`:

1. **TMC Thread (`TMC_THREAD_MEM`):** **1024 words (4 KB)** per axis (`TMC4671.h`). Worst-case static stack call chain during cogging calibration (`Run()` → `handleStateCoggingCalibration()` → `TMC4671CoggingCalibrator::run()` → `profileMultiSpeedRipple()`) is **1,016 bytes** (verified via `-fstack-usage` in `TMC4671.su` / `TMC4671_cogging.su`), leaving > 2.5 KB of headroom for SPI/HAL/FreeRTOS calls.
2. **TMC External Encoder Updater (`TMCENC`):** **80 words (320 B)** (`TMC4671.cpp`, 8 B static frame).
3. **BISSENC:** **128 words (512 B)** (`EncoderBissC.cpp`, increased from 64 words to prevent stack overflow).
4. **CDCCMD:** **384 words (1.5 KB)** (`CommandInterface.cpp`, reduced from 512 words).
5. **CMD_MAIN:** **600 words (2.4 KB)** (`FFBoardMainCommandThread.cpp`, reduced from 700 words).
6. **FFBMAIN:** **312 words (1.25 KB)** (`FFBHIDMain.cpp`).
7. **ErrorPrinter (`errprint`):** **512 words (2.0 KB)** (`ErrorHandler.cpp`, increased from 256 words after a Debug `-O0` stack overflow of `1,152 B`).
8. **USB:** **330 words (1.32 KB)** (`USBdevice.cpp`).

### Debug Build (`-O0`) Stack Watermarks & Watchlist (F407VG)

Measured via STM32CubeIDE FreeRTOS Task List on F407VG Debug (`-O0`, USB connected, idle FFB, no active effects):

| Thread | Measured Peak / Stack Size | Usage | Free Margin | Notes / Future Increase if Debug Overflow Occurs |
| :--- | :---: | :---: | :---: | :--- |
| **`CMD_MAIN`** | **2,172 B / 2,400 B** (600 words) | **90.5 %** | **228 B** | Tight in `-O0`; heavy commands (`tmc`, `fx`, `help`) may overflow. If needed, bump `FFBoardMainCommandThread.cpp` to **768 words (3,072 B)** *(+672 B)*. |
| **`USB`** | **1,008 B / 1,320 B** (330 words) | **76.4 %** | **312 B** | Tight during USB enumeration (`snprintf` in `getUsbSerial`) + HID FFB callbacks. If needed, bump `USBdevice.cpp` to **448 words (1,792 B)** *(+472 B)*. |
| **`defaultTask`** | 1,576 B / 2,048 B (512 words) | 77.0 % | 472 B | Peak reached during boot (`cppmain()`). Monitor if startup sequence grows. |
| **`FFBMAIN`** | 748 B / 1,248 B (312 words) | 59.9 % | 500 B | Idle measurement (no active FFB effects). In `-O0`, active `EffectsCalculator` frames add ~360 B (~88–95 %). If needed, bump `FFBHIDMain.cpp` to **448 words (1,792 B)** *(+544 B)*. |
| **`errprint`** | 1,152 B / 2,048 B (512 words) | 56.3 % | 896 B | Fixed from 256 words (1,024 B). Safe margin in `-O0`. |
| **`CDCCMD`** | 896 B / 1,536 B (384 words) | 58.3 % | 640 B | Safe. |
| **`TMCENC`** | 156 B / 320 B (80 words) | 48.8 % | 164 B | Safe. |
| **`HIDCMD`** | 244 B / 512 B (128 words) | 47.7 % | 268 B | Safe. |
| **`BISSENC`** | 204 B / 512 B (128 words) | 39.8 % | 308 B | Safe. |
| **`UARTCMD`** | 156 B / 600 B (150 words) | 26.0 % | 444 B | Safe. |
| **`TMC`** | 848 B / 4,096 B (1,024 words) | 20.7 % | 3,248 B | Safe (headroom reserved for cogging calibration). |

## Linker & SRAM Constraints (Verified via `arm-none-eabi-size`)

The FreeRTOS heap is sized at **78 KB** (`79,872 B`) across F407VG, F407VG_DISCO, and F411RE within the 128 KB (`131,072 B`) hardware SRAM:

| Target | Flash (`text + data`) | `data` | `bss` *(incl. 78 KB `ucHeap`)* | Static `.bss`/`.data` *(excl. heap)* | Total Linked SRAM | Free SRAM / 128 KB |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **F407VG** | 354,932 B (346.6 KB) | 2,524 B | 100,012 B | ~22.1 KB | **102,536 B (100.1 KB)** | **27.9 KB (21.8 %)** |
| **F407VG_DISCO** | 275,096 B (268.6 KB) | 2,444 B | 99,540 B | ~21.6 KB | **101,984 B (99.6 KB)** | **28.4 KB (22.2 %)** |
| **F411RE** | 280,416 B (273.8 KB) | 2,352 B | 95,232 B | ~17.3 KB | **97,584 B (95.3 KB)** | **32.7 KB (25.5 %)** |

## Recommendations
- **Keep temporary calibration buffers on the FreeRTOS heap:** Allocating `CoggingDftBuffers` (3 KB) via `pvPortMalloc` / `vPortFree` (`heap_4.c`) uses 0 bytes during normal FFB operation, leaves ~40 KB of FreeRTOS heap free during calibration, and avoids permanently consuming static SRAM or TMC thread stack.
- **Verify stack high-water marks after major changes:** Use `sys.tasklist` to monitor `BISSENC` (128 words) and `TMCENC` (80 words) whenever SPI/encoder drivers are updated.
- **Optional future optimization:** If additional heap is ever needed in 2-axis builds, `TMC_THREAD_MEM` (currently 1024 words = 8 KB for 2 axes) could be trimmed after `sys.tasklist` verification, though current ~40 KB headroom makes this unnecessary.
