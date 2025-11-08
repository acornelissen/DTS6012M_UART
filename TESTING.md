# Desktop Testing Guide

These instructions document how to build and run the DTS6012M UART library’s unit tests on a desktop machine (no Arduino toolchain needed). The tests rely on the lightweight stubs under `tests/` (e.g., `tests/Arduino.h`) and the conditional `DTS6012M_TEST_MODE` code paths inside the library.

## Prerequisites

- C++17-capable compiler (tested with `clang++`/`g++` on macOS).

## Build

```bash
cd /path/to/DTS6012M_UART
g++ -std=c++17 -DDTS6012M_TEST_MODE -I. -Itests \
    tests/test_DTS6012M_UART.cpp DTS6012M_UART.cpp \
    -o build/dts_tests
```

## Run

```bash
./build/dts_tests
```

The executable prints a PASS/FAIL summary for every unit test and exits with a non-zero status if a failure occurs.

## Notes

- The conditional stub in `DTS6012M_UART.h` replaces `HardwareSerial` when `DTS6012M_TEST_MODE` is defined; ensure that macro stays enabled for desktop builds.
- The host tests do **not** exercise real UART hardware. Always re-test on physical hardware before releasing.***
