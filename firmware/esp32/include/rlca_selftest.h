/**
 * On-target self test.
 *
 * The same vectors that firmware/esp32/test/test_protocol.c checks on the PC,
 * run once at boot on the real CPU: it catches endianness / alignment /
 * optimiser surprises in protocol.c, and gives the operator a one-line
 * PASS/FAIL they can read over USB or the console port (command `selftest`).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "rlca_esp32_config.h"

/** Run the built-in checks.  Fills `report` (always NUL terminated) with a
 *  one-line summary and returns true when everything passed. */
bool rlca_selftest_run(char *report, size_t report_len);
