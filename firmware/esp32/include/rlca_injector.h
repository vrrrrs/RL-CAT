/**
 * Injector: turns console text commands into standard protocol frames and
 * enforces the arbitration rule (docs/32-esp32-bridge.md §4).
 *
 * P1 provides the plumbing plus two bench-test commands (`raw`, `hb`).
 * P2 adds the joint/mode/calibration command set (`i`, `s`, `z`, `c`, `r`),
 * P3 connects the arbitration gate fed by TEL_FAST.status.
 */
#pragma once

#include <stddef.h>
#include <stdbool.h>

/** Handle one console line.
 *  @param line       NUL-terminated, already trimmed, non-empty
 *  @param reply      buffer for the console answer (always '\0' terminated)
 *  @param reply_len  size of `reply`
 *  @return true when the command was recognised. */
bool rlca_injector_handle_line(const char *line, char *reply, size_t reply_len);
