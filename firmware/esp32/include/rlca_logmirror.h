/**
 * Mirror ESP_LOG output to the console TCP port (3334).
 *
 * Rationale: the ESP32's own diagnostics are the only way to see what the board
 * thinks while the robot is on the bench, and the USB-Serial-JTAG port may be
 * unreachable once the board is mounted.  Port 3334 is the ESP32's own channel
 * (docs/36-esp32-plan.md §3.4), so the log belongs there.
 */
#pragma once

/** Install the log hook.  Call after rlca_net_start() so the console port exists. */
void rlca_logmirror_start(void);
