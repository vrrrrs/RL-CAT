/**
 * Host unit tests for the injection gate (rlca_gate_decide).
 *
 * This is the safety-critical decision of the whole ESP32 firmware: it decides
 * whether the ESP32 may command the robot.  Keeping it a pure function lets it
 * be verified on a PC, with no board and no STM32 (docs/36-esp32-plan.md §6).
 *
 * Build & run: firmware/tools/test_esp32_host.ps1
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "rlca_arbiter.h"
#include "rlca_esp32_config.h"

static int g_pass = 0;
static int g_fail = 0;

static void expect(const char *what, rlca_gate_t got, rlca_gate_t want)
{
    if (got == want) {
        g_pass++;
        printf("  ok   %s\n", what);
    } else {
        g_fail++;
        printf("  FAIL %s (got %d, want %d)\n", what, (int)got, (int)want);
    }
}

static void expect_true(const char *what, bool got)
{
    if (got) {
        g_pass++;
        printf("  ok   %s\n", what);
    } else {
        g_fail++;
        printf("  FAIL %s (got false)\n", what);
    }
}

static void expect_false(const char *what, bool got)
{
    if (!got) {
        g_pass++;
        printf("  ok   %s\n", what);
    } else {
        g_fail++;
        printf("  FAIL %s (got true)\n", what);
    }
}

int main(void)
{
    const uint8_t LINK = RLCA_STAT_LINK_OK;
    const int64_t stale = (int64_t)RLCA_ARB_STATUS_STALE_MS;

    printf("=== RL-CAT ESP32 injection gate tests ===\n");
    printf("(RLCA_ARB_ENABLE=%d, BLOCK_ON_UNKNOWN=%d, stale=%lld ms)\n",
           RLCA_ARB_ENABLE, RLCA_ARB_BLOCK_ON_UNKNOWN, (long long)stale);

    /* No telemetry at all: the Pi's state is unknown. */
    expect("no telemetry -> NO_TELEMETRY",
           rlca_gate_decide(false, -1, 0), RLCA_GATE_NO_TELEMETRY);
    expect("no telemetry, even with LINK_OK set -> NO_TELEMETRY",
           rlca_gate_decide(false, -1, LINK), RLCA_GATE_NO_TELEMETRY);

    /* Fresh telemetry, Pi silent: injection allowed (phase-1 debug case). */
    expect("fresh, Pi silent -> ALLOW",
           rlca_gate_decide(true, 0, 0), RLCA_GATE_ALLOW);
    expect("fresh (100 ms), Pi silent -> ALLOW",
           rlca_gate_decide(true, 100, 0), RLCA_GATE_ALLOW);

    /* Fresh telemetry, Pi online: injection refused (the Pi owns the robot). */
    expect("fresh, LINK_OK -> PI_ONLINE",
           rlca_gate_decide(true, 0, LINK), RLCA_GATE_PI_ONLINE);
    expect("fresh (499 ms), LINK_OK -> PI_ONLINE",
           rlca_gate_decide(true, 499, LINK), RLCA_GATE_PI_ONLINE);

    /* Boundary: exactly at the stale limit is still fresh. */
    expect("age == stale -> still decided by status",
           rlca_gate_decide(true, stale, 0), RLCA_GATE_ALLOW);

    /* Older than the limit: unknown -> fail safe (blocked). */
    expect("stale +1 ms -> STALE",
           rlca_gate_decide(true, stale + 1, 0), RLCA_GATE_STALE);
    expect("stale +1 ms with LINK_OK -> STALE (stale outranks)",
           rlca_gate_decide(true, stale + 1, LINK), RLCA_GATE_STALE);
    expect("very stale -> STALE",
           rlca_gate_decide(true, 60000, 0), RLCA_GATE_STALE);

    /* Any other status bit must not open or close the gate. */
    expect("fresh, other status bits only -> ALLOW",
           rlca_gate_decide(true, 10, RLCA_STAT_READY | RLCA_STAT_IMU_OK | RLCA_STAT_ESTOP),
           RLCA_GATE_ALLOW);

    /* ---- cycle_cnt continuity (decimation tolerant) -------------------- */
    printf("-- TEL_FAST cycle_cnt check --\n");
    expect_true("delta 1 (100 Hz mirror) is fine", rlca_cycle_ok(0, 1));
    expect_true("delta 10 (10 Hz decimated mirror) is fine", rlca_cycle_ok(0, 10));
    expect_true("delta 50 is fine", rlca_cycle_ok(1000, 1050));
    expect_true("16-bit wrap is fine", rlca_cycle_ok(65530, 5));
    expect_false("duplicate counter is an anomaly", rlca_cycle_ok(7, 7));
    expect_false("backwards counter is an anomaly", rlca_cycle_ok(100, 99));
    expect_false("implausible jump is an anomaly", rlca_cycle_ok(0, 501));

    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
