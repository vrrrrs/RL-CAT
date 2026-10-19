<div align="center">

# RL-CAT

**A 12-DOF quadruped robot: mechanical design, electronics, embedded firmware,
real-time runtime and reinforcement-learning pipeline.**

![RL-CAT — real hardware](docs/pictures/stand.gif)

[中文说明](README.zh-CN.md) · [Documentation](docs/) · [Design record](docs/49-progress-review.md) · [License](LICENSE)

</div>

---

## Status

Results below are measured on hardware unless stated otherwise.

| Milestone | State | Evidence |
|---|---|---|
| **M1 · Static standing** | **Complete** (2026-10-18) | 10 s hold: position error **1.23°**, drift **0.00°**, jitter **0.00°** (σ), **0 of 926** samples outside `ACTIVE` |
| M2 · Scripted gait | Code complete, not yet run on hardware | `pi/gait.py` is open-loop and joint-space only |
| M3 · Sim-to-real locomotion | Toolchain complete, not yet achieved | Observation pipeline verified bit-exact against simulation (max\|diff\| = 0) |
| Sim-to-real gap | **Open** | [G1 / G2](docs/48-sim2real-gaps.md) |

### Scope of this release

The repository contains a complete and reproducible implementation of the robot's
hardware description, firmware, host runtime and training pipeline. It does not yet
contain a working locomotion controller: the robot stands, but does not walk.

The two outstanding sim-to-real discrepancies (joint-limit mismatch and actuator
latency) are analysed with supporting measurements in
[`docs/48-sim2real-gaps.md`](docs/48-sim2real-gaps.md).

---

## Hardware

| Item | Specification |
|---|---|
| Body | 12 DOF (3 per leg: abduction, hip pitch, knee); 3D-printed structure |
| Actuators | 12 × Feetech ST-3215-C018 serial bus servos, 4096 steps/rev |
| Realtime controller | STM32F103C8 (Cortex-M3, 72 MHz), 100 Hz control loop |
| Host computer | Raspberry Pi 5 (4 GB), policy inference and command generation |
| Wireless link | ESP32-C3 transparent bridge over Wi-Fi/TCP (optional) |
| IMU | WitMotion, bit-banged I²C |
| Battery | 3S LiPo, 100 kΩ / 20 kΩ divider on ADC |

<table>
<tr>
<td width="50%"><img src="docs/pictures/robot.jpg" alt="RL-CAT, front view"></td>
<td width="50%"><img src="docs/pictures/robot2.jpg" alt="RL-CAT, side view"></td>
</tr>
</table>

Mechanical CAD is published as STEP and STL under
[`hardware/mechanical/`](hardware/mechanical/) together with the printing parameters
used for the physical build.

---

## Architecture

![RL-CAT system architecture](docs/pictures/architecture.svg)

```
        ┌──────────────────────┐
        │  Raspberry Pi 5      │   RL policy (numpy MLP, 47-dim obs -> 12 actions)
        │  host runtime        │   tools: pose / gait / stand_test / rl_run
        └──────────┬───────────┘
                   │ UART 500 kbps, framed protocol, CRC-16
                   │   v CMD_JOINT (38 B @ 100 Hz)    ^ TEL_FAST (103 B @ 100 Hz)
        ┌──────────┴───────────┐
        │  STM32F103C8         │   100 Hz realtime loop
        │  - safety layer      │   - 4 e-stop conditions (link / battery / temperature / operator)
        │  - calibration       │   - geometric calibration table (no flash dependency)
        │  - servo bus master  │   - SyncWrite to 12 servos @ 1 Mbps
        └──────────┬───────────┘
                   │
        ┌──────────┴───────────┐
        │  12 x ST-3215 servos │
        └──────────────────────┘

        ESP32-C3 -- UART 115200 -- STM32 USART3   (optional wireless bridge)
```

The division of labour follows from timing requirements. The Raspberry Pi runs
general-purpose Linux, where scheduling latency of up to 500 ms has been observed
(see [`docs/49`](docs/49-progress-review.md) §4-A2). All operations with a hard deadline
therefore reside on the microcontroller; the host only produces joint targets.

### Simulation versus hardware

![Policy in Isaac Gym](docs/pictures/sim_all_speeds.gif)

The image above is the **simulation**: the trained policy walking at several commanded
velocities on flat terrain in Isaac Gym. It is not the physical robot.

On hardware the robot currently performs static standing (M1, quantified above).
The difference between the two behaviours is the subject of
[`docs/48-sim2real-gaps.md`](docs/48-sim2real-gaps.md).

---

## Quick start

### Software tests — no hardware, no Isaac Gym (about 30 s)

```bash
cd pi
./run_checks.sh
```

Requires Python 3.9 or later, `numpy` and `pyserial`. This is the same suite executed by CI.

### Firmware host tests — no hardware (about 1 min)

```bash
cd firmware
tools/test-host.ps1                  # Windows
bash firmware/tools/test-host.sh     # Linux / macOS
```

Seven suites, 1370 assertions.

### Policy observation verification (about 10 s)

```bash
cd pi
python3 rl_policy.py --selftest
```

This replays a recorded simulation trace and verifies that the 47-dimensional observation
layout, the scaling factors, the joint ordering and the action mapping are reproduced
bit-exactly (expected `max|diff| = 0`). It is the most important single check in the
repository: a deviation here dispatches actions to the wrong joints, which no other test
detects.

---

## Repository layout

```
docs/           25 documents: design, bring-up, acceptance, and the engineering record
firmware/
  stm32/        STM32F103 firmware (Keil MDK project; modules testable on the host)
  esp32/        ESP32-C3 wireless bridge (PlatformIO)
  tests/host/   7 suites / 1370 assertions, no hardware required
  tools/        build and test scripts
pi/             Raspberry Pi runtime: protocol, link, calibration, pose, gait, RL policy
  models/       exported policy (numpy .npz, ONNX) and the validation trace
  tests/        offline end-to-end tests
hardware/
  mechanical/   STEP and STL parts, printing parameters, robot.urdf
  electronics/  schematic and PCB
Sim/
  legged_gym/   training framework fork, with the rlcat environment
  RL/           robot assets, training scripts, implementation reports
.github/        CI, issue and pull-request templates, repository hygiene checks
```

---

## Design decisions

The following decisions account for most of the development effort. Each is documented
with the evidence that motivated it.

| Decision | Rationale |
|---|---|
| **Three distinct zero positions** | URDF zero (bent-leg stance), calibration pose (straight legs) and encoder zero are different references. Conflating them caused every joint to report ±3.14 rad. [`docs/30`](docs/30-firmware.md) §3.3 |
| **Geometric calibration table** | Given that all twelve servos read encoder 2048 in the calibration pose, every offset follows from constants. This removes the flash dependency and the intermittent "uncalibrated after reboot" failure mode. |
| **Leg-major joint ordering in the policy** | Isaac Gym orders DOFs by URDF declaration order, while the wire protocol uses a group-major order. Two independent criteria confirm the required permutation. |
| **No blocking output in realtime loops** | A blocking `print` starves the UART, the link goes stale and the robot leaves `ACTIVE`. This fault was introduced and corrected four times during development. |
| **`q_range` applies `dir` exactly once** | The stored limits are already expressed in joint space; applying `dir` a second time mirrors the limits of the six joints with `dir = -1`. |
| **Hysteresis on servo temperature** | A single erroneous reading above 70 °C released all twelve joints for approximately 87 ms, seven times in ten seconds, while the servos were at 30 °C. |

A complete catalogue of 25 engineering issues, with root causes and corrections, is in
[`docs/49-progress-review.md`](docs/49-progress-review.md) §4.

---

## Documentation

| Purpose | Document |
|---|---|
| Engineering record and fault analysis | [`docs/49`](docs/49-progress-review.md) |
| Sim-to-real gaps (G1 limits, G2 actuator latency) | [`docs/48`](docs/48-sim2real-gaps.md) |
| Host computer setup | [`docs/42`](docs/42-pi-setup.md) |
| Firmware debug shell | [`docs/41`](docs/41-firmware-debug-commands.md) |
| Firmware architecture | [`docs/30`](docs/30-firmware.md) |
| Training a policy | [`docs/45`](docs/45-linux-tasks.md), [`docs/40`](docs/40-simulation.md) |
| Complete document index | [`docs/README.md`](docs/README.md) |

> Documents `docs/33`, `34`, `35`, `36`, `37`, `38`, `39` and `50` are delivery-time
> snapshots dated 2026-10-10. They are retained as historical records and should not be
> used as current reference; the live documents are those listed above.

---

## Known limitations

1. **Joint limits differ between models.** The simulation URDF permits the thigh joint up
   to 90°, whereas the hardware allows 30°. The policy commands up to 46.9°, so hardware
   runs are clamped with `rl_run --clamp-real`. The performance cost of this clamp has not
   yet been quantified. See [`docs/48`](docs/48-sim2real-gaps.md) §G1.
2. **Actuator latency is absent from simulation.** Physical servos lag by 10–30 ms
   (one to three control cycles), which is the principal reason a policy that is stable
   in simulation may oscillate on hardware. See §G2.
3. **Static balance only.** The robot stands; locomotion is not implemented.
4. **Host link runs at 500 kbps rather than 1 Mbps.** The rate was reduced because
   20 µs per byte left insufficient margin for interrupt latency on single-byte UART reception.
5. **Link thresholds in `pi/config.py` are relaxed for development** (150 / 1200 ms instead
   of 80 / 300 ms). They must be restored before any safety validation.
6. **Mechanical CAD is part-level only.** An assembly STEP file has not yet been exported;
   see [`hardware/mechanical/README.md`](hardware/mechanical/README.md) §2.

---

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Required checks before a pull request:

```bash
cd pi && ./run_checks.sh
cd firmware && tools/test-host.ps1
cd pi && python3 rl_policy.py --selftest
```

---

## License

| Scope | License |
|---|---|
| Software — firmware, host runtime, scripts, tools | [Apache-2.0](LICENSE) |
| Hardware — CAD, PCB, URDF | [CERN-OHL-W-2.0](LICENSE-hardware) |
| Documentation — `docs/`, `*.md` | [CC-BY-4.0](LICENSE-docs) |

Third-party components and the boundaries of this release are described in
[`NOTICE.md`](NOTICE.md).

**Isaac Gym is proprietary software and is not included.** It must be obtained from NVIDIA
separately; training cannot be reproduced without it.

---

## Safety

The platform incorporates a lithium-polymer battery, twelve servos capable of high stall
current, and a moving assembly of approximately 1.5 kg.

It is provided **as is, without warranty of any kind** (Apache-2.0 §7 and §8). Read
[`docs/46-mainline-plan.md`](docs/46-mainline-plan.md) and
[`docs/42-pi-setup.md`](docs/42-pi-setup.md) before applying power for the first time.
