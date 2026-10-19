# Mechanical design

> **Status:** part-level CAD released; full-assembly CAD pending re-export.
> **License:** CERN-OHL-W-2.0 — see [`LICENSE-hardware`](../../LICENSE-hardware).

---

## 1 · What is in this directory

| Directory | Format | Purpose |
|---|---|---|
| `step/` | STEP AP214 | Neutral CAD — opens in SolidWorks, Fusion 360, FreeCAD, Onshape, NX, CATIA |
| `stl/` | STL (binary) | Ready for slicing; the geometry as exported from the same solids |

Both formats are exported from the same SolidWorks model, so they are geometrically
consistent. Use `step/` when you need to modify a part; use `stl/` when you only need to print it.

### Parts

| Part | `step/` | `stl/` | Solidity | Notes |
|---|---|---|---|---|
| `base_link` (chassis) | ✅ | — | 8 solids | Body frame, servo brackets and electronics tray |
| `thigh` | ✅ | ✅ | 2 solids | Upper leg link, motor → knee |
| `calf` | ✅ | ✅ | 1 solid | Lower leg link, knee → foot |
| `hip1` | ✅ | ✅ | 2 solids | Abduction bracket, left-hand variant |
| `hip2` | ✅ | ✅ | 2 solids | Abduction bracket, right-hand variant (mirrored) |
| `link` | — | ✅ | — | Servo-to-leg coupling |
| assembly | — | — | — | ⚠️ **not yet exported**, see §2 |

**⚠️ Two known gaps in the current export:**

1. **`link` has an STL but no STEP.** Print from `stl/link.stl`; there is no editable CAD for it yet.
2. **`base_link` has a STEP but no STL.** Slice it directly from `step/base_link.step` after
   converting, or wait for an STL export.

---

## 2 · ⚠️ Full-assembly CAD is not included

The current assembly export (`CAT.STEP`) contains **13 component references and 0 solids** —
it is an empty shell, i.e. the parts were referenced but not embedded during export.
Publishing it would be misleading, so it is excluded.

**To regenerate it** (SolidWorks):

```
File → Save As → STEP AP214 → Options…
   ☑ Export assembly as a single part   (or ☑ include all components)
   ☑ Include all solids in the assembly
```

Until then, the per-part files above are complete enough to manufacture the robot.

---

## 3 · Printing parameters

**These are the settings the robot was actually printed with.** Deviating from them will
change the strength of the load-bearing parts (thigh, hip1, hip2).

| Parameter | Value |
|---|---|
| **Material** | **PLA Basic** |
| **Infill pattern** | **Cubic** |
| **Infill density** | **30 %** |
| **Wall loops** | **5** |
| **Alternating extra wall** | **Enabled** |
| **Bottom shell layers** | **4** |
| **Support** | **Tree (auto)** |
| Everything else | slicer default |

### Notes

- **Five wall loops plus alternating extra wall** is the main reason the legs survive
  repeated servo torque. Reducing walls to the usual 2–3 will make the thigh and hip
  brackets noticeably more flexible.
- **Cubic infill at 30 %** gives roughly isotropic stiffness; a rectilinear or grid pattern
  is weaker in the direction that matters here.
- Servo pockets and screw holes print without supports if the part is oriented as exported.
- **PLA is adequate for bench testing but creeps under sustained load and is sensitive to heat.**
  For longer sessions, PETG or ABS/ASA are more forgiving. If you switch material, raise the
  wall count rather than the infill.

---

## 4 · Bill of materials

The mechanical parts above are printed. Everything they bolt onto — servos, controller,
battery, fasteners — is listed in [`../BOM.csv`](../BOM.csv).

**⚠️ The fastener list (screws, standoffs, bearings, servo horns) is not yet complete.**
If you are building from these files, count the mounting holes in the STEP files and verify
against your own hardware before ordering.

---

## 5 · Assembly

**⚠️ An assembly guide is not yet written.** Until then, the STEP files carry the full
geometry, and the kinematic structure is documented in
[`robot.urdf`](URDF/robot.urdf) plus [`docs/30-firmware.md`](../../docs/30-firmware.md) §3.2.

Points that are easy to get wrong, from the build record:

| Topic | Detail |
|---|---|
| **Servo horn alignment** | All twelve servos are zeroed so that the **calibration pose** (legs straight) reads encoder **2048**. See [`docs/30-firmware.md`](../../docs/30-firmware.md) §3.3. |
| **Joint directions** | `dir` differs per joint and is **not** the same for all four legs — `hip` uses `+1, −1, −1, +1`. See `RLCA_JOINT_DIR_INIT` in [`rlcat_config.h`](../../firmware/stm32/Core/Inc/rlcat_config.h). |
| **Joint limits** | `hip 0° … +40°`, `thigh −35° … +30°`, `calf −60° … +60°`. The **simulation** URDF deliberately permits `thigh +90°`; the hardware does not. |
| **Left/right** | `hip1` and `hip2` are mirrored; do not substitute one for the other. |

---

## 6 · What is deliberately not released

The following exist in the author's workspace and are **not** part of this repository:

| Excluded | Reason |
|---|---|
| SolidWorks native files (`.SLDASM`, `.SLDPRT`) | Binary, version-control hostile; the STEP export carries the same geometry |
| Vendor component models (PCB footprints, Raspberry Pi, servo) | Third-party 3D models with unclear redistribution terms; not needed to reproduce the robot |
| Assembly backups | Working history, not a deliverable |
| Print files (`.gcode`, `.3mf`) | Slicer- and printer-specific; parameters in §3 are reproducible |

---

## 7 · Reproducing the kinematics in simulation

`URDF/robot.urdf` is the hardware description used by the firmware and by
`rl_run --clamp-real`. The simulation uses a **modified** copy with a wider thigh limit,
under `Sim/`:

| File | `thigh` upper | Used by |
|---|---|---|
| `URDF/robot.urdf` | **+30°** (hardware) | Firmware limits, `--clamp-real` |
| `Sim/legged_gym/resources/robots/rlcat/urdf/robot_sim.urdf` | **+90°** (training) | Training and evaluation |

**This difference is the project's most important known sim-to-real gap.** It is analysed in
[`docs/48-sim2real-gaps.md`](../../docs/48-sim2real-gaps.md) §G1.
