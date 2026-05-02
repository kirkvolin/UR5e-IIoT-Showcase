# UR5e Candy Roulette — Industrial Robot Game System

A fully integrated industrial automation project combining a Universal Robots UR5e collaborative robot, Allen-Bradley CompactLogix PLC, PanelView HMI, and PS5 controller into an interactive candy dispensing game and manual robot control system.

Built as a capstone demonstration of EtherNet/IP communication, PLC programming, robot programming, HMI design, and Python-based controller integration.

---

## What It Does

### Candy Roulette Game
Four indicator lights cycle in sequence like a roulette wheel. A player presses a button to stop the lights — whichever light is active becomes the selected candy. The PLC sends the selection to the UR5e over EtherNet/IP, and the robot picks up the corresponding candy bar with a Robotiq gripper and delivers it to the player. A win/lose mechanic checks whether the player pressed the button matching the active light, and the HMI displays the result with animated fireworks for winners.

### PS5 Manual Control
A PS5 DualSense controller provides real-time joystick control of the robot arm, routed through the PLC's EtherNet/IP connection. An operator can seamlessly switch between the autonomous candy game and manual joystick control from the HMI touchscreen. Software safety limits restrict the robot to a semicircular operating envelope with soft boundary zones that gradually reduce speed near limits.

### Dance and Motion Routines
Several pre-programmed motion routines are callable from the PLC, including a multi-act flowing dance routine, an expanding spiral pattern, a coil/unwind animation, and a simple wave. All routines use dynamically calculated positions relative to the robot's current pose, so they work from any starting position.

---

## System Architecture

```
┌────────────┐  pylogix   ┌─────────────────┐  EtherNet/IP  ┌─────────────┐
│   Python   │───────────►│  CompactLogix   │◄────────────►│    UR5e     │
│  Script    │    CIP     │     5370        │ GP Registers  │ Controller  │
│(PS5 Ctrl)  │            │                 │               │  + Gripper  │
└─────┬──────┘            └──┬──────────┬───┘               └──────┬──────┘
      │                      │          │                          │
      │ Robotiq Socket       │ CIP      │ EtherNet/IP              │
      │ (port 63352)         │          │                          │
      │                 ┌────┴─────┐  ┌─┴────────────┐            │
      │                 │ PanelView│  │  1734-AENTR  │            │
      │                 │   5310   │  │    + OB8     │            │
      │                 │  (HMI)   │  │(Lights/Btns) │            │
      │                 └──────────┘  └──────────────┘            │
      │                                                           │
      └───────────────────────────────────────────────────────────┘
                    (gripper only — does not conflict
                     with EtherNet/IP connection)
```

All devices communicate over a single Ethernet network. The PLC acts as the central coordinator — the UR5e never receives commands directly from the HMI or PS5 controller. Everything routes through the PLC's ladder logic.

---

## EtherNet/IP Communication Protocol

The UR5e has no built-in remote control system like some industrial robots. Instead, it exposes general purpose registers over EtherNet/IP that both sides can read and write. This project uses a custom handshake protocol built on those registers:

### PLC → Robot (Commands)
| Register | Purpose |
|----------|---------|
| Bool Register 0 | Start flag — latch to trigger a move |
| Int Register 0 | Program command (1-7) |
| Float Registers 0-6 | Velocity commands for PS5 manual control |

### Robot → PLC (Status)
| Register | Purpose |
|----------|---------|
| Bool Register 0 | Busy — TRUE while executing |
| Bool Register 1 | Ready — TRUE when idle |
| Bool Register 2 | Done — TRUE when move complete |
| Int Register 0 | Echo of current program number |

### Handshake Sequence
1. UR signals Ready
2. PLC writes command number and latches Start flag
3. UR clears Ready, sets Busy, executes the command
4. PLC sees Busy, unlatches Start flag
5. UR completes move, sets Done, clears Busy
6. PLC processes result, cycle repeats

---

## Program Commands

| Command | Function | Description |
|---------|----------|-------------|
| 1 | Candy 1 | Pick candy from position 1, deliver to player |
| 2 | Candy 2 | Pick candy from position 2, deliver to player |
| 3 | Candy 3 | Pick candy from position 3, deliver to player |
| 4 | Candy 4 | Pick candy from position 4, deliver to player |
| 5 | Wave | Wave routine using taught waypoints |
| 6 | Dance | Multi-act flowing dance with PLC cancel support |
| 7 | Manual | PS5 joystick velocity control via float registers |

---

## Game State Machine

The candy roulette game runs as a state machine in PLC ladder logic:

| State | Name | Description |
|-------|------|-------------|
| 0 | IDLE | Attract mode — all lights pulse, waiting for button press |
| 1 | CYCLING | Lights chase in sequence via SQO sequencer, player presses button to stop |
| 2 | SELECTED | Frozen selection sent to UR via EtherNet/IP handshake |
| 3 | DELIVERY | Robot picks and delivers candy, selected light stays on |
| 4 | DONE | Win/lose display on HMI with firework animation, then reset |
| 5 | FAIL | Wrong button flash pattern, candy still delivered |

### Light Control
All four indicator lights are driven by a single OTE instruction each at the end of the ladder program, with branched OR conditions determining which state drives each light. This pattern prevents flickering caused by multiple OTE instructions fighting over the same output across different state rungs.

### Sequencer
The light chase in the cycling state uses an SQO (Sequencer Output) instruction stepping through a 4-element array of bit patterns (1, 2, 4, 8) at a configurable timer rate. This separates the chase pattern from the logic — changing the sequence order or adding patterns only requires editing the data array, not the ladder rungs.

---

## PS5 Controller Integration

The Python script connects to the PLC via pylogix (CIP over Ethernet) — not directly to the robot. This avoids RTDE port conflicts when the UR is simultaneously running the candy game program.

### Control Mapping
| Input | Function |
|-------|----------|
| Left Stick X/Y | Move robot in X/Y plane |
| Right Stick Y | Move robot in Z |
| Right Stick X | Rotate tool around Z |
| D-Pad | Rotate tool around X/Y |
| L1/R1 | Rotate gripper axis (joint 6) |
| L2/R2 | Close/open gripper |
| Cross | Soft stop |
| Options | Exit script |

### Safety Envelope
The script enforces software safety limits that prevent the robot from reaching its physical joint limits:

- **Outer radial limit:** 698mm (6 inches inside max reach)
- **Inner radial limit:** 200mm (prevents reaching toward base)
- **Angular limits:** ±90° semicircle (restricts to front of robot)
- **Z ceiling/floor:** Configurable height boundaries

All boundaries use a 50mm soft buffer zone where velocity gradually scales down, creating a natural "pushing through thick air" feel rather than a hard wall stop.

### Frame Rotation
A configurable rotation angle aligns the joystick X/Y axes with the operator's viewing perspective, so pushing the stick forward always moves the robot away from the operator regardless of the robot's base orientation.

---

## Waypoint System

All robot positions are defined as offsets from a single taught reference point using URScript's `pose_trans()` function. When the physical setup moves, only the reference point needs to be re-taught — all other positions (candy locations, entry points, delivery position, home) automatically shift with it.

```
ref = p[0.350, -0.100, 0.200, 0, 3.14, 0]   # Teach this one point

pick_up_1 = ref
pick_up_2 = pose_trans(ref, p[0, 0.08333, 0, 0, 0, 0])
pick_up_3 = pose_trans(ref, p[0, 0.08333 * 2, 0, 0, 0, 0])
pick_up_4 = pose_trans(ref, p[0, 0.08333 * 3, 0, 0, 0, 0])

entry_1 = pose_trans(pick_up_1, p[0, 0, -entry_height, 0, 0, 0])
```

Offsets are applied in the reference pose's local frame, so if the candy tray is tilted, all positions follow the tilt automatically.

---

## Hardware

See [HARDWARE_LIST.md](HARDWARE_LIST.md) for a complete bill of materials including part numbers, network configuration, and power budget.

### Network
| Device | IP Address | Role |
|--------|-----------|------|
| CompactLogix 5370 | 192.168.1.10 | PLC / EtherNet/IP Scanner |
| UR5e Controller | 192.168.1.20 | Robot / EtherNet/IP Adapter |
| PanelView 5310 | 192.168.1.30 | HMI |
| 1734-AENTR | 192.168.1.40 | Remote I/O Adapter |

### Power
24VDC bus powers the AENTR, OB8 outputs, indicator lights, tower light, and PanelView. Total draw is approximately 1.5A steady state. The UR5e runs on a separate AC mains circuit.

---

## File Structure

```
├── README.md                          # This file
├── HARDWARE_LIST.md                   # Complete bill of materials
├── Project_Summary_For_New_Chat.txt   # Detailed technical reference
│
├── plc/
│   ├── Candy_Roulette_IO_List.xlsx    # I/O list with all tags
│   └── UR_DataTypes.L5X               # UR data types for Studio 5000
│
├── ur_programs/
│   ├── dance.script                   # Standalone dance routine
│   ├── flowy_dance.script             # 10-act flowing dance
│   ├── ur_plc_program.script          # Full PLC-controlled program
│   └── ur_scripts_combined/           # Individual script blocks for teach pendant
│       └── README.md                  # Program tree map
│
├── python/
│   └── pygame_controller_plc.py       # PS5 controller script (pylogix)
│
├── docs/
│   ├── UR_CompactLogix_EthernetIP_Integration_Guide.md
│   ├── PS5_PLC_Integration_Guide.txt
│   └── simon_says_corrected.txt       # Original Simon Says game (replaced by roulette)
│
└── guides/
    └── Epson_RC700A_CompactLogix_EthernetIP_Integration_Guide.md
```

---

## Key Technical Decisions

**Generic Ethernet Module over EDS:** The UR's EDS file creates vendor-specific tag structures with unreadable parameter names. Using a Generic Ethernet Module with manually imported UR_DataTypes.L5X provides human-readable structured tags like `URI.Safety.ES` instead of `Param34`.

**CPS for data synchronization:** Raw EtherNet/IP I/O data must be copied atomically into structured tags using CPS (Synchronous Copy File) instructions to prevent partial data updates during a PLC scan.

**Single OTE per output:** Every physical output has exactly one OTE instruction at the end of the ladder program with branched OR conditions. This eliminates flickering caused by multiple OTEs across different state rungs where the last one in scan order wins.

**movej over movep for dance routines:** Process moves (movep) frequently trigger protective stop C204A3 on direction changes. Joint-space moves (movej) create natural arcs and almost never trigger stops because the path planner has full freedom.

**PS5 through PLC, not RTDE:** Connecting the PS5 controller directly to the UR via RTDE conflicts with the running EtherNet/IP program. Routing through the PLC via pylogix uses the existing EtherNet/IP connection, allowing seamless switching between autonomous and manual modes.

**Single timer for attract blink:** Two chained TON timers create a circular dependency where each timer's completion kills the other on the next scan. A single timer with accumulator comparison (ACC < 500 = on, ACC >= 500 = off) produces a clean oscillation.

---

## Bugs Found and Resolved

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Lights flicker across states | Multiple OTEs for same output | Single OTE per output with branched conditions |
| Attract lights erratic | Two chained TON timers in circular dependency | Single timer with ACC comparison |
| Multiple lights on after selection | LT (less than) instead of EQ for candy comparison | Changed to EQ |
| ONS + TON never reaches .DN | ONS passes power for one scan, TON can't accumulate | Separate TON onto its own rung |
| UR dance protective stops | movep with direction changes fails path sanity check | Use movej, remove blend radii, cap acceleration at 2.0 |
| HMI buttons show red X | Buttons assigned to read-only physical DI tags | Create internal BOOL tags, OR with physical inputs |
| UR spiral won't start | Missing `p` prefix on pose arrays | Add `p[...]` to all pose definitions |
| UR loop variable not updating | Variable assigned in Script node not visible to Loop condition | Keep loop variables in Assignment nodes |
| PS5 RTDE port conflict | RTDE takes over UR, kills EtherNet/IP program | Route PS5 through PLC via pylogix |
| Robot stuck busy after PS5 mode exit | UR_ProgCmd still holds 7, start flag still latched | Clear both on mode transition |

---

## Acknowledgments

Built using documentation from Universal Robots, Rockwell Automation, PATLITE, AutomationDirect, and Robotiq. EtherNet/IP integration based on UR's official Allen-Bradley integration guide with significant extensions for game logic, motion routines, and controller integration.
