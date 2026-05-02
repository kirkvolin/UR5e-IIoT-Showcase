"""
UR5e PS5 Controller via PLC (EtherNet/IP)
------------------------------------------
Controls a UR5e robot using a PS5 DualSense controller, routed through
a CompactLogix PLC via pylogix. Mode switching (manual/auto) is handled
by the HMI — this script only reads the current mode and sends velocity
commands when in manual mode.

Includes software safety envelope:
  - Outer radial limit (6 inches inside UR5e max reach)
  - Inner radial limit (prevents reaching back toward base)
  - Angular limits (restricts to front semicircle)
  - Z ceiling and floor limits
  - Soft buffer zones on all boundaries for gradual slowdown

Architecture:
    PS5 Controller → Python (this script) → PLC (pylogix/CIP) → UR (EtherNet/IP GP registers)
    HMI buttons control PS5_Mode tag (0=auto, 7=manual)

Requirements:
    pip install pylogix pygame

Controls:
    Left Stick X/Y   → Move robot in X/Y (Cartesian)
    Right Stick Y     → Move robot in Z
    Right Stick X     → Rotate tool around Z
    D-Pad Up/Down     → Rotate tool around X
    D-Pad Left/Right  → Rotate tool around Y
    L1/R1             → Rotate joint 6 (gripper axis) CCW/CW
    L2 (hold)         → Close gripper
    R2 (hold)         → Open gripper
    Cross             → Soft stop (zeros all velocities)
    Options           → Exit program

    Mode switching is done on the HMI, NOT the controller.
"""

import pygame
import socket
import time
import sys
import math

try:
    from pylogix import PLC
except ImportError:
    print("[ERROR] pylogix not installed. Run: pip install pylogix")
    sys.exit(1)

# ──────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────

PLC_IP          = "192.168.1.10"
ROBOT_IP        = "192.168.1.20"   # Only used for gripper socket

# PLC Tag Names — commands to robot
TAG_VEL_X       = "PS5_VelX"
TAG_VEL_Y       = "PS5_VelY"
TAG_VEL_Z       = "PS5_VelZ"
TAG_ROT_X       = "PS5_RotX"
TAG_ROT_Y       = "PS5_RotY"
TAG_ROT_Z       = "PS5_RotZ"
TAG_MODE        = "PS5_Mode"
TAG_JOINT6_VEL  = "PS5_Joint6Vel"

# PLC Tag Names — position feedback from robot
TAG_TCP_X       = "UR_TCP_X"
TAG_TCP_Y       = "UR_TCP_Y"
TAG_TCP_Z       = "UR_TCP_Z"

# Command numbers
CMD_AUTO        = 0
CMD_MANUAL      = 7

# Speed limits
LINEAR_SPEED      = 0.1
ROTATION_SPEED    = 0.3
GRIPPER_ROT_SPEED = 0.5

# ──────────────────────────────────────────────
# SAFETY ENVELOPE
# ──────────────────────────────────────────────
# UR5e max reach: 850mm (0.850m) from base center
# All limits measured from robot base origin

OUTER_LIMIT     = 0.698    # 850mm - 6 inches (152mm) = 698mm max radial reach
INNER_LIMIT     = 0.200    # 200mm minimum radial distance from base center
Z_CEILING       = 0.700    # Max height (700mm above base)
Z_FLOOR         = -0.100   # Min height (100mm below base — adjust to your table)

# Angular limits — restricts to front semicircle
# 0 degrees = robot's +X direction in base frame
# Positive = counterclockwise, Negative = clockwise
ANGLE_MIN_DEG   = -90      # Right boundary
ANGLE_MAX_DEG   = 90       # Left boundary

# If robot's "forward" in your setup doesn't align with +X,
# set this offset to rotate the allowed zone.
# Example: if forward is along +Y, set to 90
ANGLE_OFFSET    = 0

# Buffer zone — velocity scales down gradually before hitting hard limit
BUFFER_ZONE     = 0.050    # 50mm for radial and Z limits
ANGLE_BUFFER    = 10.0     # 10 degrees for angular limits

# Deadzone
DEADZONE        = 0.08

# Gripper
GRIPPER_PORT      = 63352
GRIPPER_SPEED     = 75
GRIPPER_FORCE     = 150
GRIPPER_STEP      = 10

# Loop rate
LOOP_HZ         = 20
LOOP_SLEEP      = 1.0 / LOOP_HZ

# Frame rotation for joystick alignment
FRAME_ROTATION_DEG = 90

# ──────────────────────────────────────────────
# PS5 DUALSENSE MAPPING
# ──────────────────────────────────────────────

AXIS_LEFT_X     = 0
AXIS_LEFT_Y     = 1
AXIS_RIGHT_X    = 2
AXIS_RIGHT_Y    = 3
AXIS_L2         = 4
AXIS_R2         = 5

BTN_CROSS       = 0
BTN_CIRCLE      = 1
BTN_SQUARE      = 2
BTN_TRIANGLE    = 3
BTN_OPTIONS     = 6
BTN_L1          = 9
BTN_R1          = 10

BTN_DPAD_UP     = 11
BTN_DPAD_DOWN   = 12
BTN_DPAD_LEFT   = 13
BTN_DPAD_RIGHT  = 14

# ──────────────────────────────────────────────
# ROBOTIQ GRIPPER
# ──────────────────────────────────────────────

class RobotiqGripper:
    def __init__(self, host: str, port: int = GRIPPER_PORT):
        self._host = host
        self._port = port
        self._sock = None

    def connect(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(8.0)
        self._sock.connect((self._host, self._port))

    def disconnect(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    def _send(self, cmd: str) -> str:
        self._sock.sendall((cmd + "\n").encode())
        return self._sock.recv(1024).decode().strip()

    def activate(self):
        resp = self._send("GET STA")
        if not resp.strip().endswith("3"):
            self._send("SET ACT 0")
            time.sleep(0.1)
            self._send("SET ACT 1")
            for _ in range(30):
                resp = self._send("GET STA")
                if resp.strip().endswith("3"):
                    break
                time.sleep(0.1)
            else:
                print("  [WARN] Gripper did not reach active state in time")
        else:
            print("  (Gripper already active)")
        self._send(f"SET SPE {GRIPPER_SPEED}")
        self._send(f"SET FOR {GRIPPER_FORCE}")
        self._send("SET GTO 1")

    def set_position(self, pos: int):
        self._send(f"SET POS {max(0, min(255, pos))}")


# ──────────────────────────────────────────────
# PLC COMMUNICATION
# ──────────────────────────────────────────────

class PLCConnection:
    def __init__(self, ip: str):
        self.plc = PLC()
        self.plc.IPAddress = ip
        self._connected = False

    def connect(self):
        result = self.plc.Read(TAG_MODE)
        if result.Status != "Success":
            raise ConnectionError(f"PLC connection failed: {result.Status}")
        self._connected = True
        print(f"  PLC connected — current mode: {result.Value}")

    def disconnect(self):
        if self._connected:
            self.plc.Close()
            self._connected = False

    def read_mode(self) -> int:
        result = self.plc.Read(TAG_MODE)
        if result.Status == "Success":
            return result.Value
        return -1

    def read_tcp_position(self) -> tuple:
        """Read current TCP position from UR via PLC tags. Returns (x, y, z) in meters."""
        x_result = self.plc.Read(TAG_TCP_X)
        y_result = self.plc.Read(TAG_TCP_Y)
        z_result = self.plc.Read(TAG_TCP_Z)

        x = x_result.Value if x_result.Status == "Success" else 0.0
        y = y_result.Value if y_result.Status == "Success" else 0.0
        z = z_result.Value if z_result.Status == "Success" else 0.0

        return (x, y, z)

    def write_velocities(self, vx: float, vy: float, vz: float,
                         rx: float, ry: float, rz: float):
        for tag_name, value in [
            (TAG_VEL_X, vx), (TAG_VEL_Y, vy), (TAG_VEL_Z, vz),
            (TAG_ROT_X, rx), (TAG_ROT_Y, ry), (TAG_ROT_Z, rz),
        ]:
            self.plc.Write(tag_name, value)

    def write_joint6_vel(self, vel: float):
        self.plc.Write(TAG_JOINT6_VEL, vel)

    def zero_velocities(self):
        self.write_velocities(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.write_joint6_vel(0.0)


# ──────────────────────────────────────────────
# SAFETY ENVELOPE
# ──────────────────────────────────────────────

def apply_safety_limits(vx: float, vy: float, vz: float,
                        tcp_x: float, tcp_y: float, tcp_z: float) -> tuple:
    """
    Clamp velocity commands to keep TCP within the safety envelope.
    Uses soft buffer zones — velocity scales down gradually as the
    robot approaches a limit, reaching zero at the hard limit.
    Only blocks velocity TOWARD the limit, not away from it.
    Returns (clamped_vx, clamped_vy, clamped_vz, warning_message).
    """
    warning = ""

    # ── Radial distance from base center (X/Y plane) ──
    radial = math.sqrt(tcp_x ** 2 + tcp_y ** 2)

    # Direction the TCP is moving radially
    if radial > 0.001:
        radial_vel = (tcp_x * vx + tcp_y * vy) / radial
    else:
        radial_vel = 0.0

    # ── OUTER LIMIT — prevent moving outward past the limit ──
    if radial > OUTER_LIMIT - BUFFER_ZONE:
        if radial_vel > 0:
            if radial >= OUTER_LIMIT:
                scale = 0.0
                warning = "OUTER LIMIT"
            else:
                remaining = OUTER_LIMIT - radial
                scale = remaining / BUFFER_ZONE
                warning = f"OUTER WARN ({remaining*1000:.0f}mm)"

            outward_vx = (tcp_x / radial) * radial_vel
            outward_vy = (tcp_y / radial) * radial_vel
            vx = vx - outward_vx * (1.0 - scale)
            vy = vy - outward_vy * (1.0 - scale)

    # ── INNER LIMIT — prevent moving inward past the limit ──
    if radial < INNER_LIMIT + BUFFER_ZONE:
        if radial_vel < 0:
            if radial <= INNER_LIMIT:
                scale = 0.0
                warning = "INNER LIMIT"
            else:
                remaining = radial - INNER_LIMIT
                scale = remaining / BUFFER_ZONE
                warning = f"INNER WARN ({remaining*1000:.0f}mm)"

            inward_vx = (tcp_x / radial) * radial_vel
            inward_vy = (tcp_y / radial) * radial_vel
            vx = vx - inward_vx * (1.0 - scale)
            vy = vy - inward_vy * (1.0 - scale)

    # ── ANGULAR LIMITS — restrict to front semicircle ──
    if radial > 0.05:
        # Calculate current angle with offset applied
        angle_deg = math.degrees(math.atan2(tcp_y, tcp_x)) - ANGLE_OFFSET

        # Normalize to -180..180
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360

        # Tangential velocity — positive = counterclockwise, negative = clockwise
        tang_vel = (-tcp_y * vx + tcp_x * vy) / radial

        # Check right boundary (ANGLE_MIN_DEG)
        if angle_deg < ANGLE_MIN_DEG + ANGLE_BUFFER:
            if tang_vel < 0:  # Moving clockwise toward right boundary
                if angle_deg <= ANGLE_MIN_DEG:
                    tang_vx = (-tcp_y / radial) * tang_vel
                    tang_vy = (tcp_x / radial) * tang_vel
                    vx = vx - tang_vx
                    vy = vy - tang_vy
                    warning = "ANGLE RIGHT LIMIT"
                else:
                    remaining_deg = angle_deg - ANGLE_MIN_DEG
                    scale = remaining_deg / ANGLE_BUFFER
                    tang_vx = (-tcp_y / radial) * tang_vel
                    tang_vy = (tcp_x / radial) * tang_vel
                    vx = vx - tang_vx * (1.0 - scale)
                    vy = vy - tang_vy * (1.0 - scale)
                    warning = f"ANGLE R WARN ({remaining_deg:.0f}deg)"

        # Check left boundary (ANGLE_MAX_DEG)
        if angle_deg > ANGLE_MAX_DEG - ANGLE_BUFFER:
            if tang_vel > 0:  # Moving counterclockwise toward left boundary
                if angle_deg >= ANGLE_MAX_DEG:
                    tang_vx = (-tcp_y / radial) * tang_vel
                    tang_vy = (tcp_x / radial) * tang_vel
                    vx = vx - tang_vx
                    vy = vy - tang_vy
                    warning = "ANGLE LEFT LIMIT"
                else:
                    remaining_deg = ANGLE_MAX_DEG - angle_deg
                    scale = remaining_deg / ANGLE_BUFFER
                    tang_vx = (-tcp_y / radial) * tang_vel
                    tang_vy = (tcp_x / radial) * tang_vel
                    vx = vx - tang_vx * (1.0 - scale)
                    vy = vy - tang_vy * (1.0 - scale)
                    warning = f"ANGLE L WARN ({remaining_deg:.0f}deg)"

    # ── Z CEILING — prevent moving up past ceiling ──
    if tcp_z > Z_CEILING - BUFFER_ZONE:
        if vz > 0:
            if tcp_z >= Z_CEILING:
                vz = 0.0
                warning = "Z CEILING"
            else:
                remaining = Z_CEILING - tcp_z
                scale = remaining / BUFFER_ZONE
                vz = vz * scale
                warning = f"Z CEIL WARN ({remaining*1000:.0f}mm)"

    # ── Z FLOOR — prevent moving down past floor ──
    if tcp_z < Z_FLOOR + BUFFER_ZONE:
        if vz < 0:
            if tcp_z <= Z_FLOOR:
                vz = 0.0
                warning = "Z FLOOR"
            else:
                remaining = tcp_z - Z_FLOOR
                scale = remaining / BUFFER_ZONE
                vz = vz * scale
                warning = f"Z FLOOR WARN ({remaining*1000:.0f}mm)"

    return (vx, vy, vz, warning)


# ──────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────

def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def trigger_to_signed(raw: float) -> float:
    return (raw + 1.0) / 2.0


def get_velocity_from_controller(joystick) -> list:
    lx = apply_deadzone(joystick.get_axis(AXIS_LEFT_X), DEADZONE)
    ly = apply_deadzone(joystick.get_axis(AXIS_LEFT_Y), DEADZONE)
    rz = apply_deadzone(joystick.get_axis(AXIS_RIGHT_X), DEADZONE)
    ry_raw = apply_deadzone(joystick.get_axis(AXIS_RIGHT_Y), DEADZONE)

    dpad_x = joystick.get_button(BTN_DPAD_RIGHT) - joystick.get_button(BTN_DPAD_LEFT)
    dpad_y = joystick.get_button(BTN_DPAD_UP) - joystick.get_button(BTN_DPAD_DOWN)

    # Raw velocities before rotation
    raw_vx = lx * LINEAR_SPEED
    raw_vy = -ly * LINEAR_SPEED

    # Rotate X/Y to match operator's viewing angle
    angle_rad = math.radians(FRAME_ROTATION_DEG)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    vx = raw_vx * cos_a - raw_vy * sin_a
    vy = raw_vx * sin_a + raw_vy * cos_a

    return [
        vx,
        vy,
        -ry_raw * LINEAR_SPEED,
        -dpad_y * ROTATION_SPEED,
        -dpad_x * ROTATION_SPEED,
        rz      * ROTATION_SPEED,
    ]


def print_status(mode_str: str, velocity: list, gripper_pos: int,
                 radial: float, tcp_z: float, angle_deg: float, warning: str):
    moving = any(abs(v) > 0.001 for v in velocity)
    status = "MOVING" if moving else "IDLE  "
    vel_str = " ".join(f"{v:+.3f}" for v in velocity[:3])
    warn_str = f" !! {warning}" if warning else ""
    print(f"\r  [{mode_str}] [{status}] [GRIP:{gripper_pos:3d}] "
          f"R:{radial*1000:.0f}mm Z:{tcp_z*1000:.0f}mm A:{angle_deg:+.0f}deg "
          f"vel:[{vel_str}]{warn_str}       ",
          end="", flush=True)


# ──────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────

def main():
    print("\n========================================")
    print("  UR5e PS5 Controller via PLC")
    print("========================================")
    print(f"  PLC IP       : {PLC_IP}")
    print(f"  Robot IP     : {ROBOT_IP} (gripper only)")
    print(f"  Linear Speed : {LINEAR_SPEED} m/s")
    print(f"  Rot Speed    : {ROTATION_SPEED} rad/s")
    print(f"  Frame Rot    : {FRAME_ROTATION_DEG}deg")
    print(f"  Outer Limit  : {OUTER_LIMIT*1000:.0f}mm")
    print(f"  Inner Limit  : {INNER_LIMIT*1000:.0f}mm")
    print(f"  Z Ceiling    : {Z_CEILING*1000:.0f}mm")
    print(f"  Z Floor      : {Z_FLOOR*1000:.0f}mm")
    print(f"  Angle Range  : {ANGLE_MIN_DEG}deg to {ANGLE_MAX_DEG}deg")
    print(f"  Angle Offset : {ANGLE_OFFSET}deg")
    print(f"  Buffer Zone  : {BUFFER_ZONE*1000:.0f}mm / {ANGLE_BUFFER}deg")
    print("  Mode is controlled by the HMI")
    print("  Press OPTIONS to exit")
    print("========================================\n")

    # ── Init pygame and controller ──
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[ERROR] No controller detected. Connect your PS5 controller and try again.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"  Controller : {joystick.get_name()}")

    # ── Connect to PLC ──
    print(f"  Connecting to PLC at {PLC_IP}...")
    plc = PLCConnection(PLC_IP)
    try:
        plc.connect()
    except Exception as e:
        print(f"\n[ERROR] Could not connect to PLC: {e}")
        print("  Check that the PLC IP is correct and the PLC is in Run mode.")
        sys.exit(1)
    print("  PLC connected!")

    # ── Verify TCP position tags exist ──
    print("  Checking TCP position tags...")
    tcp_x, tcp_y, tcp_z = plc.read_tcp_position()
    radial = math.sqrt(tcp_x ** 2 + tcp_y ** 2)
    angle = math.degrees(math.atan2(tcp_y, tcp_x)) - ANGLE_OFFSET
    print(f"  Current TCP: X={tcp_x*1000:.1f}mm Y={tcp_y*1000:.1f}mm "
          f"Z={tcp_z*1000:.1f}mm  R={radial*1000:.1f}mm  A={angle:+.1f}deg")

    # ── Connect to gripper ──
    print(f"  Connecting to Robotiq gripper at {ROBOT_IP}:{GRIPPER_PORT}...")
    gripper = RobotiqGripper(ROBOT_IP)
    gripper_available = False
    for attempt in range(1, 6):
        try:
            gripper.connect()
            gripper.activate()
            gripper_available = True
            break
        except Exception as e:
            if attempt < 5:
                print(f"  Gripper attempt {attempt} failed — retrying in 3 s... ({e})")
                try:
                    gripper.disconnect()
                except Exception:
                    pass
                time.sleep(3.0)
    if gripper_available:
        print("  Gripper ready!")
    else:
        print("  [WARN] Gripper not available — continuing without gripper control")

    print("\n  Ready! Waiting for HMI to set Manual mode (PS5_Mode = 7).")
    print("  Press OPTIONS on controller to exit.\n")

    prev_options   = False
    gripper_pos    = 0
    was_manual     = False

    try:
        while True:
            pygame.event.pump()
            loop_start = time.time()

            # ── Read current mode from PLC (set by HMI) ──
            current_mode = plc.read_mode()
            is_manual = (current_mode == CMD_MANUAL)

            # ── Read buttons and triggers ──
            cross_pressed   = joystick.get_button(BTN_CROSS)
            options_pressed = joystick.get_button(BTN_OPTIONS)
            l2              = trigger_to_signed(joystick.get_axis(AXIS_L2))
            r2              = trigger_to_signed(joystick.get_axis(AXIS_R2))

            # ── Options: exit ──
            if options_pressed and not prev_options:
                print("\n\n  OPTIONS pressed — exiting...")
                break
            prev_options = options_pressed

            # ── Detect mode transitions ──
            if was_manual and not is_manual:
                plc.zero_velocities()
                print(f"\n  >> HMI switched to AUTONOMOUS — velocities zeroed\n")
            elif not was_manual and is_manual:
                print(f"\n  >> HMI switched to MANUAL — you have control\n")
            was_manual = is_manual

            # ── Gripper: L2 closes, R2 opens (always active regardless of mode) ──
            if gripper_available:
                new_pos = gripper_pos
                if r2 > 0.05:
                    new_pos = min(255, gripper_pos + int(r2 * GRIPPER_STEP))
                elif l2 > 0.05:
                    new_pos = max(0, gripper_pos - int(l2 * GRIPPER_STEP))
                if new_pos != gripper_pos:
                    gripper_pos = new_pos
                    try:
                        gripper.set_position(gripper_pos)
                    except Exception as e:
                        print(f"\n  [GRIPPER ERROR] {e}")
                        gripper_available = False

            # ── Manual control — only when HMI has set mode to 7 ──
            if is_manual:
                try:
                    # Cross: soft stop
                    if cross_pressed:
                        plc.zero_velocities()
                        print("\n  [SOFT STOP] Cross pressed — all velocities zeroed")
                        time.sleep(0.2)
                        continue

                    # Read current TCP position for safety limits
                    tcp_x, tcp_y, tcp_z = plc.read_tcp_position()
                    radial = math.sqrt(tcp_x ** 2 + tcp_y ** 2)
                    angle_deg = math.degrees(math.atan2(tcp_y, tcp_x)) - ANGLE_OFFSET

                    # Get velocity from controller
                    velocity = get_velocity_from_controller(joystick)

                    # Apply safety envelope limits (radial + angular + Z)
                    safe_vx, safe_vy, safe_vz, warning = apply_safety_limits(
                        velocity[0], velocity[1], velocity[2],
                        tcp_x, tcp_y, tcp_z
                    )

                    # Shoulder buttons: joint 6 rotation
                    shoulder = joystick.get_button(BTN_R1) - joystick.get_button(BTN_L1)
                    j6_vel = shoulder * GRIPPER_ROT_SPEED

                    # Write safety-limited velocities to PLC
                    plc.write_velocities(
                        safe_vx, safe_vy, safe_vz,
                        velocity[3], velocity[4], velocity[5]
                    )
                    plc.write_joint6_vel(j6_vel)

                    # Display with safety info
                    print_status("MANUAL",
                                 [safe_vx, safe_vy, safe_vz,
                                  velocity[3], velocity[4], velocity[5]],
                                 gripper_pos, radial, tcp_z, angle_deg, warning)

                except Exception as e:
                    print(f"\n  [PLC ERROR] {e}")
                    try:
                        plc.zero_velocities()
                    except Exception:
                        pass
                    time.sleep(2.0)

            else:
                print(f"\r  [AUTO] — Waiting for HMI to set Manual mode (PS5_Mode = 7)  ",
                      end="", flush=True)

            # ── Maintain loop rate ──
            elapsed = time.time() - loop_start
            sleep_time = LOOP_SLEEP - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n  Ctrl+C detected — shutting down...")

    finally:
        print("  Zeroing velocities...")
        try:
            plc.zero_velocities()
            plc.disconnect()
        except Exception:
            pass
        try:
            if gripper_available:
                gripper.disconnect()
        except Exception:
            pass
        pygame.quit()
        print("  Done. Goodbye.\n")


if __name__ == "__main__":
    main()
