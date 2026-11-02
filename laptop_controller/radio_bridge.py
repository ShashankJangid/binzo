#!/usr/bin/env python3
"""
RadioMaster Pocket (USB HID) to M5Core2 Mecanum Trash Can UDP Bridge.
Captures joystick sticks, computes 4x4 Mecanum kinematics, streams packets over WiFi,
and collects live debug & telemetry logs over UDP port 4212.
"""

import os
import sys
import time
import json
import socket
import argparse
import select
import collections

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

# Enable 1ms high-precision multimedia timer on Windows for smooth 40Hz packet streaming
if os.name == 'nt':
    try:
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

try:
    import pygame
except ImportError:
    print("[ERROR] pygame is required. Run: pip install pygame")
    sys.exit(1)

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")
SESSION_LOG_PATH = os.path.join(os.path.dirname(__file__), "session_debug.log")

event_logs = collections.deque(maxlen=10)


def log_event(text, to_file=True):
    """Adds a timestamped entry to the rolling event queue and appends to session_debug.log."""
    ts = time.strftime("%H:%M:%S.") + f"{int(time.time()*1000)%1000:03d}"
    entry = f"[{ts}] {text}"
    event_logs.append(entry)
    if to_file:
        try:
            with open(SESSION_LOG_PATH, "a") as f:
                f.write(entry + "\n")
        except Exception:
            pass


def load_config():
    default_config = {
        "network": {
            "target_ip": "192.168.0.121",
            "udp_port": 4210,
            "discovery_port": 4211,
            "log_port": 4212,
            "send_rate_hz": 40
        },
        "joystick": {
            "deadband": 0.05,
            "expo": 0.15,
            "max_speed_pct": 100,
            "axes": {
                "right_x_strafe": 0,
                "right_y_forward": 3,
                "left_y_throttle": 2,
                "left_x_yaw": 1
            },
            "invert": {
                "right_x_strafe": False,
                "right_y_forward": True,
                "left_y_throttle": False,
                "left_x_yaw": False
            },
            "throttle": {
                "mode": "unipolar",
                "idle_is_bottom": True,
                "center_sprung": False,
                "bottom_threshold": -0.70
            },
            "safety": {
                "require_arm_switch": False,
                "arm_button_index": 0,
                "failsafe_on_zero_throttle": True
            }
        },
        "drivetrain": {
            "type": "mecanum_4x4",
            "motor_channels": {
                "front_left": 0,
                "front_right": 1,
                "rear_left": 2,
                "rear_right": 3
            },
            "motor_inversions": {
                "front_left": 1,
                "front_right": 1,
                "rear_left": 1,
                "rear_right": 1
            }
        }
    }
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r") as f:
                loaded = json.load(f)
                return loaded
        except Exception as e:
            print(f"[WARN] Failed to read {CONFIG_PATH}: {e}. Using defaults.")
    return default_config


def apply_deadband_and_expo(val, deadband=0.05, expo=0.15):
    """Applies deadband zone and cubic exponential sensitivity curve."""
    if abs(val) < deadband:
        return 0.0
    # Rescale to [0, 1] range after deadband
    sign = 1.0 if val > 0 else -1.0
    rescaled = (abs(val) - deadband) / (1.0 - deadband)
    rescaled = min(1.0, max(0.0, rescaled))
    # Apply expo: linear + cubic mix
    shaped = (1.0 - expo) * rescaled + expo * (rescaled ** 3)
    return sign * shaped


def compute_throttle_scale(raw_val, invert=False, center_sprung=False, bottom_thresh=-0.70):
    """
    Computes a 0.0 to 1.0 master throttle scalar from stick position.
    Strictly clamps any resting stick position below bottom_thresh (-0.70) to 0.0% power.
    """
    v = -raw_val if invert else raw_val
    if center_sprung:
        return max(0.0, min(1.0, v))
    else:
        # Deadband at bottom of throttle stick: prevents resting stick from creeping forward
        if v <= bottom_thresh:
            return 0.0
        # Map [bottom_thresh, 0.95] to [0.0, 1.0]
        norm = (v - bottom_thresh) / (0.95 - bottom_thresh)
        return max(0.0, min(1.0, norm))


def compute_mecanum_powers(vx, vy, yaw, throttle, max_speed_pct=100, inversions=None):
    """4x4 Mecanum kinematics calculation."""
    if throttle <= 0.001:
        return [0, 0, 0, 0]

    if inversions is None:
        inversions = [1, 1, 1, 1]

    # Standard Mecanum inverse kinematics
    w_fl = vy + vx + yaw
    w_fr = vy - vx - yaw
    w_rl = vy - vx + yaw
    w_rr = vy + vx - yaw

    # Normalize if any wheel exceeds 1.0 magnitude to preserve directional heading
    max_mag = max(1.0, abs(w_fl), abs(w_fr), abs(w_rl), abs(w_rr))
    w_fl /= max_mag
    w_fr /= max_mag
    w_rl /= max_mag
    w_rr /= max_mag

    # Scale by master throttle and max power setting (127 max duty on STM32 driver)
    scale = throttle * (max_speed_pct / 100.0) * 127.0

    m0 = int(round(w_fl * scale * inversions[0]))
    m1 = int(round(w_fr * scale * inversions[1]))
    m2 = int(round(w_rl * scale * inversions[2]))
    m3 = int(round(w_rr * scale * inversions[3]))

    # Clamp
    m0 = max(-127, min(127, m0))
    m1 = max(-127, min(127, m1))
    m2 = max(-127, min(127, m2))
    m3 = max(-127, min(127, m3))

    return [m0, m1, m2, m3]


def discover_m5_ip(discovery_port=4211, timeout_sec=2.5):
    """Listens for UDP broadcast beacon from M5Core2."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.bind(("", discovery_port))
    except Exception as e:
        print(f"[WARN] Unable to bind discovery port {discovery_port}: {e}")
        sock.close()
        return None

    sock.setblocking(False)
    start_t = time.time()
    print(f"[*] Scanning WiFi for M5Core2 auto-discovery beacon on port {discovery_port}...")

    while time.time() - start_t < timeout_sec:
        ready = select.select([sock], [], [], 0.2)
        if ready[0]:
            try:
                data, addr = sock.recvfrom(512)
                text = data.decode('utf-8', errors='ignore').strip()
                if "M5_TRASHCAN" in text:
                    try:
                        info = json.loads(text)
                        ip = info.get("ip", addr[0])
                        sock.close()
                        return ip
                    except Exception:
                        sock.close()
                        return addr[0]
            except Exception:
                pass
    sock.close()
    return None


def init_joystick():
    """Initializes pygame joystick subsystem and returns connected joystick."""
    try:
        pygame.joystick.quit()
    except Exception:
        pass
    pygame.init()
    pygame.joystick.init()
    pygame.event.pump()
    count = pygame.joystick.get_count()
    if count == 0:
        return None
    joy = pygame.joystick.Joystick(0)
    joy.init()
    return joy


def clear_screen():
    if os.name == 'nt':
        print("\033[H\033[J", end="", flush=True)
    else:
        print("\033[H\033[2J", end="", flush=True)


def main():
    parser = argparse.ArgumentParser(description="RadioMaster Pocket to M5Core2 UDP Bridge")
    parser.add_argument("--ip", type=str, help="Target M5Core2 IP address (overrides config)")
    parser.add_argument("--port", type=int, help="Target UDP port (default 4210)")
    parser.add_argument("--broadcast", action="store_true", help="Send to 255.255.255.255 broadcast")
    parser.add_argument("--rate", type=int, help="Packet rate in Hz (default 40)")
    args = parser.parse_args()

    cfg = load_config()

    net_cfg = cfg.get("network", {})
    target_ip = args.ip or net_cfg.get("target_ip", "192.168.0.121")
    udp_port = args.port or net_cfg.get("udp_port", 4210)
    discovery_port = net_cfg.get("discovery_port", 4211)
    log_port = net_cfg.get("log_port", 4212)
    send_rate_hz = args.rate or net_cfg.get("send_rate_hz", 40)
    period = 1.0 / send_rate_hz

    joy_cfg = cfg.get("joystick", {})
    axes_cfg = joy_cfg.get("axes", {})
    invert_cfg = joy_cfg.get("invert", {})
    throttle_cfg = joy_cfg.get("throttle", {})
    safety_cfg = joy_cfg.get("safety", {})
    drive_cfg = cfg.get("drivetrain", {})

    deadband = joy_cfg.get("deadband", 0.05)
    expo = joy_cfg.get("expo", 0.15)
    max_speed_pct = joy_cfg.get("max_speed_pct", 100)
    bottom_thresh = throttle_cfg.get("bottom_threshold", -0.70)

    # Motor inversions [FL, FR, RL, RR]
    m_inv_cfg = drive_cfg.get("motor_inversions", {})
    motor_inversions = [
        m_inv_cfg.get("front_left", 1),
        m_inv_cfg.get("front_right", 1),
        m_inv_cfg.get("rear_left", 1),
        m_inv_cfg.get("rear_right", 1)
    ]

    # Setup UDP Sockets
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    log_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        log_sock.bind(("", log_port))
        log_sock.setblocking(False)
    except Exception as e:
        print(f"[WARN] Unable to bind UDP log port {log_port}: {e}")

    # Initialize session log file header
    try:
        with open(SESSION_LOG_PATH, "a") as f:
            f.write(f"\n{'='*70}\n[SESSION START] {time.ctime()} | Target M5 IP: {target_ip}:{udp_port}\n{'='*70}\n")
    except Exception:
        pass

    log_event(f"BRIDGE_START: Network 0xMakers_Dev | Target: {target_ip}:{udp_port}")

    print("=" * 65)
    print("   M5CORE2 4X4 MECANUM TRASH CAN - RADIO BRIDGE")
    print("=" * 65)
    print("WiFi Network: 0xMakers_Dev")

    # Resolve IP
    if args.broadcast:
        target_ip = "255.255.255.255"
        print("[*] Mode: Subnet Broadcast (255.255.255.255)")
    elif target_ip == "auto":
        found_ip = discover_m5_ip(discovery_port, timeout_sec=2.5)
        if found_ip:
            target_ip = found_ip
            print(f"[OK] Auto-discovered M5Core2 at IP: {target_ip}")
        else:
            target_ip = "192.168.0.121"
            print(f"[INFO] Using configured IP: {target_ip}")
    else:
        print(f"[*] Target M5Core2 IP: {target_ip}:{udp_port}")

    # Wait for Joystick
    joy = init_joystick()
    while joy is None:
        print("\n[!] No USB Joystick detected!")
        print("    1. Connect RadioMaster Pocket to Laptop via USB-C.")
        print("    2. On the radio LCD screen, choose 'USB Joystick (HID)'.")
        print("    Waiting 2s...")
        time.sleep(2.0)
        joy = init_joystick()

    print(f"[OK] Controller Connected: {joy.get_name()}")
    log_event(f"JOYSTICK_CONNECTED: {joy.get_name()} ({joy.get_numaxes()} axes, {joy.get_numbuttons()} buttons)")
    print(f"Starting transmission loop at {send_rate_hz} Hz...")
    time.sleep(0.5)

    seq = 0
    last_print = 0
    armed = True
    last_armed = None
    last_throttle_active = False
    consecutive_joy_errors = 0
    m5_telemetry_summary = "Listening on UDP :4212..."
    last_logged_m5_msg = ""

    try:
        while True:
            t_loop_start = time.time()
            pygame.event.pump()

            # Debounced Joystick check
            joy_ok = False
            raw_rx = 0.0
            raw_ry = 0.0
            raw_lt = -1.0
            raw_ly = 0.0

            try:
                if pygame.joystick.get_count() > 0 and joy is not None:
                    ax_rx = axes_cfg.get("right_x_strafe", 0)
                    ax_ry = axes_cfg.get("right_y_forward", 3)
                    ax_lt = axes_cfg.get("left_y_throttle", 2)
                    ax_ly = axes_cfg.get("left_x_yaw", 1)

                    num_axes = joy.get_numaxes()
                    raw_rx = joy.get_axis(ax_rx) if ax_rx < num_axes else 0.0
                    raw_ry = joy.get_axis(ax_ry) if ax_ry < num_axes else 0.0
                    raw_lt = joy.get_axis(ax_lt) if ax_lt < num_axes else -1.0
                    raw_ly = joy.get_axis(ax_ly) if ax_ly < num_axes else 0.0
                    joy_ok = True
                    consecutive_joy_errors = 0
            except Exception as e:
                joy_ok = False

            if not joy_ok:
                consecutive_joy_errors += 1
                if consecutive_joy_errors == 1:
                    log_event(f"BRIDGE_WARN: Joystick frame read dropped (attempt 1)")
                if consecutive_joy_errors >= 15:
                    log_event("BRIDGE_ERROR: Joystick disconnected (>15 frames)! Sending STOP packet.")
                    stop_pkt = f"TRASH,{seq},0,0,0,0,0,0,0,0,0\n".encode('utf-8')
                    sock.sendto(stop_pkt, (target_ip, udp_port))
                    joy = init_joystick()
                    time.sleep(0.05)
                    continue
                else:
                    raw_lt = -1.0
                    raw_rx = 0.0
                    raw_ry = 0.0
                    raw_ly = 0.0

            # Apply inversions
            inv_rx = invert_cfg.get("right_x_strafe", False)
            inv_ry = invert_cfg.get("right_y_forward", True)
            inv_lt = invert_cfg.get("left_y_throttle", False)
            inv_ly = invert_cfg.get("left_x_yaw", False)

            val_rx = -raw_rx if inv_rx else raw_rx
            val_ry = -raw_ry if inv_ry else raw_ry
            val_ly = -raw_ly if inv_ly else raw_ly

            # Compute throttle scalar [0.0, 1.0]
            center_sprung = throttle_cfg.get("center_sprung", False)
            throttle = compute_throttle_scale(raw_lt, invert=inv_lt, center_sprung=center_sprung, bottom_thresh=bottom_thresh)

            # Apply deadband & expo
            vx = apply_deadband_and_expo(val_rx, deadband, expo)
            vy = apply_deadband_and_expo(val_ry, deadband, expo)
            yaw = apply_deadband_and_expo(val_ly, deadband, expo)

            # Safety Arming
            require_arm = safety_cfg.get("require_arm_switch", False)
            if require_arm:
                arm_btn = safety_cfg.get("arm_button_index", 0)
                if joy and arm_btn < joy.get_numbuttons():
                    armed = bool(joy.get_button(arm_btn))
            else:
                armed = True

            # Track and log state transitions
            if armed != last_armed:
                last_armed = armed
                log_event(f"BRIDGE: Arm State changed -> {'ARMED' if armed else 'DISARMED'}")

            throttle_active = (throttle > 0.001)
            if throttle_active != last_throttle_active:
                last_throttle_active = throttle_active
                if throttle_active:
                    log_event(f"BRIDGE: Throttle ACTIVE ({int(throttle*100)}%) -> Driving")
                else:
                    log_event(f"BRIDGE: Throttle IDLE (0%) -> Motors Commanded Stop")

            # Motor Kinematics
            if not armed:
                m = [0, 0, 0, 0]
            else:
                m = compute_mecanum_powers(vx, vy, yaw, throttle, max_speed_pct, motor_inversions)

            # Format UDP packet
            # Format: TRASH,<seq>,<armed>,<vx_100>,<vy_100>,<yaw_100>,<thr_100>,<m0>,<m1>,<m2>,<m3>\n
            vx_i = int(round(vx * 100))
            vy_i = int(round(vy * 100))
            yaw_i = int(round(yaw * 100))
            thr_i = int(round(throttle * 100))
            arm_i = 1 if armed else 0

            packet_str = f"TRASH,{seq},{arm_i},{vx_i},{vy_i},{yaw_i},{thr_i},{m[0]},{m[1]},{m[2]},{m[3]}\n"
            sock.sendto(packet_str.encode('utf-8'), (target_ip, udp_port))
            seq = (seq + 1) % 65536

            # Drain incoming M5Core2 debug & telemetry logs from UDP port 4212
            while True:
                ready = select.select([log_sock], [], [], 0)
                if ready[0]:
                    try:
                        data, _ = log_sock.recvfrom(512)
                        line = data.decode('utf-8', errors='ignore').strip()
                        if line.startswith("DBG,"):
                            # Format: DBG,<millis>,<vbat>,<arm>,[TAG] message
                            parts = line.split(",", 4)
                            msg = parts[4] if len(parts) >= 5 else line
                            log_event(f"M5: {msg}")
                        elif line.startswith("M5LOG,"):
                            # Format: M5LOG,<millis>,<vbat>,<batPct>,<armed>,<estop>,<age>,<packetCount>,<m0>,<m1>,<m2>,<m3>,<msg>
                            parts = line.split(",", 12)
                            if len(parts) >= 13:
                                vbat = f"{int(parts[2])/1000.0:.2f}V"
                                bat_pct = parts[3] + "%"
                                rx_age = parts[6] + "ms"
                                status_msg = parts[12]
                                m5_telemetry_summary = f"{vbat} ({bat_pct}) | Age: {rx_age} | {status_msg}"
                                if status_msg != last_logged_m5_msg and "Idle" not in status_msg:
                                    last_logged_m5_msg = status_msg
                                    log_event(f"M5_STATUS: {status_msg}")
                    except Exception:
                        break
                else:
                    break

            # Periodic HUD refresh (10Hz)
            if t_loop_start - last_print > 0.1:
                last_print = t_loop_start
                clear_screen()
                print("=" * 68)
                print("   M5CORE2 4X4 MECANUM TRASH CAN - UDP RADIO BRIDGE & MONITOR")
                print("=" * 68)
                arm_label = "[ARMED]" if armed else "[DISARMED / SAFE]"
                print(f" Target M5 IP : {target_ip}:{udp_port}")
                print(f" Controller   : {joy.get_name() if joy else 'None'}")
                print(f" State        : {arm_label}  |  Throttle: {thr_i}%")
                print("-" * 68)
                print(" STICK INPUTS:")
                print(f"  Right Stick (Swerve Translation) : Vx(Strafe)={vx:+.2f}  Vy(Fwd)={vy:+.2f}")
                print(f"  Left Stick  (Yaw Rotation)       : Yaw={yaw:+.2f}")
                print(f"  Master Throttle Stick            : {throttle * 100.0:5.1f}%")
                print("-" * 68)
                print(" MECANUM MOTOR SPEED VALUES (-127 to +127):")
                print(f"  Front-Left  (CH0): {m[0]:+4d}   |   Front-Right (CH1): {m[1]:+4d}")
                print(f"  Rear-Left   (CH2): {m[2]:+4d}   |   Rear-Right  (CH3): {m[3]:+4d}")
                print("-" * 68)
                print(f" M5 TELEMETRY (UDP :4212): {m5_telemetry_summary}")
                print("-" * 68)
                print(" LIVE DEBUG & EVENT LOG (Last 6 entries):")
                entries = list(event_logs)[-6:]
                if entries:
                    for entry in entries:
                        print(f"  {entry}")
                else:
                    print("  [Waiting for debug events from M5Core2...]")
                print("-" * 68)
                print(f" Streaming: Sent {seq} pkts @ {send_rate_hz}Hz | Log saved: session_debug.log")
                print("=" * 68)

            # Sleep to maintain send rate
            elapsed = time.time() - t_loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping bridge. Sending failsafe STOP commands to M5...")
        log_event("BRIDGE_STOP: User pressed Ctrl+C. Sending 5x STOP packets.")
        stop_pkt = f"TRASH,{seq},0,0,0,0,0,0,0,0,0\n".encode('utf-8')
        for _ in range(5):
            try:
                sock.sendto(stop_pkt, (target_ip, udp_port))
            except Exception:
                pass
            time.sleep(0.02)
        sock.close()
        print("[OK] Stopped cleanly. Motors disarmed.")


if __name__ == "__main__":
    main()
