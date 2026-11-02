#!/usr/bin/env python3
"""
Interactive Joystick & RadioMaster Pocket Diagnostic / Calibration Tool.
Compatible with Windows, Linux, and macOS.
"""

import os
import sys
import time
import json

# Suppress pygame startup prompt
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

try:
    import pygame
except ImportError:
    print("[ERROR] pygame is not installed. Run: pip install pygame")
    sys.exit(1)

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")


def make_bar(val, width=20):
    """Generate an ASCII visual bar for values between -1.0 and +1.0."""
    val = max(-1.0, min(1.0, val))
    center = width // 2
    pos = int((val + 1.0) / 2.0 * (width - 1))
    bar = list("-" * width)
    bar[center] = "|"
    bar[pos] = "O"
    return "".join(bar)


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def wait_for_joystick():
    """Waits until a joystick/controller is connected."""
    pygame.init()
    pygame.joystick.init()

    last_dots = 0
    while pygame.joystick.get_count() == 0:
        clear_screen()
        print("=" * 65)
        print("   RADIOMASTER POCKET / USB HID JOYSTICK SCANNER")
        print("=" * 65)
        dots = "." * (last_dots % 4)
        last_dots += 1
        print(f"\nSearching for connected USB Joysticks{dots}")
        print("\nINSTRUCTIONS FOR RADIOMASTER POCKET:")
        print(" 1. Turn on your RadioMaster Pocket.")
        print(" 2. Connect the TOP USB-C port to your laptop.")
        print(" 3. When the popup appears on the radio screen, select:")
        print("      --> 'USB Joystick (HID)'")
        print(" 4. This tool will automatically detect it once connected.\n")
        print("Press Ctrl+C to exit.")
        pygame.joystick.quit()
        time.sleep(1.0)
        pygame.joystick.init()

    joy = pygame.joystick.Joystick(0)
    joy.init()
    return joy


def monitor_mode(joy):
    """Real-time live monitoring of all axes, buttons, and hats."""
    print(f"\n[OK] Connected: {joy.get_name()}")
    print(f"Axes: {joy.get_numaxes()}, Buttons: {joy.get_numbuttons()}, Hats: {joy.get_numhats()}")
    print("\nStarting live view... (Press Ctrl+C to return/exit)\n")
    time.sleep(1.0)

    try:
        while True:
            pygame.event.pump()
            clear_screen()
            print("=" * 65)
            print(f"   DEVICE: {joy.get_name()} (Axes: {joy.get_numaxes()}, Buttons: {joy.get_numbuttons()})")
            print("=" * 65)

            # Axes
            print("\n--- ANALOG AXES (-1.0 to +1.0) ---")
            for i in range(joy.get_numaxes()):
                axis_val = joy.get_axis(i)
                bar = make_bar(axis_val, 24)
                print(f" Axis {i:2d}: [{bar}]  {axis_val:+.3f}")

            # Buttons
            print("\n--- BUTTONS / SWITCHES ---")
            btn_str = []
            for b in range(min(16, joy.get_numbuttons())):
                state = joy.get_button(b)
                btn_str.append(f"B{b}: {'[ON]' if state else ' -- '}")
            # Print in rows of 4
            for r in range(0, len(btn_str), 4):
                print("   " + "   ".join(btn_str[r:r+4]))

            # Hats (if any)
            if joy.get_numhats() > 0:
                print("\n--- HATS / D-PAD ---")
                for h in range(joy.get_numhats()):
                    print(f" Hat {h}: {joy.get_hat(h)}")

            print("\nTip: Move sticks and switches to see which numbers react.")
            print("Run 'python joystick_tester.py --calibrate' to auto-configure.")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting monitor mode.")


def prompt_axis_motion(joy, prompt_msg):
    """Detects which axis moves significantly from resting position."""
    input(f"\n{prompt_msg}\nPress Enter to begin motion detection...")

    # Sample resting baseline
    pygame.event.pump()
    time.sleep(0.2)
    pygame.event.pump()
    baselines = [joy.get_axis(i) for i in range(joy.get_numaxes())]

    print("Listening... MOVE the requested stick now!")
    start_time = time.time()
    best_axis = None
    best_delta = 0.0
    detected_direction = 1

    while time.time() - start_time < 8.0:
        pygame.event.pump()
        for i in range(joy.get_numaxes()):
            val = joy.get_axis(i)
            delta = val - baselines[i]
            if abs(delta) > abs(best_delta):
                best_delta = delta
                best_axis = i
                detected_direction = 1 if delta > 0 else -1

        if abs(best_delta) > 0.45:
            # Axis moved significantly
            break
        time.sleep(0.05)

    if best_axis is not None and abs(best_delta) > 0.4:
        print(f" [OK] Detected Axis {best_axis} (deflection: {best_delta:+.2f})")
        return best_axis, best_delta
    else:
        print(" [WARNING] No clear movement detected. Keeping default.")
        return None, 0.0


def calibrate_wizard(joy):
    """Step-by-step interactive configuration wizard."""
    clear_screen()
    print("=" * 65)
    print("   RADIOMASTER POCKET CONFIGURATION WIZARD")
    print("=" * 65)
    print("This wizard will identify your stick channels and update config.json.")
    print("Target layout:")
    print(" - RIGHT STICK: Swerve / Direction (X: Strafe, Y: Forward/Back)")
    print(" - LEFT STICK:  Throttle (Y: Speed level) & Yaw (X: Rotate)")
    print("=" * 65)

    input("\nEnsure all sticks are centered (throttle down if un-sprung).\nPress Enter to start...")

    # 1. Right Stick X (Strafe)
    axis_rx, delta_rx = prompt_axis_motion(joy, "Step 1: Push the RIGHT STICK to the RIGHT.")
    invert_rx = False
    if delta_rx < 0:
        # Pushing right gave negative value -> invert
        invert_rx = True

    # 2. Right Stick Y (Forward)
    axis_ry, delta_ry = prompt_axis_motion(joy, "Step 2: Push the RIGHT STICK FORWARD (UP).")
    invert_ry = True
    if delta_ry > 0:
        # Standard joystick up is negative, if it gave positive, invert accordingly
        invert_ry = False

    # 3. Left Stick Y (Throttle)
    axis_lt, delta_lt = prompt_axis_motion(joy, "Step 3: Push the LEFT STICK (THROTTLE) ALL THE WAY UP.")
    invert_lt = True
    if delta_lt > 0:
        invert_lt = False

    # 4. Left Stick X (Yaw)
    axis_ly, delta_ly = prompt_axis_motion(joy, "Step 4: Push the LEFT STICK (YAW) to the RIGHT (Clockwise).")
    invert_ly = False
    if delta_ly < 0:
        invert_ly = True

    # Check for safety arm switch
    print("\nStep 5: Safety Arming (Optional)")
    ans = input("Do you want to assign an ARM switch/button? (y/N): ").strip().lower()
    arm_enabled = False
    arm_btn = 0
    if ans == 'y':
        print("Flip your desired ARM switch or press button...")
        t0 = time.time()
        detected_btn = None
        while time.time() - t0 < 6.0:
            pygame.event.pump()
            for b in range(joy.get_numbuttons()):
                if joy.get_button(b):
                    detected_btn = b
                    break
            if detected_btn is not None:
                break
            time.sleep(0.05)
        if detected_btn is not None:
            print(f" [OK] Detected Button/Switch {detected_btn}")
            arm_enabled = True
            arm_btn = detected_btn
        else:
            print(" [INFO] No button press detected. Arming disabled.")

    # Load existing config
    cfg = {}
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r") as f:
                cfg = json.load(f)
        except Exception:
            cfg = {}

    if "joystick" not in cfg:
        cfg["joystick"] = {}
    if "axes" not in cfg["joystick"]:
        cfg["joystick"]["axes"] = {}
    if "invert" not in cfg["joystick"]:
        cfg["joystick"]["invert"] = {}
    if "safety" not in cfg["joystick"]:
        cfg["joystick"]["safety"] = {}

    if axis_rx is not None:
        cfg["joystick"]["axes"]["right_x_strafe"] = axis_rx
        cfg["joystick"]["invert"]["right_x_strafe"] = invert_rx
    if axis_ry is not None:
        cfg["joystick"]["axes"]["right_y_forward"] = axis_ry
        cfg["joystick"]["invert"]["right_y_forward"] = invert_ry
    if axis_lt is not None:
        cfg["joystick"]["axes"]["left_y_throttle"] = axis_lt
        cfg["joystick"]["invert"]["left_y_throttle"] = invert_lt
    if axis_ly is not None:
        cfg["joystick"]["axes"]["left_x_yaw"] = axis_ly
        cfg["joystick"]["invert"]["left_x_yaw"] = invert_ly

    cfg["joystick"]["safety"]["require_arm_switch"] = arm_enabled
    cfg["joystick"]["safety"]["arm_button_index"] = arm_btn

    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    print("\n" + "=" * 65)
    print(" Calibration complete! Saved to config.json.")
    print(f"  Right Stick Strafe (X):  Axis {cfg['joystick']['axes']['right_x_strafe']} (Invert: {cfg['joystick']['invert']['right_x_strafe']})")
    print(f"  Right Stick Forward (Y): Axis {cfg['joystick']['axes']['right_y_forward']} (Invert: {cfg['joystick']['invert']['right_y_forward']})")
    print(f"  Left Stick Throttle (Y): Axis {cfg['joystick']['axes']['left_y_throttle']} (Invert: {cfg['joystick']['invert']['left_y_throttle']})")
    print(f"  Left Stick Yaw (X):      Axis {cfg['joystick']['axes']['left_x_yaw']} (Invert: {cfg['joystick']['invert']['left_x_yaw']})")
    print(f"  Arm switch required:     {arm_enabled} (Button {arm_btn})")
    print("=" * 65)
    print("\nYou can now start the bridge by running: python radio_bridge.py")


def main():
    calibrate = ("--calibrate" in sys.argv) or ("-c" in sys.argv)
    joy = wait_for_joystick()

    if calibrate:
        calibrate_wizard(joy)
    else:
        monitor_mode(joy)


if __name__ == "__main__":
    main()
