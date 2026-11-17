import lift_controller
import arx_lift_python as arx
import time
import pygame
import sys

# Initialize the control loop
control_loop = arx.LiftHeadControlLoop("can5", arx.RobotType.X7S)

# Initialize pygame for keyboard input
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Base Lift Keyboard Control")

# Control parameters
HEIGHT_MIN = 0.0
HEIGHT_MAX = 22.0
lift_height = 0.0
height_increment = 0.5
running_state = 2

print("=== Base Lift Keyboard Control ===")
print("Controls:")
print("  W/S - Raise/Lower lift")
print("  +/- - Increase/Decrease increment size")
print("  R   - Reset height to 0")
print("  Q   - Quit")
print("  1   - Set running state to 1")
print("  2   - Set running state to 2")
print("")
print("Starting with increment: {:.2f}".format(height_increment))
print("CAUTION: Start with small movements and observe robot behavior!")
print("")

try:
    while True:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    print("Quitting...")
                    sys.exit()
                elif event.key == pygame.K_w:
                    lift_height = min(lift_height + height_increment, HEIGHT_MAX)
                    print(f"LIFT UP: {lift_height:.3f}")
                elif event.key == pygame.K_s:
                    lift_height = max(lift_height - height_increment, HEIGHT_MIN)
                    print(f"LIFT DOWN: {lift_height:.3f}")
                elif event.key == pygame.K_r:
                    lift_height = 0.0
                    print("RESET: Height set to 0")
                elif event.key == pygame.K_PLUS or event.key == pygame.K_KP_PLUS:
                    height_increment = min(height_increment * 1.5, 5.0)
                    print(f"Increment increased to: {height_increment:.3f}")
                elif event.key == pygame.K_MINUS or event.key == pygame.K_KP_MINUS:
                    height_increment = max(height_increment / 1.5, 0.05)
                    print(f"Increment decreased to: {height_increment:.3f}")
                elif event.key == pygame.K_1:
                    running_state = 1
                    print("Running state set to 1")
                elif event.key == pygame.K_2:
                    running_state = 2
                    print("Running state set to 2")

        # Get current values from robot
        current_height = control_loop.get_height()

        # Set the new lift height
        control_loop.set_height(lift_height)

        # Keep other values stable
        control_loop.set_head_yaw(0.0)
        control_loop.set_head_pitch(0.0)
        control_loop.set_waist_pos(0.0)
        control_loop.set_chassis_cmd(0, 0, 0, running_state)

        # Execute control loop
        control_loop.loop()

        # Print status every ~500ms to avoid spam
        if pygame.time.get_ticks() % 500 < 20:
            print(f"Target: {lift_height:.3f} | Actual: {current_height:.3f} | "
                  f"Diff: {abs(lift_height - current_height):.3f}")

        # Run at 500Hz to match the other control scripts (test_lift.py,
        # vr_robot_control_safe.py) - the CAN motor driver appears to need
        # commands fed at this rate or it ignores/zeroes them.
        time.sleep(0.002)

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Stopping...")
except Exception as e:
    print(f"\nError occurred: {e}")
finally:
    # Reset to safe position before exit
    print("Resetting lift height to 0 before exit...")
    control_loop.set_height(0.0)
    control_loop.set_head_yaw(0.0)
    control_loop.set_head_pitch(0.0)
    control_loop.set_waist_pos(0.0)
    control_loop.set_chassis_cmd(0, 0, 0, 2)

    # Send reset commands a few times to ensure they're received
    for _ in range(10):
        control_loop.loop()
        time.sleep(0.01)

    pygame.quit()
    print("Script terminated safely.")
