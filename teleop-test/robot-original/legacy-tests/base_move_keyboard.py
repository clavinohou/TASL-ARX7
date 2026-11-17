import lift_controller
import arx_lift_python as arx
import time
import pygame
import sys

# Initialize the control loop
control_loop = arx.LiftHeadControlLoop("can5", arx.RobotType.X7S)

# Initialize pygame for keyboard input
pygame.init()
screen = pygame.display.set_mode((480, 320))
pygame.display.set_caption("Base Move Keyboard Control")
font = pygame.font.SysFont(None, 24)

BG_COLOR = (20, 20, 20)
TEXT_COLOR = (230, 230, 230)
ARMED_COLOR = (80, 220, 80)
BRAKED_COLOR = (220, 80, 80)

# Control parameters
linear_speed = 0.5   # v_x / v_y units per second
yaw_speed = 0.3       # w_z units per second
running_state = 2    # 2 = braked/safe, 1 = armed/driving

HELP_LINES = [
    "W/S: forward/back   A/D: strafe   Q/E: rotate",
    "1: arm   2: brake   +/-: speed   SPACE/R: stop   ESC: quit",
]

print("=== Base Move Keyboard Control ===")
print("Click into the pygame window to give it keyboard focus - status is")
print("shown in that window, so you don't need to look at this terminal.")
print("CAUTION: Chassis starts BRAKED (state 2). Press 1 to arm before driving.")
print("")


def draw_hud(v_x, v_y, w_z):
    screen.fill(BG_COLOR)
    state_text = "ARMED" if running_state == 1 else "BRAKED"
    state_color = ARMED_COLOR if running_state == 1 else BRAKED_COLOR
    lines = [
        (f"STATE: {state_text}", state_color),
        (f"v_x={v_x:+.3f}  v_y={v_y:+.3f}  w_z={w_z:+.3f}", TEXT_COLOR),
        (f"linear_speed={linear_speed:.3f}  yaw_speed={yaw_speed:.3f}", TEXT_COLOR),
        ("", TEXT_COLOR),
    ] + [(line, TEXT_COLOR) for line in HELP_LINES]
    y = 16
    for text, color in lines:
        surf = font.render(text, True, color)
        screen.blit(surf, (16, y))
        y += 28
    pygame.display.flip()


try:
    loop_count = 0
    while True:
        # Handle discrete key events (mode/speed changes, quit)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("Quitting...")
                    sys.exit()
                elif event.key == pygame.K_1:
                    running_state = 1
                    print("Chassis ARMED (running_state = 1)")
                elif event.key == pygame.K_2:
                    running_state = 2
                    print("Chassis BRAKED (running_state = 2)")
                elif event.key == pygame.K_PLUS or event.key == pygame.K_KP_PLUS:
                    linear_speed = min(linear_speed * 1.5, 1.0)
                    yaw_speed = min(yaw_speed * 1.5, 1.5)
                    print(f"Speed increased: linear={linear_speed:.3f}, yaw={yaw_speed:.3f}")
                elif event.key == pygame.K_MINUS or event.key == pygame.K_KP_MINUS:
                    linear_speed = max(linear_speed / 1.5, 0.02)
                    yaw_speed = max(yaw_speed / 1.5, 0.02)
                    print(f"Speed decreased: linear={linear_speed:.3f}, yaw={yaw_speed:.3f}")
                elif event.key in (pygame.K_SPACE, pygame.K_r):
                    running_state = 2
                    print("EMERGENCY STOP: velocities zeroed, chassis braked")

        # Continuous drive keys -> velocity command (only while armed)
        keys = pygame.key.get_pressed()
        v_x = 0.0
        v_y = 0.0
        w_z = 0.0
        if running_state == 1:
            if keys[pygame.K_w]:
                v_x += linear_speed
            if keys[pygame.K_s]:
                v_x -= linear_speed
            if keys[pygame.K_a]:
                v_y += linear_speed
            if keys[pygame.K_d]:
                v_y -= linear_speed
            if keys[pygame.K_q]:
                w_z += yaw_speed
            if keys[pygame.K_e]:
                w_z -= yaw_speed

        # Keep lift/head/waist stable, drive the chassis
        control_loop.set_head_yaw(0.0)
        control_loop.set_head_pitch(0.0)
        control_loop.set_waist_pos(0.0)
        control_loop.set_height(0.0)
        control_loop.set_chassis_cmd(v_x, v_y, w_z, running_state)

        # Execute control loop
        control_loop.loop()

        # Redraw the HUD at ~50Hz (every 10th control loop) to keep the
        # 500Hz CAN feed rate cheap - font rendering every 2ms is wasteful.
        loop_count += 1
        if loop_count % 10 == 0:
            draw_hud(v_x, v_y, w_z)

        # Run at 500Hz to match the other control scripts (test_lift.py,
        # vr_robot_control_safe.py) - the CAN motor driver appears to need
        # commands fed at this rate or it ignores/zeroes them.
        time.sleep(0.002)

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Stopping...")
except Exception as e:
    print(f"\nError occurred: {e}")
finally:
    # Reset to safe/braked state before exit
    print("Braking chassis before exit...")
    control_loop.set_head_yaw(0.0)
    control_loop.set_head_pitch(0.0)
    control_loop.set_waist_pos(0.0)
    control_loop.set_height(0.0)
    control_loop.set_chassis_cmd(0.0, 0.0, 0.0, 2)

    # Send stop commands a few times to ensure they're received
    for _ in range(10):
        control_loop.loop()
        time.sleep(0.01)

    pygame.quit()
    print("Script terminated safely.")
