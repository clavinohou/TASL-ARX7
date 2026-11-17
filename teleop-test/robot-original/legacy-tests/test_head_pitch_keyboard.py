import arx_lift_python as arx
import time
import pygame
import sys

# Initialize the control loop
control_loop = arx.LiftHeadControlLoop("can5", arx.RobotType.X7S)

# Initialize pygame for keyboard input
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Head Pitch Keyboard Control")
clock = pygame.time.Clock()

# Control parameters
head_pitch = 0.0
pitch_increment = 0.01  # Start with small increments (radians)
running_state = 2

print("=== Head Pitch Keyboard Control ===")
print("Controls:")
print("  W/S - Increase/Decrease head pitch")
print("  +/- - Increase/Decrease increment size")
print("  R   - Reset pitch to 0")
print("  Q   - Quit")
print("  1   - Set running state to 1")
print("  2   - Set running state to 2")
print("")
print("Starting with increment: {:.4f} radians ({:.2f} degrees)".format(
    pitch_increment, pitch_increment * 180 / 3.14159))
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
                    head_pitch += pitch_increment
                    print(f"Pitch UP: {head_pitch:.4f} rad ({head_pitch * 180 / 3.14159:.2f} deg)")
                elif event.key == pygame.K_s:
                    head_pitch -= pitch_increment
                    print(f"Pitch DOWN: {head_pitch:.4f} rad ({head_pitch * 180 / 3.14159:.2f} deg)")
                elif event.key == pygame.K_r:
                    head_pitch = 0.0
                    print("RESET: Pitch set to 0")
                elif event.key == pygame.K_PLUS or event.key == pygame.K_KP_PLUS:
                    pitch_increment = min(pitch_increment * 1.5, 0.1)  # Cap at 0.1 rad
                    print(f"Increment increased to: {pitch_increment:.4f} rad ({pitch_increment * 180 / 3.14159:.2f} deg)")
                elif event.key == pygame.K_MINUS or event.key == pygame.K_KP_MINUS:
                    pitch_increment = max(pitch_increment / 1.5, 0.001)  # Min 0.001 rad
                    print(f"Increment decreased to: {pitch_increment:.4f} rad ({pitch_increment * 180 / 3.14159:.2f} deg)")
                elif event.key == pygame.K_1:
                    running_state = 1
                    print("Running state set to 1")
                elif event.key == pygame.K_2:
                    running_state = 2
                    print("Running state set to 2")

        # Get current values from robot
        current_pitch = control_loop.get_head_pitch()
        current_yaw = control_loop.get_head_yaw()
        current_height = control_loop.get_height()
        
        # Set the new head pitch
        control_loop.set_head_pitch(head_pitch)
        
        # Keep other values stable
        control_loop.set_head_yaw(0.0)
        control_loop.set_waist_pos(0.0)
        control_loop.set_height(0.0)
        control_loop.set_chassis_cmd(0, 0, 0, running_state)
        
        # Execute control loop
        control_loop.loop()
        
        # Print status every 100 loops to avoid spam
        if pygame.time.get_ticks() % 500 < 20:  # Roughly every 500ms
            print(f"Target: {head_pitch:.4f} rad | Actual: {current_pitch:.4f} rad | "
                  f"Diff: {abs(head_pitch - current_pitch):.4f} rad")
        
        # Small delay
        time.sleep(0.02)
        clock.tick(50)  # 50 FPS

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Stopping...")
except Exception as e:
    print(f"\nError occurred: {e}")
finally:
    # Reset to safe position before exit
    print("Resetting head pitch to 0 before exit...")
    control_loop.set_head_pitch(0.0)
    control_loop.set_head_yaw(0.0)
    control_loop.set_waist_pos(0.0)
    control_loop.set_height(0.0)
    control_loop.set_chassis_cmd(0, 0, 0, 2)
    
    # Send reset commands a few times to ensure they're received
    for _ in range(10):
        control_loop.loop()
        time.sleep(0.01)
    
    pygame.quit()
    print("Script terminated safely.")