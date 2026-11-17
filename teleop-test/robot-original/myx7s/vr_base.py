"""
VR Base Robot Controller

This application controls a base robot using VR controller input.
It now receives VR data through the fan-out service on port 7001.

To use:
1. Start the fan-out service: python vr_fanout.py
2. Start this application: python vr_base.py
"""

import lift_controller
import arx_lift_python as arx
import time
import socket
import struct
import threading
import queue
from dataclasses import dataclass
from typing import Optional
import signal
import sys

# VR Controller button constants
PRIMARY    = (1 << 0)
SECONDARY  = (1 << 1)
TRIGGERBTN = (1 << 2)
GRIPBTN    = (1 << 3)
MENUBTN    = (1 << 4)

# UDP settings (receives from fan-out service)
UDP_IP   = "127.0.0.1"  # Listen on localhost for fan-out data
UDP_PORT = 7001         # Fan-out port for vr_base

# Struct format for VR controller data
FMT = "<d3f4f2f2fB3f4f2f2fB"
PACKET_SIZE = struct.calcsize(FMT)

# Global shutdown flag
shutdown_event = threading.Event()

@dataclass
class VRControllerState:
    """Container for VR controller state"""
    time: float
    left_pos: tuple
    left_rot: tuple
    left_joy: tuple
    left_trigger: float
    left_grip: float
    left_buttons: int
    right_pos: tuple
    right_rot: tuple
    right_joy: tuple
    right_trigger: float
    right_grip: float
    right_buttons: int

class VRControllerReceiver:
    """Handles UDP reception of VR controller data"""
    
    def __init__(self, data_queue: queue.Queue):
        self.data_queue = data_queue
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)  # Add timeout for clean shutdown
        
    def parse_packet(self, data: bytes) -> Optional[VRControllerState]:
        """Parse UDP packet into VR controller state"""
        if len(data) < PACKET_SIZE:
            print(f"Packet too short: {len(data)} < {PACKET_SIZE}")
            return None
            
        try:
            unpacked = struct.unpack(FMT, data[:PACKET_SIZE])
            
            # Timestamp
            time = unpacked[0]
            
            # Left controller
            l0 = 1
            left_pos = unpacked[l0+0:l0+3]
            left_rot = unpacked[l0+3:l0+7]
            left_joy = unpacked[l0+7:l0+9]
            left_trigger = unpacked[l0+9]
            left_grip = unpacked[l0+10]
            left_buttons = unpacked[l0+11]
            
            # Right controller
            r0 = 1 + 12
            right_pos = unpacked[r0+0:r0+3]
            right_rot = unpacked[r0+3:r0+7]
            right_joy = unpacked[r0+7:r0+9]
            right_trigger = unpacked[r0+9]
            right_grip = unpacked[r0+10]
            right_buttons = unpacked[r0+11]
            
            return VRControllerState(
                time=time,
                left_pos=left_pos,
                left_rot=left_rot,
                left_joy=left_joy,
                left_trigger=left_trigger,
                left_grip=left_grip,
                left_buttons=left_buttons,
                right_pos=right_pos,
                right_rot=right_rot,
                right_joy=right_joy,
                right_trigger=right_trigger,
                right_grip=right_grip,
                right_buttons=right_buttons
            )
        except Exception as e:
            print(f"Error parsing packet: {e}")
            return None
    
    def run(self):
        """Main UDP receiver loop"""
        self.sock.bind((UDP_IP, UDP_PORT))
        print(f"VR Controller receiver listening on UDP {UDP_IP}:{UDP_PORT} (fan-out service)")
        
        try:
            while not shutdown_event.is_set():
                try:
                    data, addr = self.sock.recvfrom(2048)
                    controller_state = self.parse_packet(data)
                    
                    if controller_state:
                        # Put latest controller state in queue (non-blocking)
                        try:
                            # Remove old data if queue is full
                            while not self.data_queue.empty():
                                try:
                                    self.data_queue.get_nowait()
                                except queue.Empty:
                                    break
                            self.data_queue.put_nowait(controller_state)
                        except queue.Full:
                            pass  # Skip if queue is full
                except socket.timeout:
                    continue  # Check shutdown_event and continue
                        
        except Exception as e:
            if not shutdown_event.is_set():
                print(f"UDP receiver error: {e}")
        finally:
            self.sock.close()
            print("VR receiver thread stopped")

class RobotController:
    """Handles robot control using VR controller input"""
    
    def __init__(self, data_queue: queue.Queue):
        self.data_queue = data_queue
        self.control_loop = arx.LiftHeadControlLoop("can5", arx.RobotType.X7S)
        self.running_state = 2
        self.lift_height = 0.0
        self.head_pitch = 0.3  # Default head pitch
        self.last_controller_state = None
        # Track previous button states for key up detection
        self.prev_left_buttons = 0
        self.prev_right_buttons = 0

    def apply_deadzone(self, value, deadzone=0.15):
        if abs(value) < deadzone:
            return 0.0
        return value
        
    def map_vr_to_robot_controls(self, vr_state: VRControllerState):
        """Map VR controller inputs to robot controls"""
        
        # Use right controller joystick for chassis movement
        # joy[0] = left/right, joy[1] = forward/backward
        chassis_x = self.apply_deadzone(vr_state.left_joy[1]) * 0.5 # forward/backward
        chassis_y = self.apply_deadzone(vr_state.left_joy[0]) * -0.5  # left/right
        chassis_yaw = self.apply_deadzone(vr_state.right_joy[0], 0.25) * -0.75  # rotation
        
        # Use left controller joystick for additional controls
        # left_joy[1] for lift height control
        lift_delta = self.apply_deadzone(vr_state.right_joy[1], 0.25) * 0.075
        self.lift_height += lift_delta
        self.lift_height = max(0, min(self.lift_height, 22))
        
        # Check for button up events (key release)
        # Right controller secondary button up: toggle running state between 1 and 2
        if (self.prev_right_buttons & SECONDARY) and not (vr_state.right_buttons & SECONDARY):
            self.running_state = 1 if self.running_state == 2 else 2
            print(f"Running state toggled to: {self.running_state}")
        
        # Left controller secondary button up: toggle head pitch between 0.3 and 0.8
        if (self.prev_left_buttons & SECONDARY) and not (vr_state.left_buttons & SECONDARY):
            self.head_pitch = 0.3 if self.head_pitch == 0.8 else 0.8
            print(f"Head pitch toggled to: {self.head_pitch}")
        
        # Update previous button states for next iteration
        self.prev_left_buttons = vr_state.left_buttons
        self.prev_right_buttons = vr_state.right_buttons
            
        # Apply controls to robot
        self.control_loop.set_head_yaw(0.0)
        self.control_loop.set_head_pitch(self.head_pitch)
        self.control_loop.set_waist_pos(0.0)
        self.control_loop.set_height(self.lift_height)
        self.control_loop.set_chassis_cmd(chassis_x, chassis_y, chassis_yaw, self.running_state)

    def safe_shutdown(self):
        """Safely return robot to default position"""
        print("Performing safe robot shutdown...")
        
        # Set all controls to safe defaults
        self.control_loop.set_head_yaw(0.0)
        self.control_loop.set_head_pitch(0.0)  # Return to neutral position
        self.control_loop.set_waist_pos(0.0)
        self.control_loop.set_height(0.0)  # Return lift to default
        self.control_loop.set_chassis_cmd(0.0, 0.0, 0.0, 2)  # Stop chassis
        
        # Run a few control loops to ensure commands are sent
        for _ in range(10):
            self.control_loop.loop()
            time.sleep(0.01)
        
        print("Robot safely returned to default position")
    
    def run(self):
        """Main robot control loop"""
        print("Robot controller started")
        
        try:
            while not shutdown_event.is_set():
                # Get latest VR controller data
                try:
                    vr_state = self.data_queue.get_nowait()
                    self.last_controller_state = vr_state
                    self.map_vr_to_robot_controls(vr_state)
                except queue.Empty:
                    # No new data, use last known state or defaults
                    if self.last_controller_state is None:
                        # No controller data yet, use safe defaults
                        self.control_loop.set_head_yaw(0.0)
                        self.control_loop.set_head_pitch(self.head_pitch)
                        self.control_loop.set_waist_pos(0.0)
                        self.control_loop.set_height(0.0)
                        self.control_loop.set_chassis_cmd(0.0, 0.0, 0.0, self.running_state)
                
                # Print debug info
                # print(f"head_yaw: {self.control_loop.get_head_yaw():.3f}")
                # print(f"head_pitch: {self.control_loop.get_head_pitch():.3f}")
                # print(f"height: {self.control_loop.get_height():.3f}")
                # print(f"running_state: {self.running_state}")
                
                # Keep the robot alive
                self.control_loop.loop()
                time.sleep(0.002)  # 500Hz control loop
                
        except Exception as e:
            print(f"Robot controller error: {e}")
        finally:
            # Always perform safe shutdown
            self.safe_shutdown()
            print("Robot controller thread stopped")

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nReceived interrupt signal. Shutting down gracefully...")
    shutdown_event.set()

def main():
    """Main function to start both threads"""
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create shared queue for VR controller data
    vr_data_queue = queue.Queue(maxsize=10)
    
    # Create VR controller receiver and robot controller
    vr_receiver = VRControllerReceiver(vr_data_queue)
    robot_controller = RobotController(vr_data_queue)
    
    # Create and start threads (NOT daemon threads)
    vr_thread = threading.Thread(target=vr_receiver.run)
    robot_thread = threading.Thread(target=robot_controller.run)
    
    print("Starting VR Robot Control System...")
    print("Controls:")
    print("  Left joystick: Chassis movement (forward/back, left/right)")
    print("  Right joystick: Lift height control (up/down) and chassis rotation (left/right)")
    print("  Right controller B button (release): Toggle running state between 1 and 2")
    print("  Left controller B button (release): Toggle head pitch between 0.3 and 0.8")
    print("  Ctrl+C to exit safely")
    
    try:
        vr_thread.start()
        robot_thread.start()
        
        # Keep main thread alive and wait for shutdown
        while not shutdown_event.is_set():
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nForced shutdown...")
        shutdown_event.set()
    
    # Wait for threads to finish properly
    print("Waiting for threads to finish...")
    robot_thread.join(timeout=5)  # Wait up to 5 seconds
    vr_thread.join(timeout=5)
    
    print("VR Robot Control System stopped safely")

if __name__ == "__main__":
    main()
