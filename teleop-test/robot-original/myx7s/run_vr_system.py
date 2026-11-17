#!/usr/bin/env python3
"""
VR System Launcher

This script helps launch the VR fan-out system and control applications.
You can run individual components or the full system.

Usage:
    python run_vr_system.py [component]
    
Components:
    fanout    - Start only the fan-out service
    base      - Start only the base robot controller (requires fanout running)
    dual      - Start only the dual arm controller (requires fanout running)
    all       - Start all components (default)
    
Examples:
    python run_vr_system.py          # Start all components
    python run_vr_system.py fanout   # Start only fan-out service
    python run_vr_system.py base     # Start only base controller
"""

import subprocess
import sys
import time
import signal
import os
from typing import List, Optional

class VRSystemLauncher:
    """Manages launching and coordination of VR system components"""
    
    def __init__(self):
        self.processes: List[subprocess.Popen] = []
        self.running = True
    
    def launch_component(self, script_name: str, component_name: str) -> Optional[subprocess.Popen]:
        """Launch a single component"""
        try:
            print(f"Starting {component_name}...")
            process = subprocess.Popen([
                sys.executable, script_name
            ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
               universal_newlines=True, bufsize=1)
            
            self.processes.append(process)
            return process
            
        except Exception as e:
            print(f"Failed to start {component_name}: {e}")
            return None
    
    def launch_fanout(self) -> Optional[subprocess.Popen]:
        """Launch the fan-out service"""
        return self.launch_component("vr_fanout.py", "VR Fan-out Service")
    
    def launch_base(self) -> Optional[subprocess.Popen]:
        """Launch the base robot controller"""
        return self.launch_component("vr_base.py", "Base Robot Controller")
    
    def launch_dual_arm(self) -> Optional[subprocess.Popen]:
        """Launch the dual arm controller"""
        return self.launch_component("vr_dual_arm.py", "Dual Arm Controller")
    
    def wait_for_fanout_ready(self, timeout: float = 5.0) -> bool:
        """Wait for fan-out service to be ready"""
        print("Waiting for fan-out service to initialize...")
        time.sleep(2)  # Give it time to bind to port
        return True
    
    def launch_all(self):
        """Launch all components in the correct order"""
        print("=== VR System Launcher ===")
        print("Starting all components...\n")
        
        # Start fan-out service first
        fanout_process = self.launch_fanout()
        if not fanout_process:
            print("Failed to start fan-out service. Aborting.")
            return
        
        # Wait for fan-out to be ready
        if not self.wait_for_fanout_ready():
            print("Fan-out service failed to initialize. Aborting.")
            self.cleanup()
            return
        
        # Start the controller applications
        print("Fan-out service ready. Starting controllers...\n")
        
        base_process = self.launch_base()
        dual_process = self.launch_dual_arm()
        
        # Check if both started successfully
        active_components = []
        if fanout_process and fanout_process.poll() is None:
            active_components.append("Fan-out Service")
        if base_process and base_process.poll() is None:
            active_components.append("Base Robot Controller")
        if dual_process and dual_process.poll() is None:
            active_components.append("Dual Arm Controller")
        
        if active_components:
            print(f"Successfully started: {', '.join(active_components)}")
            print("\nPress Ctrl+C to stop all components")
            self.monitor_processes()
        else:
            print("No components started successfully.")
            self.cleanup()
    
    def monitor_processes(self):
        """Monitor all processes and handle shutdown"""
        try:
            while self.running:
                # Check if any process has died
                for process in self.processes[:]:  # Copy list to avoid modification during iteration
                    if process.poll() is not None:
                        print(f"\nProcess {process.pid} has terminated")
                        self.processes.remove(process)
                
                # If all processes are dead, exit
                if not self.processes:
                    print("All processes have terminated")
                    break
                    
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\nReceived interrupt signal. Shutting down all components...")
            self.running = False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all processes"""
        print("Stopping all processes...")
        
        for process in self.processes:
            if process.poll() is None:  # Process is still running
                try:
                    process.terminate()
                    # Give it a moment to terminate gracefully
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        print(f"Force killing process {process.pid}")
                        process.kill()
                        process.wait()
                except Exception as e:
                    print(f"Error stopping process {process.pid}: {e}")
        
        self.processes.clear()
        print("All components stopped")

def print_usage():
    """Print usage information"""
    print(__doc__)

def main():
    """Main function"""
    launcher = VRSystemLauncher()
    
    # Set up signal handler
    def signal_handler(signum, frame):
        launcher.running = False
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Parse command line arguments
    component = "all"
    if len(sys.argv) > 1:
        component = sys.argv[1].lower()
    
    # Validate component choice
    valid_components = ["fanout", "base", "dual", "all"]
    if component not in valid_components:
        print(f"Invalid component: {component}")
        print(f"Valid options: {', '.join(valid_components)}")
        print_usage()
        return 1
    
    # Check if required files exist
    required_files = {
        "fanout": ["vr_fanout.py"],
        "base": ["vr_base.py"],
        "dual": ["vr_dual_arm.py"],
        "all": ["vr_fanout.py", "vr_base.py", "vr_dual_arm.py"]
    }
    
    missing_files = []
    for file in required_files[component]:
        if not os.path.exists(file):
            missing_files.append(file)
    
    if missing_files:
        print(f"Missing required files: {', '.join(missing_files)}")
        return 1
    
    # Launch requested component(s)
    try:
        if component == "fanout":
            process = launcher.launch_fanout()
            if process:
                print("Fan-out service started. Press Ctrl+C to stop.")
                launcher.monitor_processes()
        
        elif component == "base":
            process = launcher.launch_base()
            if process:
                print("Base robot controller started. Press Ctrl+C to stop.")
                print("Note: Make sure the fan-out service is running on port 7000->7001")
                launcher.monitor_processes()
        
        elif component == "dual":
            process = launcher.launch_dual_arm()
            if process:
                print("Dual arm controller started. Press Ctrl+C to stop.")
                print("Note: Make sure the fan-out service is running on port 7000->7002")
                launcher.monitor_processes()
        
        elif component == "all":
            launcher.launch_all()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())