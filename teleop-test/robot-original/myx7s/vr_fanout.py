#!/usr/bin/env python3
"""
VR Controller UDP Fan-out Service

This service receives VR controller data on the original UDP port (7000)
and fans it out to multiple destination ports, allowing multiple applications
to receive the same VR data simultaneously.

Usage:
    python vr_fanout.py

The service will:
- Listen on UDP port 7000 for incoming VR controller data
- Forward packets to port 7001 (for vr_base.py)
- Forward packets to port 7002 (for vr_dual_arm.py)
- Support adding more destinations easily
"""

import socket
import struct
import threading
import time
import signal
import sys
from dataclasses import dataclass
from typing import List, Tuple

# VR Controller packet format (same as original)
FMT = "<d3f4f2f2fB3f4f2f2fB"
PACKET_SIZE = struct.calcsize(FMT)

# Network configuration
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 7000
FORWARD_IP = "127.0.0.1"  # Forward to localhost

# Destination ports for fan-out
DESTINATIONS = [
    7001,  # vr_base.py
    7002,  # vr_dual_arm.py
    # Add more ports here as needed
]

@dataclass
class FanOutStats:
    """Statistics for monitoring the fan-out service"""
    packets_received: int = 0
    packets_forwarded: int = 0
    forward_errors: int = 0
    last_packet_time: float = 0.0

class VRFanOutService:
    """UDP fan-out service for VR controller data"""
    
    def __init__(self, listen_port: int, destinations: List[int]):
        self.listen_port = listen_port
        self.destinations = destinations
        self.stats = FanOutStats()
        self.running = False
        
        # Create listening socket
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.settimeout(1.0)  # For clean shutdown
        
        # Create forwarding sockets
        self.forward_sockets = []
        for _ in destinations:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.forward_sockets.append(sock)
    
    def validate_packet(self, data: bytes) -> bool:
        """Validate that the packet has the expected VR controller format"""
        if len(data) < PACKET_SIZE:
            return False
        
        try:
            # Try to unpack the packet to verify format
            struct.unpack(FMT, data[:PACKET_SIZE])
            return True
        except struct.error:
            return False
    
    def forward_packet(self, data: bytes):
        """Forward packet to all destination ports"""
        for i, (sock, port) in enumerate(zip(self.forward_sockets, self.destinations)):
            try:
                sock.sendto(data, (FORWARD_IP, port))
                self.stats.packets_forwarded += 1
            except Exception as e:
                self.stats.forward_errors += 1
                print(f"Error forwarding to port {port}: {e}")
    
    def print_stats(self):
        """Print current statistics"""
        print(f"\r[{time.strftime('%H:%M:%S')}] "
              f"RX: {self.stats.packets_received:6d} | "
              f"FWD: {self.stats.packets_forwarded:6d} | "
              f"ERR: {self.stats.forward_errors:3d} | "
              f"Dest: {len(self.destinations)} ports", end="", flush=True)
    
    def run(self):
        """Main fan-out loop"""
        self.running = True
        
        try:
            self.listen_sock.bind((LISTEN_IP, self.listen_port))
            print(f"VR Fan-out Service started")
            print(f"Listening on: {LISTEN_IP}:{self.listen_port}")
            print(f"Forwarding to ports: {', '.join(map(str, self.destinations))}")
            print(f"Expected packet size: {PACKET_SIZE} bytes")
            print("Press Ctrl+C to stop\n")
            
            stats_counter = 0
            
            while self.running:
                try:
                    data, addr = self.listen_sock.recvfrom(2048)
                    
                    # Validate packet format
                    if not self.validate_packet(data):
                        print(f"\nInvalid packet from {addr}: {len(data)} bytes")
                        continue
                    
                    # Update stats
                    self.stats.packets_received += 1
                    self.stats.last_packet_time = time.time()
                    
                    # Forward to all destinations
                    self.forward_packet(data)
                    
                    # Print stats every 100 packets
                    stats_counter += 1
                    if stats_counter >= 100:
                        self.print_stats()
                        stats_counter = 0
                    
                except socket.timeout:
                    # Check if we should continue running
                    continue
                    
        except Exception as e:
            if self.running:
                print(f"\nFan-out service error: {e}")
        finally:
            self.cleanup()
    
    def stop(self):
        """Stop the fan-out service"""
        self.running = False
    
    def cleanup(self):
        """Clean up sockets and resources"""
        print("\nCleaning up fan-out service...")
        
        self.listen_sock.close()
        for sock in self.forward_sockets:
            sock.close()
        
        print("Final statistics:")
        print(f"  Packets received: {self.stats.packets_received}")
        print(f"  Packets forwarded: {self.stats.packets_forwarded}")
        print(f"  Forward errors: {self.stats.forward_errors}")
        print("VR Fan-out Service stopped")

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nReceived interrupt signal. Stopping fan-out service...")
    fanout_service.stop()

def main():
    """Main function"""
    global fanout_service
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and start fan-out service
    fanout_service = VRFanOutService(LISTEN_PORT, DESTINATIONS)
    
    try:
        fanout_service.run()
    except KeyboardInterrupt:
        print("\nForced shutdown...")
        fanout_service.stop()

if __name__ == "__main__":
    main()