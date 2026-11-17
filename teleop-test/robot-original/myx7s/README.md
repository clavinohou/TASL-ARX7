# VR Robot Control System with UDP Fan-out

This system allows multiple VR robot control applications to receive the same VR controller data simultaneously using a UDP fan-out service.

## Overview

The system consists of three main components:

1. **VR Fan-out Service** (`vr_fanout.py`) - Receives VR data on port 7000 and distributes it to multiple applications
2. **Base Robot Controller** (`vr_base.py`) - Controls base robot movement and lift (receives on port 7001)  
3. **Dual Arm Controller** (`vr_dual_arm.py`) - Controls dual robotic arms (receives on port 7002)

## Quick Start

### Option 1: Run Everything at Once
```bash
python run_vr_system.py
```

### Option 2: Run Components Individually
```bash
# Terminal 1: Start fan-out service
python vr_fanout.py

# Terminal 2: Start base robot controller  
python vr_base.py

# Terminal 3: Start dual arm controller
python vr_dual_arm.py
```

### Option 3: Run Specific Components
```bash
# Start only the fan-out service
python run_vr_system.py fanout

# Start only the base controller (requires fanout running)
python run_vr_system.py base

# Start only the dual arm controller (requires fanout running)  
python run_vr_system.py dual
```

## How It Works

### Before (Problem)
Both `vr_base.py` and `vr_dual_arm.py` tried to bind to the same UDP port (7000), causing conflicts when running simultaneously.

### After (Solution)
1. **Fan-out Service** listens on the original port 7000 for VR controller data
2. Validates incoming packets to ensure they match the expected VR controller format
3. Forwards each packet to multiple destination ports:
   - Port 7001 → `vr_base.py`
   - Port 7002 → `vr_dual_arm.py`
4. Each application receives the same VR data on its dedicated port

### Network Flow
```
VR Controller → Port 7000 (Fan-out) → Port 7001 (Base Robot)
                                   → Port 7002 (Dual Arms)
```

## Configuration

### Adding More Applications
To add additional VR control applications:

1. Edit `vr_fanout.py` and add a new port to the `DESTINATIONS` list:
```python
DESTINATIONS = [
    7001,  # vr_base.py
    7002,  # vr_dual_arm.py
    7003,  # your_new_app.py
]
```

2. Configure your new application to listen on the new port:
```python
UDP_IP = "127.0.0.1"
UDP_PORT = 7003
```

### Custom Ports
If you need to use different ports, modify the constants in each file:
- `vr_fanout.py`: Change `LISTEN_PORT` and `DESTINATIONS`
- `vr_base.py`: Change `UDP_PORT`
- `vr_dual_arm.py`: Change `UDP_PORT`

## Monitoring

The fan-out service provides real-time statistics:
```
[14:30:25] RX:   1234 | FWD:   2468 | ERR:   0 | Dest: 2 ports
```
- **RX**: Packets received from VR controller
- **FWD**: Total packets forwarded to all destinations  
- **ERR**: Number of forwarding errors
- **Dest**: Number of destination ports

## VR Controller Data Format

The system uses a binary UDP packet format:
```
Format: "<d3f4f2f2fB3f4f2f2fB"
Size: 73 bytes

Structure:
- 1 double:  timestamp
- 12 values: left controller (pos[3], rot[4], joy[2], trigger, grip, buttons)  
- 12 values: right controller (pos[3], rot[4], joy[2], trigger, grip, buttons)
```

## Troubleshooting

### Port Already in Use
```
OSError: [Errno 98] Address already in use
```
- Check if another instance is running: `sudo netstat -tulpn | grep :7000`
- Kill existing processes or change ports

### No VR Data Received
1. Verify VR controller is sending to port 7000
2. Check fan-out service is running and shows "RX" count increasing
3. Verify applications are listening on correct ports (7001, 7002)

### Applications Not Receiving Data
1. Ensure fan-out service started before applications
2. Check firewall settings for localhost traffic
3. Verify packet format matches between components

### Performance Issues
- Fan-out service forwards at ~500Hz with minimal latency
- Each packet generates N forwards (N = number of destinations)
- Monitor "ERR" count in statistics for forwarding failures

## Dependencies

- `numpy` (for dual arm controller)
- `scipy` (for dual arm controller)  
- `lift_controller` (for base robot)
- `arx_lift_python` (for base robot)
- `bimanual` (for dual arm controller)

## Safety Notes

- All applications include graceful shutdown handling (Ctrl+C)
- Robot controllers implement safe position defaults
- Fan-out service validates packet format before forwarding
- System designed for real-time operation with minimal buffering