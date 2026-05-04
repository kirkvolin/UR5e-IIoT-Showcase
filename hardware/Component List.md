# Hardware List

## Robot System

| Component | Manufacturer | Part Number | Description |
|-----------|-------------|-------------|-------------|
| Robot Arm | Universal Robots | UR5e | 6-axis collaborative robot, 850mm reach, 5kg payload |
| Robot Controller | Universal Robots | - | UR5e controller cabinet (included with robot) |
| Teach Pendant | Universal Robots | - | PolyScope touchscreen interface (included with robot) |
| Gripper | Robotiq | 2F-85 | 2-finger adaptive gripper, 85mm stroke |

## PLC and I/O

| Component | Manufacturer | Part Number | Description |
|-----------|-------------|-------------|-------------|
| PLC Controller | Allen-Bradley | 1769-L33ER | CompactLogix 5370 controller |
| Remote I/O Adapter | Allen-Bradley | 1734-AENTR | POINT I/O Dual Port EtherNet/IP adapter |
| Output Module | Allen-Bradley | 1734-OB8 | POINT I/O 8-point 24VDC sourcing output module |

## HMI

| Component | Manufacturer | Part Number | Description |
|-----------|-------------|-------------|-------------|
| Touchscreen HMI | Allen-Bradley | PanelView 5310 | Touchscreen operator interface, programmed with View Designer |

## Pilot Devices

| Component | Manufacturer | Part Number | Description |
|-----------|-------------|-------------|-------------|
| Indicator Light (Green) | AutomationDirect | GCX1232-24L | 22mm metal LED pilot light, 24VDC, green |
| Indicator Light (Yellow) | AutomationDirect | GCX1233-24L | 22mm metal LED pilot light, 24VDC, yellow |
| Indicator Light (Red) | AutomationDirect | GCX1231-24L | 22mm metal LED pilot light, 24VDC, red |
| Indicator Light (Blue) | AutomationDirect | GCX1234-24L | 22mm metal LED pilot light, 24VDC, blue |
| Push Buttons | - | - | 4x momentary push buttons (one per candy station) |
| Tower Light | PATLITE | LME-402FB | 4-tier LED signal tower, 24V AC/DC, with buzzer |

## Networking

| Component | Manufacturer | Part Number | Description |
|-----------|-------------|-------------|-------------|
| Ethernet Switch | - | - | Unmanaged industrial Ethernet switch, minimum 5 ports |
| Ethernet Cables | - | - | Cat5e patch cables (PLC, UR, HMI, AENTR) |

## Power

| Component | Manufacturer | Part Number | Description |
|-----------|-------------|-------------|-------------|
| 24VDC Power Supply | - | - | 24VDC, 3A (72W) minimum, DIN-rail mount |

## PS5 Controller (Manual Control Mode)

| Component | Manufacturer | Part Number | Description |
|-----------|-------------|-------------|-------------|
| PS5 Controller | Sony | DualSense | Wireless controller, connected via USB or Bluetooth |
| Laptop / PC | - | - | Windows PC running Python script for controller interface |

## Software

| Software | Manufacturer | Purpose |
|----------|-------------|---------|
| Studio 5000 Logix Designer | Rockwell Automation | PLC programming (ladder logic) |
| View Designer | Rockwell Automation | HMI screen design for PanelView 5000 series |
| PolyScope | Universal Robots | UR teach pendant programming |
| Python 3.x | - | PS5 controller script runtime |
| pylogix | - | Python library for CIP communication with CompactLogix |
| pygame | - | Python library for PS5 controller input |

## Network Configuration

| Device | IP Address | Subnet Mask | Protocol |
|--------|-----------|-------------|----------|
| CompactLogix 5370 | 192.168.1.10 | 255.255.255.0 | EtherNet/IP (Scanner) |
| UR5e Controller | 192.168.1.20 | 255.255.255.0 | EtherNet/IP (Adapter) |
| PanelView 5310 | 192.168.1.30 | 255.255.255.0 | CIP |
| 1734-AENTR | 192.168.1.40 | 255.255.255.0 | EtherNet/IP (Adapter) |

## Power Budget Summary

| Device | Current Draw | Notes |
|--------|-------------|-------|
| 1734-AENTR | 430 mA | Adapter logic + backplane |
| 1734-OB8 | 32 mA | Backplane overhead |
| 4x Indicator Lights | 320 mA | 80 mA each (worst case with incandescent bulbs) |
| PATLITE Tower Light | 215 mA | 4 tiers + buzzer steady state |
| PanelView 5310 | 500 mA | Estimated (verify per model) |
| **24VDC Total** | **~1.5 A** | **Recommended: 24VDC 3A supply** |
| UR5e Controller | ~4 A @ 120VAC | Separate AC mains circuit |
