# Pan/Tilt LiDAR Room Mapper

## Components Needed
- 1 × TF-Luna LiDAR  
- 2 × MG90S Servos (Pan & Tilt)  
- 1 × Raspberry Pi 4 Model B  
- Separate 5 V power supply for servos  

## Wiring

### TF-Luna
| Pin | Connection |
|-----|------------|
| 5 V  | Pin 2 (5 V power) |
| RXD  | Pin 8 (GPIO14)    |
| TXD  | Pin 10 (GPIO15)   |
| GND  | Pin 6 (Ground)    |

### Pan Servo (MG90S)
| Wire  | Connection |
|-------|------------|
| Orange | Pin 12 (GPIO18) |
| Red    | 5 V from separate power supply |
| Brown  | GND (connect all grounds together) |

### Tilt Servo (MG90S)
| Wire  | Connection |
|-------|------------|
| Orange | Pin 11 (GPIO17) |
| Red    | 5 V from separate power supply |
| Brown  | GND (connect all grounds together) |

> **Note:** Make sure all grounds are connected together

## How the Room Mapper Program Works
The `room_map` program sweeps the pan and tilt servos across the room, taking distance measurements from the TF-Luna LiDAR at each position. It logs the raw pan, tilt, and distance values to a file and can also display the data live. This allows you to map the room in a 3D-like coordinate system, which can later be visualized in Python.

## Pan/Tilt Turret Example
![Pan/Tilt LiDAR Setup](Raspberry%20Pi%20Code/Images/Pan_and_Tilt_Turret.jpg)

## Whole Setup Example
![Pan/Tilt LiDAR Setup](Raspberry%20Pi%20Code/Images/Whole_Setupt.jpg)
