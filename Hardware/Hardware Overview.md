# HARDWARE OVERVIEW
Detailed overview of the hardware architecture of our WRO Future Engineers robot.

---

##  Table of Contents
- [Overview](#overview)
- [Hardware Overview](#hardware-overview)
  - [Processing Unit](#processing-unit)
  - [Locomotion & Actuators](#locomotion--actuators)
  - [Motor Driver](#motor-driver)
  - [Power Management](#power-management)
  - [Sensors](#sensors)
  - [Camera System](#camera-system)
  - [Custom Electronics & PCB](#custom-electronics--pcb)
  - [Mechanical Structure](#mechanical-structure)
  - [User Interface](#user-interface)
- [Dimensions & Weight](#dimensions--weight)
- [Bill of Materials](#bill-of-materials)
- [Advantages](#advantages)

---

## Overview
Our robot is designed around **stability, modularity, and speed**.  
The hardware integrates:
- High-efficiency DC motors with optimised gearboxes  
- Raspberry Pi 4 for advanced vision and decision-making  
- Multiple feedback sensors (IMU, IR, Limit Switches)  
- Custom 3D-printed and LEGO-based chassis for compactness and modularity  
- Stable power distribution with buck converters and Li-ion batteries  

This ensures consistent performance across the **Obstacle Challenge** and **Open Challenge**.

---

## Hardware Overview

### Processing Unit
- **Raspberry Pi 4B (4GB RAM)**  
  - Central brain of the robot.  
  - Runs computer vision (OpenCV) and control algorithms in real time.  
  - Handles communication with sensors via I²C and GPIO.  
  - Selected for its balance of processing power, cost, and community support.  

---

### Locomotion & Actuators
The robot uses a carefully tested drive + steering system optimised for torque-speed balance.

- **Drive Motor: D360 Brushed DC Motor + 22:1 Gearbox**  
  - Provides sufficient torque for speed and manoeuvrability.  
  - Mounted in a **3D-printed housing** for vibration dampening and stable alignment.  

- **Steering: REV Robotics 2000 Series Dual Mode Servo**  
  - Dual functionality: can operate as a servo (angular control) or a continuous motor.  
  - Provides precise steering and a smooth turning radius.  
  - Mounted with GoBilda servo frame for durability.  

Other motors considered during iterations:  
- **N20 DC Motor** → lightweight but insufficient torque under load.  
- **REV NEO 550 Brushless** → powerful, but bulky and harder to tune with Spark MAX.  
- **LEGO Medium Motor** → easy integration but limited performance.  

Final choice balanced **speed, torque, and compactness** for competition.

---

### Motor Driver
- **TB6612FNG Dual H-Bridge Driver**  
  - Drives the brushed DC motor.  
  - Supports sufficient current with minimal heating.  
  - Provides braking and smooth acceleration.  

---

### Power Management
A robust power system ensures stability and prevents brownouts.

- **7.4V Li-ion Battery Pack (2S, 1300 mAh, 25C)**  
  - High energy density, rechargeable, stable voltage.  

- **Buck Converters**  
  - **5V 3A Buck Converter**: powers Raspberry Pi and sensors.  
  - **USB Step-Down Converter**: additional regulated output for Pi Camera and low-voltage peripherals.  
  - Efficient DC-DC regulation ensures minimal energy loss and protects sensitive electronics.  

---

### Sensors
The robot uses a combination of sensors for feedback and environment perception:

1. **Raspberry Pi Camera Module 3 (Wide)**  
   - Vision system for obstacle detection, wall following, lane recognition, and parking.  
   - Works with OpenCV for HSV-based color detection (red/green pillars, black walls, blue/orange lap lines, magenta parking zone).  

2. **BNO055 IMU**  
   - 9-axis (Gyroscope + Accelerometer + Magnetometer).  
   - Provides orientation for stable steering corrections.  
   - Runs built-in sensor fusion for accurate yaw, pitch, and roll readings.  

3. **IR Sensors (×4)**  
   - Used for short-range obstacle detection and alignment during parking.  

4. **VEX Limit Switches**  
   - Detect physical collisions or if the robot gets stuck.  
   - Provides fail-safe redundancy.  

---

### Camera System
- **Raspberry Pi Camera Module 3 (Wide-Angle)**  
  - Mounted on the front of the robot.  
  - Wide field of view enables simultaneous wall, obstacle, and marker detection.  
  - Paired with OpenCV for real-time image processing.  

---

### Custom Electronics & PCB
- Soldered a zero board to ensure strong connections and reliability 
- Functions:  
  - Power distribution from battery to buck converters.  
  - Organised sensor and motor connections.  
  - Reduced wiring complexity and improved reliability.
Picture of the Circuit diagram is attached

---

### Mechanical Structure
- **Chassis**: hybrid of LEGO Technic and 3D-printed parts.  
- **Design goals**: compact, modular, easy to modify.  
- **3D-Printed Custom Components**:  
  - Motor mounts for vibration reduction.  
  - Camera housing to block false detections from internal circuits.  
  - Protective covers for sensitive electronics.  

---

### User Interface
- **Push Buttons**  
  - Start/Stop robot.  
  - Emergency reset if required.  

---

## Dimensions & Weight
- **Weight**: ~650–700 g  
- **Dimensions**: ~16 cm (L) × 12 cm (W) × 10 cm (H)  

Compact enough for agility while maintaining balance during turns.

---

## Bill of Materials
| Component | Quantity | Source/Link |
|-----------|----------|-------------|
| Raspberry Pi 4B (4GB) | 1 | [Robu.in](https://robu.in/product/raspberry-pi-4-model-b-with-4-gb-ram/) |
| Raspberry Pi Camera Module 3 (Wide) | 1 | [Robu.in](https://robu.in/product/raspberry-pi-camera-module-3-wide/) |
| D360 Brushed DC Motor + Gearbox | 1 | [Temu](https://www.temu.com/goods_snapshot.html?goods_id=601099518898704) |
| TB6612FNG Motor Driver | 1 | [Robu.in](https://robu.in/product/motor-driver-tb6612fng-module) |
| REV 2000 Series Servo | 1 | [GoBilda](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/) |
| GoBilda Servo Mount | 1 | [GoBilda](https://www.gobilda.com/1802-series-servo-frame-43mm-width-for-standard-size-servos/) |
| 7.4V 2S Li-ion Battery Pack (1300 mAh, 25C) | 1 | [Robu.in](https://robu.in/product/orange-1300mah-2s-25c-7-4-v-lithium-polymer-battery-pack-li-po/) |
| Buck Converter (5V, 3A) | 1 | [Robu.in](https://robu.in/product/ultra-small-size-dc-dc-5v-3a-bec-power-supply-buck-step-down-module/) |
| USB Buck Converter | 1 | [Robu.in](https://robu.in/product/dc-to-dc-6-24v-to-5v-usb-output-step-down-power-charger-buck-converter/) |
| BNO055 IMU | 1 | [ThinkRobotics](https://thinkrobotics.com/products/9-dof-absolute-orientation-bno055-sensor) |
| VEX Limit Switches | 2 | [VEX Robotics](https://www.vexrobotics.com/276-2174.html) |
| IR Sensors | 4 | [Robu.in](https://robu.in/product/ir-infrared-obstacle-avoidance-sensor-module/) |
| LEGO Technic Parts | Multiple | — |
| 3D Printed Parts | Custom | — |

---

## Advantages
- **High Performance**: Custom D360 motor with gearbox tuned for speed + torque.  
- **Vision-Driven**: Pi Camera enables wall following, lap counting, and obstacle navigation.  
- **Failsafe Redundancy**: Limit switches prevent deadlocks if the robot gets stuck.  
- **Stable Power**: 5V regulation avoids Pi brownouts.  
- **Compact & Modular**: Hybrid LEGO + 3D printed structure supports quick modifications.  
- **Reliable Navigation**: BNO055 IMU stabilizes turns and corrections.  
- **Consistent Parking**: IR sensors ensure accurate parallel parking.  

---
## Why did we use a custom motor
Our motor is based on the D360 DC motor with a custom gearbox. We used this motor because we could not find any other motor that had the appropriate speed, torque, and weight. We found that using a custom motor was the best choice.

## Why did we use a white cover
Frequently, our camera would detect the electronics on our PCB as obstacles or walls. Therefore, we found that the best way to overcome this was to put a white cover over our PCB, so that our camera would not be confused by the colours on our PCB.

## Why did we feel that LEGO was the best option
The lego chassis is completly custom designed from scratch, and we found it was the best option as it gave us the flexibility to use whichever parts we want and the lego chassis is extremly modular in comparison to other of the shelf options. Lego parts are also strong and the chassis is extremly easy to change or repair at a moments notice in comparison to off the shelf options.


##CAD and Lego
The instructions for building our robot are provided in this repository [Robot Assembly Instructions](Robot_Assembly_Instructions.pdf). We have also attached an Excel sheet containing all the LEGO parts required to make the robot. We have also attached the CAD files, the cover and the custom housing in this repository. The complete CAD of the robot is also attached in a .io file format, as the CAD was made in LEGO Studio. The cad for the custom parts is given in STEP format, as that was made in onshape.
 
