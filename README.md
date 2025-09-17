# RoboMisk_####
**Contemplate · Create · Conquer**

This repository contains details about the design, evolution, and continuous development of our self-driving robot vehicle, built and programmed by Team Circuitron for the Future Engineers 2025 category of the World Robot Olympiad (WRO).

---

## Member Details
1. Abdullah Alobaidi (Software)
2. Abdullah Almodaimigh (Hardware)  

---

## Table of Contents

- `t-photos`: Contains 2 photos of the team, including an official and a funny photo  
- `v-photos`: Contains 6 photos of the vehicle from all sides, including top and bottom views  
- `video`: Contains a `video.md` file linking to a video showing the robot in action  
- `Hardware`  Includes hardware overview, design process, final design, and component choices  
- `Code`: Includes all source code controlling the vehicle's behaviour 
- `Strategy`: Explanation of wall-following logic and decision-making approach  
- `models`  Files for 3D printing, laser cutting, or CNC (if applicable)  
- `other`: Supporting documents like connection instructions, datasets, specs  
- `special-thanks`  Acknowledgements and credits

---
## Engineering Notebook
Our engineering notebook contains more information about our robot and can be found here:

## Hardware Overview
---

## Table of Contents
1. [Hardware Overview](#hardware-overview)
2. [Processing Unit](#processing-unit)
3. [Vision System](#vision-system)
4. [Locomotion](#locomotion)
5. [Steering System](#steering-system)
6. [Power Management](#power-management)
7. [Chassis](#chassis)
8. [License](#license)



Our robotic platform is engineered with a modular hardware architecture  
to deliver high performance, flexibility, and support iterative development cycles.

---

## Processing Unit

**Raspberry Pi 4 Model B**

- CPU: Quad-core Cortex-A72 (ARM v8)
- RAM: 2GB / 4GB / 8GB (depending on configuration)
- Interfaces: GPIO, USB, HDMI, CSI (camera)

This board supports seamless integration of sensor arrays and enables  
Real-time control algorithms are essential for autonomous navigation and decision-making.

---

## Vision System

**Camera:** Raspberry Pi Camera Module

**Key Features:**
- Obstacle detection
- Line tracking
- Environmental mapping
- Native compatibility with the Raspberry Pi ecosystem

---

## Locomotion

**Wheels:** LEGO wheels for modularity and superior material quality  
**Motor:** D360 DC motor  

**Optimised for:**
- Size
- Weight
- Torque output
- Efficient mobility across diverse terrains

---

## Steering System

**Servo Motor:** [Insert Model] high-precision servo  

**Features:**
- Responsive and accurate control
- Reliable navigation in dynamic environments

---

## Power Management

**Buck Converter:** [Insert Model]  
**Input Voltage:** [Insert Voltage]V  

**Features:**
- Steps down the voltage to the required levels for all subsystems
- Ensures stable operation
- Protects sensitive electronics

---

## Chassis

**Framework:** LEGO-based chassis  

**Benefits:**
- Modularity for rapid iterations
- Ease of customisation
- Supports quick repositioning of components and continuous mechanical optimisation



## Strategy
Open Challenge Strategy
The Open Challenge begins with the robot aligning to the nearest wall and determining its driving orientation through initial line detection, where a blue line initiates anticlockwise motion and an orange line initiates clockwise motion. The robot applies wall-following algorithms to maintain a central path while incrementally counting laps using blue and orange corner lines, incorporating a debounce delay to prevent multiple counts per pass. Upon completion of three laps, confirmed by twelve line detections, the robot transitions into parking mode. It follows the outer black wall until detecting a pink wall, where it uses IMU feedback for angular correction and executes a precise parallel parking maneuver.

Obstacle Challenge Strategy
For the Obstacle Challenge, the robot begins within the parking zone and determines its initial direction based on black wall detection before exiting. During navigation, its vision system identifies red and green obstacles, steering right in the presence of red and left for green, while simultaneously applying wall-following and line detection for lap counting. After completing three laps, the robot performs an additional alignment lap to optimise its parking approach. Using IR sensors for accurate wall distance measurement and IMU-based angle corrections, the robot executes a reliable parallel parking sequence between two boundaries to conclude the challenge.


## Special Thanks
We are extremely grateful to our mentor, Mr. Sunil Solanki, for his invaluable guidance, encouragement, and belief in our abilities throughout this journey. His mentorship has been the driving force behind our learning and progress. We would also like to express our sincere gratitude to the honourable and venerable judges for giving us the privilege to present our work before such a distinguished panel. Your time, expertise, and dedication to this competition mean the world to us. It is an incredible honour to have our efforts reviewed by such esteemed individuals, and we deeply value this opportunity to showcase our passion, creativity, and hard work in robotics under your guidance.
