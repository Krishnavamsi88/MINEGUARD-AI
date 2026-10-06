# MINEGUARD AI
### Smart Mine Safety and Autonomous HEMM Fleet Coordination System

> **Smart India Hackathon 2026 — SIH26007**

MINEGUARD AI is a simulation-based mine safety and autonomous HEMM fleet coordination system designed for safe and efficient operation of mine vehicles in fog and low-visibility conditions.

The system combines autonomous route following, LiDAR-based obstacle detection, safety risk assessment, vehicle-to-vehicle (V2V) coordination, simulated thermal imaging, GPS/IMU telemetry, and a real-time monitoring dashboard.

---

## Problem Statement

**SIH26007 — Safe and Efficient Operation of Mine Vehicles in Fog and Low-Visibility Conditions in Open Cast Iron Ore Mines**

Open-cast mining operations can experience severe fog and low-visibility conditions during monsoon periods. Visibility may reduce to only a few metres, increasing the risk of collisions and unsafe vehicle movement.

MINEGUARD AI addresses this challenge through a simulation-first architecture that continuously monitors vehicle state, detects obstacles, coordinates multiple HEMMs, and provides safety-aware speed control.

---

## Project Overview

The system simulates a mining environment containing three autonomous Heavy Earth Moving Machines (HEMMs).

Each vehicle:

- Follows a predefined mine route autonomously
- Receives an operator-defined target speed
- Uses LiDAR for obstacle awareness
- Uses simulated GPS and IMU telemetry
- Provides RGB and simulated thermal camera data
- Publishes vehicle state for fleet coordination
- Receives safety and V2V constraints

The operator does not directly steer the vehicle. Instead, the operator specifies the desired target speed while the autonomy and safety systems determine the safe operating speed.

### Control Hierarchy

```text
AUTONOMOUS ROUTE
       ↓
OPERATOR TARGET SPEED
       ↓
SAFETY LIMIT
       ↓
V2V COORDINATION
       ↓
     /cmd_vel
       ↓
     HEMM

Key Features
🚛 Autonomous HEMM Fleet
Three simulated HEMMs operate within the mine environment using lightweight waypoint-based route following.
🛡️ Safety System
The safety layer continuously evaluates obstacle distance and generates risk levels:
CLEAR
  ↓
ADVISORY
  ↓
CAUTION
  ↓
WARNING
  ↓
CRITICAL

The safety system can reduce the vehicle speed or stop the vehicle when required.
📡 V2V Fleet Coordination
The vehicles exchange state information including:
- Vehicle position
- Speed
- Heading
- Route information
- Traffic state
- Safety/risk state
At shared conflict zones, deterministic right-of-way rules are used to prevent conflicting vehicle movements.
A vehicle without priority waits until the conflict zone is clear.
📷 Multi-Sensor Simulation
Each HEMM includes simulated:
- LiDAR
- GPS
- IMU
- RGB camera
- Depth camera
- Thermal camera representation
The thermal view is generated in simulation using image processing and is intended as a simulated FLIR-like visualization rather than physically calibrated thermal hardware.
🖥️ Real-Time Dashboard
The monitoring dashboard provides:
- Live mine map
- HEMM positions
- Vehicle headings
- Speed
- Safety status
- Risk level
- V2V state
- Camera feeds
- Thermal visualization
- Fleet telemetry
System Architecture
                    ┌─────────────────────┐
                    │   Mine Environment  │
                    │   Gazebo Simulation  │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │   Sensor Simulation  │
                    │ LiDAR GPS IMU Camera │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
       ┌────────────┐   ┌────────────┐   ┌────────────┐
       │ Autonomous │   │   Safety   │   │    V2V     │
       │ Controller │   │   System   │   │ Coordination│
       └─────┬──────┘   └─────┬──────┘   └─────┬──────┘
             │                │                │
             └────────────────┼────────────────┘
                              ▼
                       ┌─────────────┐
                       │ Final Speed │
                       │  /cmd_vel   │
                       └──────┬──────┘
                              ▼
                         ┌─────────┐
                         │  HEMM   │
                         └────┬────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │ Monitoring       │
                    │ Dashboard        │
                    └──────────────────┘

Technology Stack
Technology	Purpose
Ubuntu 24.04 LTS	Development platform
ROS 2 Jazzy	Robot communication and control
Gazebo Sim 8.11.0	Mine and vehicle simulation
Blender 5.2.1	Mine environment modelling
Python	Control, safety and dashboard logic
OpenCV	Image processing and simulated thermal visualization
ros_gz_bridge	ROS 2 ↔ Gazebo communication
HTML/CSS/JavaScript	Dashboard interface


ROS 2 Communication
HEMM Topics
Each vehicle publishes and subscribes through its namespace.
Example:
/hemm_01/cmd_vel
/hemm_01/odom
/hemm_01/scan
/hemm_01/gps
/hemm_01/imu
/hemm_01/camera/image_raw
/hemm_01/depth/image_raw
/hemm_01/safety/warning
/hemm_01/safety/obstacle_distance
/hemm_01/safety/risk_level

Equivalent topics are used for HEMM-02 and HEMM-03.
Fleet Communication
/fleet/vehicle_states

The fleet state contains information required for multi-vehicle coordination.
Repository Structure
SIH_Mine_Safety/
│
├── blender/
├── gazebo/
├── dashboard/
│
├── docs/
│   └── MINEGUARD_AI_SIH_Final_Project_Report.pdf
│
├── media/
│   ├── demo/
│   │   └── MINEGUARD_AI_SIH_Demo.mp4
│   │
│   └── screenshots/
│
├── hemm_autonomous_controller.py
├── hemm_safety_system.py
├── hemm_dumper_v2_model.sdf
├── sih_mine_environment.py
│
├── launch_3_hemm_autonomous.sh
├── launch_3hemm_bridge.sh
├── launch_camera_view.sh
├── launch_driver_assistance.sh
├── launch_hemm_sim.sh
├── launch_mine_world.sh
├── launch_sensors.sh
│
└── README.md

Installation
Prerequisites
Install and configure:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Sim
- Python 3
- Required ROS-Gazebo bridge packages
Source ROS 2:
source /opt/ros/jazzy/setup.bash

Clone the repository:
git clone <YOUR_GITHUB_REPOSITORY_URL>
cd SIH_Mine_Safety

## Running the Final 3-HEMM Simulation

The main demonstration uses the three-HEMM autonomous fleet world.

### 1. Launch the 3-HEMM Autonomous Simulation

```bash
./launch_3_hemm_autonomous.sh

Safety Logic
The final vehicle speed is constrained by both the route controller and the safety system.
Conceptually:
Final Speed =
MIN(Route Speed, Safety Speed Limit)

Safety limits progressively reduce vehicle speed as the risk level increases.
At critical risk, the vehicle is commanded to stop.
Safety has priority over fleet coordination.
V2V Coordination
The V2V system uses deterministic rule-based coordination rather than machine learning.
At a shared conflict zone:
Vehicle A → PRIORITY / GO
Vehicle B → WAITING
Vehicle C → WAITING

Once the priority vehicle clears the conflict zone:
Vehicle A → CLEAR
Vehicle B → RESUME
Vehicle C → RESUME

If safety detects a critical hazard, safety commands override V2V coordination.
Dashboard
The dashboard provides a centralized view of the simulated fleet.
Dashboard Overview
 
Live Mine Map
 
Fleet Telemetry
 
V2V Coordination
 
Safety Warning
 
Simulation Results
The simulation demonstrates:
- Three-vehicle autonomous fleet operation
- Route following
- Obstacle detection
- Safety-based speed limitation
- Critical-risk stopping
- V2V right-of-way coordination
- Fleet telemetry
- Camera visualization
- Simulated thermal visualization
- Real-time dashboard monitoring
Demo Video
🎥 MINEGUARD AI Simulation Demo
The complete demonstration is available in:
media/demo/MINEGUARD_AI_SIH_Demo.mp4

Screenshots
Mine Environment
 
Three HEMM Simulation
 
Thermal Camera
 Key Features
🚛 Autonomous HEMM Fleet
Three simulated HEMMs operate within the mine environment using lightweight waypoint-based route following.
🛡️ Safety System
The safety layer continuously evaluates obstacle distance and generates risk levels:
CLEAR
  ↓
ADVISORY
  ↓
CAUTION
  ↓
WARNING
  ↓
CRITICAL

The safety system can reduce the vehicle speed or stop the vehicle when required.
📡 V2V Fleet Coordination
The vehicles exchange state information including:
- Vehicle position
- Speed
- Heading
- Route information
- Traffic state
- Safety/risk state
At shared conflict zones, deterministic right-of-way rules are used to prevent conflicting vehicle movements.
A vehicle without priority waits until the conflict zone is clear.
📷 Multi-Sensor Simulation
Each HEMM includes simulated:
- LiDAR
- GPS
- IMU
- RGB camera
- Depth camera
- Thermal camera representation
The thermal view is generated in simulation using image processing and is intended as a simulated FLIR-like visualization rather than physically calibrated thermal hardware.
🖥️ Real-Time Dashboard
The monitoring dashboard provides:
- Live mine map
- HEMM positions
- Vehicle headings
- Speed
- Safety status
- Risk level
- V2V state
- Camera feeds
- Thermal visualization
- Fleet telemetry
System Architecture
                    ┌─────────────────────┐
                    │   Mine Environment  │
                    │   Gazebo Simulation  │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │   Sensor Simulation  │
                    │ LiDAR GPS IMU Camera │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
       ┌────────────┐   ┌────────────┐   ┌────────────┐
       │ Autonomous │   │   Safety   │   │    V2V     │
       │ Controller │   │   System   │   │ Coordination│
       └─────┬──────┘   └─────┬──────┘   └─────┬──────┘
             │                │                │
             └────────────────┼────────────────┘
                              ▼
                       ┌─────────────┐
                       │ Final Speed │
                       │  /cmd_vel   │
                       └──────┬──────┘
                              ▼
                         ┌─────────┐
                         │  HEMM   │
                         └────┬────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │ Monitoring       │
                    │ Dashboard        │
                    └──────────────────┘

Technology Stack
Technology	Purpose
Ubuntu 24.04 LTS	Development platform
ROS 2 Jazzy	Robot communication and control
Gazebo Sim 8.11.0	Mine and vehicle simulation
Blender 5.2.1	Mine environment modelling
Python	Control, safety and dashboard logic
OpenCV	Image processing and simulated thermal visualization
ros_gz_bridge	ROS 2 ↔ Gazebo communication
HTML/CSS/JavaScript	Dashboard interface


ROS 2 Communication
HEMM Topics
Each vehicle publishes and subscribes through its namespace.
Example:
/hemm_01/cmd_vel
/hemm_01/odom
/hemm_01/scan
/hemm_01/gps
/hemm_01/imu
/hemm_01/camera/image_raw
/hemm_01/depth/image_raw
/hemm_01/safety/warning
/hemm_01/safety/obstacle_distance
/hemm_01/safety/risk_level

Equivalent topics are used for HEMM-02 and HEMM-03.
Fleet Communication
/fleet/vehicle_states

The fleet state contains information required for multi-vehicle coordination.
Repository Structure
SIH_Mine_Safety/
│
├── blender/
├── gazebo/
├── dashboard/
│
├── docs/
│   └── MINEGUARD_AI_SIH_Final_Project_Report.pdf
│
├── media/
│   ├── demo/
│   │   └── MINEGUARD_AI_SIH_Demo.mp4
│   │
│   └── screenshots/
│
├── hemm_autonomous_controller.py
├── hemm_safety_system.py
├── hemm_dumper_v2_model.sdf
├── sih_mine_environment.py
│
├── launch_3_hemm_autonomous.sh
├── launch_3hemm_bridge.sh
├── launch_camera_view.sh
├── launch_driver_assistance.sh
├── launch_hemm_sim.sh
├── launch_mine_world.sh
├── launch_sensors.sh
│
└── README.md

Installation
Prerequisites
Install and configure:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Sim
- Python 3
- Required ROS-Gazebo bridge packages
Source ROS 2:
source /opt/ros/jazzy/setup.bash

Clone the repository:
git clone <YOUR_GITHUB_REPOSITORY_URL>
cd SIH_Mine_Safety

Running the Simulation
Launch the Mine World
./launch_mine_world.sh

Launch the HEMM Simulation
./launch_hemm_sim.sh

Start ROS-Gazebo Bridge
./launch_3hemm_bridge.sh

Start Autonomous Fleet
./launch_3_hemm_autonomous.sh

Start Sensors
./launch_sensors.sh

Start Dashboard
From the dashboard directory, start the dashboard using the project's dashboard launcher/server configuration.
The dashboard is available at:
http://localhost:8080

Safety Logic
The final vehicle speed is constrained by both the route controller and the safety system.
Conceptually:
Final Speed =
MIN(Route Speed, Safety Speed Limit)

Safety limits progressively reduce vehicle speed as the risk level increases.
At critical risk, the vehicle is commanded to stop.
Safety has priority over fleet coordination.
V2V Coordination
The V2V system uses deterministic rule-based coordination rather than machine learning.
At a shared conflict zone:
Vehicle A → PRIORITY / GO
Vehicle B → WAITING
Vehicle C → WAITING

Once the priority vehicle clears the conflict zone:
Vehicle A → CLEAR
Vehicle B → RESUME
Vehicle C → RESUME

If safety detects a critical hazard, safety commands override V2V coordination.
Dashboard
The dashboard provides a centralized view of the simulated fleet.
Dashboard Overview
 
Live Mine Map
 
Fleet Telemetry
 
V2V Coordination
 
Safety Warning
 
Simulation Results
The simulation demonstrates:
- Three-vehicle autonomous fleet operation
- Route following
- Obstacle detection
- Safety-based speed limitation
- Critical-risk stopping
- V2V right-of-way coordination
- Fleet telemetry
- Camera visualization
- Simulated thermal visualization
- Real-time dashboard monitoring
Demo Video
🎥 MINEGUARD AI Simulation Demo
The complete demonstration is available in:
media/demo/MINEGUARD_AI_SIH_Demo.mp4

Screenshots
Mine Environment
 
Three HEMM Simulation
 
Thermal Camera
 
Simulation Environment
 
Limitations
The current implementation is simulation-based.
The system does not currently include:
- Physical HEMM deployment
- Calibrated FLIR thermal hardware
- Physical V2V radios
- Real mine vehicle CAN integration
- Real-world GNSS correction
- Hardware radar integration
The thermal camera representation is simulated, and V2V communication is modelled through ROS 2 topics.
Future Scope
Future development can include:
- Physical sensor integration
- Real HEMM deployment
- Calibrated thermal cameras
- Dedicated V2V communication hardware
- Real mine GNSS integration
- CAN/vehicle interface integration
- Advanced perception models
- Real-world mine trials
- Edge deployment on embedded computing platforms
Documentation
📄 Final Project Report
docs/MINEGUARD_AI_SIH_Final_Project_Report.pdf

Smart India Hackathon
Problem Statement: SIH26007
Title: Safe and Efficient Operation of Mine Vehicles in Fog and Low-Visibility Conditions in Open Cast Iron Ore Mines
Theme: Smart Automation
Category: Hardware
Ministry: Ministry of Steel / NMDC
Project Status
Simulation Prototype — Completed
The current repository contains the simulation environment, autonomous fleet controller, safety system, V2V coordination, dashboard, documentation and demonstration media.
License
This project is developed as an academic and Smart India Hackathon prototype.
© 2026 MINEGUARD AI Project Team
Simulation Environment
 
Limitations
The current implementation is simulation-based.
The system does not currently include:
- Physical HEMM deployment
- Calibrated FLIR thermal hardware
- Physical V2V radios
- Real mine vehicle CAN integration
- Real-world GNSS correction
- Hardware radar integration
The thermal camera representation is simulated, and V2V communication is modelled through ROS 2 topics.
Future Scope
Future development can include:
- Physical sensor integration
- Real HEMM deployment
- Calibrated thermal cameras
- Dedicated V2V communication hardware
- Real mine GNSS integration
- CAN/vehicle interface integration
- Advanced perception models
- Real-world mine trials
- Edge deployment on embedded computing platforms
Documentation
📄 Final Project Report
docs/MINEGUARD_AI_SIH_Final_Project_Report.pdf

Smart India Hackathon
Problem Statement: SIH26007
Title: Safe and Efficient Operation of Mine Vehicles in Fog and Low-Visibility Conditions in Open Cast Iron Ore Mines
Theme: Smart Automation
Category: Hardware
Ministry: Ministry of Steel / NMDC
Project Status
Simulation Prototype — Completed
The current repository contains the simulation environment, autonomous fleet controller, safety system, V2V coordination, dashboard, documentation and demonstration media.
License
This project is developed as an academic and Smart India Hackathon prototype.
© 2026 MINEGUARD AI Project Team
