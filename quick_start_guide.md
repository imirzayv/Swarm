# Quick Start Guide: CV + AI + Drone Swarms — What You Need to Know Before You Start

> This document gives you a fast on-ramp into the research area. Read it in one sitting (~45 minutes), then use the references to go deeper.

---

## 1. The Big Picture

The intersection of **computer vision**, **artificial intelligence**, and **drone swarms** is about giving a fleet of autonomous aerial vehicles the ability to collectively perceive, reason about, and act upon their environment — without a human pilot for each drone.

A single drone with a camera can detect objects, map terrain, or track a target. But a *swarm* of camera-equipped drones, coordinating intelligently, can cover vastly larger areas, maintain persistent surveillance from multiple angles, and adapt to changing conditions in ways a single platform never could. The research challenge lies in making this coordination work: how do drones decide where to look, how to share what they see, and how to collectively act on that information?

Three pillars hold up this field:

**Computer Vision (CV)** gives each drone eyes. Onboard cameras feed into detection models (typically YOLO-family architectures for real-time performance on edge hardware) to identify objects of interest, classify terrain types, or extract features for mapping. The key constraint is computational: drones carry small, low-power processors, so inference must be fast and lightweight.

**Artificial Intelligence (AI)** gives the swarm a brain. This spans classical path planning algorithms (A*, RRT, Voronoi tessellation), bio-inspired coordination (flocking, ant colony optimization, particle swarm optimization), and modern learning approaches — particularly Multi-Agent Reinforcement Learning (MARL), which lets drones learn cooperative strategies through trial and error in simulation.

**Swarm Robotics** provides the coordination substrate. Drawing from decades of multi-agent systems research, swarm robotics defines how drones communicate, form consensus, allocate tasks, maintain formations, and degrade gracefully when individuals fail.

---

## 2. Core Concepts You Must Understand

### 2.1 Swarm Architectures

**Centralized:** A ground station or leader drone computes all decisions and broadcasts commands. Simple to implement, fragile to single-point failure, doesn't scale well.

**Decentralized:** Each drone makes its own decisions based on local information and neighbor communication. Robust and scalable, but harder to guarantee global optimality.

**CTDE (Centralized Training, Decentralized Execution):** The dominant paradigm in MARL research. During training (in simulation), a centralized critic has access to all agents' states to guide learning. During execution, each agent acts on its own local observation. This gives you the best of both worlds: globally informed learning, locally executable policies.

### 2.2 Key CV Models for UAVs

The VisDrone dataset is the standard benchmark for object detection from drone perspectives. Models used in UAV research include:

- **YOLOv8 / YOLOv11** — Currently the go-to for real-time detection. Runs at 30+ FPS on embedded GPUs.
- **SSD (Single Shot Detector)** — Slightly less accurate, very fast.
- **Faster R-CNN** — More accurate but slower; used when real-time isn't critical.
- **ORB-SLAM3** — Visual SLAM for simultaneous localization and mapping from monocular/stereo cameras.

### 2.3 MARL Algorithms Worth Knowing

| Algorithm | Type | Key Property |
|-----------|------|-------------|
| **MAPPO** | Policy Gradient | Multi-agent PPO; works surprisingly well out of the box |
| **QMIX** | Value Decomposition | Learns a mixing network that combines per-agent Q-values |
| **MADDPG** | Actor-Critic | Continuous action spaces; each agent has its own critic |
| **VDN** | Value Decomposition | Simpler than QMIX; sums individual Q-values |
| **IQL** | Independent Learners | Each agent learns independently; surprisingly competitive baseline |

### 2.4 Simulation Stack at a Glance

The standard research simulation stack in 2025/2026 looks like this:

```
┌─────────────────────────────────────────────┐
│              Your Algorithm (Python)         │
│   (CV pipeline, coordination logic, RL)      │
├─────────────────────────────────────────────┤
│         ROS 2 Humble (middleware)            │
│   Topics, services, actions, tf2 transforms  │
├─────────────────────────────────────────────┤
│    PX4-Autopilot SITL (flight controller)    │
│  Realistic flight dynamics, sensor models    │
├─────────────────────────────────────────────┤
│    Gazebo (physics + rendering engine)       │
│  3D world, cameras, LiDAR, collision physics │
└─────────────────────────────────────────────┘
```

PX4 SITL runs the actual flight controller firmware on your laptop instead of on a drone's microcontroller. It communicates with Gazebo for physics and with ROS 2 for your code. This means algorithms tested here can theoretically be deployed on real PX4-based drones with minimal changes.

For MARL training, you typically don't use Gazebo (too slow for millions of training steps). Instead, you build a lightweight custom environment in PettingZoo or a custom Gym wrapper, train your policies there, and then *validate* the learned policies in the full Gazebo + PX4 stack.

---

## 3. What the Literature Says Right Now

### 3.1 Swarm Coordination

UAV swarm research has matured significantly. Recent comprehensive surveys from Springer and MDPI (2025) cover coordinated path planning, task assignment, formation control, and security. The field is moving from centralized, handcrafted coordination to learned, decentralized policies. Bio-inspired approaches (flocking, PSO) remain popular for their simplicity, but MARL methods are rapidly gaining traction because they can handle complex, dynamic environments.

### 3.2 CV on UAV Platforms

The YOLO family dominates drone-based detection. Recent work applying YOLOv8 and YOLOv10 to real field conditions (agricultural drones in orchards, for instance) reports 72–75% accuracy under operational conditions — notably lower than lab benchmarks, highlighting the sim-to-real gap. Meanwhile, a 2025 paper in *Communications Engineering* demonstrated a drone swarm using anomaly detection on synthetic aperture images to track objects under forest canopy, achieving 93% precision and 96% recall — a compelling demonstration of multi-drone CV fusion.

### 3.3 MARL for UAVs

A 2025 survey in MDPI *Drones* specifically covers MARL for UAV control, categorizing approaches into policy-based (MAPPO, MATD3) and value-based (QMIX, VDN) methods. MAPPO has emerged as the most versatile algorithm for UAV swarms, with applications spanning target tracking, surveillance, collaborative exploration, and search-and-rescue. The CTDE paradigm is the default training approach. Key remaining challenges: sample efficiency, reward shaping for cooperation, and scaling beyond 10 agents.

### 3.4 Simulation Environments

The simulation landscape has consolidated around ROS 2 + Gazebo + PX4 as the standard for realistic drone simulation. The PX4 Swarm Controller project provides a ready-made ROS 2 package for multi-drone simulation with leader-follower control. Aerostack2 and Crazyswarm2 offer higher-level swarm management. For lightweight MARL training, custom environments built on PettingZoo are the norm.

AirSim (Microsoft) was archived in 2022 but remains usable, especially for high-fidelity rendering — its Unreal Engine integration produces photorealistic imagery that Gazebo cannot match. Its successor, Colosseum, is community-maintained. For projects where visual realism matters (training CV models on synthetic data), AirSim is still worth considering.

---

## 4. Gaps and Opportunities

Based on the current literature, here are concrete research gaps you can target:

1. **CV-in-the-loop coordination:** Most swarm coordination papers use oracle state information (true positions of targets). Very few close the loop between actual camera-based detection and swarm behavior adaptation. This is a wide-open, practical research direction.

2. **Scalability beyond toy scenarios:** The majority of MARL papers test on 3–6 agents. Demonstrating effective coordination at 10–20+ agents, even in simulation, would be a meaningful contribution.

3. **Energy-aware task allocation with CV quality trade-offs:** Current energy-aware work optimizes flight time. But detection quality degrades with altitude, speed, and angle. Jointly optimizing for detection quality *and* energy is unexplored.

4. **Feature-level fusion across swarm members:** Almost all multi-drone CV work either shares raw images (expensive) or shares only final detections (lossy). Sharing intermediate CNN features is a promising middle ground that has barely been explored in the aerial domain.

5. **Heterogeneous swarms with diverse sensors:** Real swarms may mix RGB, thermal, and multispectral cameras. How to fuse detections from different modalities across different drones is an open challenge.

---

## 5. Fastest Path to a Running Demo

If you want something working on screen within a weekend, here is the fastest route:

**Day 1 (3 hours):**
1. Install ROS 2 Humble following the official one-liner: `sudo apt install ros-humble-desktop`
2. Install PX4-Autopilot: `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`
3. Run `make px4_sitl gz_x500` — you should see a drone in Gazebo
4. In another terminal, install MAVROS2 and verify you can read drone state

**Day 2 (3 hours):**
1. Spawn multiple drones using PX4's multi-vehicle support
2. Write a Python script using MAVSDK that makes 3 drones take off and fly to different waypoints
3. Attach a camera sensor to each drone in the Gazebo model
4. Verify you can subscribe to camera image topics in ROS 2

**Day 3 (3 hours):**
1. Run YOLOv8 inference on the ROS 2 camera topic stream
2. Spawn some target objects in the Gazebo world
3. Detect them from one drone's camera → print detection results
4. Celebrate — you now have multi-drone CV working in simulation

From this baseline, you can build any of the six paper topics described in the main guideline.

---

## 6. Essential References

### Surveys & Overviews

1. Chung, S.J., Paranjape, A.A., Dames, P., Shen, S., & Kumar, V. (2018). "A Survey on Aerial Swarm Robotics." *IEEE Transactions on Robotics*, 34(4), 837–855. https://doi.org/10.1109/TRO.2018.2857475

2. Sai, S., Garg, A., Jhawar, K., Chamola, V., & Sikdar, B. (2023). "A Comprehensive Survey on Artificial Intelligence for Unmanned Aerial Vehicles." *IEEE Open Journal of Vehicular Technology*, 4, 713–738.

3. "UAV Swarms: Research, Challenges, and Future Directions." (2025). *Journal of Engineering and Applied Science*, Springer. https://doi.org/10.1186/s44147-025-00582-3

4. "A Survey on UxV Swarms and the Role of Artificial Intelligence as a Technological Enabler." (2025). *Drones*, 9(10), 700. https://doi.org/10.3390/drones9100700

5. "A Survey on UAV Control with Multi-Agent Reinforcement Learning." (2025). *Drones*, 9(7), 484. https://doi.org/10.3390/drones9070484

### Key Technical Papers

6. Nathan, R.J.A.A., et al. (2025). "An Autonomous Drone Swarm for Detecting and Tracking Anomalies among Dense Vegetation." *Communications Engineering*. https://doi.org/10.1038/s44172-025-00546-8

7. "Fault-Tolerant Multi-Agent RL Framework for UAV-UGV Coverage Path Planning." (2024). *Drones*, 8(10), 537. https://doi.org/10.3390/drones8100537

8. "MARLander: A Local Path Planning for Drone Swarms using Multiagent Deep Reinforcement Learning." (2024). arXiv:2406.04159

9. "ROS-Based Multi-Domain Swarm Framework for Fast Prototyping." (2025). *Aerospace*, 12(8), 702. https://doi.org/10.3390/aerospace12080702

10. "AI-based Autonomous UAV Swarm System for Weed Detection and Treatment." (2024). *Journal of Industrial Information Integration*. https://doi.org/10.1016/j.jii.2024.100690

### Simulation & Tools

11. Shah, S., Dey, D., Lovett, C., & Kapoor, A. (2018). "AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles." *Field and Service Robotics*. GitHub: https://github.com/microsoft/AirSim

12. PX4-Autopilot. https://github.com/PX4/PX4-Autopilot

13. Aerostack2. https://github.com/aerostack2/aerostack2

14. Crazyswarm2. https://github.com/IMRCLab/crazyswarm2

15. PX4 Swarm Controller (ROS 2). https://github.com/artastier/PX4_Swarm_Controller

### MARL Foundations

16. Yu, C., et al. (2022). "The Surprising Effectiveness of PPO in Cooperative Multi-Agent Games." *NeurIPS 2022*.

17. Rashid, T., et al. (2018). "QMIX: Monotonic Value Function Factorisation for Deep Multi-Agent Reinforcement Learning." *ICML 2018*.

18. Lowe, R., et al. (2017). "Multi-Agent Actor-Critic for Mixed Cooperative-Competitive Environments." *NeurIPS 2017*.

### Datasets

19. VisDrone Dataset — Standard benchmark for object detection from drone imagery. https://github.com/VisDrone/VisDrone-Dataset

20. UAV123 — UAV tracking benchmark. https://cemse.kaust.edu.sa/ivul/uav123

---

## 7. Glossary

| Term | Meaning |
|------|---------|
| **SITL** | Software In The Loop — running flight controller firmware on a PC instead of a real drone |
| **HITL** | Hardware In The Loop — connecting a real flight controller to a simulated environment |
| **CTDE** | Centralized Training, Decentralized Execution — the dominant MARL paradigm |
| **MAPPO** | Multi-Agent Proximal Policy Optimization — currently the most popular MARL algorithm |
| **PX4** | Open-source flight controller firmware, the most widely used in research |
| **MAVLink** | Lightweight messaging protocol for drone communication |
| **MAVROS** | ROS package that bridges MAVLink and ROS topics |
| **MAVSDK** | High-level SDK for drone control, simpler than MAVROS |
| **Voronoi Tessellation** | Partitioning space into regions based on distance to a set of points; used for coverage optimization |
| **mAP** | Mean Average Precision — standard metric for object detection performance |
| **FOV** | Field of View — the angular extent visible through a camera |
| **PSO** | Particle Swarm Optimization — bio-inspired optimization used in swarm coordination |
| **RRT** | Rapidly-exploring Random Tree — path planning algorithm |
| **Sim-to-real gap** | The performance drop when transferring a policy trained in simulation to a real robot |

---

*This document is a companion to the full Project Guideline. Start here, then follow the phased plan.*
