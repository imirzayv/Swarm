# Research Project Guideline: Computer Vision & AI for UAV Swarm Systems

> **Estimated Total Duration:** 16–20 weeks (at 2–3 hours/day)
> **Author Role:** AI Engineer — Computer Vision & UAV Platforms
> **Deliverable:** Research paper with simulation-backed experiments

---

## Phase 0 — Project Scoping & Environment Setup (Week 1–2)

**Goal:** Decide your paper topic, set up your machine, and get a baseline simulation running.

### 0.1 Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 8-core (Ryzen 7 / i7) | 12–16 core (Ryzen 9 / i9) |
| RAM | 16 GB | 32 GB |
| GPU | NVIDIA GTX 1660 (6 GB VRAM) | RTX 3070+ (8 GB+ VRAM) |
| Storage | 256 GB SSD | 512 GB+ NVMe SSD |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

> **Why Ubuntu 22.04?** It is the best-supported platform for ROS 2 Humble, Gazebo, PX4 SITL, and most open-source drone toolchains. Dual-boot or a dedicated machine is strongly preferred over a VM — Gazebo rendering in VMs is notoriously slow and buggy.

### 0.2 Core Software Stack Installation

Install in this order — each layer depends on the one above it:

**Layer 1 — Operating System & Basics**
- Ubuntu 22.04 LTS
- Python 3.10+, pip, conda (Miniconda recommended)
- Git, curl, cmake, build-essential

**Layer 2 — ROS 2 & Simulation**
- ROS 2 Humble Hawksbill (LTS, supported until 2027)
- Gazebo Harmonic (next-gen) or Gazebo Classic 11
- RViz2 (3D visualization companion to ROS 2)

**Layer 3 — Flight Stack**
- PX4-Autopilot (SITL — Software In The Loop)
- MAVROS2 / px4_msgs (ROS 2 ↔ PX4 bridge)
- QGroundControl (ground station GUI)

**Layer 4 — AI/CV Libraries**
- PyTorch 2.x + CUDA toolkit
- OpenCV 4.x (with DNN module)
- Ultralytics YOLOv8/v11 (object detection)
- Stable-Baselines3 (reinforcement learning)
- PettingZoo or EPyMARL (multi-agent RL environments)

**Layer 5 — Swarm-Specific Frameworks (pick based on topic)**
- Aerostack2 — full autonomous aerial robotics framework on ROS 2
- Crazyswarm2 — swarm control for Crazyflie nano-drones (sim + real)
- PX4 Swarm Controller — leader-follower swarm on ROS 2 + Gazebo
- CrazyChoir — distributed optimization and formation control

**Layer 6 — Supporting Tools**
- MAVSDK-Python (high-level drone control API)
- Plotly / Matplotlib (experiment visualization)
- TensorBoard (training monitoring)
- Docker (optional, for reproducible environments)
- LaTeX / Overleaf (paper writing)

### 0.3 Milestone Checklist (end of Week 2)
- [ ] Ubuntu 22.04 running natively with GPU drivers
- [ ] ROS 2 Humble installed and `ros2 topic list` works
- [ ] PX4 SITL launches a single drone in Gazebo
- [ ] Can control the drone via MAVROS2 or MAVSDK-Python
- [ ] Spawned 3+ drones simultaneously in Gazebo
- [ ] Python environment with PyTorch + OpenCV + YOLOv8 ready

---

## Phase 1 — Literature Review (Week 2–5)

**Goal:** Understand the landscape, identify gaps, and finalize your contribution.

### 1.1 Key Questions to Answer

Answer each of these in 2–3 paragraphs during your reading. These will form the backbone of your Related Work section.

**Q1. What are the dominant architectures for UAV swarm coordination?**
- Centralized vs. decentralized vs. hybrid (CTDE — Centralized Training, Decentralized Execution)
- Leader-follower vs. consensus-based vs. bio-inspired (PSO, ACO, flocking)
- How does communication topology affect swarm performance?

**Q2. How is computer vision currently integrated into multi-drone systems?**
- Onboard vs. offboard processing trade-offs
- Object detection models used (YOLO variants, SSD, Faster R-CNN)
- Visual SLAM for multi-agent mapping (ORB-SLAM3, RTAB-Map)
- Synthetic aperture imaging from drone swarms

**Q3. What role does reinforcement learning play in swarm UAV operations?**
- Single-agent RL for individual drone tasks vs. MARL for coordination
- Key MARL algorithms: MAPPO, QMIX, MADDPG, VDN, IQL
- Sim-to-real transfer challenges
- Reward shaping for cooperative behaviors

**Q4. What are the open problems and gaps?**
- Scalability (most papers test 3–6 drones; what about 10–50?)
- Heterogeneous swarms (different sensor payloads, capabilities)
- Real-time CV inference under swarm communication bandwidth limits
- Joint optimization of coverage, detection quality, and energy

**Q5. What simulation environments are used in the literature?**
- Gazebo + ROS 2 + PX4 (most common for realistic physics)
- AirSim / Colosseum (high-fidelity rendering, archived but usable)
- Custom OpenAI Gym / PettingZoo environments (for MARL training)
- How are results validated? What metrics are standard?

### 1.2 Reading Strategy

Start with survey papers for breadth, then drill into specific topic papers:

**Tier 1 — Survey Papers (read fully, ~5 papers)**
- Chung et al., "A Survey on Aerial Swarm Robotics," IEEE Trans. Robotics, 2018
- Sai et al., "A Comprehensive Survey on AI for UAVs," IEEE Open J. Vehicular Tech., 2023
- The ACM Computing Surveys paper on "Future UAV/Drone Systems for Intelligent Active Surveillance" (2024)
- MDPI Drones: "A Survey on UxV Swarms and the Role of AI" (2025)
- MDPI Drones: "A Survey on UAV Control with Multi-Agent Reinforcement Learning" (2025)

**Tier 2 — Topic-Specific Papers (read selectively, ~15–20 papers)**
- Target tracking with drone swarms (PSO-based, RL-based)
- Coverage path planning with MARL (QMIX, MAPPO applications)
- YOLO-based detection from UAV platforms (VisDrone dataset papers)
- Formation control algorithms (weighted topology, virtual structure)
- Cooperative mapping and exploration

**Tier 3 — Simulation & Tooling Papers (~5 papers)**
- PX4 SITL documentation and architecture papers
- Aerostack2 framework paper
- Crazyswarm2 / CrazySim ICRA 2024 paper
- ROS 2-based UAV security simulation framework (2024)

### 1.3 Literature Review Deliverables
- [ ] Annotated bibliography (30–40 papers) in a spreadsheet or Zotero
- [ ] 2-page gap analysis identifying what is missing in the literature
- [ ] Clear statement of your paper's contribution (1 paragraph)

---

## Phase 2 — Topic Selection (Week 4–5, overlapping with Phase 1)

**Goal:** Pick one focused, simulation-provable topic for your paper.

Below are **six candidate topics** ranked by feasibility and novelty. Each is designed to be achievable in simulation with open-source tools.

---

### Topic A: Vision-Guided Adaptive Area Monitoring with Drone Swarms

**Core idea:** A swarm of drones cooperatively monitors a large area, dynamically reallocating coverage based on real-time object detections from onboard cameras. When one drone detects an event of interest (e.g., a person, vehicle, fire), nearby drones converge to form a tighter surveillance cluster while the rest maintain baseline coverage.

**Why it's novel:** Most coverage optimization papers treat drones as point sensors. This integrates actual CV detections into the coverage replanning loop.

**Simulation approach:**
- Gazebo world with spawned "target" objects at random locations
- Each drone runs YOLOv8 on its simulated camera feed
- A decentralized Voronoi-based coverage algorithm adjusts drone positions
- Detection events trigger local swarm reconfigurations
- Metrics: coverage percentage, detection latency, redundancy ratio, energy

**Difficulty:** ★★★☆☆ — Moderate. Well-defined problem, clear metrics.

---

### Topic B: MARL-Based Cooperative Target Search and Tracking

**Core idea:** Train a swarm of drones using Multi-Agent Reinforcement Learning to cooperatively search for and track moving targets in an unknown environment. Each drone has a limited field-of-view camera and must learn to coordinate without explicit communication protocols.

**Why it's novel:** Combines MARL (MAPPO or QMIX) with vision-based observations rather than oracle position data. Tests whether agents can learn implicit coordination from visual cues alone.

**Simulation approach:**
- Custom PettingZoo environment wrapping a simplified 2D/3D grid world
- Targets move with varying patterns (random walk, evasive, patrol)
- Observation space: local camera-like FOV + relative neighbor positions
- Train with MAPPO using Stable-Baselines3 or EPyMARL
- Validate trained policies in Gazebo with PX4 drones for visual fidelity
- Metrics: time-to-first-detection, tracking continuity, coverage efficiency

**Difficulty:** ★★★★☆ — Challenging. MARL training can be finicky.

---

### Topic C: Federated Object Detection Across a Drone Swarm

**Core idea:** Each drone in the swarm trains a local object detection model on images it captures during flight. Periodically, drones aggregate their model weights using federated learning, improving global detection performance without sharing raw imagery (preserving bandwidth and privacy).

**Why it's novel:** Federated learning on UAV swarms is emerging but very few papers combine it with actual CV model training in simulation.

**Simulation approach:**
- Multiple Gazebo drones flying over heterogeneous terrain (urban, rural, forest)
- Each drone captures frames and fine-tunes a lightweight YOLO model locally
- A simulated communication round aggregates weights (FedAvg)
- Compare: centralized training vs. federated vs. isolated models
- Metrics: mAP, communication cost (bytes transferred), convergence speed

**Difficulty:** ★★★★☆ — Challenging. Requires careful experiment design.

---

### Topic D: Swarm-Based Cooperative 3D Reconstruction

**Core idea:** A swarm of drones cooperatively reconstructs a 3D model of a structure (building, disaster site) by planning complementary viewpoints. A next-best-view planner assigns each drone a unique angle to maximize surface coverage and minimize occlusion.

**Why it's novel:** Single-drone 3D reconstruction is well-studied, but multi-drone viewpoint planning with collision avoidance and overlap minimization is less explored.

**Simulation approach:**
- Gazebo world with a target structure (building model)
- Drones capture images from their assigned positions
- Point cloud merging using Open3D or COLMAP
- Compare: random viewpoints vs. greedy next-best-view vs. swarm-optimized
- Metrics: surface coverage %, reconstruction error (Chamfer distance), mission time

**Difficulty:** ★★★☆☆ — Moderate. Heavy on geometry, lighter on RL.

---

### Topic E: Energy-Aware Swarm Coverage with Dynamic Task Handoff

**Core idea:** Drones in a swarm have finite battery. The system must maintain continuous area coverage by dynamically handing off surveillance tasks to freshly charged drones while low-battery ones return to a charging station. A learned policy optimizes the handoff schedule.

**Why it's novel:** Most swarm coverage papers ignore battery dynamics or treat them as a fixed constraint. This treats energy as a first-class variable in the optimization.

**Simulation approach:**
- Gazebo with a charging pad and patrol zones
- Battery modeled as a depleting resource tied to flight time
- RL agent (or heuristic) decides when each drone should return/deploy
- Compare: fixed schedule vs. threshold-based vs. RL-optimized
- Metrics: coverage gap duration, total area monitored, battery waste

**Difficulty:** ★★★☆☆ — Moderate. Clean problem with clear baselines.

---

### Topic F: Anomaly Detection in Agricultural Fields Using Drone Swarm with Shared Feature Maps

**Core idea:** A swarm of drones surveys an agricultural field. Instead of each drone running full inference independently, drones share compressed feature maps with neighbors to build a wider contextual understanding — enabling detection of large-scale anomalies (crop disease spreading, irrigation failures) that no single drone could see from its limited view.

**Why it's novel:** Feature-level fusion across multiple aerial agents, rather than raw image sharing or independent detection.

**Simulation approach:**
- Gazebo world with a textured ground plane representing a crop field
- Inject visual anomalies (color patches, missing crop rows)
- Each drone extracts intermediate CNN features and broadcasts to neighbors
- A fusion module merges multi-viewpoint features for classification
- Compare: independent detection vs. feature fusion vs. raw image sharing
- Metrics: detection accuracy, communication bandwidth, latency

**Difficulty:** ★★★★★ — Hard. Novel but requires solid deep learning chops.

---

### Topic Selection Criteria

| Criterion | Weight |
|-----------|--------|
| Feasibility in simulation (can I actually build this?) | 30% |
| Novelty (does it add something new to the literature?) | 25% |
| Publication potential (is this interesting to reviewers?) | 20% |
| Alignment with my skills and available time | 15% |
| Clear, measurable metrics for comparison | 10% |

**Recommendation:** If this is your first paper in this space, go with **Topic A** or **Topic E** — they are well-scoped, have clear baselines, and produce visually compelling results. If you want higher novelty and are comfortable with RL training, go with **Topic B**.

---

## Phase 3 — System Design & Prototyping (Week 5–8)

**Goal:** Design your system architecture, implement the core pipeline, and get a minimal working prototype.

### 3.1 System Architecture Design (Week 5)
- Draw a block diagram of your full system (drones, sensors, algorithms, communication)
- Define the ROS 2 node graph: which nodes exist, what topics they publish/subscribe
- Specify the observation space, action space, and reward function (if using RL)
- Define your baselines (what will you compare against?)

### 3.2 Simulation World Setup (Week 5–6)
- Build or download a Gazebo world that matches your scenario
- Configure multi-drone spawning (PX4 SITL multi-vehicle)
- Attach simulated cameras to each drone (Gazebo camera plugin)
- Verify camera feeds are accessible via ROS 2 topics
- Spawn target objects / anomalies in the world

### 3.3 Core Algorithm Implementation (Week 6–8)
- Implement CV pipeline: camera → preprocessing → detection/feature extraction
- Implement swarm coordination logic (coverage, assignment, formation)
- Implement communication protocol between drones (ROS 2 topics or simulated mesh)
- Integrate the pieces: detection triggers coordination updates
- Unit test each component independently before integration

### 3.4 Milestone Checklist (end of Week 8)
- [ ] Multi-drone Gazebo world running with cameras active
- [ ] CV model running on simulated camera feeds
- [ ] Swarm coordination algorithm moving drones based on detections
- [ ] Basic end-to-end demo works (even if performance is poor)

---

## Phase 4 — Experiments & Data Collection (Week 8–12)

**Goal:** Run systematic experiments, collect data, and produce publication-quality results.

### 4.1 Experiment Design
- Define independent variables (number of drones, area size, target density, algorithm variant)
- Define dependent variables / metrics (coverage %, detection rate, latency, energy consumed)
- Plan at least 3 comparison conditions (e.g., single drone vs. uncoordinated swarm vs. your method)
- Each experiment should run 10–30 trials for statistical significance

### 4.2 Experiment Execution
- Automate experiment launches with bash scripts or ROS 2 launch files
- Log all data: drone positions, timestamps, detections, battery levels, actions taken
- Use ROS 2 bag files for raw data recording
- Monitor GPU/CPU utilization to report computational cost

### 4.3 Metrics & Visualization
- Coverage efficiency: percentage of area monitored over time
- Detection performance: precision, recall, F1, mAP on simulated targets
- Coordination quality: redundancy ratio, inter-drone distance statistics
- Energy efficiency: mission time per unit of battery consumed
- Scalability: how metrics change as swarm size increases (3 → 5 → 10 drones)
- Communication cost: messages per second, bandwidth consumption

### 4.4 Visualization Tools
- Matplotlib / Seaborn for line plots, box plots, bar charts
- RViz2 for 3D trajectory visualization
- Gazebo screen recordings for qualitative demo videos
- TensorBoard for RL training curves (if applicable)

### 4.5 Milestone Checklist (end of Week 12)
- [ ] All experiments completed with sufficient trials
- [ ] Raw data organized and backed up
- [ ] Key result figures generated
- [ ] Statistical analysis done (mean, std, confidence intervals)

---

## Phase 5 — Paper Writing (Week 12–16)

**Goal:** Write a complete, publication-ready manuscript.

### 5.1 Paper Structure

1. **Abstract** (200–250 words) — Write last
2. **Introduction** (~1.5 pages) — Problem, motivation, contributions
3. **Related Work** (~2 pages) — From Phase 1 literature review
4. **System Architecture** (~1.5 pages) — From Phase 3 design
5. **Methodology** (~2–3 pages) — Algorithms, models, training procedures
6. **Experimental Setup** (~1 page) — Simulation environment, parameters, baselines
7. **Results & Discussion** (~2–3 pages) — Figures, tables, analysis
8. **Conclusion & Future Work** (~0.5 pages)
9. **References** (30–50 entries)

### 5.2 Writing Schedule
- Week 12–13: Sections 3, 4, 5 (technical core — write while experiments are fresh)
- Week 13–14: Sections 6, 7 (results and analysis)
- Week 14–15: Sections 1, 2 (intro and related work — easier to frame now)
- Week 15–16: Abstract, conclusion, references, formatting, proofreading

### 5.3 Target Venues

**Conferences:**
- IEEE/RSJ IROS (International Conference on Intelligent Robots and Systems)
- IEEE ICRA (International Conference on Robotics and Automation)
- IEEE/CVF CVPR Workshops (if CV is the primary contribution)
- AAMAS (Autonomous Agents and Multi-Agent Systems) — if MARL-focused

**Journals:**
- MDPI Drones (open access, fast review, good for first papers)
- IEEE Robotics and Automation Letters (RA-L)
- Elsevier Robotics and Autonomous Systems
- Springer Journal of Intelligent & Robotic Systems

---

## Phase 6 — Polish, Submit & Supplement (Week 16–18)

### 6.1 Final Checklist
- [ ] All figures are high-resolution (300+ DPI), vector where possible
- [ ] Reproducibility: document all hyperparameters, random seeds, software versions
- [ ] Prepare supplementary material: demo video, code repository
- [ ] Run a final round of proofreading (or use a tool like Grammarly + manual pass)
- [ ] Format to target venue template (IEEE, ACM, etc.)
- [ ] Submit

### 6.2 Bonus: Open-Source Your Work
- Clean up code and push to GitHub
- Write a clear README with installation and reproduction instructions
- This significantly increases citation potential and community impact

---

## Complete Software Reference Table

| Category | Tool | Purpose | Required? |
|----------|------|---------|-----------|
| **OS** | Ubuntu 22.04 LTS | Base operating system | Yes |
| **Middleware** | ROS 2 Humble | Inter-process communication, sensor integration | Yes |
| **Simulator** | Gazebo Harmonic / Classic 11 | Physics simulation, sensor simulation | Yes |
| **Visualization** | RViz2 | 3D visualization of ROS data | Yes |
| **Flight Stack** | PX4-Autopilot (SITL) | Realistic flight controller in simulation | Yes |
| **ROS-PX4 Bridge** | MAVROS2 / px4_msgs + MicroXRCE-DDS | Communication between ROS 2 and PX4 | Yes |
| **Ground Station** | QGroundControl | Mission planning, telemetry | Recommended |
| **Drone Control API** | MAVSDK-Python | High-level drone commands | Recommended |
| **Object Detection** | Ultralytics YOLOv8/v11 | Real-time object detection | Yes (for CV topics) |
| **Deep Learning** | PyTorch 2.x + CUDA | Model training and inference | Yes |
| **Computer Vision** | OpenCV 4.x | Image processing, transformations | Yes |
| **RL Framework** | Stable-Baselines3 | Single-agent RL training | For RL topics |
| **Multi-Agent RL** | EPyMARL / PettingZoo / MARLLIB | MARL algorithm implementations | For MARL topics |
| **3D Reconstruction** | Open3D / COLMAP | Point cloud processing | For Topic D |
| **Swarm Framework** | Aerostack2 / Crazyswarm2 | High-level swarm management | Recommended |
| **Data Logging** | ROS 2 bag | Recording experiment data | Yes |
| **Plotting** | Matplotlib / Seaborn / Plotly | Result visualization | Yes |
| **Training Monitor** | TensorBoard | RL training curves | Recommended |
| **Paper Writing** | LaTeX (Overleaf) | Manuscript preparation | Yes |
| **Reference Manager** | Zotero / Mendeley | Citation management | Recommended |
| **Version Control** | Git + GitHub | Code management | Yes |
| **Containerization** | Docker | Reproducible environments | Optional |

---

## Daily Time Allocation Guide (2–3 hours/day)

| Phase | Suggested daily split |
|-------|----------------------|
| Phase 0 (Setup) | 100% hands-on installation and testing |
| Phase 1 (Literature) | 80% reading + 20% note-taking |
| Phase 2 (Topic Selection) | 50% reading + 50% brainstorming and prototyping feasibility |
| Phase 3 (Prototyping) | 20% design + 80% coding |
| Phase 4 (Experiments) | 10% planning + 70% running experiments + 20% analysis |
| Phase 5 (Writing) | 90% writing + 10% polishing figures |
| Phase 6 (Polish) | 100% editing, formatting, submission logistics |

---

*Last updated: March 2026*
