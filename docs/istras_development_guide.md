# ISTRAS'26 — 10-Day Development Guide - Energy-Efficient Adaptive UAV Swarm Coverage for Sustainable Aerial Monitoring of Transport Corridors

> **⚠ HISTORICAL — superseded by `istras_development_guide_v2.md`,
> then by the energy-scaling pivot (2026-04-25).** The active paper
> is now lawnmower N-scaling. Read **`istras_energy_scaling_guide.md`**
> first; this doc is retained only for the original adaptive-vs-baseline
> framing.

> **Paper:** Energy-Efficient Adaptive UAV Swarm Coverage for Sustainable Aerial Monitoring of Transport Corridors
> **Start:** 20 April 2026 (Monday)
> **Deadline:** 1 May 2026 (Thursday)
> **Active days:** 8 · **Buffer days:** 2
> **Daily budget:** 2–3 hours

---

## At a Glance

| # | Date | Phase | Deliverable |
|---|---|---|---|
| 1 | Mon 20 Apr | Fix detection pipeline | Working HSV detection on 3 target classes |
| 2 | Tue 21 Apr | Integration + minimal tuning | End-to-end demo recorded |
| 3 | Wed 22 Apr | Run experiments | Raw CSV logs for 9 trials |
| 4 | Thu 23 Apr | Analysis + figures | 1 table + 2 charts generated |
| 5 | Fri 24 Apr | Draft: Intro + Related Work | ~1.25 pages written |
| 6 | Sat 25 Apr | Draft: System + Methodology | ~1.5 pages written |
| 7 | Sun 26 Apr | Draft: Experiments + Results | ~1 page written |
| 8 | Mon 27 Apr | Draft: Conclusion + Abstract + References | Full draft complete |
| 9 | Tue 28 Apr | **BUFFER** — polish, format, proofread | Submission-ready paper |
| 10 | Wed 29 Apr | **BUFFER** — Springer forms, plagiarism check, submit | Submitted |

Days 9–10 are genuine buffer. If anything slips on Days 1–8, absorb into buffer. If Days 1–8 go smoothly, use buffer for a second proofread pass or for catching up on MDPI work.

---

## Solving the Detection Problem (Day 1)

### Why YOLOv8 failed

YOLOv8 is trained on the COCO dataset: 80 classes of real-world photographs with realistic textures, shadows, and occlusions. Your Gazebo primitives (boxes, spheres with flat colors) have none of that visual richness, so the learned feature distributions don't overlap. The "kite" hits are the model grabbing its nearest COCO class for small diamond-shaped objects against the sky — another drone seen by a drone camera.

### The fix: HSV color-based detection

Replace the YOLOv8 call with OpenCV HSV thresholding. Your cartoonish colored Gazebo objects become perfect detection targets because colors are deterministic in simulation. This takes about one afternoon to implement and delivers 100% reliable detection.

### Replacement for `scripts/detection_publisher.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from swarm_msgs.msg import Detection
from cv_bridge import CvBridge
import cv2
import numpy as np

# HSV ranges: (lower, upper) per class
# Tune these after visually inspecting Gazebo camera feeds
COLOR_CLASSES = {
    'person':  {'hsv_lower': (0,   150, 100), 'hsv_upper': (10,  255, 255), 'priority': 0.9},
    'vehicle': {'hsv_lower': (100, 150, 100), 'hsv_upper': (140, 255, 255), 'priority': 0.6},
    'fire':    {'hsv_lower': (10,  150, 150), 'hsv_upper': (25,  255, 255), 'priority': 1.0},
}
MIN_AREA_PX = 150  # reject blobs below this pixel area

class ColorDetectionPublisher(Node):
    def __init__(self):
        super().__init__('color_detection_publisher')
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('image_topic', '/drone1/camera/image')
        self.drone_id = self.get_parameter('drone_id').value
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_cb, 10)
        self.pub = self.create_publisher(
            Detection, f'/drone{self.drone_id}/detection', 10)

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for class_name, spec in COLOR_CLASSES.items():
            mask = cv2.inRange(hsv,
                               np.array(spec['hsv_lower']),
                               np.array(spec['hsv_upper']))
            # Clean up noise
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_AREA_PX:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                confidence = min(1.0, area / 2000.0)  # area-normalized

                det = Detection()
                det.drone_id = self.drone_id
                det.class_name = class_name
                det.confidence = float(confidence)
                det.bbox = [float(x), float(y), float(w), float(h)]
                det.stamp = self.get_clock().now().to_msg()
                # world_position: left for coordinator to compute from drone pose + bbox center
                self.pub.publish(det)

def main():
    rclpy.init()
    rclpy.spin(ColorDetectionPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Gazebo target objects

Make sure your `spawn_targets.sh` spawns boxes with saturated colors. Example SDF snippet:

```xml
<model name="person_target">
  <static>true</static>
  <link name="link">
    <visual name="visual">
      <geometry><box><size>1.0 1.0 1.8</size></box></geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
    <collision name="collision">
      <geometry><box><size>1.0 1.0 1.8</size></box></geometry>
    </collision>
  </link>
</model>
```

Use **pure red `(1,0,0)` for person, pure blue `(0,0,1)` for vehicle, pure orange `(1,0.5,0)` for fire**. Saturated colors give much wider HSV tolerance windows and make the detector robust to lighting variation.

### Calibrating HSV ranges

If the default ranges don't catch your colors, record a single camera frame, open it in any image viewer, and hover over the colored object — read the HSV values, then widen the ranges by ±15 on hue and ±50 on saturation/value. Alternatively, run this quick interactive calibrator:

```bash
python3 -c "
import cv2
img = cv2.imread('/path/to/frame.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
print('HSV range in image: min', hsv.reshape(-1,3).min(0), 'max', hsv.reshape(-1,3).max(0))
"
```

### Paper framing for this choice

Write the Methodology section like this: *"We employ HSV-based color segmentation as the perception module, which is appropriate for controlled simulation environments with known target palettes. In real-world deployment, this module is replaced with a CNN detector such as YOLOv8 fine-tuned on domain-specific imagery, with the rest of the coordination pipeline unchanged."* This is honest and isolates your contribution (coordination, not perception).

---

## Day 1 (Mon 20 Apr) — Fix Detection

**Budget:** 3 hours
**Goal:** Detection pipeline publishes valid `Detection` messages for all three color classes

- Back up current `yolo_detector.py` / `detection_publisher.py` to `scripts/_archive/`
- Replace with the HSV version above
- Update `spawn_targets.sh` to use pure-color materials if not already
- Launch a single drone + one colored box target in Gazebo
- Verify with `ros2 topic echo /drone1/detection` — should see messages when the drone flies over the target

**Definition of done:** You see one `Detection` message per target per camera frame, with correct `class_name`.

---

## Day 2 (Tue 21 Apr) — Integration + Demo Recording

**Budget:** 3 hours
**Goal:** Full 3-drone adaptive pipeline end-to-end with detection → Voronoi update → waypoint publish → drone movement

- Launch 3-drone world with `launch_multi_drone.sh 3`
- Spawn 3 colored targets at fixed positions
- Start all ROS 2 nodes: detection publishers × 3, waypoint executors × 3, voronoi coordinator, data logger
- Watch RViz2 or Gazebo: drones should spread out via Voronoi at start, then converge when a target is detected
- Record a 2-minute Gazebo screen capture for the paper
- Fix any crashes or topic-mismatch bugs that surface

**Definition of done:** A clean 2-minute video showing spread → detect → converge → disperse behavior. CSV files are being written by `data_logger.py`.

---

## Day 3 (Wed 22 Apr) — Run Experiments

**Budget:** 3 hours (includes ~30 minutes of actual simulation time)
**Goal:** 9 completed trials (3 methods × 3 trials) with CSV logs

### The minimal experiment set

| Parameter | Value |
|---|---|
| Drones | 3 |
| Area | 40 m × 40 m monitoring zone |
| Altitude | 10 m (constant, downward-facing camera) |
| Targets per trial | 3 (1 person, 1 vehicle, 1 fire) |
| Target placement | Random seed, different per trial |
| Trial duration | 120 s |
| Methods | adaptive_voronoi, lawnmower, static_grid |
| Trials per method | 3 |
| **Total simulation time** | **~18 minutes wall-clock** |

This is genuinely minimal. If you want more robust statistics for the paper's results section, expand each method to 5 trials (adds ~12 min more, still fits in the day).

### Execution

```bash
# Run sequentially - automate with a shell loop
for method in adaptive_voronoi lawnmower static_grid; do
  for trial in 1 2 3; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 3 \
      --targets 3 \
      --duration 120 \
      --trial $trial \
      --seed $trial \
      --output-dir data/logs/istras/
    sleep 5   # let processes settle between trials
  done
done
```

### What to log per trial

Your `data_logger.py` should produce one CSV per drone per trial:

- `drone{N}_position.csv`: timestamp, x, y, z
- `drone{N}_detection.csv`: timestamp, class_name, confidence, target_id
- `swarm_mode.csv`: timestamp, mode (EXPLORE/EXPLOIT), exploit_drone_ids

Plus one `trial_meta.json` per trial capturing: method name, seed, target positions, drone starting positions.

**Definition of done:** 9 trial folders under `data/logs/istras/`, each with position CSVs for all 3 drones, detection log, and metadata.

---

## Day 4 (Thu 23 Apr) — Analysis & Figures

**Budget:** 3 hours
**Goal:** Table and 2 charts ready for paper

### The 5 metrics you need

| Metric | How to compute |
|---|---|
| **Total flight distance** (m) | Sum of ‖pos(t+1) − pos(t)‖ over all timesteps, summed across all 3 drones |
| **Mission energy** (Wh) | `P_hover × mission_time_s / 3600`, where `P_hover ≈ 150 W` for a small quadrotor. Use this **per drone**, then sum. |
| **CO₂ equivalent** (g) | `energy_kWh × grid_factor`, where `grid_factor_Azerbaijan = 440 g/kWh`, `grid_factor_EU_avg = 250 g/kWh`. Report both. |
| **Detection coverage** (%) | `(targets_detected / targets_spawned) × 100`, averaged per-trial |
| **Detection latency** (s) | For each detected target: `time_of_first_detection − trial_start_time`. Report mean across targets and trials. |

### Energy reference rate — citation

When you use `P_hover = 150 W`, cite a published source. The commonly cited figure is from the DJI M100/M210 class papers — a useful anchor is approximately 150 W for a 1.5 kg quadrotor in hover. A good specific citation is Abeywickrama et al. 2018 ("Comprehensive Energy Consumption Model for Unmanned Aerial Vehicles, Based on Empirical Studies of Battery Performance," IEEE Access), which reports 142 W for a DJI Matrice 100 at hover. Use 150 W as a round figure and cite this paper.

### Grid emission factor — citation

Azerbaijan grid factor varies across sources. Use the IEA Azerbaijan country profile figure of roughly 0.44 kg CO₂/kWh for 2022 as a defensible anchor. For European average, cite EEA (~0.25 kg CO₂/kWh for 2022).

### Figures to produce

1. **Figure 1 — System architecture diagram** (one-time draw in draw.io or similar)
   - ROS 2 node graph: detection publishers → coordinator → waypoint executors
   - Shows the open-source stack (PX4 + Gazebo + ROS 2 + MAVSDK + OpenCV)

2. **Figure 2 — Flight distance comparison** (bar chart)
   - X-axis: method (adaptive / lawnmower / static_grid)
   - Y-axis: total flight distance (m), mean ± std
   - Label with percentage reduction vs. lawnmower baseline

3. **Figure 3 — Detection latency comparison** (bar chart)
   - X-axis: method
   - Y-axis: mean time to first detection (s), mean ± std

4. **Table 1 — All metrics summary**

| Method | Distance (m) | Energy (Wh) | CO₂ AZ (g) | CO₂ EU (g) | Coverage (%) | Latency (s) |
|---|---|---|---|---|---|---|
| Static grid | ... ± ... | ... | ... | ... | ... | ... |
| Lawnmower | ... ± ... | ... | ... | ... | ... | ... |
| Adaptive (ours) | ... ± ... | ... | ... | ... | ... | ... |

### Expected result pattern (rough forecast)

- **Static grid** will have the lowest flight distance (drones barely move) but the worst detection coverage (targets outside cells never seen)
- **Lawnmower** will have the highest flight distance (drones sweep everywhere) with good eventual coverage but high detection latency
- **Adaptive** should have moderate flight distance but lowest detection latency because it concentrates on detected regions

If adaptive does not beat lawnmower on distance, that's still a publishable result — frame it as *"Our adaptive method reduces detection latency by X% while using comparable energy to the lawnmower baseline, offering a favorable coverage-responsiveness trade-off."* Don't force a story the data doesn't support.

**Definition of done:** `figures/` directory with 4 PDFs (one architecture diagram + 2 bar charts + 1 results table as LaTeX snippet), ready to drop into the paper.

---

## Days 5–8 — Writing

Follow this section split. Target 3–5 pages total (ISTRAS limit).

### Day 5 (Fri 24 Apr) — Introduction + Related Work (~1.25 pages)

**Introduction (3 paragraphs, ~0.75 page):**
1. Motivation: ITS increasingly uses UAVs for corridor monitoring; operational energy footprint of aerial monitoring is non-negligible.
2. Problem: existing coverage strategies (lawnmower, grid) are energy-inefficient because they do not adapt to where events actually occur.
3. Contribution: (i) decentralized adaptive Voronoi coverage driven by onboard detection, (ii) open-source PX4 + Gazebo + ROS 2 implementation, (iii) quantitative evaluation showing energy savings versus conventional baselines.

**Related Work (~0.5 page):** Short, 8–10 references. Group into three: (a) multi-UAV coverage (Cortés et al., CVT flood 2025); (b) UAV-based ITS monitoring (IST25-37 airport safety, urban drone management); (c) energy-aware UAV operations (Abeywickrama et al. energy model, relevant sustainability-focused UAV papers).

### Day 6 (Sat 25 Apr) — System Architecture + Methodology (~1.5 pages)

**System Architecture (~0.5 page):** One paragraph of prose + Figure 1. Describe the node graph and messages. Emphasize open-source + modularity.

**Methodology (~1 page):**
- Voronoi coverage and Lloyd's algorithm (one equation)
- Confidence-weighted density function (one equation)
- Event-triggered mode switching (brief pseudocode or bullet list)
- Baseline methods (lawnmower, static grid) — 2 sentences each
- Energy model: `E = P_hover × t + ε(distance)` — one sentence, one citation

### Day 7 (Sun 26 Apr) — Experiments + Results (~1 page)

**Experimental Setup (~0.25 page):** Simulation stack, parameters table (drones=3, area=40×40, trial=120s, trials=3), target classes.

**Results (~0.75 page):**
- Present Table 1 with the summary
- Present Figure 2 (distance) with 1–2 sentences of analysis
- Present Figure 3 (latency) with 1–2 sentences of analysis
- One paragraph of discussion covering: energy savings at comparable coverage; trade-off with static grid (which saves energy but loses coverage); computational overhead of Voronoi computation is negligible compared to flight energy.

### Day 8 (Mon 27 Apr) — Conclusion + Abstract + References

**Conclusion (~0.25 page):** One paragraph summarizing contributions and numerical headline result. One paragraph on future work: real-world deployment, larger swarms, integration with actual CNN detectors.

**Abstract:** Use the one already drafted in `ISTRAS26_Abstract_A.docx`. Update numbers to match your actual results.

**References:** APA 7th style as ISTRAS requires. 12–18 entries.

---

## Days 9–10 — Buffer

### Day 9 (Tue 28 Apr) — Polish

- Full read-through, aloud if possible
- Check every figure is referenced in text
- Check every reference in text appears in bibliography
- Verify font is Book Antiqua as template requires; sizes per ISTRAS template (13pt title, 12pt H1, 11pt H2, 10pt body)
- Anonymize for double-blind review: remove author names, affiliations, acknowledgements

### Day 10 (Wed 29 Apr) — Submission

- Fill out 4 Springer paperwork forms (License, Publishing Agreement, Copyright Declaration, Co-Author Consent)
- Run plagiarism self-check. Watch specifically for overlap with your MDPI draft and with `swarm_drones_report.md` — if they're high, rewrite.
- Upload to https://submission-system.istras.org/ with all supporting PDFs
- Confirmation email screenshot for your records

Submit before midnight on 29 April. This leaves 1 May as absolute-worst-case backup.

---

## What Not to Do in 10 Days

To stay on schedule, explicitly **skip** these items — they are MDPI paper material, not ISTRAS short paper material:

- ❌ The full ablation study (binary_voronoi, all_converge) — save for MDPI
- ❌ The 5-drone scalability experiment — save for MDPI
- ❌ The multi-class formation analysis — mention in 1 sentence, save detail for MDPI
- ❌ Fine-tuning YOLOv8 — not needed; HSV works
- ❌ Porting to real hardware — 0% in scope

---

## Risk Register

| Risk | Likelihood | Mitigation |
|---|---|---|
| HSV detection doesn't reliably catch objects | Low | Use pure-saturated colors (1,0,0 etc); widen hue window to ±15 |
| Adaptive doesn't beat lawnmower on distance | Medium | Reframe around detection latency; still a positive result |
| Gazebo crashes at 3-drone full pipeline | Low-medium | Lower camera resolution to 320×240; process every 5th frame |
| Experiment runs slip past Day 3 | Medium | Use buffer days; worst case 3 trials per method is still publishable |
| Plagiarism overlap with MDPI draft | Medium | Rewrite shared passages on Day 9; don't copy-paste between papers |
| Springer forms take longer than expected | Medium | Start them Day 8 evening, not Day 10 |

---

## Daily Checklist Template

At the end of each day, confirm:

- [ ] Today's deliverable complete?
- [ ] Code committed to git with a dated message?
- [ ] Any issue carried over to tomorrow? (write it down)
- [ ] On schedule, or into buffer?

If you hit end of Day 8 and the draft is not complete, cut features aggressively on Day 9. The paper existing beats the paper being perfect.

---

*Guide prepared 19 April 2026. For the full repo setup and code structure, see `README.md`.*
