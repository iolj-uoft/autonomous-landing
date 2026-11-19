# Project 1 — Small Tasks (Isaac Sim + Pegasus + PX4 + ROS 2)

Goal: a UAV that lands on a moving pad, built step by step.
Each task should take about **1–3 hours**. Tick it only when its **"Done when"** check is true.

## Working rules
- If a task runs past a day, split it into smaller tasks.
- When stuck, give it **30 minutes** with the docs, GitHub issues and error searches first. Then ask for help with the **exact error + what you tried**.
- After every task, add 2–3 lines to `NOTES.md`: what worked, what broke, how you fixed it. These become your README and interview stories later.

---

## Phase 0 — Setup

- [ ] **0.1** Check your GPU, driver and OS against Isaac Sim's current system requirements.
  - *Done when:* GPU, driver and OS are written in `NOTES.md` and confirmed as supported.
- [ ] **0.2** Install Isaac Sim and open a sample scene.
  - *Done when:* a sample scene runs at a usable frame rate.
- [ ] **0.3** Install the ROS 2 version that Isaac Sim's ROS 2 bridge supports.
  - *Done when:* `ros2 run demo_nodes_cpp talker` and `listener` talk to each other.
- [ ] **0.4** Enable the Isaac Sim ROS 2 bridge.
  - *Done when:* `ros2 topic list` shows topics from Isaac Sim (e.g. `/clock`).

---

## Phase 1 — Make the drone fly from your own code

- [ ] **1.1** Install Pegasus Simulator, using the version that matches your Isaac Sim (check its README).
  - *Done when:* Pegasus's PX4 example scene loads with a drone in it.
- [ ] **1.2** Install QGroundControl and connect it to the simulated PX4.
  - *Done when:* QGC shows the drone's attitude and GPS.
- [ ] **1.3** Take off, hover and land manually from QGC.
  - *Done when:* the drone completes a full flight.
- [ ] **1.4** Read about MAVLink: heartbeat, flight modes, offboard mode.
  - *Done when:* you can explain in two sentences what offboard mode is and why it needs a steady setpoint stream.
- [ ] **1.5** Write a small C++ (MAVSDK) program that prints the heartbeat and attitude.
  - Includes: `mavsdk.h`, `plugins/telemetry/telemetry.h`, `plugins/mavlink_passthrough/mavlink_passthrough.h`
  - Check the connection URL format for *your* MAVSDK version. The API port is usually 14540 (QGC uses 14550).
  - *Done when:* the printed values match what QGC shows.
- [ ] **1.6** Set up the PX4 ↔ ROS 2 bridge (micro XRCE-DDS agent).
  - *Done when:* `ros2 topic echo /fmu/out/vehicle_status` prints data.
- [ ] **1.7** Learn the frame conventions: PX4 uses NED/FRD, ROS uses ENU/FLU.
  - *Done when:* you can convert a position and a velocity by hand between the two.
- [ ] **1.8** First ROS 2 node: arm → take off to 2 m → hover 10 s → land, using **position setpoints**.
  - *Done when:* it works 3 times in a row.
- [ ] **1.9** Fly a 2 m square using **velocity setpoints only** (see the note below).
  - Write your own P controller per leg: `v = Kp * (corner - position)`, saturated to a max speed. Switch to the next corner when within a tolerance.
  - *Done when:* a logged x–y trajectory plot shows a clear square, with no position setpoints sent.
- [ ] **1.10** GitHub repo with a README and a short screen recording.
  - ✅ **Milestone 1:** you can honestly say "PX4, MAVLink, ROS 2, sim".

### Note: position setpoints vs. velocity setpoints vs. mission waypoints
| Type | What you send | Who closes the position loop | Used for |
|---|---|---|---|
| **Mission waypoints** | A list of waypoints uploaded once (Mission mode) | PX4 | Pre-planned surveys. Not offboard. |
| **Position setpoint** (offboard) | "Be at (x, y, z)", streamed | PX4's position controller | Simple go-to, hover (task 1.8) |
| **Velocity setpoint** (offboard) | "Move at (vx, vy, vz) right now", streamed | **You** (your own controller) | Tracking a moving target (Phase 2) |

- Offboard setpoints must be **streamed continuously** (faster than 2 Hz). If they stop, PX4 leaves offboard mode as a failsafe.
- In PX4's ROS 2 interface (`TrajectorySetpoint`), set the position fields to `NaN` and fill the velocity fields for velocity-only control. Also set `OffboardControlMode` to velocity.
- Landing on a moving pad needs velocity setpoints, because your controller combines tag error feedback with the pad's velocity as feedforward. Task 1.9 is practice for exactly that.

---

## Phase 2 — Land on a pad (static first, then moving)

- [ ] **2.1** Add a flat pad with an AprilTag to the scene.
  - *Done when:* it's visible from above at 3 m.
- [ ] **2.2** Attach a downward camera to the drone and publish it to ROS 2.
  - *Done when:* you can see the image in `rqt_image_view`.
- [ ] **2.3** Tag-detection node (OpenCV ArUco or an AprilTag library).
  - *Done when:* it publishes the tag pose in the camera frame.
- [ ] **2.4** Set up TF2: a static transform camera → base_link, then express the tag in the world frame.
  - *Done when:* RViz shows the tag in the right place as the drone moves.
- [ ] **2.5** Hover above the static tag using P control in x and y (velocity setpoints).
  - *Done when:* it stays within about 20 cm for 30 s.
- [ ] **2.6** Add descent and touchdown logic to land on the static tag.
  - *Done when:* it lands on the pad 5 times out of 5.
- [ ] **2.7** Script the pad to move at a constant velocity.
  - *Done when:* the pad moves in a straight line, repeatably.
- [ ] **2.8** Filter that estimates the pad's relative velocity (reuse the interceptor's KF).
  - *Done when:* a plot of estimated vs. true velocity matches closely.
- [ ] **2.9** Land on the moving pad, using the pad's velocity as feedforward.
  - ✅ **Milestone 2:** lands on the moving pad most of the time.
- [ ] **2.10** Add roll, pitch and heave to the pad.
  - *Done when:* landings still work at mild motion.

---

## Phase 3 and 4 — break these down after Milestone 2
- [ ] **Phase 3 (robustness):** fault injection (tag dropout, GPS loss, latency, wind), abort and go-around logic, Monte Carlo runs, failure-mode table.
- [ ] **Phase 4 (hardware):** choose a PX4-capable flight controller (F405 boards generally can't run PX4), add a companion computer and camera, and fly real tests.
