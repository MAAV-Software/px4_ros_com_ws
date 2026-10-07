# Navigation Subteam Plan

Status as of 2026-09-15. Pacing is **rolling/reactive** — milestones below are ordered, not dated. Dates get filled in and adjusted as the team actually makes progress; don't treat this file as a fixed calendar.

## End goal

The search is split into two phases:

- **Phase 1**: 3 drones complete an autonomous grid search over a predetermined track, while a 4th drone verifies object detections from the first 3.
- **Phase 2**: Once the original 3 drones finish their initial grid search, they continue verifying detected object positions. Drone 4 then scans a calculated (generated) route, making a third check on object locations along the path.

Final code changes are due **end of March 2027**. Features are built incrementally toward this two-phase system.

## Team

- 3-5 people, mostly new members (1 returning).
- No fixed subgroup or fixed meeting days per person — attendance varies week to week.
- 3 structured meetings/week, 2 hrs each.

## Hardware

- 2 old-design drones available now. Not the production competition platform — a formal switch to the new platform will be decided later.
- Used to validate physical systems at every milestone where feasible, relatively soon after sim validation (not necessarily the same week).
- 4-drone-scale milestones stay **sim-only** until more airframes are available.

## External dependency

Object detection and path-finding are being built by other subteams. Navigation does not yet know their tech stack or how that information will be relayed (message format, API, timing). This is the single largest schedule risk in the plan (see milestones 6-7 below).

## Onboarding (Sept 15 – Oct 11, 2026)

ROS2 + PX4 SITL fundamentals from scratch. No prior curriculum exists — this is a placeholder skeleton, not final form.

| Week | Focus | Outcome |
|---|---|---|
| 1 (Sep 15-21) | Linux/dev env, ROS2 core concepts (nodes, topics, services, launch files) | Everyone can build/run a basic ROS2 pub-sub node |
| 2 (Sep 22-28) | PX4 SITL + Gazebo setup, MAVLink/uXRCE-DDS bridge basics, this repo's structure | Everyone can launch SITL + Gazebo and see PX4 topics in ROS2 |
| 3 (Sep 29-Oct 5) | Offboard control basics: arm, takeoff, waypoint nav via `px4_msgs` | Each new member flies a single simulated drone through a scripted waypoint path |
| 4 (Oct 6-11) | Git workflow, multi-vehicle SITL (2 instances), intro to Milestone 1 codebase | Team is functionally at the starting line for Milestone 1 |

Official work begins the 2nd week of October 2026 (week of Oct 12).

## Milestone schedule (strictly sequential)

| # | Milestone | Sim | Real-life (2 old drones) | Notes |
|---|---|---|---|---|
| 1 | 1 drone autonomous | ✓ | ✓ | Baseline offboard control, single track |
| 2 | 2 drone | ✓ | ✓ | Independent multi-vehicle SITL + 2 real airframes |
| 3 | 2 drone, anti-collision, pre-determined routes | ✓ | ✓ | Anti-collision algorithm choice still open — decide early in this milestone |
| 4 | 4 drone, anti-collision, pre-determined routes | ✓ | Sim only (hardware gap) | Stays sim-only until more airframes exist |
| 5 | Integration: webapp launcher | ✓ | ✓ (at 2-drone scale) | Need webapp team's launch interface/API |
| 6 | Integration: object detection system | — (not feasible in sim) | ✓ | Real camera + real objects required. **Hardest gate in the roadmap** — can't be de-risked in sim, blocked on the other subteam's detection stack being ready/mountable, plus hardware access. |
| 7 | Integration: widest-path-finding from detections | — (not feasible in sim) | ✓ | Needs real detected object positions — inherits #6's blocker directly. |
| 8 | 2 drone, anti-collision, generated routes | ✓ (synthetic detection feed) | ✓ | Sim testing uses a simulated/injected detection input along a predetermined path as a stand-in for the live detection system, decoupling this milestone's logic testing from #6/#7's real-world dependency. |
| 9 | 4 drone, anti-collision, generated routes | ✓ (synthetic detection feed) | Sim only (hardware gap) | |
| 10 | 4 drone, Phase 1 functionality | ✓ (synthetic detection feed) | Sim only (hardware gap) | 3 drones grid search + drone 4 verification, integrated |
| 11 | 4 drone, Phase 2 functionality | ✓ (synthetic detection feed) | Sim only (hardware gap) | Final deliverable — due end of March 2027 |

## Open decisions (deferred, revisit when reached)

- Whether #8's sim work (synthetic-feed harness) can start before #6/#7 get real-life validation, or whether the team holds strictly to the listed order even if the other subteam's detection stack slips.
