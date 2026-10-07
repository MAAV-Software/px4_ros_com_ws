# Milestone 1 test sessions: plan

**Goal:** finish the Milestone 1 tests in three working sessions:

| Session | Length | Stage | Outcome |
|---|---|---|---|
| 1 | 1.5 h | SITL | Node changes written, reviewed and smoke-tested; first full-path run scored |
| 2 | 1.5 h | SITL | Every in-scope test passes in 3 consecutive runs; failsafe (F1) recorded; go/no-go for HITL |
| 3 | 3 h | HITL (real airframe) | The same tests pass on one of the old-design drones |

Related docs:
- The test matrix and thresholds: [`milestone-1-single-vehicle.md`](milestone-1-single-vehicle.md)
- The full scoring design: [`flight-scoring-plan.md`](flight-scoring-plan.md)

## Scope

| In scope | Deferred (Milestone 1 follow-up) | Why deferred |
|---|---|---|
| T1–T9 (translation), Y1–Y2 (yaw), R1 (indirect roll/pitch), H1 (hover hold), S1, F1, L1 | V1 (velocity mode), R2 (direct attitude mode) | PLAN.md defines Milestone 1 as baseline offboard control on a single track. Later milestones use position control. V1 and R2 each add a new control mode with its own safety work. V1 can be tried in SITL in Session 2's [stretch slot](#session-2-sitl-runs-90-min). |
| Post-flight scoring from the `.ulg` log (`m1_analyze`) | Live monitor node from `flight-scoring-plan.md` | The `.ulg` logs contain everything needed, so one tool covers both SITL and HITL. During a flight, the node's own logs show progress. |

## The test path

One path covers every in-scope row. It stays inside a 5 m × 5 m square at 3–6 m altitude, so the guide's 15 m × 15 m HITL area still applies. Yaw is in radians in the NED frame, as in the YAML and PX4: 0 = north, π/2 = east (clockwise seen from above). The table below shows degrees for readability.

| # | x | y | z | Yaw | Hold (s) | Tests |
|---|---|---|---|---|---|---|
| 0 | 0 | 0 | −5 | 0 | 5 | T1 climb |
| 1 | 5 | 0 | −5 | 0 | 5 | T3 north |
| 2 | 5 | 5 | −5 | 0 | 5 | T5 east |
| 3 | 0 | 5 | −5 | 0 | 5 | T4 south |
| 4 | 0 | 0 | −5 | 0 | 5 | T6 west |
| 5 | 0 | 0 | −3 | 0 | 5 | T2 descend |
| 6 | 4 | 4 | −3 | 0 | 5 | T7 diagonal (NE) |
| 7 | 4 | 0 | −3 | 0 | 3 | — (reposition) |
| 8 | 0 | 4 | −3 | 0 | 5 | T7 diagonal (NW) |
| 9 | 4 | 0 | −6 | 0 | 5 | T8 3D (SE + climb) |
| 10 | 0 | 4 | −4 | 0 | 5 | T8 3D (NW + descend) |
| 11 | 1 | 4 | −4 | 0 | 3 | T9 small (1 m north) |
| 12 | 1 | 5 | −4 | 0 | 3 | T9 small (1 m east) |
| 13 | 2 | 2 | −5 | 0 | 10 | H1 hover hold |
| 14 | 2 | 2 | −5 | 90 | 3 | Y1 yaw clockwise |
| 15 | 2 | 2 | −5 | 180 | 3 | Y1 yaw clockwise |
| 16 | 2 | 2 | −5 | 90 | 3 | Y1 yaw counter-clockwise |
| 17 | 2 | 2 | −5 | 0 | 3 | Y1 yaw counter-clockwise |
| 18 | 2 | 2 | −5 | −90 | 3 | Y1 yaw counter-clockwise |
| 19 | 2 | 2 | −5 | 0 | 3 | Y1 yaw clockwise |
| 20 | 4 | 2 | −5 | 90 | 5 | Y2 yaw while moving |
| 21 | 2 | 2 | −5 | 0 | 5 | Y2 yaw while moving, then land |

- Yaw changes never exceed 90° in one step, so the turn direction is never ambiguous.
- R1 (tilt) is scored on every leg, and S1, F1 and L1 from `vehicle_status`.
- Expected flight time is about 3–4 minutes with the speed limits below: about 100 s of hold time, plus travel, takeoff and landing.
- A second file, `milestone1_path_square.yaml`, holds the original 4-waypoint square at 3 m altitude. It is used for the first flight of each session.

## Code changes

### Prepared before Session 1: the scorer

The lead prepares this, with Claude's help if wanted (about 45 min). It's test tooling rather than flight code, and Session 1 needs it working to score the first run.

1. **`m1_analyze`:** a post-flight scorer.
   ```
   ros2 run milestone1_singlevehicle m1_analyze <flight.ulg> --stage sitl|hitl [--path <yaml>]
   ```
   - It splits the flight into legs wherever the logged `trajectory_setpoint` changes. This is the setpoint the node sent. It corrects `flight-scoring-plan.md`, which used `vehicle_local_position_setpoint`. That topic can change continuously, which would make leg splitting unreliable.
   - It scores each leg's measures against the thresholds in the path YAML and writes a PASS/FAIL report to `log/m1_reports/`.
   - It needs `pip install pyulog`.
2. **Unit tests** for the scoring maths, using made-up flight data.
3. **The path files:**
   - `milestone1_path.yaml`: the full path above, with per-waypoint `yaw`, `hold_s` and `tests` IDs, plus the SITL and HITL `thresholds`.
   - `milestone1_path_square.yaml`.

### Written in Session 1: node changes

In pairs, in `offboard_control_solution.py` and the launch file:

| Pair | Change |
|---|---|
| A | **Yaw and hold.** Read `yaw` (radians) and `hold_s` per waypoint. Send yaw in `TrajectorySetpoint.yaw` as is. A waypoint is reached when the drone is within 0.5 m **and** its heading (`VehicleLocalPosition.heading`) is within 5°. The node then holds there for `hold_s` before moving on. |
| B | **S1 and path selection.** Subscribe to `vehicle_status` again. Resend arm and offboard once per second until the drone reports armed and in Offboard mode. Log the confirmation. If it isn't confirmed after 10 s, log an error and stop. Add a `path_file` parameter (default `milestone1_path.yaml`) and a matching `path` launch argument. |

Both changes are in the same file, so pair B rebases on pair A's change before the review.

### Decisions to make before Session 1

- **Agree the pass thresholds** in the guide. They're still marked "proposed".
- **Agree the speed limits.** Use the same values in SITL and HITL so the results are comparable:

  | Parameter | PX4 default | For these tests |
  |---|---|---|
  | `MPC_XY_VEL_MAX` | 12 m/s | 2 m/s |
  | `MPC_Z_VEL_MAX_UP` | 3 m/s | 1 m/s |
  | `MPC_Z_VEL_MAX_DN` | 1.5 m/s | 1 m/s |
  | `MPC_YAWRAUTO_MAX` | 60°/s | 45°/s |

## Session 1: SITL, build (90 min)

**Roles:**
- **Pairs A and B:** write the node changes.
- **Operator:** sets up SITL and validates the scorer while the pairs code.
- **Recorder:** notes decisions and follow-ups, and commits.

| Time | Activity |
|---|---|
| 0:00–0:10 | Pull the branch, `colcon build`, `colcon test`. Start SITL headless, and set the speed limits in the PX4 console with `param set`. Run `param show` after the next SITL restart to confirm they were kept. |
| 0:10–0:50 | **Pairs A and B write the node changes.** Meanwhile the operator flies the *current* node on `milestone1_path_square.yaml` (pass it with `path:=` once pair B's change is in, or temporarily swap the file) and runs `m1_analyze` on the `.ulg`. This checks the scorer on a real flight before the new node is ready. |
| 0:50–1:05 | Code review of both changes as a group. Fix anything that blocks flying; log everything else as follow-ups. |
| 1:05–1:15 | **Smoke test:** fly `milestone1_path_square.yaml` with the new node. Confirms S1, T1, T3–T5, L1 and the hold behaviour. |
| 1:15–1:25 | **First full-path run**, scored with `m1_analyze`. |
| 1:25–1:30 | Commit. List anything that failed, and who fixes it before Session 2. |

**If running behind:** skip the first full-path run. It becomes the first run of Session 2.

## Between Sessions 1 and 2

- Fix anything from Session 1's failure list.
- If any measure failed because the threshold looks unrealistic rather than because of a bug, raise it with the team before Session 2. Don't change thresholds silently.

## Session 2: SITL, runs (90 min)

**Roles:**
- **Operator:** runs SITL and the node.
- **Analyst:** runs `m1_analyze` and checks the numbers.
- **Recorder:** fills in the results log and commits.
- **Everyone else:** watch QGroundControl and the node logs.

| Time | Activity |
|---|---|
| 0:00–0:10 | Pull, build, start SITL. Check the speed limits are still set. |
| 0:10–0:40 | **Full path × 3 consecutive runs.** Restart SITL between runs, so each run takes off from the origin and T1 is a pure climb. Score each run as soon as it lands. Any failure restarts the count. |
| 0:40–0:50 | **F1 run:** `Ctrl-C` the node mid-path. Record the failsafe action PX4 took. |
| 0:50–1:10 | **Buffer / stretch.** Fix and re-run anything that failed. If all passed, optionally try V1 (velocity mode) in SITL. It isn't required for Milestone 1. |
| 1:10–1:30 | Fill in the results log, make the go/no-go call for HITL, commit, and assign the HITL prerequisites. |

**Go to HITL only if:**
- 3 consecutive full-path runs pass every in-scope row within the SITL thresholds
- The F1 action is recorded and is acceptable for hardware (hold or land, not something unexpected)
- No open bugs in the node

If these aren't met, use the first part of Session 3 indoors for the fixes and the bench test, and only fly if there's time for the full Session 3 flight plan below.

## Before Session 3: HITL prerequisites

These should be done **before** Session 3, so the session is spent flying.

| Task | Notes |
|---|---|
| Fill in the guide's 3 HITL `TODO`s | Flight controller link (serial or UDP), the matching `MicroXRCEAgent` command, the drone's PX4 version, airframe notes |
| **Indoor bench test, props off** (~30 min) | Connect, check that the `/fmu/out/...` topic names match what the node uses (they may differ on the drone's PX4 version), and run the node to confirm S1: it arms and enters Offboard mode. Disarm. If this can't be done beforehand, it's the first block of Session 3 and Flight 5 is cut. |
| Set parameters on the drone | The speed limits above. Check `MPC_TILTMAX_AIR`, `COM_OBL_RC_ACT` and `COM_OF_LOSS_T` (default 1 s), and set a geofence of about 20 m horizontal and 10 m vertical. |
| Logistics | Test site booked, weather checked, at least 6 charged batteries, a safety pilot confirmed, and the scoring laptop with `pyulog` installed |

## Session 3: HITL (3 h)

**Roles:**
- **Safety pilot:** holds the RC transmitter. This is their only job.
- **Operator:** runs the ground station and the node.
- **Spotter:** watches the area and calls abort.
- **Analyst/recorder:** pulls logs, runs `m1_analyze`, fills in the results log.

Rotate the operator, spotter and analyst roles between flights so more of the team gets hands-on time. The safety pilot doesn't rotate.

| Time | Activity |
|---|---|
| 0:00–0:30 | Set up at the site, then run the guide's every-flight checklist: area clear, position lock, RC and kill switch tested, agent connected, topics present. Confirm the drone's parameters (speed limits, failsafes, geofence). |
| 0:30–0:45 | **Flight 1:** `milestone1_path_square.yaml` at 3 m. Pull the log, run a quick `m1_analyze`, and decide whether to fly the full path. |
| 0:45–1:00 | **Flight 2:** full path. Score it before the next flight. |
| 1:00–1:15 | **Flight 3:** F1. Stop the node mid-path, with the safety pilot ready to take over. |
| 1:15–1:30 | Break. Swap batteries, check the airframe (props, motor temperatures, mounts), and review scores so far. |
| 1:30–1:45 | **Flight 4:** full path repeated |
| 1:45–2:00 | **Flight 5:** full path repeated again. This gives 3 full-path flights to compare with SITL. |
| 2:00–2:30 | **Buffer:** re-fly anything that failed or was aborted (weather, position lock, takeover). |
| 2:30–3:00 | Pull all `.ulg` logs, score with `--stage hitl`, fill in the results log, debrief, pack up, commit |

**If running behind:** cut Flight 5 first, then Flight 4. The order of priority is the square, then the full path, then F1.

**Abort the session** if wind is strong enough to affect the hover, position lock is poor, or the safety pilot has to take over twice.

**HITL passes if** every in-scope row is within the HITL thresholds in at least one full-path flight (preferably in all of them), and F1 matches the SITL result.

## Risks

| Risk | Mitigation |
|---|---|
| Scorer not ready before Session 1 | The pairs still write the node changes; scoring moves into Session 2's buffer, and the 3 runs are scored afterwards |
| Node changes take longer than 40 min in Session 1 | Cut the first full-path run. Session 2's buffer absorbs the rest. |
| Topic names or the link differ on the drone | The indoor bench test before Session 3 |
| PX4's default speed (12 m/s) on old airframes | The speed limits above, the same in all sessions |
| SITL fails thresholds | Tune the node, or relax a threshold with the team's agreement and record why. Don't change thresholds silently. |
| Weather or site problems | Book a backup date for Session 3. Run Flight 1 as early as possible. |

## Done when

- SITL and HITL checklists in the guide are ticked for all in-scope rows
- The results log contains the `m1_analyze` reports and flight logs for each run
- V1, R2 and the live monitor are logged as Milestone 1 follow-ups
