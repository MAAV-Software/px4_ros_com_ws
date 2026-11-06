// What the hub sends. Field names match HubCore.snapshot() in hub_core.py.

export type MissionState =
  | 'idle'
  | 'preflight'
  | 'ready'
  | 'starting'
  | 'searching'
  | 'landing'
  | 'complete'
  | 'aborting'
  | 'aborted';

export interface Mission {
  id: string;
  name: string;
  inputs: string[];
  drones: Record<string, { role: string }>;
}

export interface Drone {
  id: string;
  role: string | null;
  connected: boolean;
  link_age_s: number | null;
  run_state: string | null;
  activity: string | null;
  armed: boolean | null;
  airborne: boolean | null;
  position_ok: boolean | null;
  position: { x: number; y: number; z: number } | null;
  battery_pct: number | null;
  global_position_ok: boolean | null;
  gps_fix: number | null;
  rc_lost: boolean | null;
  waypoint: number | null;
  waypoint_total: number | null;
  recovery: string | null;
  hold_left_s: number | null;
  failure: string | null;
}

export type AbortResult =
  | 'sending'
  | 'no_response'
  | 'holding'
  | 'not_armed'
  | 'hold_unconfirmed'
  | 'pilot_control'
  | 'already_landing';

export interface Snapshot {
  type: 'state';
  state: MissionState;
  reason: string;
  mission: Mission | null;
  run_id: string | null;
  committed: boolean;
  clock: { running: boolean; elapsed_ms: number };
  checks: Record<string, { ok: boolean; problems: string[] } | null>;
  abort_results: Record<string, AbortResult>;
  recovery: string | null;
  drones: Drone[];
  log_seq: number;
}

export interface LogLine {
  seq: number;
  source: string;
  level: string;
  text: string;
}

export interface CommandResult {
  ok: boolean;
  state: MissionState | null;
  reason: string;
}
