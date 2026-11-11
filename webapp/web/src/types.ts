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

// One package folder in a drone's src, from LaunchLibrary.catalog() in launch_library.py.
export interface Folder {
  folder: string;
  package: string;
  files: { file: string; built: boolean }[];
}

// One argument a launch file declares, from launch_args.py. A null default is one only
// the launch can work out.
export interface LaunchArg {
  name: string;
  default: string | null;
  description: string;
  choices: string[];
  included: boolean;
}

export type Mode = 'sitl' | 'hitl';

// One SITL drone the hub started, from HubCore._sitl_view() in hub_core.py.
export interface SitlDrone {
  drone_id: string;
  instance: number;
  state: 'starting' | 'ready' | 'stuck' | 'stopping' | 'exited';
  detail: string;
}

export type Corner = 'south_west' | 'south_east' | 'north_east' | 'north_west';

export interface LatLon {
  lat: number;
  lon: number;
}

// The search area, from Field.view() in grid.py.
export interface Area {
  corners: Record<Corner, LatLon>;
  sides_m: Record<'south' | 'north' | 'west' | 'east', number>;
  flags: string[];
}

// Where a drone is on the 40 by 150 grid: fractional column and row.
export interface GridCell {
  col: number;
  row: number;
  on_field: boolean;
}

export interface Mission {
  id: string;
  name: string;
  folder: string;
  package: string;
  file: string;
  drones: string[];
  args: Record<string, string>;
}

export interface Drone {
  id: string;
  in_mission: boolean;
  sim: boolean | null;
  color: number | null;
  connected: boolean;
  link_age_s: number | null;
  run_state: string | null;
  activity: string | null;
  armed: boolean | null;
  airborne: boolean | null;
  position_ok: boolean | null;
  position: { x: number; y: number; z: number } | null;
  global: LatLon | null;
  grid: GridCell | null;
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
  mode: Mode;
  sitl: { available: boolean; drones: SitlDrone[] };
  area: Area | null;
  voice: { available: boolean; reason: string };
  https: { port: number } | null;
  mission: Mission | null;
  launch_args: { source: string; args: LaunchArg[] } | null;
  catalogs: Record<string, Folder[]>;
  run_id: string | null;
  committed: boolean;
  clock: { running: boolean; elapsed_ms: number };
  checks: Record<string, { ok: boolean; problems: string[] } | null>;
  abort_results: Record<string, AbortResult>;
  recovery: string | null;
  drones: Drone[];
  log_seq: number;
}

// Where each picked drone has been this mission, as [column, row] points on the grid.
export interface Trails {
  version: number;
  points: Record<string, [number, number][]>;
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
