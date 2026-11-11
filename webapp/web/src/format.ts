import type { AbortResult, MissionState } from './types';

export const STATE_LABEL: Record<MissionState, string> = {
  idle: 'Idle',
  preflight: 'Preflight',
  ready: 'Ready',
  starting: 'Starting',
  searching: 'Searching',
  landing: 'Landing',
  complete: 'Complete',
  aborting: 'Aborting',
  aborted: 'Aborted',
};

export const ABORT_LABEL: Record<AbortResult, string> = {
  sending: 'Sending',
  no_response: 'No response',
  holding: 'Holding',
  not_armed: 'Not armed',
  hold_unconfirmed: 'Hold not confirmed',
  pilot_control: 'Pilot has control, left alone',
  already_landing: 'Already coming down',
};

const GPS_FIX: Record<number, string> = {
  0: 'No GPS fix',
  1: 'No GPS fix',
  2: 'GPS 2D',
  3: 'GPS 3D',
  4: 'DGPS',
  5: 'RTK float',
  6: 'RTK fixed',
  8: 'GPS extrapolated',
};

export function droneName(id: string) {
  const match = /^drone(\d+)$/.exec(id);
  if (match) return `Drone ${match[1]}`;
  const sim = /^sim(\d+)$/.exec(id);
  return sim ? `SITL ${sim[1]}` : id;
}

export const MODE_LABEL = { sitl: 'SITL', hitl: 'HITL' } as const;

// Each drone's colour on the map, by the slot the hub gives it, in this fixed order. Blue,
// green, red, orange and yellow are left for the path and mines, as in the simulator. Checked
// with the data-viz palette validator against the map's background, all pairs: the first four
// are apart by CVD dE 14 and normal dE 22, all six by 9.8 and 15. A drone past the sixth is
// drawn in ink. The number on each marker identifies the drone too, so colour isn't alone.
export const DRONE_COLORS = ['#422ade', '#a10354', '#9d850b', '#9c72fe', '#f531ac', '#984b01'];
export const INK = '#111111';

export function droneColor(slot: number | null | undefined) {
  return slot === null || slot === undefined ? INK : (DRONE_COLORS[slot] ?? INK);
}

// Falls back to whether PX4 has a global position, for firmware whose vehicle_gnss can't be read.
export function gpsLabel(fix: number | null, positionOk: boolean | null) {
  if (fix !== null) return GPS_FIX[fix] ?? `GPS fix ${fix}`;
  return positionOk === null ? 'GPS unknown' : positionOk ? 'GPS OK' : 'No GPS position';
}

export function rcLabel(lost: boolean | null) {
  return lost === null ? 'RC unknown' : lost ? 'No RC' : 'RC OK';
}

export function clockText(ms: number) {
  const total = Math.max(0, Math.floor(ms / 1000));
  const minutes = String(Math.floor(total / 60)).padStart(2, '0');
  const seconds = String(total % 60).padStart(2, '0');
  return `${minutes}:${seconds}`;
}
