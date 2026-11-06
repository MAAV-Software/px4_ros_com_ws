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
  return match ? `Drone ${match[1]}` : id;
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
