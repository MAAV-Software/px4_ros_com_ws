import { droneName, gpsLabel, rcLabel } from './format';
import type { Drone } from './types';

function linkText(drone: Drone, offlineS: number, phoneLinked: boolean) {
  // With no phone link, nothing here is live: say how old it is instead.
  if (!phoneLinked) return `${Math.round((drone.link_age_s ?? 0) + offlineS)} s ago`;
  return drone.connected ? 'Linked' : `Lost, ${Math.round(drone.link_age_s ?? 0)} s ago`;
}

function DroneCard({ drone, offlineS, phoneLinked }: { drone: Drone; offlineS: number; phoneLinked: boolean }) {
  const lost = phoneLinked && !drone.connected;
  const route =
    drone.waypoint !== null && drone.waypoint_total !== null && drone.airborne
      ? `, waypoint ${drone.waypoint + 1} of ${drone.waypoint_total}`
      : '';
  return (
    <article className={lost ? 'card card-lost' : 'card'}>
      <header className="card-head">
        <strong>
          {droneName(drone.id)}
          {drone.role ? `, ${drone.role}` : ''}
        </strong>
        <span className={lost ? 'strong' : 'quiet'}>{linkText(drone, offlineS, phoneLinked)}</span>
      </header>
      <p>
        {drone.battery_pct === null ? 'Battery unknown' : `Battery ${drone.battery_pct}%`},{' '}
        {gpsLabel(drone.gps_fix, drone.global_position_ok)}, {rcLabel(drone.rc_lost)}
      </p>
      <p className="quiet">
        {drone.activity ?? 'No report yet'}
        {route}
      </p>
      {drone.failure && <p className="failure">{drone.failure}</p>}
    </article>
  );
}

export function FleetTab({ drones, offlineS, phoneLinked }: { drones: Drone[]; offlineS: number; phoneLinked: boolean }) {
  if (drones.length === 0) return <p className="quiet center">No drones have connected yet.</p>;
  return (
    <div className="stack">
      {drones.map((drone) => (
        <DroneCard key={drone.id} drone={drone} offlineS={offlineS} phoneLinked={phoneLinked} />
      ))}
    </div>
  );
}
