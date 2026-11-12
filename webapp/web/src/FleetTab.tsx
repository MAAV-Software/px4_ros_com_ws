import { useState } from 'react';
import { droneColor, droneName, gpsLabel, rcLabel } from './format';
import { placeText } from './MapTab';
import { ModeToggle } from './Mode';
import type { Hub } from './useHub';
import type { Drone, SitlDrone, Snapshot } from './types';

const SITL_STATE: Record<SitlDrone['state'], string> = {
  starting: 'Starting',
  ready: 'Ready',
  stuck: 'Not coming up',
  stopping: 'Stopping',
  exited: 'Stopped by itself',
};

function linkText(drone: Drone, offlineS: number, phoneLinked: boolean) {
  // With no phone link, nothing here is live: say how old it is instead.
  if (!phoneLinked) return `${Math.round((drone.link_age_s ?? 0) + offlineS)} s ago`;
  return drone.connected ? 'Linked' : `Lost, ${Math.round(drone.link_age_s ?? 0)} s ago`;
}

function RemoveButton({ onRemove, disabled }: { onRemove: () => void; disabled: boolean }) {
  return (
    <button className="button button-small" disabled={disabled} onClick={onRemove}>
      Remove
    </button>
  );
}

function DroneCard({
  drone,
  sitl,
  offlineS,
  phoneLinked,
  onRemove,
}: {
  drone: Drone;
  sitl: SitlDrone | undefined;
  offlineS: number;
  phoneLinked: boolean;
  onRemove: (() => void) | null;
}) {
  const lost = phoneLinked && !drone.connected && sitl?.state !== 'stopping';
  const route =
    drone.waypoint !== null && drone.waypoint_total !== null && drone.airborne
      ? `, waypoint ${drone.waypoint + 1} of ${drone.waypoint_total}`
      : '';
  const starting = sitl && sitl.state !== 'ready';
  return (
    <article className={lost ? 'card card-lost' : 'card'}>
      <header className="card-head">
        <strong className="card-name">
          <span className="map-dot" style={{ background: droneColor(drone.color) }} aria-hidden="true">
            {/(\d+)$/.exec(drone.id)?.[1] ?? '?'}
          </span>
          {droneName(drone.id)}
          {drone.in_mission ? ', picked' : ''}
        </strong>
        <span className={lost ? 'strong' : 'quiet'}>
          {starting ? SITL_STATE[sitl.state] : linkText(drone, offlineS, phoneLinked)}
        </span>
      </header>
      {starting && sitl.detail && <p className={sitl.state === 'starting' ? 'quiet' : 'failure'}>{sitl.detail}</p>}
      <p>
        {drone.battery_pct === null ? 'Battery unknown' : `Battery ${drone.battery_pct}%`},{' '}
        {gpsLabel(drone.gps_fix, drone.global_position_ok)}
        {drone.sim ? '' : `, ${rcLabel(drone.rc_lost)}`}
      </p>
      <p className="quiet">
        {drone.activity ?? 'No report yet'}
        {route}
      </p>
      {drone.grid && <p className="quiet">{placeText(drone)}</p>}
      {drone.failure && <p className="failure">{drone.failure}</p>}
      {onRemove && <RemoveButton onRemove={onRemove} disabled={!phoneLinked || Boolean(drone.airborne)} />}
    </article>
  );
}

// A SITL drone whose agent hasn't connected yet, so the hub has no report from it.
function PendingSitlCard({ sitl, onRemove }: { sitl: SitlDrone; onRemove: (() => void) | null }) {
  return (
    <article className="card">
      <header className="card-head">
        <strong>{droneName(sitl.drone_id)}</strong>
        <span className="quiet">{SITL_STATE[sitl.state]}</span>
      </header>
      {sitl.detail && <p className={sitl.state === 'starting' ? 'quiet' : 'failure'}>{sitl.detail}</p>}
      {onRemove && <RemoveButton onRemove={onRemove} disabled={false} />}
    </article>
  );
}

export function FleetTab({ hub, snapshot, offlineS }: { hub: Hub; snapshot: Snapshot; offlineS: number }) {
  const [refused, setRefused] = useState('');
  const phoneLinked = hub.linked;
  const sitl = snapshot.sitl;
  const sitlMode = snapshot.mode === 'sitl';
  const live = sitl.drones.filter((entry) => entry.state !== 'stopping');
  // Each SITL drone joins the world the first one started, so they go newest first.
  const newest = live.length > 0 ? live[live.length - 1].drone_id : null;
  const starting = sitl.drones.some((entry) => entry.state === 'starting');
  const layoutMax = sitl.layouts.find((layout) => layout.id === sitl.layout)?.max ?? Infinity;
  const full = sitl.drones.length >= layoutMax;
  const pending = sitl.drones.filter((entry) => !snapshot.drones.some((drone) => drone.id === entry.drone_id));

  const run = async (path: string, body: object = {}) => {
    const result = await hub.command(path, body);
    setRefused(result.ok ? '' : result.reason);
  };
  const removeFor = (id: string) =>
    sitlMode && id === newest ? () => void run('/api/sitl/remove', { drone_id: id }) : null;

  return (
    <div className="stack">
      <ModeToggle hub={hub} snapshot={snapshot} />
      {sitlMode && sitl.available && (
        <section className="box">
          <h2>SITL drones</h2>
          <p className="quiet small">
            Each one is a PX4 instance in a shared Gazebo world on the hub's computer. They are removed newest first,
            since the first one runs the world.
          </p>
          <label className="field">
            <span className="field-label small">Spawn layout</span>
            <select
              value={sitl.layout ?? ''}
              disabled={!phoneLinked || sitl.drones.length > 0}
              onChange={(event) => void run('/api/sitl/layout', { layout: event.target.value })}
            >
              {sitl.layouts.map((layout) => (
                <option key={layout.id} value={layout.id}>
                  {layout.label}, up to {layout.max}
                </option>
              ))}
            </select>
            {sitl.drones.length > 0 && (
              <span className="quiet small">Stop all SITL drones to change where they spawn.</span>
            )}
          </label>
          <div className="row">
            <button
              className="button button-primary"
              disabled={!phoneLinked || starting || full}
              onClick={() => void run('/api/sitl/add')}
            >
              {starting ? 'Starting' : full ? 'Layout full' : 'Add SITL drone'}
            </button>
            {sitl.drones.length > 0 && (
              <button className="button" disabled={!phoneLinked} onClick={() => void run('/api/sitl/stop')}>
                Stop all
              </button>
            )}
          </div>
        </section>
      )}
      {!sitlMode && live.length > 0 && (
        <p className="banner">
          {live.length} SITL {live.length === 1 ? 'drone is' : 'drones are'} still running. Switch to SITL to stop them.
        </p>
      )}
      {refused && (
        <p className="banner banner-warn" role="status">
          {refused}
        </p>
      )}
      {snapshot.drones.length === 0 && pending.length === 0 && (
        <p className="quiet center">
          {sitlMode ? 'No SITL drones yet.' : "No drones have connected yet. Start each drone's agent with this hub's address."}
        </p>
      )}
      {pending.map((entry) => (
        <PendingSitlCard key={entry.drone_id} sitl={entry} onRemove={removeFor(entry.drone_id)} />
      ))}
      {snapshot.drones.map((drone) => (
        <DroneCard
          key={drone.id}
          drone={drone}
          sitl={sitl.drones.find((entry) => entry.drone_id === drone.id)}
          offlineS={offlineS}
          phoneLinked={phoneLinked}
          onRemove={removeFor(drone.id)}
        />
      ))}
    </div>
  );
}
