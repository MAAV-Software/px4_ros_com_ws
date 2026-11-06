import { useEffect, useState } from 'react';
import type { Hub } from './useHub';
import { useTicker } from './useHub';
import { ABORT_LABEL, clockText, droneName } from './format';
import type { CommandResult, Snapshot } from './types';

const CONFIRM_WINDOW_MS = 10000;

function Dot({ tone }: { tone: 'good' | 'warn' | 'bad' | 'quiet' }) {
  return <span className={`dot dot-${tone}`} aria-hidden="true" />;
}

function ReasonBanner({ text }: { text: string }) {
  return text ? (
    <p className="banner banner-warn" role="status">
      {text}
    </p>
  ) : null;
}

function StartChecks({ snapshot }: { snapshot: Snapshot }) {
  const names = Object.keys(snapshot.checks);
  if (names.length === 0) return null;
  return (
    <section className="box">
      <h2>Start checks</h2>
      <ul className="plain">
        {names.map((id) => {
          const check = snapshot.checks[id];
          return (
            <li key={id} className="check">
              <Dot tone={check === null ? 'quiet' : check.ok ? 'good' : 'warn'} />
              <span>
                {droneName(id)}:{' '}
                {check === null ? 'not checked' : check.ok ? 'pass' : check.problems.join('; ')}
              </span>
            </li>
          );
        })}
      </ul>
    </section>
  );
}

// Start and confirm are separate tasks, so a stray tap can't launch anything.
// The clock is measured from the first tap, and the hub is told how long ago that was.
function StartConfirm({
  missionName,
  onConfirm,
  onCancel,
  pressedAt,
}: {
  missionName: string;
  onConfirm: (pressAgeMs: number) => void;
  onCancel: () => void;
  pressedAt: number;
}) {
  const now = useTicker(250);
  const left = CONFIRM_WINDOW_MS - (now - pressedAt);
  useEffect(() => {
    if (left <= 0) onCancel();
  }, [left, onCancel]);
  return (
    <div className="sheet-backdrop" role="dialog" aria-modal="true" aria-label="Confirm start">
      <div className="sheet">
        <h2>Start {missionName}?</h2>
        <p className="quiet">The timer has started. This closes in {Math.max(0, Math.ceil(left / 1000))} s.</p>
        <button className="button button-primary" onClick={() => onConfirm(performance.now() - pressedAt)}>
          Confirm start
        </button>
        <button className="button" onClick={onCancel}>
          Cancel
        </button>
      </div>
    </div>
  );
}

function SetupPanel({ hub, snapshot }: { hub: Hub; snapshot: Snapshot }) {
  const [refused, setRefused] = useState('');
  const [pressedAt, setPressedAt] = useState<number | null>(null);
  const missionId = snapshot.mission?.id ?? '';

  const report = (result: CommandResult) => setRefused(result.ok ? '' : result.reason);
  const pick = async (id: string) => report(await hub.command('select', { mission_id: id }));
  const confirm = async (pressAgeMs: number) => {
    setPressedAt(null);
    report(await hub.command('start', { press_age_ms: pressAgeMs }));
  };

  const needs = snapshot.mission ? Object.keys(snapshot.mission.drones).length : 0;
  const checking = snapshot.state === 'preflight';
  const canStart = snapshot.state === 'ready' && hub.linked;

  return (
    <>
      <label className="field">
        <span className="field-label">Mission</span>
        <select
          value={missionId}
          disabled={checking || !hub.linked}
          onChange={(event) => void pick(event.target.value)}
        >
          <option value="" disabled>
            Choose a mission
          </option>
          {hub.missions.map((mission) => (
            <option key={mission.id} value={mission.id}>
              {mission.name}
            </option>
          ))}
        </select>
      </label>
      {snapshot.mission && (
        <p className="quiet">
          Needs {needs} {needs === 1 ? 'drone' : 'drones'}
        </p>
      )}
      <ReasonBanner text={refused || snapshot.reason} />
      <StartChecks snapshot={snapshot} />
      <div className="row">
        <button
          className="button"
          disabled={!snapshot.mission || checking || !hub.linked}
          onClick={() => void pick(missionId)}
        >
          {checking ? 'Checking' : 'Check again'}
        </button>
        <button
          className="button button-primary"
          disabled={!canStart}
          onClick={() => setPressedAt(performance.now())}
        >
          Start
        </button>
      </div>
      {pressedAt !== null && snapshot.mission && (
        <StartConfirm
          missionName={snapshot.mission.name}
          pressedAt={pressedAt}
          onConfirm={(age) => void confirm(age)}
          onCancel={() => setPressedAt(null)}
        />
      )}
    </>
  );
}

function FlightPanel({ snapshot, linked }: { snapshot: Snapshot; linked: boolean }) {
  const drones = snapshot.drones.filter((drone) => drone.role !== null);
  return (
    <>
      {snapshot.committed ? (
        <p className="banner banner-good">Committed. Losing Wi-Fi now has no effect on the drones.</p>
      ) : (
        <p className="banner">Sending the mission to every drone. Nothing flies until all have it.</p>
      )}
      <section className="box">
        <h2>{linked ? 'Drones' : 'Drones, last known'}</h2>
        <ul className="plain">
          {drones.map((drone) => (
            <li key={drone.id} className="check">
              <Dot tone={drone.run_state === 'failed' ? 'bad' : linked && drone.connected ? 'good' : 'quiet'} />
              <span>
                {droneName(drone.id)}: {drone.activity ?? 'waiting'}
                {drone.waypoint !== null && drone.waypoint_total !== null && drone.airborne
                  ? `, waypoint ${drone.waypoint + 1} of ${drone.waypoint_total}`
                  : ''}
              </span>
            </li>
          ))}
        </ul>
      </section>
    </>
  );
}

function AbortPanel({ hub, snapshot }: { hub: Hub; snapshot: Snapshot }) {
  const [refused, setRefused] = useState('');
  const results = Object.entries(snapshot.abort_results);
  const missing = results.some(([, result]) => result === 'no_response' || result === 'hold_unconfirmed');
  const holding = snapshot.drones.filter((drone) => drone.airborne && drone.hold_left_s !== null);
  const anyAirborne = snapshot.drones.some((drone) => drone.role !== null && drone.airborne);

  const run = async (path: string, body: object = {}) => {
    const result = await hub.command(path, body);
    setRefused(result.ok ? '' : result.reason);
  };
  const secondsLeft = holding.length ? Math.min(...holding.map((drone) => drone.hold_left_s ?? 0)) : null;

  return (
    <>
      <ReasonBanner text={refused || snapshot.reason} />
      {missing && (
        <p className="banner banner-bad" role="alert">
          Call the safety pilot for any drone not holding. The app may not be able to stop it.
        </p>
      )}
      <section className="box">
        <h2>Abort results</h2>
        <ul className="plain">
          {results.map(([id, result]) => {
            // After Hold, say what the drone is doing now: Returning, Landing or On ground.
            const activity = snapshot.drones.find((drone) => drone.id === id)?.activity;
            const label = result === 'holding' && activity ? activity : (ABORT_LABEL[result] ?? result);
            return (
              <li key={id} className="check">
                <Dot
                  tone={
                    result === 'sending' ? 'quiet' : result === 'no_response' || result === 'hold_unconfirmed' ? 'bad' : 'good'
                  }
                />
                <span>
                  {droneName(id)}: {label}
                </span>
              </li>
            );
          })}
        </ul>
      </section>
      {snapshot.state === 'aborted' && holding.length > 0 && (
        <section className="box">
          <h2>Bring them back</h2>
          <p className="quiet">Returning automatically in {secondsLeft} s if you don't choose.</p>
          <div className="row">
            <button className="button button-primary" onClick={() => void run('recover', { action: 'return' })}>
              Return
            </button>
            <button className="button button-primary" onClick={() => void run('recover', { action: 'land' })}>
              Land here
            </button>
          </div>
        </section>
      )}
      {snapshot.state === 'aborted' && (
        <button className="button" disabled={anyAirborne} onClick={() => void run('reset')}>
          {anyAirborne ? 'Waiting for drones to land' : 'Finish and reset'}
        </button>
      )}
    </>
  );
}

function CompletePanel({ hub, snapshot, elapsedMs }: { hub: Hub; snapshot: Snapshot; elapsedMs: number }) {
  const failed = snapshot.drones.filter((drone) => drone.role !== null && drone.failure);
  return (
    <>
      <p className="banner banner-good">Mission complete in {clockText(elapsedMs)}.</p>
      {failed.map((drone) => (
        <p key={drone.id} className="banner banner-warn">
          {droneName(drone.id)}: {drone.failure}
        </p>
      ))}
      <button className="button button-primary" onClick={() => void hub.command('reset')}>
        New mission
      </button>
    </>
  );
}

export function MissionTab({ hub, snapshot, elapsedMs }: { hub: Hub; snapshot: Snapshot; elapsedMs: number }) {
  switch (snapshot.state) {
    case 'idle':
    case 'preflight':
    case 'ready':
      return <SetupPanel hub={hub} snapshot={snapshot} />;
    case 'starting':
    case 'searching':
    case 'landing':
      return <FlightPanel snapshot={snapshot} linked={hub.linked} />;
    case 'aborting':
    case 'aborted':
      return <AbortPanel hub={hub} snapshot={snapshot} />;
    case 'complete':
      return <CompletePanel hub={hub} snapshot={snapshot} elapsedMs={elapsedMs} />;
  }
}
