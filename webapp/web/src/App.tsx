import { useState } from 'react';
import { FleetTab } from './FleetTab';
import { clockText, STATE_LABEL } from './format';
import { LogTab } from './LogTab';
import { MissionTab } from './MissionTab';
import type { Hub } from './useHub';
import { useHub, useTicker } from './useHub';
import type { MissionState } from './types';

const TABS = ['Mission', 'Fleet', 'Log'] as const;
type Tab = (typeof TABS)[number];

const STOPPABLE: MissionState[] = ['preflight', 'ready', 'starting', 'searching', 'landing'];

function AbortBar({ hub }: { hub: Hub }) {
  const state = hub.snapshot?.state;
  if (hub.abortHeld && !hub.linked) {
    return (
      <button className="abort abort-held" onClick={hub.cancelHeldAbort}>
        <span>Abort held, no link: use RC</span>
        <small>Tap to cancel</small>
      </button>
    );
  }
  if (!hub.linked) {
    return (
      <button className="abort" onClick={hub.abort}>
        No link: use RC
      </button>
    );
  }
  const stoppable = state !== undefined && STOPPABLE.includes(state);
  return (
    <button className={stoppable ? 'abort' : 'abort abort-quiet'} disabled={!stoppable} onClick={hub.abort}>
      {state === 'aborting' ? 'Aborting' : state === 'aborted' ? 'Aborted' : 'ABORT'}
    </button>
  );
}

export function App() {
  const hub = useHub();
  const [tab, setTab] = useState<Tab>('Mission');
  const now = useTicker(250);
  const { snapshot, linked, receivedAt } = hub;

  const sinceReceived = now - receivedAt;
  // The timer keeps counting through a dropout, and the hub corrects it on reconnect.
  const elapsedMs = snapshot
    ? snapshot.clock.elapsed_ms + (snapshot.clock.running ? sinceReceived : 0)
    : 0;
  const clockShown = snapshot && (snapshot.clock.running || snapshot.clock.elapsed_ms > 0);
  const stateText = snapshot ? STATE_LABEL[snapshot.state] : 'Connecting';

  return (
    <div className="app">
      <header className="top">
        <strong className="top-name">{snapshot?.mission?.name ?? 'Swarm launcher'}</strong>
        <span className="top-state">
          {!linked && 'No link'}
          {linked && stateText}
          {clockShown ? `, ${clockText(elapsedMs)}` : ''}
        </span>
      </header>
      <main className="body">
        {!snapshot && <p className="quiet center">Connecting to the hub.</p>}
        {snapshot && tab === 'Mission' && <MissionTab hub={hub} snapshot={snapshot} elapsedMs={elapsedMs} />}
        {snapshot && tab === 'Fleet' && (
          <FleetTab drones={snapshot.drones} offlineS={linked ? 0 : sinceReceived / 1000} phoneLinked={linked} />
        )}
        {tab === 'Log' && <LogTab logs={hub.logs} />}
      </main>
      <AbortBar hub={hub} />
      <nav className="tabs" aria-label="Sections">
        {TABS.map((name) => (
          <button
            key={name}
            className={name === tab ? 'tab tab-active' : 'tab'}
            aria-current={name === tab ? 'page' : undefined}
            onClick={() => setTab(name)}
          >
            {name}
          </button>
        ))}
      </nav>
    </div>
  );
}
