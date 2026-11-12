import { useEffect, useState } from 'react';
import { FleetTab } from './FleetTab';
import { clockText, MODE_LABEL, STATE_LABEL } from './format';
import { LogTab } from './LogTab';
import { MapTab } from './MapTab';
import { MissionTab } from './MissionTab';
import type { Hub } from './useHub';
import { useHub, useTicker } from './useHub';
import type { MissionState } from './types';

const TABS = ['Mission', 'Fleet', 'Map', 'Log'] as const;
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

const ACTIVE: MissionState[] = ['starting', 'searching', 'landing', 'aborting', 'aborted'];

// Keeps the phone's screen on through a run, so the timer and abort stay in view. Only over
// HTTPS, like the microphone; the browser drops it when the page is hidden.
function useWakeLock(active: boolean) {
  useEffect(() => {
    if (!active || !('wakeLock' in navigator)) return;
    let lock: WakeLockSentinel | null = null;
    let stopped = false;
    const take = () => {
      if (document.visibilityState !== 'visible') return;
      navigator.wakeLock.request('screen').then(
        (taken) => {
          if (stopped) void taken.release();
          else lock = taken;
        },
        () => undefined,
      );
    };
    take();
    document.addEventListener('visibilitychange', take);
    return () => {
      stopped = true;
      document.removeEventListener('visibilitychange', take);
      void lock?.release();
    };
  }, [active]);
}

// The HTTPS addresses to try for a page opened over plain HTTP. Without a port (that is, through
// port 80) the hub's 443 is tried first, since serve_on_80_and_443.sh sets both up together.
export function httpsCandidates(location: { hostname: string; port: string }, httpsPort: number) {
  const explicit = `https://${location.hostname}:${httpsPort}`;
  return location.port === '' || location.port === '80' ? [`https://${location.hostname}`, explicit] : [explicit];
}

// On plain HTTP, move to HTTPS if this phone trusts the hub's certificate, for the microphone and
// the wake lock. The check only succeeds once the hub's CA is installed, so a new phone stays put,
// where the talk button shows how to install it.
function useMoveToHttps(httpsPort: number | null) {
  useEffect(() => {
    if (window.isSecureContext || httpsPort === null) return;
    let cancelled = false;
    void (async () => {
      for (const origin of httpsCandidates(location, httpsPort)) {
        const controller = new AbortController();
        const timer = window.setTimeout(() => controller.abort(), 3000);
        try {
          // An opaque reply is enough: it means the connection and its certificate were accepted.
          await fetch(`${origin}/api/state`, { mode: 'no-cors', cache: 'no-store', signal: controller.signal });
          if (!cancelled) location.replace(`${origin}${location.pathname}${location.search}${location.hash}`);
          return;
        } catch {
          // Not trusted yet, or nothing listening there: try the next, then stay on HTTP.
        } finally {
          window.clearTimeout(timer);
        }
      }
    })();
    return () => {
      cancelled = true;
    };
  }, [httpsPort]);
}

export function App() {
  const hub = useHub();
  const [tab, setTab] = useState<Tab>('Mission');
  const now = useTicker(250);
  const { snapshot, linked, receivedAt } = hub;
  useWakeLock(snapshot !== null && ACTIVE.includes(snapshot.state));
  useMoveToHttps(snapshot?.https?.port ?? null);

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
          {snapshot && <span className={`mode-badge mode-badge-${snapshot.mode}`}>{MODE_LABEL[snapshot.mode]}</span>}
          {!linked && 'No link'}
          {linked && stateText}
          {clockShown ? `, ${clockText(elapsedMs)}` : ''}
        </span>
      </header>
      <main className="body">
        {!snapshot && <p className="quiet center">Connecting to the hub.</p>}
        {snapshot && tab === 'Mission' && <MissionTab hub={hub} snapshot={snapshot} elapsedMs={elapsedMs} />}
        {snapshot && tab === 'Fleet' && (
          <FleetTab hub={hub} snapshot={snapshot} offlineS={linked ? 0 : sinceReceived / 1000} />
        )}
        {snapshot && tab === 'Map' && <MapTab hub={hub} snapshot={snapshot} />}
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
