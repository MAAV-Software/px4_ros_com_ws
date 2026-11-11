import { useEffect, useState } from 'react';
import type { Hub } from './useHub';
import { useTicker } from './useHub';
import { ABORT_LABEL, clockText, droneName } from './format';
import { ModeToggle } from './Mode';
import type { VoiceCommand } from './VoiceBar';
import { VoiceBar } from './VoiceBar';
import type { Folder, LaunchArg, Snapshot } from './types';

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
  snapshot,
  linked,
  onVoice,
}: {
  missionName: string;
  onConfirm: (pressAgeMs: number) => void;
  onCancel: () => void;
  pressedAt: number;
  snapshot: Snapshot;
  linked: boolean;
  onVoice: (command: VoiceCommand, at: number) => void;
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
        <VoiceBar snapshot={snapshot} linked={linked} onCommand={onVoice} showTyping={false} />
      </div>
    </div>
  );
}

// Set by the launcher on every launch, so the phone doesn't offer them.
const SET_BY_LAUNCHER = ['launch_sitl', 'topic_prefix', 'system_id'];

interface FileInfo {
  file: string;
  foundOn: string[];
  builtOn: string[];
}

interface FolderInfo {
  folder: string;
  package: string;
  files: FileInfo[];
}

// Every connected drone's src in one list: a folder or file shows if any drone has it.
function mergeCatalogs(catalogs: Record<string, Folder[]>): FolderInfo[] {
  const folders = new Map<string, FolderInfo>();
  for (const [droneId, list] of Object.entries(catalogs)) {
    for (const entry of list) {
      let folder = folders.get(entry.folder);
      if (!folder) {
        folder = { folder: entry.folder, package: entry.package, files: [] };
        folders.set(entry.folder, folder);
      }
      for (const { file, built } of entry.files) {
        let info = folder.files.find((known) => known.file === file);
        if (!info) {
          info = { file, foundOn: [], builtOn: [] };
          folder.files.push(info);
        }
        info.foundOn.push(droneId);
        if (built) info.builtOn.push(droneId);
      }
    }
  }
  const sorted = [...folders.values()].sort((a, b) => a.folder.localeCompare(b.folder));
  sorted.forEach((folder) => folder.files.sort((a, b) => a.file.localeCompare(b.file)));
  return sorted;
}

// An emptied field means the file's default, so it isn't sent.
function changedOnly(drafts: Record<string, string>) {
  return Object.fromEntries(Object.entries(drafts).filter(([, value]) => value !== ''));
}

function sameArgs(a: Record<string, string>, b: Record<string, string>) {
  const keys = Object.keys(a);
  return keys.length === Object.keys(b).length && keys.every((key) => a[key] === b[key]);
}

function ArgField({
  arg,
  value,
  onDraft,
  onApply,
}: {
  arg: LaunchArg;
  value: string;
  onDraft: (value: string) => void;
  onApply: (value: string) => void;
}) {
  const boolean = arg.default === 'true' || arg.default === 'false';
  const options = arg.choices.length > 0 ? arg.choices : boolean ? ['true', 'false'] : null;
  const shownDefault = arg.default === null ? 'worked out by the launch' : arg.default === '' ? 'empty' : arg.default;
  return (
    <label className="field arg">
      <span className="arg-name">
        {arg.name}
        {value !== '' && <span className="arg-changed">changed</span>}
      </span>
      {arg.description && <span className="quiet small">{arg.description}</span>}
      {options ? (
        <select value={value} onChange={(event) => onApply(event.target.value)}>
          <option value="">Default ({shownDefault})</option>
          {options.map((option) => (
            <option key={option} value={option}>
              {option}
            </option>
          ))}
        </select>
      ) : (
        <>
          <input
            type="text"
            value={value}
            placeholder="Default"
            autoCapitalize="off"
            autoCorrect="off"
            spellCheck={false}
            enterKeyHint="done"
            onChange={(event) => onDraft(event.target.value)}
            onBlur={(event) => onApply(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === 'Enter') event.currentTarget.blur();
            }}
          />
          <span className="quiet small arg-default">Default: {shownDefault}</span>
        </>
      )}
    </label>
  );
}

function SetupPanel({ hub, snapshot }: { hub: Hub; snapshot: Snapshot }) {
  const [refused, setRefused] = useState('');
  const [pressedAt, setPressedAt] = useState<number | null>(null);
  const mission = snapshot.mission;
  const [folderName, setFolderName] = useState(mission?.folder ?? '');
  const [drafts, setDrafts] = useState<Record<string, string>>(mission?.args ?? {});

  // Follow the hub's pick when it changes, which includes one made on another phone.
  const missionKey = mission ? `${mission.id}|${mission.drones.join()}|${JSON.stringify(mission.args)}` : '';
  useEffect(() => {
    if (!mission) return;
    setFolderName(mission.folder);
    setDrafts(mission.args);
  }, [missionKey]);

  // Only drones of this mode's kind: SITL drones in SITL, real ones in HITL.
  const sitlMode = snapshot.mode === 'sitl';
  const connected = snapshot.drones
    .filter((drone) => drone.connected && drone.sim === sitlMode)
    .map((drone) => drone.id);
  const folders = mergeCatalogs(
    Object.fromEntries(Object.entries(snapshot.catalogs).filter(([id]) => connected.includes(id))),
  );
  const folder = folders.find((entry) => entry.folder === folderName);
  const file = mission && folder && mission.package === folder.package ? mission.file : '';
  const fileInfo = folder?.files.find((entry) => entry.file === file);
  const shownDrones = [...new Set([...connected, ...(mission?.drones ?? [])])].sort();
  const args = (file && snapshot.launch_args?.args) || [];
  const editable = args.filter((arg) => !SET_BY_LAUNCHER.includes(arg.name));
  const changedCount = Object.keys(changedOnly(drafts)).length;
  const dirty = mission !== null && !sameArgs(changedOnly(drafts), mission.args);

  const submit = async (next: { file: string; drones: string[]; args: Record<string, string> }) => {
    if (!folder) return;
    const result = await hub.command('select', { package: folder.package, ...next });
    setRefused(result.ok ? '' : result.reason);
  };

  const pickFolder = (name: string) => {
    setFolderName(name);
    setDrafts({});
  };

  const pickFile = (name: string) => {
    const info = folder?.files.find((entry) => entry.file === name);
    const ready = connected.filter((id) => info?.builtOn.includes(id));
    setDrafts({});
    // With no drone able to run it, check them all anyway so the checks say why.
    void submit({ file: name, drones: ready.length > 0 ? ready : connected, args: {} });
  };

  const toggleDrone = (id: string) => {
    if (!mission) return;
    const drones = mission.drones.includes(id) ? mission.drones.filter((d) => d !== id) : [...mission.drones, id];
    if (drones.length === 0) {
      setRefused('Pick at least one drone');
      return;
    }
    void submit({ file: mission.file, drones, args: changedOnly(drafts) });
  };

  const applyArg = (name: string, value: string) => {
    const next = { ...drafts, [name]: value };
    setDrafts(next);
    if (mission && !sameArgs(changedOnly(next), mission.args)) {
      void submit({ file: mission.file, drones: mission.drones, args: changedOnly(next) });
    }
  };

  const checking = snapshot.state === 'preflight';
  const canStart = snapshot.state === 'ready' && hub.linked && !dirty;

  const empty =
    connected.length === 0
      ? sitlMode
        ? 'No SITL drones yet. Add them on the Fleet tab.'
        : "No real drones connected. Start each drone's agent with this hub's address."
      : folders.length === 0
        ? 'Waiting for the drones to list their launch files.'
        : '';
  if (empty) {
    return (
      <>
        <ModeToggle hub={hub} snapshot={snapshot} />
        <p className="quiet center">{empty}</p>
      </>
    );
  }

  return (
    <>
      <ModeToggle hub={hub} snapshot={snapshot} />
      <label className="field">
        <span className="field-label">Folder</span>
        <select value={folder ? folderName : ''} disabled={!hub.linked} onChange={(event) => pickFolder(event.target.value)}>
          <option value="" disabled>
            Choose a folder in src
          </option>
          {folders.map((entry) => (
            <option key={entry.folder} value={entry.folder}>
              {entry.folder}
            </option>
          ))}
        </select>
      </label>
      {folder && (
        <label className="field">
          <span className="field-label">Launch file</span>
          <select value={file} disabled={!hub.linked} onChange={(event) => pickFile(event.target.value)}>
            <option value="" disabled>
              Choose a launch file
            </option>
            {folder.files.map((entry) => (
              <option key={entry.file} value={entry.file}>
                {entry.file}
                {entry.builtOn.length === 0 ? ' (not built)' : ''}
              </option>
            ))}
          </select>
        </label>
      )}
      {mission && file && (
        <section className="box">
          <h2>Drones</h2>
          <ul className="plain">
            {shownDrones.map((id) => {
              const checked = mission.drones.includes(id);
              const note = !connected.includes(id)
                ? 'not connected'
                : !fileInfo?.foundOn.includes(id)
                  ? 'no such file'
                  : !fileInfo.builtOn.includes(id)
                    ? 'not built'
                    : '';
              return (
                <li key={id}>
                  <label className="choice">
                    <input
                      type="checkbox"
                      checked={checked}
                      disabled={!hub.linked || (!checked && note !== '')}
                      onChange={() => toggleDrone(id)}
                    />
                    <span>
                      {droneName(id)}
                      {note && <span className="quiet">, {note}</span>}
                    </span>
                  </label>
                </li>
              );
            })}
          </ul>
        </section>
      )}
      {mission && file && editable.length > 0 && (
        <details className="box">
          <summary className="strong">
            Launch arguments{changedCount > 0 ? `, ${changedCount} changed` : ''}
          </summary>
          <div className="stack args">
            {editable.map((arg) => (
              <ArgField
                key={arg.name}
                arg={arg}
                value={drafts[arg.name] ?? ''}
                onDraft={(value) => setDrafts({ ...drafts, [arg.name]: value })}
                onApply={(value) => applyArg(arg.name, value)}
              />
            ))}
            <p className="quiet small">
              launch_sitl is always false, and topic_prefix and system_id are set for each drone, so none of
              them is listed.
              {mission.drones.length > 1 && snapshot.launch_args
                ? ` Defaults are as ${droneName(snapshot.launch_args.source)} has them, and each drone uses its own.`
                : ''}
            </p>
          </div>
        </details>
      )}
      <ReasonBanner text={refused || snapshot.reason} />
      <StartChecks snapshot={snapshot} />
      <div className="row">
        <button
          className="button"
          disabled={!mission || !file || checking || !hub.linked}
          onClick={() => mission && void submit({ file: mission.file, drones: mission.drones, args: changedOnly(drafts) })}
        >
          {checking ? 'Checking' : 'Check again'}
        </button>
        <button className="button button-primary" disabled={!canStart} onClick={() => setPressedAt(performance.now())}>
          Start
        </button>
      </div>
      <VoiceBar snapshot={snapshot} linked={hub.linked} onCommand={onVoice} />
      {pressedAt !== null && mission && (
        <StartConfirm
          missionName={mission.name}
          pressedAt={pressedAt}
          onConfirm={(age) => void confirm(age)}
          onCancel={() => setPressedAt(null)}
          snapshot={snapshot}
          linked={hub.linked}
          onVoice={onVoice}
        />
      )}
    </>
  );

  // "Start mission" opens the same confirm a tap on Start does, timed from when the talk
  // button went down. Abort never comes here: the hub has already done it.
  function onVoice(command: VoiceCommand, at: number) {
    if (command === 'start') {
      if (canStart) setPressedAt(at);
      else setRefused(dirty ? 'Wait for the checks on your changes, then say it again.' : 'Not ready to start yet.');
    } else if (command === 'confirm') {
      if (pressedAt !== null) void confirm(performance.now() - pressedAt);
      else setRefused('Nothing to confirm. Say "start mission" first.');
    } else if (command === 'cancel') {
      setPressedAt(null);
    }
  }

  async function confirm(pressAgeMs: number) {
    setPressedAt(null);
    const result = await hub.command('start', { press_age_ms: pressAgeMs });
    setRefused(result.ok ? '' : result.reason);
  }
}

function FlightPanel({ snapshot, linked }: { snapshot: Snapshot; linked: boolean }) {
  const drones = snapshot.drones.filter((drone) => drone.in_mission);
  return (
    <>
      {/* Until the drones are airborne, "abort" by voice works too; the hub then turns voice off. */}
      {snapshot.state === 'starting' && snapshot.voice.available && (
        <VoiceBar snapshot={snapshot} linked={linked} onCommand={() => undefined} showTyping={false} />
      )}
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
  const anyAirborne = snapshot.drones.some((drone) => drone.in_mission && drone.airborne);

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
  const failed = snapshot.drones.filter((drone) => drone.in_mission && drone.failure);
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
