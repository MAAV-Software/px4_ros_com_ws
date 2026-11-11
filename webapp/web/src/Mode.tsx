import { useState } from 'react';
import { MODE_LABEL } from './format';
import type { Hub } from './useHub';
import type { Mode, Snapshot } from './types';

const MODES: Mode[] = ['sitl', 'hitl'];
const CHANGEABLE = ['idle', 'preflight', 'ready', 'complete'];

const MODE_HINT: Record<Mode, string> = {
  sitl: "Simulated drones, started on the hub's computer from the Fleet tab.",
  hitl: "Real drones. Each one's agent, PX4 link and RC have to be up before it passes its checks.",
};

// Which drones a launch goes to. The launch files are the same either way.
export function ModeToggle({ hub, snapshot }: { hub: Hub; snapshot: Snapshot }) {
  const [refused, setRefused] = useState('');
  const locked = !CHANGEABLE.includes(snapshot.state);

  const choose = async (mode: Mode) => {
    if (mode === snapshot.mode) return;
    const result = await hub.command('/api/mode', { mode });
    setRefused(result.ok ? '' : result.reason);
  };

  return (
    <div className="field">
      <span className="field-label">Launching in</span>
      <div className="segmented" role="radiogroup" aria-label="Launching in">
        {MODES.map((mode) => (
          <button
            key={mode}
            role="radio"
            aria-checked={snapshot.mode === mode}
            className={snapshot.mode === mode ? `segment segment-on segment-${mode}` : 'segment'}
            disabled={!hub.linked || locked}
            onClick={() => void choose(mode)}
          >
            {MODE_LABEL[mode]}
          </button>
        ))}
      </div>
      <span className="quiet small">{locked ? 'Fixed until this run is over.' : MODE_HINT[snapshot.mode]}</span>
      {refused && (
        <p className="banner banner-warn" role="status">
          {refused}
        </p>
      )}
    </div>
  );
}
