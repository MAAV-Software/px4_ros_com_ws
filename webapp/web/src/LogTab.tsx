import { useMemo, useState } from 'react';
import { droneName } from './format';
import type { LogLine } from './types';

const LEVELS = ['All', 'WARN', 'ERROR'] as const;
type LevelFilter = (typeof LEVELS)[number];
const SHOWN_LINES = 300;

function matchesLevel(line: LogLine, filter: LevelFilter) {
  if (filter === 'All') return true;
  if (filter === 'WARN') return line.level === 'WARN' || line.level === 'ERROR';
  return line.level === 'ERROR';
}

export function LogTab({ logs }: { logs: LogLine[] }) {
  const [level, setLevel] = useState<LevelFilter>('All');
  const [source, setSource] = useState('All');
  const sources = useMemo(() => ['All', ...new Set(logs.map((line) => line.source))], [logs]);
  const shown = logs
    .filter((line) => matchesLevel(line, level) && (source === 'All' || line.source === source))
    .slice(-SHOWN_LINES)
    .reverse();

  return (
    <>
      <div className="row log-filters">
        <select aria-label="Filter by source" value={source} onChange={(event) => setSource(event.target.value)}>
          {sources.map((name) => (
            <option key={name} value={name}>
              {name === 'All' ? 'All sources' : droneName(name)}
            </option>
          ))}
        </select>
        <select
          aria-label="Filter by severity"
          value={level}
          onChange={(event) => setLevel(event.target.value as LevelFilter)}
        >
          <option value="All">All levels</option>
          <option value="WARN">Warnings and errors</option>
          <option value="ERROR">Errors only</option>
        </select>
      </div>
      {shown.length === 0 && <p className="quiet center">No log lines yet.</p>}
      <ol className="log plain">
        {shown.map((line) => (
          <li key={line.seq} className={`log-line log-${line.level.toLowerCase()}`}>
            <span className="log-source">{line.source}</span> {line.text}
          </li>
        ))}
      </ol>
    </>
  );
}
