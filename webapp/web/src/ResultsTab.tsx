import { useCallback, useEffect, useMemo, useState } from 'react';
import type { ScanResult } from './types';

const STRATEGY_LABEL: Record<string, string> = {
  mow: '3 mowers + checker',
  mow4: 'All four lawnmower',
  swarm2x2: '2x2 swarm',
  quadrant: 'Quadrants',
  corridor: 'Corridor',
};

function strategyLabel(id: string) {
  return STRATEGY_LABEL[id] ?? id;
}

function layoutLabel(result: ScanResult) {
  return result.mine_case ? result.mine_case : `seed ${result.mine_seed}`;
}

function num(value: number | null, digits = 0) {
  return value === null || value === undefined ? '-' : value.toFixed(digits);
}

export function ResultsTab() {
  const [results, setResults] = useState<ScanResult[] | null>(null);
  const [error, setError] = useState('');

  const refresh = useCallback(async () => {
    try {
      const response = await fetch('/api/scan/results', { cache: 'no-store' });
      const data = (await response.json()) as { results: ScanResult[] };
      setResults(data.results ?? []);
      setError('');
    } catch {
      setError('Could not reach the hub.');
    }
  }, []);

  useEffect(() => {
    void refresh();
  }, [refresh]);

  // Best score per strategy, so the comparison shows which pattern wins on this field.
  const bestByStrategy = useMemo(() => {
    const best: Record<string, number> = {};
    for (const result of results ?? []) {
      if (typeof result.score === 'number' && (best[result.strategy] === undefined || result.score > best[result.strategy])) {
        best[result.strategy] = result.score;
      }
    }
    return best;
  }, [results]);

  const topScore = useMemo(() => {
    const scores = (results ?? []).map((r) => r.score).filter((s): s is number => typeof s === 'number');
    return scores.length ? Math.max(...scores) : null;
  }, [results]);

  return (
    <>
      <div className="row log-filters">
        <button className="tab" onClick={() => void refresh()}>
          Refresh
        </button>
        <span className="quiet">Each finished scan run appends a result the drones record.</span>
      </div>
      {error && <p className="quiet center">{error}</p>}
      {results !== null && results.length === 0 && !error && (
        <p className="quiet center">No scan runs recorded yet. Fly a scan with mines to compare strategies here.</p>
      )}
      {results !== null && results.length > 0 && (
        <div className="results-wrap">
          <table className="results">
            <thead>
              <tr>
                <th>Strategy</th>
                <th>Field</th>
                <th>Found</th>
                <th>Minutes</th>
                <th>Score</th>
                <th>Oracle</th>
                <th>Path</th>
              </tr>
            </thead>
            <tbody>
              {results.map((result, index) => {
                const isTop = topScore !== null && result.score === topScore;
                const isStrategyBest = typeof result.score === 'number' && result.score === bestByStrategy[result.strategy];
                return (
                  <tr key={index} className={isTop ? 'results-top' : isStrategyBest ? 'results-best' : undefined}>
                    <td>
                      {strategyLabel(result.strategy)}
                      {result.optimal && <span className="results-badge"> optimal</span>}
                    </td>
                    <td>{layoutLabel(result)}</td>
                    <td>
                      {result.mines_found}/{result.mines_total}
                    </td>
                    <td>{num(result.minutes, 1)}</td>
                    <td>{num(result.score, 0)}</td>
                    <td>{num(result.oracle_score, 0)}</td>
                    <td>
                      {result.width_ft} ft wide, {result.length_ft} ft long
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
    </>
  );
}
