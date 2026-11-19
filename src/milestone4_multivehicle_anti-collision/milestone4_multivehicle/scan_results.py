"""A tiny record of each field-scan run, for comparing scan strategies and their path options.

Every run of a scan launch file appends one JSON line here (default
~/launcher_runs/scan_results.jsonl) when its best path is final: which strategy flew,
which mine layout, how many mines it found, how long it took, and how the path scored
against the oracle's optimal one. No ROS, so tests and the web hub can read it directly.

  python3 -m milestone4_multivehicle.scan_results          # print the table
  python3 -m milestone4_multivehicle.scan_results FILE      # from a specific file
"""
import json
import os
import sys
import time

# Columns shown by the table and the web Results tab, in order.
FIELDS = ('time', 'strategy', 'mine_case', 'mine_seed', 'mines_total', 'mines_found',
          'scanned_fraction', 'minutes', 'width_ft', 'length_ft', 'score', 'oracle_score',
          'optimal', 'path_text')


def record(path, fields):
    """Append one run as a JSON line, adding a wall-clock time if none is given. Best effort:
    a write that fails (bad path, read-only) is swallowed so a flight is never interrupted."""
    row = {'time': time.time(), **fields}
    path = os.path.expanduser(path)
    try:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'a') as stream:
            stream.write(json.dumps(row) + '\n')
    except OSError:
        return None
    return row


def load(path):
    """Every recorded run, oldest first; an empty list if the file is missing or unreadable."""
    path = os.path.expanduser(path)
    rows = []
    try:
        with open(path) as stream:
            for line in stream:
                line = line.strip()
                if line:
                    try:
                        rows.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
    except OSError:
        return []
    return rows


def summarize(records):
    """The best-scoring run and the per-strategy best, for a quick comparison."""
    scored = [r for r in records if isinstance(r.get('score'), (int, float))]
    best = max(scored, key=lambda r: r['score'], default=None)
    by_strategy = {}
    for row in scored:
        key = row.get('strategy', '?')
        if key not in by_strategy or row['score'] > by_strategy[key]['score']:
            by_strategy[key] = row
    return {'runs': len(records), 'best': best, 'by_strategy': by_strategy}


def _cell(value):
    if isinstance(value, float):
        return f'{value:.3f}'
    return '' if value is None else str(value)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else '~/launcher_runs/scan_results.jsonl'
    rows = load(path)
    if not rows:
        print(f'no runs recorded in {path}')
        return
    columns = [f for f in FIELDS if f != 'path_text']
    widths = {c: max(len(c), *(len(_cell(r.get(c))) for r in rows)) for c in columns}
    print('  '.join(c.ljust(widths[c]) for c in columns))
    for row in rows:
        print('  '.join(_cell(row.get(c)).ljust(widths[c]) for c in columns))
    best = summarize(rows)['best']
    if best is not None:
        print(f"\nbest: {best.get('strategy')} scored {best['score']:.1f} "
              f"(case {best.get('mine_case') or 'seed ' + str(best.get('mine_seed'))})")


if __name__ == '__main__':
    main()
