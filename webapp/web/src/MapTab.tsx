import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { droneColor, droneName, INK } from './format';
import { FreeMap } from './FreeMap';
import type { Hub } from './useHub';
import type { Corner, Drone, Snapshot, Trails } from './types';

const COLUMNS = 40;
const ROWS = 150;
const SQUARE_M = 0.6096;
const CORNERS: Corner[] = ['south_west', 'south_east', 'north_east', 'north_west'];
const CORNER_LABEL: Record<Corner, string> = {
  south_west: 'South-west, column 0 row 0',
  south_east: 'South-east',
  north_east: 'North-east',
  north_west: 'North-west',
};
const EDITABLE = ['idle', 'preflight', 'ready', 'complete'];
// Blue, green, red, orange and yellow are kept for the path and mines, as in the simulator.
const QUIET = '#5c5c57';
const FIELD = '#f6f6f1';
const LINE = '#d8d8d0';
const MAJOR = '#b8b8b0';

function droneNumber(id: string) {
  return /(\d+)$/.exec(id)?.[1] ?? '?';
}

// How far outside the field a drone is, in metres, from its fractional column and row.
function offFieldMetres(col: number, row: number) {
  const dx = col < 0 ? -col : col > COLUMNS ? col - COLUMNS : 0;
  const dy = row < 0 ? -row : row > ROWS ? row - ROWS : 0;
  return Math.hypot(dx, dy) * SQUARE_M;
}

export function placeText(drone: Drone) {
  if (!drone.grid) return drone.global ? 'No search area set' : 'No global position';
  const { col, row, on_field: onField } = drone.grid;
  if (!onField) return `${offFieldMetres(col, row).toFixed(0)} m off the field`;
  return `Column ${Math.floor(col)}, row ${Math.floor(row)}`;
}

function FieldCanvas({ drones, trails }: { drones: Drone[]; trails: Trails['points'] }) {
  const wrapper = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const [box, setBox] = useState({ width: 320, height: 480 });

  useLayoutEffect(() => {
    const element = wrapper.current;
    if (!element) return;
    const measure = () => setBox({ width: element.clientWidth, height: element.clientHeight });
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  // The whole field in view: squares as big as the space allows, at least 3 px.
  const margin = { left: 30, bottom: 18, top: 16, right: 8 };
  const square = Math.max(
    3,
    Math.min(
      Math.floor((box.width - margin.left - margin.right) / COLUMNS),
      Math.floor((box.height - margin.top - margin.bottom) / ROWS),
    ),
  );
  const width = margin.left + COLUMNS * square + margin.right;
  const height = margin.top + ROWS * square + margin.bottom;

  useEffect(() => {
    const element = canvas.current;
    const context = element?.getContext('2d');
    if (!element || !context) return;
    const scale = window.devicePixelRatio || 1;
    element.width = width * scale;
    element.height = height * scale;
    context.setTransform(scale, 0, 0, scale, 0, 0);
    context.clearRect(0, 0, width, height);
    const left = margin.left;
    const top = margin.top;
    // Row 0 is the south edge, drawn at the bottom.
    const x = (col: number) => left + col * square;
    const y = (row: number) => top + (ROWS - row) * square;

    context.fillStyle = FIELD;
    context.fillRect(left, top, COLUMNS * square, ROWS * square);
    context.lineWidth = 1;
    for (let col = 0; col <= COLUMNS; col += 1) {
      const major = col % 10 === 0;
      if (!major && square < 6) continue;
      context.strokeStyle = major ? MAJOR : LINE;
      context.beginPath();
      context.moveTo(x(col) + 0.5, top);
      context.lineTo(x(col) + 0.5, top + ROWS * square);
      context.stroke();
    }
    for (let row = 0; row <= ROWS; row += 1) {
      const major = row % 10 === 0;
      if (!major && square < 6) continue;
      context.strokeStyle = major ? MAJOR : LINE;
      context.beginPath();
      context.moveTo(left, y(row) + 0.5);
      context.lineTo(left + COLUMNS * square, y(row) + 0.5);
      context.stroke();
    }
    context.strokeStyle = INK;
    context.strokeRect(left + 0.5, top + 0.5, COLUMNS * square, ROWS * square);

    context.fillStyle = QUIET;
    context.font = '11px -apple-system, BlinkMacSystemFont, sans-serif';
    context.textAlign = 'right';
    context.textBaseline = 'middle';
    for (const row of [0, 50, 100, 149]) context.fillText(String(row), left - 4, y(row) - square / 2);
    context.textAlign = 'center';
    context.textBaseline = 'top';
    for (const col of [0, 10, 20, 30, 39]) context.fillText(String(col), x(col) + square / 2, top + ROWS * square + 4);
    context.textBaseline = 'bottom';
    context.fillText('N', left + (COLUMNS * square) / 2, top - 2);

    // Each drone's trail this mission, in its colour, kept inside the field's edge.
    context.save();
    context.beginPath();
    context.rect(left, top, COLUMNS * square, ROWS * square);
    context.clip();
    context.lineWidth = 2;
    context.lineJoin = 'round';
    context.lineCap = 'round';
    for (const drone of drones) {
      const points = trails[drone.id];
      if (!points || points.length === 0) continue;
      context.strokeStyle = droneColor(drone.color);
      context.beginPath();
      let drawing = false;
      for (const [, , col, row] of points) {
        if (col === null || row === null) {
          drawing = false;
          continue;
        }
        if (drawing) context.lineTo(x(col), y(row));
        else context.moveTo(x(col), y(row));
        drawing = true;
      }
      context.stroke();
    }
    context.restore();

    for (const drone of drones) {
      if (!drone.grid) continue;
      const { col, row, on_field: onField } = drone.grid;
      const color = droneColor(drone.color);
      // A drone off the field sits on its edge, drawn hollow.
      const cx = x(Math.min(Math.max(col, 0), COLUMNS));
      const cy = y(Math.min(Math.max(row, 0), ROWS));
      const radius = 9;
      context.beginPath();
      context.arc(cx, cy, radius, 0, Math.PI * 2);
      context.fillStyle = onField ? color : '#ffffff';
      context.fill();
      // A ring the colour of the field keeps a marker clear of the trails under it.
      context.lineWidth = 2;
      context.strokeStyle = onField ? '#ffffff' : color;
      context.stroke();
      context.fillStyle = onField ? '#ffffff' : color;
      context.font = 'bold 11px -apple-system, BlinkMacSystemFont, sans-serif';
      context.textAlign = 'center';
      context.textBaseline = 'middle';
      context.fillText(droneNumber(drone.id), cx, cy + 0.5);
    }
  }, [drones, trails, width, height, square]);

  return (
    // At least as tall as the canvas, so nothing after it can overlap the field.
    <div className="map-wrap" ref={wrapper} style={{ minHeight: height }}>
      <canvas
        ref={canvas}
        style={{ width, height }}
        role="img"
        aria-label={`Field grid with ${drones.filter((d) => d.grid).length} drones`}
      />
    </div>
  );
}

function parseLatLon(text: string) {
  const parts = text.split(/[\s,]+/).filter(Boolean).map(Number);
  if (parts.length !== 2 || parts.some((n) => !Number.isFinite(n))) return null;
  return { lat: parts[0], lon: parts[1] };
}

function CornersForm({ hub, snapshot, onDone }: { hub: Hub; snapshot: Snapshot; onDone: () => void }) {
  const current = snapshot.area?.corners;
  const [text, setText] = useState<Record<Corner, string>>(() =>
    Object.fromEntries(
      CORNERS.map((corner) => [corner, current ? `${current[corner].lat}, ${current[corner].lon}` : '']),
    ) as Record<Corner, string>,
  );
  const [refused, setRefused] = useState('');
  const locked = !EDITABLE.includes(snapshot.state) || !hub.linked;
  const parsed = CORNERS.map((corner) => parseLatLon(text[corner]));

  const save = async () => {
    if (parsed.some((point) => point === null)) {
      setRefused('Each corner needs a latitude and a longitude, as "47.3979710, 8.5461637".');
      return;
    }
    const corners = Object.fromEntries(CORNERS.map((corner, index) => [corner, parsed[index]]));
    const result = await hub.command('/api/area', { corners });
    setRefused(result.ok ? '' : result.reason);
    if (result.ok) onDone();
  };

  return (
    <section className="box stack">
      <h2>Field corners</h2>
      <p className="quiet small">
        WGS84 degrees from an RTK fix. Phone GPS is not close enough: a square is only 0.61 m wide.
      </p>
      {CORNERS.map((corner, index) => (
        <label key={corner} className="field">
          <span className="field-label small">{CORNER_LABEL[corner]}</span>
          <input
            type="text"
            inputMode="decimal"
            placeholder="latitude, longitude"
            autoCapitalize="off"
            autoCorrect="off"
            spellCheck={false}
            value={text[corner]}
            disabled={locked}
            className={text[corner] && !parsed[index] ? 'input-bad' : undefined}
            onChange={(event) => setText({ ...text, [corner]: event.target.value })}
          />
        </label>
      ))}
      {refused && (
        <p className="banner banner-warn" role="status">
          {refused}
        </p>
      )}
      <div className="row">
        <button className="button button-primary" disabled={locked} onClick={() => void save()}>
          Save corners
        </button>
        {snapshot.area && (
          <button className="button" onClick={onDone}>
            Cancel
          </button>
        )}
      </div>
    </section>
  );
}

type MapView = 'competition' | 'free';
const MAP_VIEWS: { id: MapView; label: string }[] = [
  { id: 'competition', label: 'Competition' },
  { id: 'free', label: 'Free' },
];
const VIEW_KEY = 'swarm-launcher-map-view';

// Which map this phone last showed. Only a convenience, so it may be lost.
function storedView(): MapView {
  try {
    return localStorage.getItem(VIEW_KEY) === 'free' ? 'free' : 'competition';
  } catch {
    return 'competition';
  }
}

function DroneList({ drones, withGrid }: { drones: Drone[]; withGrid: boolean }) {
  return (
    <ul className="plain map-drones">
      {drones.map((drone) => (
        <li key={drone.id} className="check">
          <span className="map-dot" style={{ background: droneColor(drone.color) }}>
            {droneNumber(drone.id)}
          </span>
          <span>
            {droneName(drone.id)}: {withGrid ? placeText(drone) : drone.global ? (drone.activity ?? 'No report yet') : 'No position'}
          </span>
        </li>
      ))}
    </ul>
  );
}

export function MapTab({ hub, snapshot }: { hub: Hub; snapshot: Snapshot }) {
  const [view, setView] = useState<MapView>(storedView);
  const choose = (next: MapView) => {
    setView(next);
    try {
      localStorage.setItem(VIEW_KEY, next);
    } catch {
      // Private browsing and the like: the choice just isn't remembered.
    }
  };
  const drones = snapshot.drones.filter((drone) => drone.connected || drone.in_mission);
  const trailNote = Object.keys(hub.trails.points).length > 0 && (
    <p className="quiet small">Lines show where each drone has been this mission, until New mission.</p>
  );

  return (
    <div className="stack map-tab">
      <div className="segmented" role="radiogroup" aria-label="Map">
        {MAP_VIEWS.map(({ id, label }) => (
          <button
            key={id}
            role="radio"
            aria-checked={view === id}
            className={view === id ? 'segment segment-on segment-plain' : 'segment'}
            onClick={() => choose(id)}
          >
            {label}
          </button>
        ))}
      </div>
      {view === 'competition' ? (
        <CompetitionMap hub={hub} snapshot={snapshot} drones={drones} trailNote={trailNote} />
      ) : (
        <>
          <DroneList drones={drones} withGrid={false} />
          {trailNote}
          <FreeMap drones={drones} trails={hub.trails.points} area={snapshot.area} />
          <p className="quiet small">
            Drag to move, pinch or scroll to zoom. Fit keeps every drone in view.
            {snapshot.area ? ' The dashed outline is the search area.' : ''}
          </p>
        </>
      )}
    </div>
  );
}

// The competition's own grid, set by the search area's corners, as the spec describes.
function CompetitionMap({
  hub,
  snapshot,
  drones,
  trailNote,
}: {
  hub: Hub;
  snapshot: Snapshot;
  drones: Drone[];
  trailNote: React.ReactNode;
}) {
  const [editing, setEditing] = useState(false);
  const [refused, setRefused] = useState('');
  const area = snapshot.area;
  const locked = !EDITABLE.includes(snapshot.state) || !hub.linked;
  // SITL drones have no measured field, so offer one placed around the first that has a position.
  const fitTo =
    snapshot.mode === 'sitl' ? snapshot.drones.find((drone) => drone.sim && drone.connected && drone.global) : undefined;

  const run = async (path: string, body: object = {}) => {
    const result = await hub.command(path, body);
    setRefused(result.ok ? '' : result.reason);
  };

  return (
    <>
      {(!area || editing) && <CornersForm hub={hub} snapshot={snapshot} onDone={() => setEditing(false)} />}
      {fitTo && (!area || editing) && (
        <button
          className="button"
          disabled={locked}
          onClick={() => void run('/api/area/fit', { drone_id: fitTo.id }).then(() => setEditing(false))}
        >
          Place a test field around {droneName(fitTo.id)}
        </button>
      )}
      {refused && (
        <p className="banner banner-warn" role="status">
          {refused}
        </p>
      )}
      {area && !editing && (
        <>
          {area.flags.map((flag) => (
            <p key={flag} className="banner banner-warn">
              Check the corners: {flag}.
            </p>
          ))}
          <DroneList drones={drones} withGrid />
          {trailNote}
          <FieldCanvas drones={drones} trails={hub.trails.points} />
          <div className="row">
            <button className="button" disabled={locked} onClick={() => setEditing(true)}>
              Change corners
            </button>
            <button className="button" disabled={locked} onClick={() => void run('/api/area', { corners: null })}>
              Clear
            </button>
          </div>
        </>
      )}
    </>
  );
}
