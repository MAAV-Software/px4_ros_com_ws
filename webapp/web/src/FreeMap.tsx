import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { droneColor } from './format';
import type { Area, Drone, Trails } from './types';

// A map with no edges: metres east and north of a fixed reference point, north up, which
// can be dragged, pinched and zoomed. It needs no search area; if there is one, the field's
// outline is drawn on it too. Positions come from the same latitudes and longitudes the hub
// sends, flattened onto a plane around the reference, which is exact to a few centimetres
// over anything a drone can fly here.

const EARTH_RADIUS_M = 6371000;
const MIN_SCALE = 0.02; // pixels per metre: about 20 km across a phone
const MAX_SCALE = 400; // pixels per metre: a few centimetres per pixel
const FIT_MAX_SCALE = 20; // a lone drone isn't fitted closer than this
const FIT_PADDING_PX = 40;
const GRID_STEPS_M = [0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
const FIELD = '#f6f6f1';
const LINE = '#e1e1da';
const INK = '#111111';
const QUIET = '#5c5c57';

type LatLon = [number, number];
interface Point {
  x: number; // metres east of the reference
  y: number; // metres north of the reference
}
interface View {
  cx: number;
  cy: number;
  scale: number;
}

function project(reference: LatLon, lat: number, lon: number): Point {
  const rad = Math.PI / 180;
  return {
    x: (lon - reference[1]) * rad * EARTH_RADIUS_M * Math.cos(reference[0] * rad),
    y: (lat - reference[0]) * rad * EARTH_RADIUS_M,
  };
}

function clampScale(scale: number) {
  return Math.min(MAX_SCALE, Math.max(MIN_SCALE, scale));
}

function droneNumber(id: string) {
  return /(\d+)$/.exec(id)?.[1] ?? '?';
}

export function FreeMap({ drones, trails, area }: { drones: Drone[]; trails: Trails['points']; area: Area | null }) {
  const wrapper = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 320, height: 400 });
  const [view, setView] = useState<View>({ cx: 0, cy: 0, scale: 8 });
  const viewRef = useRef(view);
  viewRef.current = view;
  // Kept in view as drones move, until the user moves the map; Fit turns it back on.
  const [following, setFollowing] = useState(true);
  const pointers = useRef(new Map<number, { x: number; y: number }>());

  // A fixed reference, chosen once, so nothing jumps when another drone appears.
  const reference = useRef<LatLon | null>(null);
  if (reference.current === null) {
    const firstTrail = Object.values(trails).find((points) => points.length > 0)?.[0];
    const firstDrone = drones.find((drone) => drone.global)?.global;
    if (area) reference.current = [area.corners.south_west.lat, area.corners.south_west.lon];
    else if (firstDrone) reference.current = [firstDrone.lat, firstDrone.lon];
    else if (firstTrail) reference.current = [firstTrail[0], firstTrail[1]];
  }
  const ref = reference.current;

  const placed = ref
    ? drones.filter((drone) => drone.global).map((drone) => ({ drone, at: project(ref, drone.global!.lat, drone.global!.lon) }))
    : [];
  const paths = ref
    ? drones
        .filter((drone) => trails[drone.id]?.length)
        .map((drone) => ({ drone, points: trails[drone.id].map(([lat, lon]) => project(ref, lat, lon)) }))
    : [];
  const field =
    ref && area
      ? (['south_west', 'south_east', 'north_east', 'north_west'] as const).map((corner) =>
          project(ref, area.corners[corner].lat, area.corners[corner].lon),
        )
      : [];

  useLayoutEffect(() => {
    const element = wrapper.current;
    if (!element) return;
    const measure = () => setSize({ width: element.clientWidth, height: element.clientHeight });
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  // Everything worth seeing, with a margin: drones, trails and the field.
  const fitted = (): View | null => {
    const all = [...placed.map((p) => p.at), ...paths.flatMap((p) => p.points), ...field];
    if (all.length === 0) return null;
    const xs = all.map((p) => p.x);
    const ys = all.map((p) => p.y);
    const [minX, maxX, minY, maxY] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
    const room = (span: number, pixels: number) => (pixels - 2 * FIT_PADDING_PX) / Math.max(span, 1e-6);
    const scale = Math.min(FIT_MAX_SCALE, room(maxX - minX, size.width), room(maxY - minY, size.height));
    return { cx: (minX + maxX) / 2, cy: (minY + maxY) / 2, scale: clampScale(scale) };
  };

  const fitKey = following ? JSON.stringify([placed.map((p) => p.at), paths.map((p) => p.points.length), field, size]) : '';
  useEffect(() => {
    if (!following) return;
    const next = fitted();
    if (next) setView(next);
    // fitKey covers what fitted() reads.
  }, [fitKey, following]);

  // Screen pixels to metres and back, for the current view.
  const toScreen = (p: Point, v: View) => ({
    x: size.width / 2 + (p.x - v.cx) * v.scale,
    y: size.height / 2 - (p.y - v.cy) * v.scale,
  });
  const toWorld = (sx: number, sy: number, v: View): Point => ({
    x: v.cx + (sx - size.width / 2) / v.scale,
    y: v.cy - (sy - size.height / 2) / v.scale,
  });
  // Zoom by a factor, keeping the spot under (sx, sy) where it is.
  const zoomAt = (factor: number, sx: number, sy: number) => {
    const current = viewRef.current;
    const anchor = toWorld(sx, sy, current);
    const scale = clampScale(current.scale * factor);
    setView({
      scale,
      cx: anchor.x - (sx - size.width / 2) / scale,
      cy: anchor.y + (sy - size.height / 2) / scale,
    });
  };

  useEffect(() => {
    const element = canvas.current;
    const context = element?.getContext('2d');
    if (!element || !context) return;
    const ratio = window.devicePixelRatio || 1;
    element.width = size.width * ratio;
    element.height = size.height * ratio;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.fillStyle = FIELD;
    context.fillRect(0, 0, size.width, size.height);

    // A square grid whose step stays at least 50 px apart, and a scale bar for it.
    const step = GRID_STEPS_M.find((metres) => metres * view.scale >= 50) ?? GRID_STEPS_M[GRID_STEPS_M.length - 1];
    const topLeft = toWorld(0, 0, view);
    const bottomRight = toWorld(size.width, size.height, view);
    context.strokeStyle = LINE;
    context.lineWidth = 1;
    context.beginPath();
    for (let x = Math.floor(topLeft.x / step) * step; x <= bottomRight.x; x += step) {
      const sx = Math.round(toScreen({ x, y: 0 }, view).x) + 0.5;
      context.moveTo(sx, 0);
      context.lineTo(sx, size.height);
    }
    for (let y = Math.floor(bottomRight.y / step) * step; y <= topLeft.y; y += step) {
      const sy = Math.round(toScreen({ x: 0, y }, view).y) + 0.5;
      context.moveTo(0, sy);
      context.lineTo(size.width, sy);
    }
    context.stroke();

    if (field.length === 4) {
      context.setLineDash([6, 4]);
      context.strokeStyle = INK;
      context.lineWidth = 1.5;
      context.beginPath();
      field.forEach((corner, index) => {
        const s = toScreen(corner, view);
        if (index === 0) context.moveTo(s.x, s.y);
        else context.lineTo(s.x, s.y);
      });
      context.closePath();
      context.stroke();
      context.setLineDash([]);
      const label = toScreen(field[0], view);
      context.fillStyle = QUIET;
      context.font = '11px -apple-system, BlinkMacSystemFont, sans-serif';
      context.textAlign = 'left';
      context.textBaseline = 'top';
      context.fillText('Field', label.x + 4, label.y + 4);
    }

    context.lineWidth = 2;
    context.lineJoin = 'round';
    context.lineCap = 'round';
    for (const { drone, points } of paths) {
      context.strokeStyle = droneColor(drone.color);
      context.beginPath();
      points.forEach((point, index) => {
        const s = toScreen(point, view);
        if (index === 0) context.moveTo(s.x, s.y);
        else context.lineTo(s.x, s.y);
      });
      context.stroke();
    }

    for (const { drone, at } of placed) {
      const s = toScreen(at, view);
      context.beginPath();
      context.arc(s.x, s.y, 9, 0, Math.PI * 2);
      context.fillStyle = droneColor(drone.color);
      context.fill();
      context.lineWidth = 2;
      context.strokeStyle = '#ffffff';
      context.stroke();
      context.fillStyle = '#ffffff';
      context.font = 'bold 11px -apple-system, BlinkMacSystemFont, sans-serif';
      context.textAlign = 'center';
      context.textBaseline = 'middle';
      context.fillText(droneNumber(drone.id), s.x, s.y + 0.5);
    }

    const barX = 12;
    const barY = size.height - 14;
    const barLength = step * view.scale;
    context.strokeStyle = INK;
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(barX, barY - 4);
    context.lineTo(barX, barY);
    context.lineTo(barX + barLength, barY);
    context.lineTo(barX + barLength, barY - 4);
    context.stroke();
    context.fillStyle = INK;
    context.font = '11px -apple-system, BlinkMacSystemFont, sans-serif';
    context.textAlign = 'left';
    context.textBaseline = 'bottom';
    context.fillText(`${step} m`, barX + barLength + 6, barY + 2);
    context.textAlign = 'center';
    context.textBaseline = 'top';
    context.fillText('N ↑', size.width / 2, 6);
  });

  // The wheel has to be a non-passive listener for preventDefault to keep the page still.
  useEffect(() => {
    const element = canvas.current;
    if (!element) return;
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      setFollowing(false);
      const box = element.getBoundingClientRect();
      zoomAt(Math.exp(-event.deltaY * 0.0015), event.clientX - box.left, event.clientY - box.top);
    };
    element.addEventListener('wheel', onWheel, { passive: false });
    return () => element.removeEventListener('wheel', onWheel);
  });

  const local = (event: React.PointerEvent) => {
    const box = canvas.current!.getBoundingClientRect();
    return { x: event.clientX - box.left, y: event.clientY - box.top };
  };

  const onPointerDown = (event: React.PointerEvent) => {
    canvas.current?.setPointerCapture(event.pointerId);
    pointers.current.set(event.pointerId, local(event));
    setFollowing(false);
  };

  // One finger drags; two pinch about the point between them, and drag as they move.
  const onPointerMove = (event: React.PointerEvent) => {
    const before = pointers.current.get(event.pointerId);
    if (!before) return;
    const now = local(event);
    const current = viewRef.current;
    if (pointers.current.size === 1) {
      setView({
        ...current,
        cx: current.cx - (now.x - before.x) / current.scale,
        cy: current.cy + (now.y - before.y) / current.scale,
      });
    } else if (pointers.current.size === 2) {
      const other = [...pointers.current.entries()].find(([id]) => id !== event.pointerId)![1];
      const oldGap = Math.hypot(before.x - other.x, before.y - other.y);
      const newGap = Math.hypot(now.x - other.x, now.y - other.y);
      const oldMid = { x: (before.x + other.x) / 2, y: (before.y + other.y) / 2 };
      const newMid = { x: (now.x + other.x) / 2, y: (now.y + other.y) / 2 };
      const anchor = toWorld(oldMid.x, oldMid.y, current);
      const scale = clampScale(current.scale * (oldGap > 0 ? newGap / oldGap : 1));
      setView({
        scale,
        cx: anchor.x - (newMid.x - size.width / 2) / scale,
        cy: anchor.y + (newMid.y - size.height / 2) / scale,
      });
    }
    pointers.current.set(event.pointerId, now);
  };

  const onPointerUp = (event: React.PointerEvent) => {
    pointers.current.delete(event.pointerId);
  };

  const zoomButton = (factor: number) => {
    setFollowing(false);
    zoomAt(factor, size.width / 2, size.height / 2);
  };

  return (
    <div className="free-map" ref={wrapper}>
      {ref === null ? (
        <p className="quiet center">No drone has a position yet.</p>
      ) : (
        <canvas
          ref={canvas}
          className="free-map-canvas"
          style={{ width: size.width, height: size.height }}
          role="img"
          aria-label={`Map of ${placed.length} drones, ${view.scale >= 1 ? '' : 'zoomed out'}`}
          onPointerDown={onPointerDown}
          onPointerMove={onPointerMove}
          onPointerUp={onPointerUp}
          onPointerCancel={onPointerUp}
        />
      )}
      {ref !== null && (
        <div className="free-map-controls">
          <button className="button map-control" aria-label="Zoom in" onClick={() => zoomButton(1.6)}>
            +
          </button>
          <button className="button map-control" aria-label="Zoom out" onClick={() => zoomButton(1 / 1.6)}>
            −
          </button>
          <button
            className={following ? 'button map-control map-control-on' : 'button map-control'}
            aria-pressed={following}
            onClick={() => setFollowing(true)}
          >
            Fit
          </button>
        </div>
      )}
    </div>
  );
}
