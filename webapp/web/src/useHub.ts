import { useCallback, useEffect, useRef, useState } from 'react';
import type { CommandResult, LogLine, Snapshot, Trails } from './types';

const RECONNECT_MS = 1000;
// The hub sends state twice a second. A locked phone or a lost Wi-Fi link doesn't
// always close the socket, so silence counts as a lost link.
const SILENCE_MS = 2500;
const LOG_LINES_KEPT = 1000;

// Not crypto.randomUUID: browsers only offer it over HTTPS, and Phase 1 serves plain HTTP.
function newCommandId() {
  return `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
}

// A bare name is a mission command; a path such as /api/mode is used as it is.
async function postCommand(path: string, body: object): Promise<CommandResult> {
  try {
    const response = await fetch(path.startsWith('/') ? path : `/api/mission/${path}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    return (await response.json()) as CommandResult;
  } catch {
    return { ok: false, state: null, reason: 'No link to the hub' };
  }
}

export function useHub() {
  const [snapshot, setSnapshot] = useState<Snapshot | null>(null);
  const [receivedAt, setReceivedAt] = useState(0);
  const [linked, setLinked] = useState(false);
  const [logs, setLogs] = useState<LogLine[]>([]);
  const [trails, setTrails] = useState<Trails>({ version: -1, points: {} });
  const [abortHeld, setAbortHeld] = useState(false);
  const lastSeq = useRef(0);
  const heldCommandId = useRef('');

  useEffect(() => {
    let socket: WebSocket | null = null;
    let closed = false;
    let reconnectTimer = 0;
    let lastMessageAt = 0;

    const connect = () => {
      if (closed) return;
      const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
      const current = new WebSocket(`${scheme}://${location.host}/ws?after=${lastSeq.current}`);
      socket = current;
      current.onopen = () => {
        lastMessageAt = performance.now();
      };
      current.onmessage = (event) => {
        lastMessageAt = performance.now();
        setLinked(true);
        const message = JSON.parse(event.data);
        if (message.type === 'state') {
          setSnapshot(message as Snapshot);
          setReceivedAt(performance.now());
        } else if (message.type === 'trail') {
          // The whole trail when the hub says so (a new socket, a new run or a new grid), else the new points.
          setTrails((previous) => {
            if (message.full || message.version !== previous.version) {
              return { version: message.version, points: message.points };
            }
            const points = { ...previous.points };
            for (const [id, added] of Object.entries(message.points as Trails['points'])) {
              points[id] = [...(points[id] ?? []), ...added];
            }
            return { version: message.version, points };
          });
        } else if (message.type === 'log') {
          const lines = message.lines as LogLine[];
          // Numbers going backwards means the hub restarted, and this is its whole log.
          const restarted = lines[0].seq <= lastSeq.current;
          lastSeq.current = lines[lines.length - 1].seq;
          setLogs((previous) => (restarted ? lines : [...previous, ...lines].slice(-LOG_LINES_KEPT)));
        }
      };
      current.onclose = () => {
        if (socket === current) {
          setLinked(false);
          reconnectTimer = window.setTimeout(connect, RECONNECT_MS);
        }
      };
    };

    const watchdog = window.setInterval(() => {
      if (socket?.readyState === WebSocket.OPEN && performance.now() - lastMessageAt > SILENCE_MS) {
        socket.close();
      }
    }, 500);

    connect();
    return () => {
      closed = true;
      window.clearTimeout(reconnectTimer);
      window.clearInterval(watchdog);
      socket?.close();
    };
  }, []);

  const command = useCallback(
    (path: string, body: object = {}) => postCommand(path, { command_id: newCommandId(), ...body }),
    [],
  );

  // A tap with no link is held and sent on reconnect, with the same command_id
  // throughout so it can only fire once. The operator can cancel it while it waits.
  const sendHeldAbort = useCallback(async () => {
    const result = await postCommand('abort', { command_id: heldCommandId.current });
    if (result.state !== null) setAbortHeld(false);
    return result;
  }, []);

  const abort = useCallback(() => {
    heldCommandId.current = newCommandId();
    setAbortHeld(true);
    void sendHeldAbort();
  }, [sendHeldAbort]);

  const cancelHeldAbort = useCallback(() => setAbortHeld(false), []);

  useEffect(() => {
    if (!abortHeld || !linked) return;
    void sendHeldAbort();
    const retry = window.setInterval(() => void sendHeldAbort(), 1000);
    return () => window.clearInterval(retry);
  }, [abortHeld, linked, sendHeldAbort]);

  return { snapshot, receivedAt, linked, logs, trails, command, abort, abortHeld, cancelHeldAbort };
}

export type Hub = ReturnType<typeof useHub>;

// Keeps a value that was true `receivedAt` moving forward by the time since then.
export function useTicker(intervalMs: number) {
  const [, setTick] = useState(0);
  useEffect(() => {
    const timer = window.setInterval(() => setTick((value) => value + 1), intervalMs);
    return () => window.clearInterval(timer);
  }, [intervalMs]);
  return performance.now();
}
