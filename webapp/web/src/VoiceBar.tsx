import { useRef, useState } from 'react';
import { Recorder, recordingProblem } from './recorder';
import type { CommandResult, Snapshot } from './types';

export type VoiceCommand = 'start' | 'confirm' | 'cancel' | 'abort';

interface HeardResult extends CommandResult {
  heard?: string;
  command?: VoiceCommand | null;
}

function newCommandId() {
  return `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
}

async function post(url: string, body: BodyInit, type: string): Promise<HeardResult> {
  try {
    const response = await fetch(url, { method: 'POST', headers: { 'Content-Type': type }, body });
    return (await response.json()) as HeardResult;
  } catch {
    return { ok: false, state: null, reason: 'No link to the hub' };
  }
}

// The page is on plain HTTP, where browsers give it no microphone. Once per phone: install
// the hub's own certificate authority, then use the HTTPS address.
function HttpsHelp({ port }: { port: number }) {
  const secure = `https://${location.hostname}:${port}${location.pathname}`;
  return (
    <div className="small quiet">
      <p>Voice needs the HTTPS address. Once per phone:</p>
      <ol className="steps">
        <li>
          <a href="/ca.crt">Download the hub's certificate</a>.
        </li>
        <li>
          iPhone: Settings, General, VPN &amp; Device Management, install it. Then Settings, General, About,
          Certificate Trust Settings, and turn it on. Android: Settings, Security, Install a certificate, CA
          certificate.
        </li>
        <li>
          Open <a href={secure}>{secure}</a>.
        </li>
      </ol>
    </div>
  );
}

// Hold to talk, or type. The hub works out the command; abort it carries out itself, and the
// rest come back to onCommand with the time the button went down, which starts the clock.
export function VoiceBar({
  snapshot,
  linked,
  onCommand,
  showTyping = true,
}: {
  snapshot: Snapshot;
  linked: boolean;
  onCommand: (command: VoiceCommand, pressedAt: number) => void;
  showTyping?: boolean;
}) {
  const recorder = useRef<Recorder | null>(null);
  const pressedAt = useRef(0);
  const [phase, setPhase] = useState<'idle' | 'listening' | 'hearing'>('idle');
  const [said, setSaid] = useState('');
  const [typed, setTyped] = useState('');
  const problem = recordingProblem() || (snapshot.voice.available ? '' : `Voice is off: ${snapshot.voice.reason}.`);

  const show = (result: HeardResult, at: number) => {
    if (!result.ok && result.heard === undefined) {
      setSaid(result.reason);
      return;
    }
    const heard = result.heard ? `"${result.heard}"` : 'nothing';
    setSaid(result.command ? `Heard ${heard}` : `Heard ${heard}, which is not a command`);
    if (result.command) onCommand(result.command, at);
  };

  const press = async (event: React.PointerEvent) => {
    event.preventDefault();
    if (problem || !linked || phase !== 'idle') return;
    pressedAt.current = performance.now();
    recorder.current = new Recorder();
    setPhase('listening');
    setSaid('');
    try {
      await recorder.current.start();
    } catch {
      recorder.current = null;
      setPhase('idle');
      setSaid('No microphone. Allow it for this page in the browser settings.');
    }
  };

  const release = async () => {
    const active = recorder.current;
    if (!active || phase !== 'listening') return;
    recorder.current = null;
    setPhase('hearing');
    const clip = await active.stop();
    if (clip.size < 44 + 16000 * 2 * 0.25) {
      setPhase('idle');
      setSaid('Too short. Hold the button while you speak.');
      return;
    }
    const result = await post(`/api/voice?command_id=${newCommandId()}`, clip, 'audio/wav');
    setPhase('idle');
    show(result, pressedAt.current);
  };

  const send = async () => {
    const text = typed.trim();
    if (!text) return;
    const at = performance.now();
    const result = await post('/api/command', JSON.stringify({ command_id: newCommandId(), text }), 'application/json');
    setTyped('');
    show(result, at);
  };

  return (
    <section className="voice stack">
      <button
        className={phase === 'listening' ? 'button voice-button voice-listening' : 'button voice-button'}
        disabled={Boolean(problem) || !linked}
        onPointerDown={(event) => void press(event)}
        onPointerUp={() => void release()}
        onPointerLeave={() => void release()}
        onPointerCancel={() => void release()}
        onContextMenu={(event) => event.preventDefault()}
      >
        {phase === 'listening' ? 'Listening, let go to send' : phase === 'hearing' ? 'Hearing' : 'Hold to talk'}
      </button>
      {!window.isSecureContext && snapshot.https ? (
        <HttpsHelp port={snapshot.https.port} />
      ) : (
        (said || problem) && <p className="quiet small">{said || problem}</p>
      )}
      {showTyping && (
        <form
          className="row voice-typed"
          onSubmit={(event) => {
            event.preventDefault();
            void send();
          }}
        >
          <input
            type="text"
            value={typed}
            placeholder="or type: start mission, confirm, cancel, abort"
            autoCapitalize="off"
            enterKeyHint="send"
            disabled={!linked}
            onChange={(event) => setTyped(event.target.value)}
          />
          <button className="button voice-send" type="submit" disabled={!linked || !typed.trim()}>
            Send
          </button>
        </form>
      )}
    </section>
  );
}
