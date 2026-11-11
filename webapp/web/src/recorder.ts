// Hold-to-talk recording: raw microphone samples, sent to the hub as 16 kHz 16-bit mono WAV,
// which is what Vosk wants. Done here so the hub needs no audio decoder.

const TARGET_RATE = 16000;
let stream: MediaStream | null = null;

// Why this phone can't record, or '' when it can. Browsers only give a page the microphone
// over HTTPS (or on localhost).
export function recordingProblem() {
  if (!window.isSecureContext) return 'Voice needs the HTTPS address.';
  if (!navigator.mediaDevices?.getUserMedia) return 'This browser does not give pages the microphone.';
  return '';
}

export class Recorder {
  private context: AudioContext | null = null;
  private source: MediaStreamAudioSourceNode | null = null;
  private node: ScriptProcessorNode | null = null;
  private chunks: Float32Array[] = [];
  private rate = 48000;

  async start() {
    // Made before any await, while the tap still counts as a user gesture (iOS needs that).
    const Context = window.AudioContext ?? (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
    this.context = new Context();
    // Asked for once, then kept, so later presses start at once.
    stream ??= await navigator.mediaDevices.getUserMedia({
      audio: { channelCount: 1, echoCancellation: true, noiseSuppression: true, autoGainControl: true },
    });
    await this.context.resume();
    this.rate = this.context.sampleRate;
    this.chunks = [];
    this.source = this.context.createMediaStreamSource(stream);
    // ScriptProcessor is old but works everywhere, iOS Safari included.
    this.node = this.context.createScriptProcessor(4096, 1, 1);
    this.node.onaudioprocess = (event) => this.chunks.push(new Float32Array(event.inputBuffer.getChannelData(0)));
    this.source.connect(this.node);
    this.node.connect(this.context.destination);
  }

  async stop(): Promise<Blob> {
    this.source?.disconnect();
    this.node?.disconnect();
    await this.context?.close();
    this.context = null;
    return toWav(this.chunks, this.rate);
  }
}

function toWav(chunks: Float32Array[], rate: number) {
  const total = chunks.reduce((sum, chunk) => sum + chunk.length, 0);
  const input = new Float32Array(total);
  let offset = 0;
  for (const chunk of chunks) {
    input.set(chunk, offset);
    offset += chunk.length;
  }
  // Average each window of input samples down to one at 16 kHz.
  const ratio = rate / TARGET_RATE;
  const count = Math.floor(total / ratio);
  const pcm = new Int16Array(count);
  for (let i = 0; i < count; i += 1) {
    const from = Math.floor(i * ratio);
    const to = Math.min(Math.floor((i + 1) * ratio), total);
    let sum = 0;
    for (let j = from; j < to; j += 1) sum += input[j];
    const value = sum / Math.max(to - from, 1);
    pcm[i] = Math.max(-32768, Math.min(32767, Math.round(value * 32767)));
  }
  const header = new DataView(new ArrayBuffer(44));
  const text = (at: number, value: string) => [...value].forEach((c, i) => header.setUint8(at + i, c.charCodeAt(0)));
  text(0, 'RIFF');
  header.setUint32(4, 36 + pcm.byteLength, true);
  text(8, 'WAVE');
  text(12, 'fmt ');
  header.setUint32(16, 16, true);
  header.setUint16(20, 1, true); // PCM
  header.setUint16(22, 1, true); // mono
  header.setUint32(24, TARGET_RATE, true);
  header.setUint32(28, TARGET_RATE * 2, true);
  header.setUint16(32, 2, true);
  header.setUint16(34, 16, true);
  text(36, 'data');
  header.setUint32(40, pcm.byteLength, true);
  return new Blob([header, pcm], { type: 'audio/wav' });
}
