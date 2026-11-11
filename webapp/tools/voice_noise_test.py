#!/usr/bin/env python3
"""Phase 4's voice gate: are commands heard through prop noise, with no false starts?

Mixes every clip in a folder with a noise recording at each signal-to-noise ratio, runs
the hub's own recognizer and parser on the result, and reports what was heard. A clip's
name says what it should mean: start_*, confirm_*, cancel_*, abort_*, stop_search_*, or
other_* for anything that must do nothing.

  python3 tools/voice_noise_test.py --clips test/voice_clips --noise props.wav --snr 20 10 5 0

Record the real thing for the gate: the team saying each command, several times each,
and the props at flying power from where the operator stands, all as 16-bit mono WAV
(any rate; the noise is resampled crudely if it differs). Without --noise it uses
synthetic noise, which only shows the tool works.

Exits 1 on any false start, or if a command's accuracy at any ratio is below --min-accuracy.
"""
import argparse
import math
import random
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from swarm_launcher.voice import DEFAULT_MODEL, VoiceRecognizer, parse_command, read_wav  # noqa: E402

EXPECTED = (('stop_search', 'abort'), ('start', 'start'), ('confirm', 'confirm'),
            ('cancel', 'cancel'), ('abort', 'abort'), ('other', None))


def expected_for(name):
    for prefix, command in EXPECTED:
        if name.startswith(prefix):
            return command
    raise SystemExit(f'{name}: name must start with one of {", ".join(p for p, _ in EXPECTED)}')


def samples(pcm):
    return list(struct.unpack(f'<{len(pcm) // 2}h', pcm))


def to_pcm(values):
    return struct.pack(f'<{len(values)}h', *(max(-32768, min(32767, int(v))) for v in values))


def rms(values):
    return math.sqrt(sum(v * v for v in values) / max(len(values), 1)) or 1.0


def resample(values, rate, target):
    if rate == target:
        return values
    step = rate / target
    return [values[int(i * step)] for i in range(int(len(values) / step))]


def synthetic_noise(rate, seconds, seed):
    """Brown-ish noise with a buzz at a prop-like blade rate. Not a substitute for a recording."""
    rng = random.Random(seed)
    level, out = 0.0, []
    for i in range(int(rate * seconds)):
        level = 0.98 * level + rng.uniform(-1, 1)
        buzz = math.sin(2 * math.pi * 180 * i / rate) + 0.5 * math.sin(2 * math.pi * 360 * i / rate)
        out.append(800 * level + 1500 * buzz)
    return out


def mix(speech, noise, snr_db, rng):
    start = rng.randrange(0, max(len(noise) - len(speech), 1))
    piece = noise[start:start + len(speech)]
    piece += [0.0] * (len(speech) - len(piece))
    gain = rms(speech) / (rms(piece) * 10 ** (snr_db / 20))
    return [s + gain * n for s, n in zip(speech, piece)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--clips', required=True, help='folder of command clips (16-bit mono WAV)')
    parser.add_argument('--noise', help='prop-noise recording (16-bit mono WAV); synthetic if left out')
    parser.add_argument('--snr', type=float, nargs='+', default=[20, 10, 5, 0], help='speech-to-noise ratios, dB')
    parser.add_argument('--model', default=DEFAULT_MODEL)
    parser.add_argument('--min-accuracy', type=float, default=0.9)
    parser.add_argument('--seed', type=int, default=1)
    args = parser.parse_args()

    recognizer = VoiceRecognizer(args.model)
    reason = recognizer.unavailable_reason()
    if reason:
        raise SystemExit(reason)
    clips = sorted(Path(args.clips).glob('*.wav'))
    if not clips:
        raise SystemExit(f'no .wav clips in {args.clips}')
    rng = random.Random(args.seed)
    noise_rate = None
    if args.noise:
        noise_pcm, noise_rate = read_wav_any(Path(args.noise).read_bytes())
        noise = samples(noise_pcm)

    results = {}  # (expected, snr) -> [correct, total]
    false_starts = []
    for snr in [None] + list(args.snr):
        for clip in clips:
            pcm, rate = read_wav(clip.read_bytes())
            speech = samples(pcm)
            if snr is not None:
                if args.noise:
                    source = resample(noise, noise_rate, rate)
                else:
                    source = synthetic_noise(rate, 30, args.seed)
                speech = mix(speech, source, snr, rng)
            heard = recognizer.recognize(to_pcm(speech), rate)
            command = parse_command(heard)
            want = expected_for(clip.stem)
            key = (want or 'nothing', 'clean' if snr is None else f'{snr:g} dB')
            tally = results.setdefault(key, [0, 0])
            tally[0] += command == want
            tally[1] += 1
            if command == 'start' and want != 'start':
                false_starts.append(f'{clip.name} at {key[1]} heard as "{heard}"')
            if command != want:
                print(f'  miss: {clip.name} at {key[1]}: heard "{heard}" ({command}), wanted {want}')

    columns = ['clean'] + [f'{snr:g} dB' for snr in args.snr]
    print(f"\n{'should mean':<12}" + ''.join(f'{c:>10}' for c in columns))
    failed = False
    for want in sorted({key[0] for key in results}):
        row = f'{want:<12}'
        for column in columns:
            correct, total = results.get((want, column), (0, 0))
            row += f'{correct:>6}/{total:<3}'
            failed |= total > 0 and correct / total < args.min_accuracy
        print(row)
    print(f'\nfalse starts: {len(false_starts)}')
    for line in false_starts:
        print(f'  {line}')
    print('noise: ' + (args.noise or 'synthetic (record real prop noise for the gate)'))
    sys.exit(1 if false_starts or failed else 0)


def read_wav_any(data):
    """Like read_wav, but no length limit, for a long noise recording."""
    import io
    import wave
    with wave.open(io.BytesIO(data)) as clip:
        if clip.getnchannels() != 1 or clip.getsampwidth() != 2:
            raise SystemExit('the noise recording must be 16-bit mono WAV')
        return clip.readframes(clip.getnframes()), clip.getframerate()


if __name__ == '__main__':
    main()
