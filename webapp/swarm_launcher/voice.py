"""Voice commands: Vosk on the hub, offline, limited to the launcher's few words.

Hold-to-talk on the phone records a short clip and sends it here as 16-bit mono WAV.
Vosk decodes it against a grammar of just the command phrases, so anything else comes
out as [unk] and does nothing. Start and abort are deliberately unequal: "start mission"
only opens the confirm (the phone runs that, with "confirm" or a tap), while "abort"
acts at once.

The recognizer needs about 300 MB, so it loads on the first clip and the hub unloads it
once drones are airborne. Vosk is optional: without it, or without a model, voice says
why it is off and typed commands still work.
"""
import io
import json
import re
import threading
import wave
from pathlib import Path

PHRASES = ['start mission', 'confirm', 'cancel', 'abort', 'stop search']
GRAMMAR = json.dumps(PHRASES + ['[unk]'])
MAX_CLIP_S = 10.0
DEFAULT_MODEL = '~/vosk-model-small-en-us-0.15'


def parse_command(text):
    """The command a phrase means: 'abort', 'start', 'confirm', 'cancel', or None.

    Abort wins over everything, since a false abort costs a re-run and a missed one is
    worse. Start needs both words, in order, so "start" alone or "mission" alone do nothing.
    """
    words = re.findall(r"[a-z\[\]]+", (text or '').lower())
    joined = ' '.join(words)
    if 'abort' in words or 'stop search' in joined:
        return 'abort'
    if 'start mission' in joined:
        return 'start'
    if 'confirm' in words:
        return 'confirm'
    if 'cancel' in words:
        return 'cancel'
    return None


def read_wav(data):
    """(pcm bytes, sample rate) from a 16-bit mono WAV clip, or ValueError saying what's wrong."""
    try:
        with wave.open(io.BytesIO(data)) as clip:
            if clip.getnchannels() != 1 or clip.getsampwidth() != 2:
                raise ValueError('voice clips must be 16-bit mono WAV')
            rate = clip.getframerate()
            frames = clip.getnframes()
            if not 8000 <= rate <= 48000:
                raise ValueError(f'unexpected sample rate {rate}')
            if frames > rate * MAX_CLIP_S:
                raise ValueError(f'voice clips are limited to {MAX_CLIP_S:.0f} s')
            return clip.readframes(frames), rate
    except (wave.Error, EOFError) as error:
        raise ValueError(f'not a WAV clip: {error}') from None


class VoiceRecognizer:
    """Vosk, loaded when first needed. recognize() blocks for a moment, so the hub runs it
    in a worker thread.
    """

    def __init__(self, model_path=DEFAULT_MODEL):
        self.model_path = Path(model_path).expanduser()
        self.model = None
        self.lock = threading.Lock()

    @property
    def loaded(self):
        return self.model is not None

    def unavailable_reason(self):
        """Why voice can't work on this hub, or '' when it can."""
        try:
            import vosk  # noqa: F401
        except ImportError:
            return 'Vosk is not installed on the hub (pip3 install vosk)'
        if not (self.model_path / 'am').is_dir() and not (self.model_path / 'conf').is_dir():
            return f'no Vosk model at {self.model_path}'
        return ''

    def recognize(self, pcm, rate):
        with self.lock:
            if self.model is None:
                import vosk
                vosk.SetLogLevel(-1)
                self.model = vosk.Model(str(self.model_path))
            recognizer = __import__('vosk').KaldiRecognizer(self.model, rate, GRAMMAR)
            recognizer.AcceptWaveform(pcm)
            text = json.loads(recognizer.FinalResult()).get('text', '')
        # [unk] is the grammar's catch-all for anything that isn't a command.
        return ' '.join(word for word in text.split() if word != '[unk]')

    def unload(self):
        with self.lock:
            self.model = None


class FakeRecognizer:
    """For tests and --fake-sitl: 'hears' the same phrase for every clip."""

    def __init__(self, heard='', reason=''):
        self.heard = heard
        self.reason = reason
        self.model = None

    @property
    def loaded(self):
        return self.model is not None

    def unavailable_reason(self):
        return self.reason

    def recognize(self, pcm, rate):
        self.model = object()
        return self.heard

    def unload(self):
        self.model = None
