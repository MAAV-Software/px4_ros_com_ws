"""Voice: what phrases mean, reading clips, and the hub's rules for them."""
import io
import os
import wave
from pathlib import Path

import pytest

from swarm_launcher.hub_core import IDLE, SEARCHING, HubCore
from swarm_launcher.voice import FakeRecognizer, VoiceRecognizer, parse_command, read_wav

CLIPS = Path(__file__).parent / 'voice_clips'


@pytest.mark.parametrize('text, command', [
    ('start mission', 'start'),
    ('START MISSION', 'start'),
    ('start', None),
    ('mission', None),
    ('mission start', None),
    ('confirm', 'confirm'),
    ('cancel', 'cancel'),
    ('abort', 'abort'),
    ('stop search', 'abort'),
    ('stop', None),
    ('start mission abort', 'abort'),
    ('', None),
    (None, None),
    ('[unk]', None),
])
def test_parse_command(text, command):
    assert parse_command(text) == command


def wav(seconds=0.5, rate=16000, channels=1, width=2):
    data = io.BytesIO()
    with wave.open(data, 'wb') as clip:
        clip.setnchannels(channels)
        clip.setsampwidth(width)
        clip.setframerate(rate)
        clip.writeframes(b'\0' * int(seconds * rate) * channels * width)
    return data.getvalue()


def test_read_wav():
    pcm, rate = read_wav(wav())
    assert rate == 16000 and len(pcm) == 16000


@pytest.mark.parametrize('clip, message', [
    (b'not audio', 'not a WAV clip'),
    (wav(channels=2), '16-bit mono'),
    (wav(width=1), '16-bit mono'),
    (wav(seconds=11), 'limited to 10 s'),
    (wav(rate=4000), 'unexpected sample rate'),
])
def test_bad_clips_are_refused(clip, message):
    with pytest.raises(ValueError, match=message):
        read_wav(clip)


def test_the_hub_aborts_on_hearing_abort_and_passes_the_rest_back(clock):
    core = HubCore(now=clock, voice=FakeRecognizer())
    assert core.voice_refusal() is None
    result = core.spoken('start mission')
    assert result['ok'] and result['command'] == 'start' and result['heard'] == 'start mission'
    assert core.spoken('what time is it')['command'] is None
    core.state = SEARCHING
    core.mission = {'drones': []}
    core.abort_results = {}
    result = core.spoken('stop search')
    assert result['command'] == 'abort' and core.state == 'aborting'


def test_voice_is_off_while_drones_fly_and_the_model_unloads(clock):
    voice = FakeRecognizer()
    core = HubCore(now=clock, voice=voice)
    voice.recognize(b'', 16000)
    assert voice.loaded
    core.agent_connected('d1', lambda message: None)
    core.agent_message('d1', {'type': 'status', 'airborne': True})
    assert core.voice_refusal()['reason'] == 'voice is off while drones fly. Use the abort button'
    assert core.snapshot()['voice'] == {'available': False,
                                        'reason': 'voice is off while drones fly. Use the abort button'}
    core.tick()
    assert not voice.loaded


def test_a_hub_without_vosk_says_why(clock, tmp_path):
    core = HubCore(now=clock, voice=VoiceRecognizer(tmp_path / 'no-model'))
    reason = core.voice_refusal()['reason']
    assert 'Vosk is not installed' in reason or 'no Vosk model' in reason
    assert HubCore(now=clock).voice_refusal()['reason'] == 'voice is not set up on this hub'
    assert core.state == IDLE


MODEL = Path(os.environ.get('VOSK_MODEL', '~/vosk-model-small-en-us-0.15')).expanduser()


@pytest.mark.skipif(not MODEL.is_dir(), reason='needs a Vosk model (set VOSK_MODEL)')
@pytest.mark.parametrize('name, command', [
    ('start_mission', 'start'), ('confirm', 'confirm'), ('cancel', 'cancel'),
    ('abort', 'abort'), ('stop_search', 'abort'), ('other_hello', None), ('other_start', None),
    ('other_mission', None), ('other_stop', None),
])
def test_vosk_hears_the_commands(name, command):
    """Clean synthetic speech only. The prop-noise test is tools/voice_noise_test.py."""
    pytest.importorskip('vosk')
    recognizer = VoiceRecognizer(MODEL)
    pcm, rate = read_wav((CLIPS / f'{name}.wav').read_bytes())
    assert parse_command(recognizer.recognize(pcm, rate)) == command
