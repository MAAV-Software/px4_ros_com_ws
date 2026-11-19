"""Bridge Milestone 4's /milestone4/map_state topic into the hub snapshot.

Runs only where the mine_field node publishes (the SITL machine). It subscribes over ROS in a
background thread and hands each decoded overlay to HubCore.set_survey, so the Map tab can
draw the hidden mines, the ones found, the green zone and the planned path. If rclpy is not
importable (a hub with no ROS, or the fake-SITL dev mode) the hub still runs; the map simply
shows no overlay.

HubCore stays free of ROS: this is the only piece that touches rclpy.
"""
import asyncio
import contextlib
import json
import threading

MAP_STATE_TOPIC = '/milestone4/map_state'
POLL_S = 0.25


async def run_survey_bridge(core, topic=MAP_STATE_TOPIC):
    """Listen on `topic` for the app's lifetime, pushing each new overlay into `core`."""
    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError as error:
        core.log('hub', 'INFO', f'map overlay off: ROS is not available here ({error})')
        return

    latest = {'survey': None, 'seen': False}
    lock = threading.Lock()

    def on_message(msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with lock:
            latest['survey'], latest['seen'] = data, True

    try:
        rclpy.init(args=None)
    except RuntimeError:
        pass  # Already initialised elsewhere in this process.
    node = rclpy.create_node('hub_survey_bridge')
    node.create_subscription(String, topic, on_message, 10)

    stop = threading.Event()

    def spin():
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    thread = threading.Thread(target=spin, name='survey_bridge', daemon=True)
    thread.start()
    core.log('hub', 'INFO', f'map overlay on: listening to {topic}')
    last_version = None
    try:
        while True:
            await asyncio.sleep(POLL_S)
            with lock:
                survey, seen = latest['survey'], latest['seen']
            if not seen:
                continue
            # Only touch the snapshot when the overlay actually changes. After a reset clears
            # it, a stale unchanged version is not re-applied; a fresh scan bumps the version.
            version = survey.get('version') if isinstance(survey, dict) else None
            if version != last_version:
                last_version = version
                core.set_survey(survey)
    finally:
        stop.set()
        thread.join(timeout=1.0)
        node.destroy_node()
        with contextlib.suppress(Exception):
            rclpy.shutdown()
