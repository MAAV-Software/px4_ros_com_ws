"""Four-vehicle all-peer priority tests (no ROS transport/dynamics)."""
from itertools import permutations
import pytest
from milestone4_multivehicle.avoidance import Avoidance, POSITION_TOPIC
from test_multivehicle import api, make_controller, position  # noqa: F401 (api is a fixture)


@pytest.mark.parametrize('own,peer', list(permutations(range(4), 2)))
def test_all_priority_pairs(own, peer):
    a = Avoidance(own, 0., 0.)
    for i in range(4):
        a.receive([i, 100., 100.], 0.)
    a.receive([peer, 3., 4.], 0.)
    assert a.conflicts(position()) == (peer > own)
    target = a.update(position(), 0.)
    assert (target is not None) == (peer > own)


def test_multiple_blockers_and_stale_peers_cannot_release_wait():
    a = Avoidance(0, 0., 0.)
    for i in (1, 2, 3): a.receive([i, 1., 0.], 0.)
    a.update(position(), 0.)
    a.update(position(), .5)
    a.update(position(z=-10), .6)
    for i in (2, 3): a.receive([i, 20., 0.], .7)
    assert a.update(position(z=-10), .7) is not None  # instance 1 still near
    a.receive([1, 20., 0.], 2.)
    assert a.update(position(z=-10), 2.) is not None  # other peers stale
    for i in (2, 3): a.receive([i, 20., 0.], 2.)
    assert a.update(position(z=-10), 2.) is None


def test_four_controllers_share_only_position_and_route_commands(api):
    nodes = [make_controller(api, i, vehicle_count=4) for i in range(4)]
    for node in nodes:
        node.on_local_position(position())
        node.publish_global_position()
    for i, node in enumerate(nodes):
        for peer in nodes: node.on_global_position(peer.global_position_pub.messages[-1])
        assert len(node.avoidance.peers) == 3
        node.on_timer(); node.arm()
        assert node.avoidance.state == 'NORMAL'
        assert node.vehicle_command_pub.messages[-1].target_system == i + 1
        assert node.global_position_pub.messages[-1].data[0] == i
    for a, b in permutations(nodes, 2):
        assert set(a.publishers) & set(b.publishers) == {POSITION_TOPIC}


@pytest.mark.parametrize('instance,count', [(4,4), (-1,4), (1.5,4), (0,1), (0,5)])
def test_invalid_fleet_identity(instance,count):
    with pytest.raises(ValueError): Avoidance(instance, 0., 0., count)
