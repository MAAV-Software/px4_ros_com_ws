"""Announcing maavnetwork.local, with a stand-in for avahi."""
from swarm_launcher.mdns import (
    AVAHI_ENTRY_GROUP_COLLISION, AVAHI_PROTO_INET, AVAHI_PROTO_INET6, AVAHI_PUBLISH_NO_REVERSE, MdnsError,
    MdnsName, parse_ip_addr)


class FakeGroup:
    def __init__(self):
        self.calls = []
        self.state = 2  # Established.

    def AddAddress(self, interface, protocol, flags, name, address):  # noqa: N802 (avahi's names)
        self.calls.append(('add', interface, protocol, flags, name, address))

    def Commit(self):  # noqa: N802
        self.calls.append(('commit',))

    def Reset(self):  # noqa: N802
        self.calls.append(('reset',))

    def GetState(self):  # noqa: N802
        return self.state

    def Free(self):  # noqa: N802
        self.calls.append(('free',))


def announcer(addresses, group=None):
    current = {'value': addresses}
    group = group or FakeGroup()
    name = MdnsName(addresses=lambda: current['value'], connect=lambda: group,
                    index=lambda interface: {'wlan0': 3, 'mesh0': 4}[interface])
    return name, group, current


def test_each_address_is_announced_on_its_own_network():
    name, group, _ = announcer({('wlan0', '10.42.0.1'), ('mesh0', '192.168.50.1')})
    line = name.refresh()
    assert group.calls == [
        ('add', 4, AVAHI_PROTO_INET, AVAHI_PUBLISH_NO_REVERSE, 'maavnetwork.local', '192.168.50.1'),
        ('add', 3, AVAHI_PROTO_INET, AVAHI_PUBLISH_NO_REVERSE, 'maavnetwork.local', '10.42.0.1'),
        ('commit',),
    ]
    assert line == 'announcing maavnetwork.local at 192.168.50.1 on mesh0, 10.42.0.1 on wlan0'


def test_nothing_changes_while_the_addresses_stay_the_same():
    name, group, _ = announcer({('wlan0', '10.42.0.1')})
    name.refresh()
    group.calls.clear()
    assert name.refresh() == '' and group.calls == []


def test_a_hotspot_coming_up_later_is_announced_too():
    name, group, current = announcer({('mesh0', '192.168.50.1')})
    name.refresh()
    current['value'] = {('mesh0', '192.168.50.1'), ('wlan0', '10.42.0.1')}
    group.calls.clear()
    assert 'wlan0' in name.refresh()
    assert group.calls[0] == ('reset',) and group.calls[-1] == ('commit',) and len(group.calls) == 4


def test_a_name_already_taken_is_reported():
    name, group, _ = announcer({('wlan0', '10.42.0.1')})
    name.refresh()
    group.state = AVAHI_ENTRY_GROUP_COLLISION
    assert name.refresh() == ('cannot announce maavnetwork.local: another device on the network '
                              'already uses maavnetwork.local')


def test_no_avahi_is_reported_not_raised():
    def missing():
        raise MdnsError('avahi is not running (org.freedesktop.DBus.Error.ServiceUnknown)')

    name = MdnsName(addresses=lambda: {('wlan0', '10.42.0.1')}, connect=missing, index=lambda i: 3)
    assert name.refresh() == ('cannot announce maavnetwork.local: avahi is not running '
                              '(org.freedesktop.DBus.Error.ServiceUnknown)')
    assert name.problem


def test_close_withdraws_the_name():
    name, group, _ = announcer({('wlan0', '10.42.0.1')})
    name.refresh()
    name.close()
    assert group.calls[-1] == ('free',) and name.group is None


# `ip -o addr show` from the SITL VM, plus a hotspot with only link-local IPv6 and a
# Docker bridge that has no IPv4-side reason to be announced on its own.
IP_ADDR = r"""
1: lo    inet 127.0.0.1/8 scope host lo\       valid_lft forever preferred_lft forever
1: lo    inet6 ::1/128 scope host \       valid_lft forever preferred_lft forever
2: enp0s1    inet 192.168.64.4/24 metric 100 brd 192.168.64.255 scope global dynamic enp0s1\       valid_lft 2862sec
2: enp0s1    inet6 fd68:b4d3:deff:3014:9ca2:d9ff:fe25:9994/64 scope global dynamic mngtmpaddr noprefixroute \
2: enp0s1    inet6 fe80::9ca2:d9ff:fe25:9994/64 scope link \       valid_lft forever preferred_lft forever
3: wlan0    inet 10.42.0.1/24 brd 10.42.0.255 scope global noprefixroute wlan0\       valid_lft forever
3: wlan0    inet6 fe80::1/64 scope link noprefixroute \       valid_lft forever preferred_lft forever
4: docker0    inet6 fe80::42:1/64 scope link \       valid_lft forever preferred_lft forever
"""


def test_each_network_gets_its_ipv4_and_one_kind_of_ipv6():
    assert parse_ip_addr(IP_ADDR) == {
        ('enp0s1', '192.168.64.4'), ('enp0s1', 'fd68:b4d3:deff:3014:9ca2:d9ff:fe25:9994'),
        ('wlan0', '10.42.0.1'), ('wlan0', 'fe80::1'),
    }


def test_ipv6_addresses_are_announced_as_ipv6():
    name, group, _ = announcer({('wlan0', '10.42.0.1'), ('wlan0', 'fe80::1')})
    line = name.refresh()
    protocols = {call[5]: call[2] for call in group.calls if call[0] == 'add'}
    assert protocols == {'10.42.0.1': AVAHI_PROTO_INET, 'fe80::1': AVAHI_PROTO_INET6}
    assert line == 'announcing maavnetwork.local at 10.42.0.1 on wlan0'
