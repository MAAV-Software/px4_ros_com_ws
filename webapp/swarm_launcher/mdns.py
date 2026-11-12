"""Announce the hub as maavnetwork.local, so a phone can type that instead of an address.

mDNS needs no DNS server: iPhones and Android 12+ ask the network, and the hub answers
through avahi, which Ubuntu runs anyway. Each address is announced only on the network it
belongs to, so a phone on Drone 1's Wi-Fi gets the Wi-Fi address and not, say, the mesh
radio's. Addresses are looked at again every so often, since a hotspot can come up after
the hub does.

The name ends in .local on purpose: that ending is reserved for mDNS and never goes to the
internet, so a phone can't be sent to someone else's site under the same name.
"""
import socket
import subprocess

# From avahi-common/defs.h.
AVAHI_PROTO_INET = 0
AVAHI_PROTO_INET6 = 1
AVAHI_PUBLISH_NO_REVERSE = 16
AVAHI_ENTRY_GROUP_COLLISION = 3
AVAHI_ENTRY_GROUP_FAILURE = 4
DEFAULT_NAME = 'maavnetwork'


class MdnsError(Exception):
    pass


def interface_addresses():
    """{(interface, address)} for every network this machine is on, loopback aside: its IPv4
    addresses, and one kind of IPv6 address (global if it has one, else link-local).

    IPv6 matters even though the hub listens on IPv4 only. A name with no IPv6 record makes
    lookups that ask for both, as browsers on a Mac do, wait 5 s for an answer that never
    comes. With one, the IPv6 attempt is refused at once and the client moves to IPv4.
    """
    try:
        output = subprocess.run(['ip', '-o', 'addr', 'show'], capture_output=True, text=True,
                                timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return set()
    return parse_ip_addr(output)


def parse_ip_addr(output):
    ipv4, global6, link6 = set(), {}, {}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) < 4 or fields[1] == 'lo':
            continue
        interface, family, address = fields[1], fields[2], fields[3].split('/')[0]
        scope = fields[fields.index('scope') + 1] if 'scope' in fields else ''
        if family == 'inet':
            ipv4.add((interface, address))
        elif family == 'inet6' and scope == 'global':
            global6.setdefault(interface, set()).add(address)
        elif family == 'inet6' and scope == 'link':
            link6.setdefault(interface, set()).add(address)
    addresses = set(ipv4)
    for interface in {i for i, _ in ipv4}:
        for address in global6.get(interface) or link6.get(interface) or ():
            addresses.add((interface, address))
    return addresses


def connect_avahi():
    """A fresh avahi entry group over D-Bus, or MdnsError saying why there can't be one."""
    try:
        import dbus
    except ImportError:
        raise MdnsError('python3-dbus is not installed') from None
    try:
        bus = dbus.SystemBus()
        server = dbus.Interface(bus.get_object('org.freedesktop.Avahi', '/'), 'org.freedesktop.Avahi.Server')
        path = server.EntryGroupNew()
        return dbus.Interface(bus.get_object('org.freedesktop.Avahi', path), 'org.freedesktop.Avahi.EntryGroup')
    except dbus.DBusException as error:
        raise MdnsError(f'avahi is not running ({error.get_dbus_name()})') from None


class MdnsName:
    def __init__(self, name=DEFAULT_NAME, addresses=interface_addresses, connect=connect_avahi,
                 index=socket.if_nametoindex):
        self.name = f'{name}.local'
        self.addresses = addresses
        self.connect = connect
        self.index = index
        self.group = None
        self.published = None
        self.problem = ''

    def refresh(self):
        """Announce the current addresses if they changed. Returns a line for the hub's log, or ''."""
        addresses = self.addresses()
        if self.group is not None:
            state = self.group.GetState()
            if state in (AVAHI_ENTRY_GROUP_COLLISION, AVAHI_ENTRY_GROUP_FAILURE):
                self.problem = f'another device on the network already uses {self.name}'
                self.published = None
                return f'cannot announce {self.name}: {self.problem}'
        if addresses == self.published:
            return ''
        try:
            if self.group is None:
                self.group = self.connect()
            else:
                self.group.Reset()
            for interface, address in sorted(addresses):
                protocol = AVAHI_PROTO_INET6 if ':' in address else AVAHI_PROTO_INET
                self.group.AddAddress(self.index(interface), protocol, AVAHI_PUBLISH_NO_REVERSE,
                                      self.name, address)
            if addresses:
                self.group.Commit()
        except MdnsError as error:
            self.problem = str(error)
            return f'cannot announce {self.name}: {self.problem}'
        except Exception as error:  # A D-Bus error from avahi, with its own explanation.
            self.problem = str(error)
            self.group = None
            return f'cannot announce {self.name}: {self.problem}'
        self.published = addresses
        self.problem = ''
        where = ', '.join(f'{address} on {interface}' for interface, address in sorted(addresses) if ':' not in address)
        return f'announcing {self.name} at {where}' if addresses else f'{self.name}: no network to announce on'

    def close(self):
        if self.group is not None:
            try:
                self.group.Free()
            except Exception:  # Avahi going away first is fine: the name goes with it.
                pass
            self.group = None
