"""HTTPS for the phone, from a certificate authority the hub makes for itself.

Phones only give a page the microphone and the screen wake lock over HTTPS, and the
field has no internet for a public certificate. So the hub makes its own CA once, keeps
it, and issues itself a certificate for the addresses it is reached on. Each operator's
phone installs the CA once, from /ca.crt. On an iPhone that is a profile install, then
turning on full trust for it in Settings > General > About > Certificate Trust Settings.

The CA key never leaves the hub's folder. The hub's own certificate is reissued whenever
its addresses change, without touching the CA, so phones don't need to install it again.
"""
import datetime
import ipaddress
import socket
import subprocess
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID

CA_NAME = 'MAAV swarm launcher local CA'
CA_DAYS = 3650
# Apple refuses server certificates valid for longer than this.
SERVER_DAYS = 397
RENEW_BEFORE_DAYS = 30


def local_hosts(extra=()):
    """Names and IPv4 addresses a phone might use to reach this machine."""
    hosts = {'localhost', '127.0.0.1', *extra}
    name = socket.gethostname()
    hosts |= {name, f'{name.split(".")[0]}.local'}
    try:
        output = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout
        hosts |= set(output.split())
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        hosts |= {info[4][0] for info in socket.getaddrinfo(name, None, socket.AF_INET)}
    except OSError:
        pass
    # The address used to reach other machines, found without sending anything.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(('10.255.255.255', 1))
            hosts.add(probe.getsockname()[0])
    except OSError:
        pass
    return sorted(host for host in hosts if host and ':' not in host)


def ensure_certificates(folder, hosts):
    """(certificate, key, CA certificate) paths, making or renewing whatever is needed."""
    folder = Path(folder).expanduser()
    folder.mkdir(parents=True, exist_ok=True)
    ca_key, ca_cert = _ensure_ca(folder)
    cert_path, key_path = folder / 'hub.crt', folder / 'hub.key'
    if not _server_cert_fits(cert_path, ca_cert, hosts):
        _issue_server_cert(folder, ca_key, ca_cert, hosts)
    return cert_path, key_path, folder / 'ca.crt'


def _now():
    return datetime.datetime.now(datetime.timezone.utc).replace(tzinfo=None)


def expires(cert):
    """When a certificate stops being valid, as a naive UTC time, on old and new cryptography."""
    after = getattr(cert, 'not_valid_after_utc', None)
    return after.replace(tzinfo=None) if after is not None else cert.not_valid_after


def _new_key():
    return rsa.generate_private_key(public_exponent=65537, key_size=2048, backend=default_backend())


def _write_key(path, key):
    path.write_bytes(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
                                       serialization.NoEncryption()))
    path.chmod(0o600)


def _ensure_ca(folder):
    key_path, cert_path = folder / 'ca.key', folder / 'ca.crt'
    if key_path.exists() and cert_path.exists():
        key = serialization.load_pem_private_key(key_path.read_bytes(), None, default_backend())
        return key, x509.load_pem_x509_certificate(cert_path.read_bytes(), default_backend())
    key = _new_key()
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, CA_NAME)])
    cert = (x509.CertificateBuilder()
            .subject_name(name).issuer_name(name).public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(_now() - datetime.timedelta(days=1))
            .not_valid_after(_now() + datetime.timedelta(days=CA_DAYS))
            .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
            .add_extension(x509.KeyUsage(digital_signature=True, key_cert_sign=True, crl_sign=True,
                                         content_commitment=False, key_encipherment=False, data_encipherment=False,
                                         key_agreement=False, encipher_only=False, decipher_only=False), critical=True)
            .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
            .sign(key, hashes.SHA256(), default_backend()))
    _write_key(key_path, key)
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    return key, cert


def _alt_names(hosts):
    names = []
    for host in hosts:
        try:
            names.append(x509.IPAddress(ipaddress.ip_address(host)))
        except ValueError:
            names.append(x509.DNSName(host))
    return names


def _server_cert_fits(path, ca_cert, hosts):
    if not path.exists() or not path.with_suffix('.key').exists():
        return False
    try:
        cert = x509.load_pem_x509_certificate(path.read_bytes(), default_backend())
        names = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    except (ValueError, x509.ExtensionNotFound):
        return False
    have = {str(n) for n in names.get_values_for_type(x509.IPAddress)} | set(names.get_values_for_type(x509.DNSName))
    return (cert.issuer == ca_cert.subject and set(hosts) <= have
            and expires(cert) - _now() > datetime.timedelta(days=RENEW_BEFORE_DAYS))


def _issue_server_cert(folder, ca_key, ca_cert, hosts):
    key = _new_key()
    cert = (x509.CertificateBuilder()
            .subject_name(x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'swarm launcher hub')]))
            .issuer_name(ca_cert.subject).public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(_now() - datetime.timedelta(days=1))
            .not_valid_after(_now() + datetime.timedelta(days=SERVER_DAYS))
            .add_extension(x509.SubjectAlternativeName(_alt_names(hosts)), critical=False)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .add_extension(x509.KeyUsage(digital_signature=True, key_encipherment=True, content_commitment=False,
                                         data_encipherment=False, key_agreement=False, key_cert_sign=False,
                                         crl_sign=False, encipher_only=False, decipher_only=False), critical=True)
            .add_extension(x509.AuthorityKeyIdentifier.from_issuer_public_key(ca_key.public_key()), critical=False)
            .sign(ca_key, hashes.SHA256(), default_backend()))
    _write_key(folder / 'hub.key', key)
    (folder / 'hub.crt').write_bytes(cert.public_bytes(serialization.Encoding.PEM))
