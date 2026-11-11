"""The hub's own certificate authority and HTTPS certificate."""
import datetime

import pytest

pytest.importorskip('cryptography')

from cryptography import x509  # noqa: E402
from cryptography.hazmat.backends import default_backend  # noqa: E402
from cryptography.hazmat.primitives.asymmetric import padding  # noqa: E402

from swarm_launcher.tls import SERVER_DAYS, ensure_certificates, expires, local_hosts  # noqa: E402


def load(path):
    return x509.load_pem_x509_certificate(path.read_bytes(), default_backend())


def names(cert):
    alt = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    return {str(n) for n in alt.get_values_for_type(x509.IPAddress)} | set(alt.get_values_for_type(x509.DNSName))


def test_the_certificate_is_signed_by_the_hubs_own_ca_and_names_its_addresses(tmp_path):
    cert_path, key_path, ca_path = ensure_certificates(tmp_path, ['localhost', '192.168.64.4'])
    cert, ca = load(cert_path), load(ca_path)
    assert names(cert) == {'localhost', '192.168.64.4'}
    assert cert.issuer == ca.subject
    ca.public_key().verify(cert.signature, cert.tbs_certificate_bytes, padding.PKCS1v15(), cert.signature_hash_algorithm)
    assert ca.extensions.get_extension_for_class(x509.BasicConstraints).value.ca
    # Apple refuses longer-lived server certificates.
    assert expires(cert) - datetime.datetime.now(datetime.timezone.utc).replace(tzinfo=None) <= datetime.timedelta(days=SERVER_DAYS)
    assert oct(key_path.stat().st_mode & 0o777) == '0o600'
    assert oct((tmp_path / 'ca.key').stat().st_mode & 0o777) == '0o600'


def test_a_new_address_reissues_the_certificate_but_keeps_the_ca(tmp_path):
    cert_path, _, ca_path = ensure_certificates(tmp_path, ['localhost'])
    ca_before, cert_before = ca_path.read_bytes(), cert_path.read_bytes()
    ensure_certificates(tmp_path, ['localhost'])
    assert cert_path.read_bytes() == cert_before  # Nothing changed, nothing reissued.
    ensure_certificates(tmp_path, ['localhost', '10.42.0.1'])
    assert ca_path.read_bytes() == ca_before  # Phones keep trusting it.
    assert cert_path.read_bytes() != cert_before
    assert '10.42.0.1' in names(load(cert_path))


def test_local_hosts_includes_localhost_and_extras():
    hosts = local_hosts(['drone1.field'])
    assert {'localhost', '127.0.0.1', 'drone1.field'} <= set(hosts)
    assert all(':' not in host for host in hosts)
