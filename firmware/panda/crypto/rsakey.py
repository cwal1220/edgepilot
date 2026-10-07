"""sign.py와 cert_header.py가 쓰는 RSA 키 읽기. 표준 라이브러리만 쓴다(pycryptodome 불필요).

load(path)는 PKCS#1 PEM 개인키(certs/debug)나 OpenSSH 공개키(certs/*.pub)를 읽어
(n, e, d)를 돌려준다. 공개키면 d는 None이다.
"""
import base64
import struct


def _der(data, i):
    """data[i]의 DER 원소 하나: (tag, 값, 그다음 위치)."""
    tag, length = data[i], data[i + 1]
    i += 2
    if length & 0x80:
        count = length & 0x7f
        length = int.from_bytes(data[i:i + count], "big")
        i += count
    return tag, data[i:i + length], i + length


def _openssh_public(text):
    blob = base64.b64decode(text.split()[1])
    fields, i = [], 0
    while i < len(blob):
        (length,) = struct.unpack(">I", blob[i:i + 4])
        fields.append(blob[i + 4:i + 4 + length])
        i += 4 + length
    if fields[0] != b"ssh-rsa":
        raise ValueError("not an ssh-rsa key")
    return int.from_bytes(fields[2], "big"), int.from_bytes(fields[1], "big"), None


def _pkcs1_private(text):
    lines = text.strip().splitlines()
    if lines[0] != "-----BEGIN RSA PRIVATE KEY-----":
        raise ValueError("not a PKCS#1 RSA private key")
    body = base64.b64decode("".join(line for line in lines if not line.startswith("-----")))
    _, sequence, _ = _der(body, 0)
    ints, i = [], 0
    while i < len(sequence):
        _, value, i = _der(sequence, i)
        ints.append(int.from_bytes(value, "big"))
    # RSAPrivateKey: version, n, e, d, p, q, dp, dq, qinv
    return ints[1], ints[2], ints[3]


def load(path):
    with open(path) as f:
        text = f.read()
    if text.startswith("ssh-rsa "):
        return _openssh_public(text)
    return _pkcs1_private(text)
