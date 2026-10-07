#!/usr/bin/env python3
"""cert.h를 쓴다: bootstub의 RSA 서명 검사가 쓰는 꼴로 debug·release 공개키를 담는다.
원래 board/SConscript의 get_key_header와 같은 내용을 낸다.

사용: cert_header.py <out.h> <certs 디렉터리>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rsakey  # noqa: E402


def to_c_uint32(x):
  nums = []
  for _ in range(0x20):
    nums.append(x % (2**32))
    x //= (2**32)
  return "{" + 'U,'.join(map(str, nums)) + "U}"


def key_header(name, certs_dir):
  n, e, _ = rsakey.load(os.path.join(certs_dir, f"{name}.pub"))
  assert n.bit_length() == 1024

  rr = pow(2**1024, 2, n)
  n0inv = 2**32 - pow(n, -1, 2**32)
  return [
    f"RSAPublicKey {name}_rsa_key = {{",
    "  .len = 0x20,",
    f"  .n0inv = {n0inv}U,",
    f"  .n = {to_c_uint32(n)},",
    f"  .rr = {to_c_uint32(rr)},",
    f"  .exponent = {e},",
    "};",
  ]


if __name__ == "__main__":
  out, certs_dir = sys.argv[1], sys.argv[2]
  with open(out, "w") as f:
    for name in ("debug", "release"):
      f.write("\n".join(key_header(name, certs_dir)) + "\n")
