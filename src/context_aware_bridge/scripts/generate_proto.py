#!/usr/bin/env python3
"""
generate_proto.py — run this script on RasPi once after cloning to generate messages_pb2.py.

Usage:
    python3 scripts/generate_proto.py

Requires: grpcio-tools  (pip install grpcio-tools)
"""
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
PKG_ROOT   = SCRIPT_DIR.parent  # wheeltec_ros2/src/context_aware_bridge
PROTO_SRC  = PKG_ROOT / 'proto' / 'messages.proto'
OUT_DIR    = PKG_ROOT / 'context_aware_bridge'

# Fallback: look for proto next to the repo root
if not PROTO_SRC.exists():
    PROTO_SRC = PKG_ROOT.parent.parent.parent / 'context-aware' / 'proto' / 'messages.proto'

if not PROTO_SRC.exists():
    print(f'[ERROR] messages.proto not found at {PROTO_SRC}')
    sys.exit(1)

cmd = [
    sys.executable, '-m', 'grpc_tools.protoc',
    f'-I{PROTO_SRC.parent}',
    f'--python_out={OUT_DIR}',
    str(PROTO_SRC),
]
print(f'[generate_proto] Running: {" ".join(str(c) for c in cmd)}')
result = subprocess.run(cmd, check=False)
if result.returncode != 0:
    print('[ERROR] protoc failed. Install: pip install grpcio-tools')
    sys.exit(result.returncode)

print(f'[generate_proto] Generated: {OUT_DIR / "messages_pb2.py"}')
