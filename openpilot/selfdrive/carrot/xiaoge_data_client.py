#!/usr/bin/env python3
"""Display the live JSON stream published by xiaoge_data.py."""

import argparse
import json
import socket
import struct
import sys
import time
from typing import Any


DEFAULT_PORT = 7711
MAX_PACKET_SIZE = 1024 * 1024
TRAFFIC_LIGHT_COLORS = {
  0: "NONE",
  1: "RED",
  2: "GREEN",
  3: "YELLOW",
}


def recv_exact(sock: socket.socket, size: int) -> bytes:
  data = bytearray()
  while len(data) < size:
    chunk = sock.recv(size - len(data))
    if not chunk:
      raise ConnectionError("服务器已断开连接")
    data.extend(chunk)
  return bytes(data)


def recv_packet(sock: socket.socket) -> dict[str, Any] | None:
  packet_size = struct.unpack("!I", recv_exact(sock, 4))[0]
  if packet_size == 0:
    return None
  if packet_size > MAX_PACKET_SIZE:
    raise ValueError(f"数据包过大: {packet_size} bytes")

  packet = json.loads(recv_exact(sock, packet_size).decode("utf-8"))
  if not isinstance(packet, dict):
    raise ValueError("JSON 数据包根节点不是对象")
  return packet


def format_value(path: str, value: Any) -> str:
  if path.endswith("trafficLightColor") and isinstance(value, (int, float)):
    color = TRAFFIC_LIGHT_COLORS.get(int(value), "UNKNOWN")
    return f"{int(value)} ({color})"
  if isinstance(value, bool):
    return "true" if value else "false"
  if isinstance(value, float):
    return f"{value:.3f}"
  if value is None:
    return "null"
  if isinstance(value, (list, tuple)):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
  return str(value)


def flatten_fields(value: Any, prefix: str = "") -> list[tuple[str, str]]:
  fields = []
  if isinstance(value, dict):
    for key, child in value.items():
      path = f"{prefix}.{key}" if prefix else str(key)
      fields.extend(flatten_fields(child, path))
  else:
    fields.append((prefix, format_value(prefix, value)))
  return fields


def render_packet(packet: dict[str, Any], address: str, port: int) -> None:
  fields = flatten_fields(packet.get("data", {}))
  name_width = max((len(name) for name, _ in fields), default=10)
  lines = [
    f"Xiaoge 实时数据  {address}:{port}",
    f"sequence={packet.get('sequence', '-')}  source={packet.get('ip', '-')}  timestamp={packet.get('timestamp', '-')}",
    "-" * max(60, name_width + 24),
  ]
  lines.extend(f"{name:<{name_width}} : {value}" for name, value in fields)
  if not fields:
    lines.append("(当前数据为空)")
  lines.append("\n按 Ctrl+C 退出")

  if sys.stdout.isatty():
    sys.stdout.write("\033[2J\033[H")
  sys.stdout.write("\n".join(lines) + "\n")
  sys.stdout.flush()


def run(address: str, port: int) -> None:
  while True:
    try:
      print(f"正在连接 {address}:{port} ...")
      with socket.create_connection((address, port), timeout=5.0) as sock:
        sock.settimeout(None)
        print("连接成功，等待实时数据...")
        while True:
          packet = recv_packet(sock)
          if packet is not None:
            render_packet(packet, address, port)
    except (ConnectionError, OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
      print(f"连接/数据错误: {error}；2 秒后重试。", file=sys.stderr)
      time.sleep(2.0)


def main() -> None:
  parser = argparse.ArgumentParser(description="显示 xiaoge_data.py 发布的实时 JSON 数据")
  parser.add_argument("ip", nargs="?", help="运行 xiaoge_data.py 的设备 IP")
  parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"TCP 端口，默认 {DEFAULT_PORT}")
  args = parser.parse_args()

  address = args.ip.strip() if args.ip else input("请输入设备 IP 地址: ").strip()
  if not address:
    parser.error("IP 地址不能为空")
  if not 1 <= args.port <= 65535:
    parser.error("端口必须在 1 到 65535 之间")

  try:
    run(address, args.port)
  except KeyboardInterrupt:
    print("\n已退出。")


if __name__ == "__main__":
  main()
