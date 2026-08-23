#!/usr/bin/env python3
"""
SocialNav Studio - Remote SSH Bridge Gateway & Tunnel Daemon
Zero-dependency Python 3 asyncio WebSocket server (RFC 6455) for remote robot management.
Listens on ws://localhost:9092 and manages SSH connections, command execution, and port forwarding.
"""

import asyncio
import base64
import hashlib
import json
import os
import signal
import struct
import subprocess
import sys
import time

PORT = 9092
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

class RemoteSSHSession:
    def __init__(self):
        self.host = ""
        self.port = 22
        self.username = ""
        self.auth_mode = "password" # password or key
        self.password = ""
        self.remote_process = None
        self.tunnel_process = None
        self.interactive_process = None
        self.interactive_task = None
        self.is_running = False
        self.launch_time = None

    def interrupt_interactive(self):
        if self.interactive_process:
            try:
                self.interactive_process.terminate()
            except Exception:
                pass
            self.interactive_process = None
        if self.interactive_task and not self.interactive_task.done():
            self.interactive_task.cancel()
            self.interactive_task = None

    def get_ssh_env(self, password=None):
        env = os.environ.copy()
        pw = password if password is not None else self.password
        if self.auth_mode == "password" and pw:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            askpass_file = os.path.join(script_dir, "askpass.py")
            if not os.path.exists(askpass_file):
                try:
                    with open(askpass_file, "w") as f:
                        f.write('#!/usr/bin/env python3\nimport os\nprint(os.environ.get("SOCIALNAV_SSH_PASS", ""))\n')
                    os.chmod(askpass_file, 0o755)
                except Exception:
                    pass
            env["SSH_ASKPASS"] = askpass_file
            env["SSH_ASKPASS_REQUIRE"] = "force"
            env["SOCIALNAV_SSH_PASS"] = pw
            env["DISPLAY"] = env.get("DISPLAY", ":0")
        return env

    def build_ssh_base_cmd(self):
        cmd = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", "-o", "ConnectTimeout=6"]
        if self.port != 22:
            cmd.extend(["-p", str(self.port)])
        if self.auth_mode == "key" and self.key_path and os.path.exists(self.key_path):
            cmd.extend(["-i", self.key_path])
        target = f"{self.username}@{self.host}" if self.username else self.host
        cmd.append(target)
        return cmd

    def wrap_with_auth(self, cmd_list, password=None):
        pw = password if password is not None else self.password
        if self.auth_mode == "password" and pw:
            sshpass_check = subprocess.run(["which", "sshpass"], capture_output=True, text=True)
            if sshpass_check.returncode == 0:
                return ["sshpass", "-p", pw] + cmd_list
        return cmd_list


session = RemoteSSHSession()
connected_clients = set()


class MinimalWebSocket:
    def __init__(self, reader, writer):
        self.reader = reader
        self.writer = writer
        self.closed = False

    async def handshake(self):
        request_line = await self.reader.readline()
        if not request_line:
            return False

        headers = {}
        while True:
            line = await self.reader.readline()
            if not line or line == b"\r\n":
                break
            parts = line.decode("utf-8", errors="ignore").split(":", 1)
            if len(parts) == 2:
                headers[parts[0].strip().lower()] = parts[1].strip()

        key = headers.get("sec-websocket-key")
        if not key:
            self.writer.write(b"HTTP/1.1 400 Bad Request\r\n\r\n")
            await self.writer.drain()
            return False

        accept_val = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("utf-8")).digest()).decode("utf-8")
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_val}\r\n\r\n"
        )
        self.writer.write(response.encode("utf-8"))
        await self.writer.drain()
        return True

    async def send_text(self, text):
        if self.closed:
            return
        payload = text.encode("utf-8")
        length = len(payload)
        header = bytearray([0x81])  # FIN + Text opcode

        if length <= 125:
            header.append(length)
        elif length <= 65535:
            header.append(126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(127)
            header.extend(struct.pack("!Q", length))

        try:
            self.writer.write(header + payload)
            await self.writer.drain()
        except Exception:
            self.closed = True

    async def read_msg(self):
        while not self.closed:
            try:
                head = await self.reader.readexactly(2)
            except Exception:
                self.closed = True
                return None

            b1, b2 = head[0], head[1]
            opcode = b1 & 0x0F
            is_masked = (b2 & 0x80) != 0
            payload_len = b2 & 0x7F

            if opcode == 0x8:  # Close
                self.closed = True
                return None

            if payload_len == 126:
                ext = await self.reader.readexactly(2)
                payload_len = struct.unpack("!H", ext)[0]
            elif payload_len == 127:
                ext = await self.reader.readexactly(8)
                payload_len = struct.unpack("!Q", ext)[0]

            mask = await self.reader.readexactly(4) if is_masked else None
            data = await self.reader.readexactly(payload_len)

            if is_masked and mask:
                unmasked = bytearray(len(data))
                for i in range(len(data)):
                    unmasked[i] = data[i] ^ mask[i % 4]
                data = bytes(unmasked)

            if opcode == 0x1:  # Text frame
                return data.decode("utf-8", errors="ignore")
            elif opcode == 0x9:  # Ping -> reply Pong
                self.writer.write(bytearray([0x8A, 0x00]))
                await self.writer.drain()

        return None


async def broadcast_json(data_dict):
    msg_str = json.dumps(data_dict)
    dead_clients = set()
    for client in connected_clients:
        try:
            await client.send_text(msg_str)
        except Exception:
            dead_clients.add(client)
    connected_clients.difference_update(dead_clients)


def sync_session_from_msg(msg_obj):
    if msg_obj.get("host"):
        session.host = msg_obj.get("host", "").strip()
    if msg_obj.get("port"):
        session.port = int(msg_obj.get("port", 22))
    if msg_obj.get("username"):
        session.username = msg_obj.get("username", "").strip()
    if msg_obj.get("auth_mode"):
        session.auth_mode = msg_obj.get("auth_mode", "password")
    if "password" in msg_obj and msg_obj.get("password") is not None:
        session.password = msg_obj.get("password", "")
    if msg_obj.get("key_path"):
        session.key_path = msg_obj.get("key_path", "")


async def handle_ssh_request(ws, msg_obj):
    op = msg_obj.get("op", "")
    sync_session_from_msg(msg_obj)

    if op == "test_connection":
        host = session.host
        port = session.port
        user = session.username
        auth_mode = session.auth_mode
        password = session.password
        key_path = session.key_path

        await ws.send_text(json.dumps({
            "op": "terminal_log",
            "type": "info",
            "text": f"\x1b[36m[SSH-PROBE]\x1b[0m Pinging {user}@{host}:{port}..."
        }))

        t0 = time.time()
        ssh_cmd = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
                   "-o", "ConnectTimeout=5", "-o", "BatchMode=" + ("yes" if auth_mode == "key" or not password else "no")]
        if port != 22:
            ssh_cmd.extend(["-p", str(port)])
        if auth_mode == "key" and key_path:
            ssh_cmd.extend(["-i", key_path])
        target = f"{user}@{host}" if user else host
        ssh_cmd.extend([target, "echo SOCIALNAV_SSH_OK && hostname && uname -s -r -m"])

        if auth_mode == "password" and password:
            sshpass_check = subprocess.run(["which", "sshpass"], capture_output=True, text=True)
            if sshpass_check.returncode == 0:
                ssh_cmd = ["sshpass", "-p", password] + ssh_cmd

        ssh_env = session.get_ssh_env(password)
        try:
            proc = await asyncio.create_subprocess_exec(
                *ssh_cmd,
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env=ssh_env
            )
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=8.0)
            latency_ms = int((time.time() - t0) * 1000)
            out_str = stdout.decode("utf-8", errors="ignore")
            err_str = stderr.decode("utf-8", errors="ignore")

            if proc.returncode == 0 and "SOCIALNAV_SSH_OK" in out_str:
                details = out_str.replace("SOCIALNAV_SSH_OK", "").strip()
                await ws.send_text(json.dumps({
                    "op": "test_result",
                    "success": True,
                    "latency_ms": latency_ms,
                    "details": details,
                    "message": f"Successfully connected to {target} ({latency_ms}ms)!\nRemote: {details}"
                }))
                await ws.send_text(json.dumps({
                    "op": "terminal_log",
                    "type": "success",
                    "text": f"\x1b[32m[SSH-OK]\x1b[0m Connected to {target} in {latency_ms}ms! Info: {details}"
                }))
            else:
                err_msg = err_str.strip() or "Connection timed out or credentials invalid."
                await ws.send_text(json.dumps({
                    "op": "test_result",
                    "success": False,
                    "message": f"SSH connection failed: {err_msg}"
                }))
                await ws.send_text(json.dumps({
                    "op": "terminal_log",
                    "type": "error",
                    "text": f"\x1b[31m[SSH-ERR]\x1b[0m Failed: {err_msg}"
                }))
        except asyncio.TimeoutError:
            await ws.send_text(json.dumps({
                "op": "test_result",
                "success": False,
                "message": "SSH Connection probe timed out after 8 seconds."
            }))
        except Exception as e:
            await ws.send_text(json.dumps({
                "op": "test_result",
                "success": False,
                "message": f"Error executing SSH probe: {str(e)}"
            }))

    elif op == "launch_ros2":
        distro = msg_obj.get("ros_distro", "humble")
        ws_setup = msg_obj.get("ws_setup", f"/opt/ros/{distro}/setup.bash")
        preset = msg_obj.get("preset", "rosbridge")
        custom_cmd = msg_obj.get("custom_cmd", "").strip()
        auto_tunnel = msg_obj.get("auto_tunnel", True)
        local_port = int(msg_obj.get("local_port", 9091))
        remote_port = int(msg_obj.get("remote_port", 9091))

        if preset == "rosbridge":
            remote_exec = f"source {ws_setup} 2>/dev/null || true; ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={remote_port}"
        elif preset == "publisher":
            remote_exec = f"source {ws_setup} 2>/dev/null || true; ros2 run ros2_bridge live_stream_publisher"
        elif preset == "custom" and custom_cmd:
            remote_exec = f"source {ws_setup} 2>/dev/null || true; {custom_cmd}"
        else:
            remote_exec = f"source {ws_setup} 2>/dev/null || true; ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={remote_port}"

        await ws.send_text(json.dumps({
            "op": "terminal_log",
            "type": "info",
            "text": f"\x1b[35m[LAUNCH-START]\x1b[0m Executing remote command on {session.username}@{session.host}:\n\x1b[33m$ {remote_exec}\x1b[0m"
        }))

        ssh_env = session.get_ssh_env()

        if auto_tunnel:
            tunnel_cmd = session.build_ssh_base_cmd()
            tunnel_cmd.extend(["-N", "-L", f"{local_port}:localhost:{remote_port}"])
            tunnel_cmd = session.wrap_with_auth(tunnel_cmd)
            try:
                session.tunnel_process = await asyncio.create_subprocess_exec(
                    *tunnel_cmd,
                    stdin=asyncio.subprocess.DEVNULL,
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.PIPE,
                    env=ssh_env
                )
                await ws.send_text(json.dumps({
                    "op": "terminal_log",
                    "type": "info",
                    "text": f"\x1b[36m[TUNNEL]\x1b[0m Established SSH Port Forwarding localhost:{local_port} <==> remote:{remote_port}"
                }))
            except Exception as e:
                await ws.send_text(json.dumps({
                    "op": "terminal_log",
                    "type": "warn",
                    "text": f"\x1b[33m[TUNNEL-WARN]\x1b[0m Could not create port tunnel: {e}"
                }))

        launch_cmd = session.build_ssh_base_cmd()
        launch_cmd.append(remote_exec)
        launch_cmd = session.wrap_with_auth(launch_cmd)

        try:
            session.remote_process = await asyncio.create_subprocess_exec(
                *launch_cmd,
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env=ssh_env
            )
            session.is_running = True
            session.launch_time = time.time()

            await ws.send_text(json.dumps({
                "op": "launch_status",
                "status": "RUNNING",
                "host": session.host,
                "local_port": local_port,
                "remote_port": remote_port,
                "message": f"ROS2 bridge launched successfully on {session.host}!"
            }))

            async def stream_output():
                while session.is_running and session.remote_process:
                    line = await session.remote_process.stdout.readline()
                    if not line:
                        break
                    decoded_line = line.decode("utf-8", errors="ignore").rstrip()
                    if decoded_line:
                        await broadcast_json({
                            "op": "terminal_log",
                            "type": "stdout",
                            "text": decoded_line
                        })
                session.is_running = False
                await broadcast_json({
                    "op": "launch_status",
                    "status": "STOPPED",
                    "message": "Remote ROS2 process exited."
                })

            asyncio.create_task(stream_output())

        except Exception as e:
            await ws.send_text(json.dumps({
                "op": "launch_status",
                "status": "ERROR",
                "message": f"Failed to start remote ROS2 launch: {str(e)}"
            }))

    elif op == "exec_command":
        cmd_text = msg_obj.get("command", "").strip()
        if not cmd_text:
            return

        if not session.host:
            await ws.send_text(json.dumps({
                "op": "terminal_log",
                "type": "error",
                "text": "\x1b[31m[EXEC-ERR]\x1b[0m No remote robot host specified. Please configure your profile in Settings."
            }))
            return

        # Stop previous command if still streaming
        session.interrupt_interactive()

        await ws.send_text(json.dumps({
            "op": "terminal_log",
            "type": "stdin",
            "text": f"\x1b[32m$\x1b[0m {cmd_text}"
        }))

        ws_setup = msg_obj.get("ws_setup") or "/opt/ros/humble/setup.bash"
        remote_cmd = f"source {ws_setup} 2>/dev/null || true; {cmd_text}"

        ssh_cmd = session.build_ssh_base_cmd()
        ssh_cmd.append(remote_cmd)
        ssh_cmd = session.wrap_with_auth(ssh_cmd)
        ssh_env = session.get_ssh_env()

        try:
            proc = await asyncio.create_subprocess_exec(
                *ssh_cmd,
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env=ssh_env
            )
            session.interactive_process = proc

            async def stream_interactive_output():
                try:
                    while proc.returncode is None:
                        line = await proc.stdout.readline()
                        if not line:
                            break
                        text = line.decode("utf-8", errors="ignore").rstrip("\r\n")
                        if text:
                            await ws.send_text(json.dumps({
                                "op": "terminal_log",
                                "type": "stdout",
                                "text": text
                            }))
                    await proc.wait()
                except asyncio.CancelledError:
                    try:
                        proc.terminate()
                    except Exception:
                        pass
                finally:
                    if session.interactive_process == proc:
                        session.interactive_process = None

            session.interactive_task = asyncio.create_task(stream_interactive_output())

        except Exception as e:
            await ws.send_text(json.dumps({
                "op": "terminal_log",
                "type": "error",
                "text": f"\x1b[31m[EXEC-ERR]\x1b[0m {str(e)}"
            }))

    elif op == "interrupt_command":
        session.interrupt_interactive()
        await ws.send_text(json.dumps({
            "op": "terminal_log",
            "type": "warn",
            "text": "\x1b[33m^C [SIGINT - Process Interrupted]\x1b[0m"
        }))

    elif op == "stop_session":
        if session.remote_process:
            try:
                session.remote_process.terminate()
            except Exception:
                pass
            session.remote_process = None

        if session.tunnel_process:
            try:
                session.tunnel_process.terminate()
            except Exception:
                pass
            session.tunnel_process = None

        session.is_running = False
        await broadcast_json({
            "op": "launch_status",
            "status": "STOPPED",
            "message": "Remote SSH session and port forward tunnel stopped."
        })
        await ws.send_text(json.dumps({
            "op": "terminal_log",
            "type": "warn",
            "text": "\x1b[33m[STOP]\x1b[0m Remote session & SSH tunnel stopped."
        }))

    elif op == "fetch_topics":
        distro = msg_obj.get("ros_distro", "humble")
        ws_setup = msg_obj.get("ws_setup", f"/opt/ros/{distro}/setup.bash")
        cmd_text = f"source {ws_setup} 2>/dev/null || true; ros2 topic list -t"

        ssh_cmd = session.build_ssh_base_cmd()
        ssh_cmd.append(cmd_text)
        ssh_cmd = session.wrap_with_auth(ssh_cmd)
        ssh_env = session.get_ssh_env()

        try:
            proc = await asyncio.create_subprocess_exec(
                *ssh_cmd,
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env=ssh_env
            )
            stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=10.0)
            out_str = stdout.decode("utf-8", errors="ignore")
            
            topics_map = {}
            for line in out_str.splitlines():
                line = line.strip()
                if not line or " " not in line:
                    continue
                parts = line.split("[", 1)
                if len(parts) == 2:
                    t_name = parts[0].strip()
                    t_type = parts[1].replace("]", "").strip()
                    topics_map[t_name] = t_type

            await ws.send_text(json.dumps({
                "op": "topics_discovered",
                "success": True,
                "topics": topics_map,
                "count": len(topics_map)
            }))
            await ws.send_text(json.dumps({
                "op": "terminal_log",
                "type": "success",
                "text": f"\x1b[32m[DISCOVERY]\x1b[0m Successfully queried {len(topics_map)} ROS2 topics & types from remote robot!"
            }))
        except Exception as e:
            await ws.send_text(json.dumps({
                "op": "topics_discovered",
                "success": False,
                "error": str(e),
                "topics": {}
            }))

    elif op == "get_status":
        await ws.send_text(json.dumps({
            "op": "status_response",
            "is_running": session.is_running,
            "host": session.host,
            "username": session.username,
            "uptime": int(time.time() - session.launch_time) if session.launch_time and session.is_running else 0
        }))


async def client_handler(reader, writer):
    ws = MinimalWebSocket(reader, writer)
    success = await ws.handshake()
    if not success:
        writer.close()
        return

    connected_clients.add(ws)
    try:
        await ws.send_text(json.dumps({
            "op": "gateway_ready",
            "version": "1.0.0",
            "message": "SocialNav SSH Remote Gateway connected on port 9092."
        }))

        while not ws.closed:
            msg_str = await ws.read_msg()
            if msg_str is None:
                break
            try:
                msg_obj = json.loads(msg_str)
                await handle_ssh_request(ws, msg_obj)
            except Exception as e:
                await ws.send_text(json.dumps({
                    "op": "error",
                    "message": f"Malformed request: {str(e)}"
                }))
    finally:
        connected_clients.discard(ws)
        writer.close()


async def main():
    server = await asyncio.start_server(client_handler, "0.0.0.0", PORT)
    print(f"\n=======================================================")
    print(f" 🚀 SocialNav Remote SSH Gateway running on ws://localhost:{PORT}")
    print(f" Ready to manage remote robot connections, tunnels & terminal logs.")
    print(f" Press Ctrl+C to stop.")
    print(f"=======================================================\n")
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[SSH Gateway] Shutting down...")
        sys.exit(0)
