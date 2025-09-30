#!/usr/bin/env python3
import threading, time, re, queue, math
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from ros2_grbl_ramps_interfaces.msg import GrblStatus
from ros2_grbl_ramps_interfaces.srv import SendGcode, JogXYZ, Home, Realtime, SetUnits

# Robust GRBL status pattern:
# Accepts MPos, optional WPos, and optional WCO=<x,y,z>
STATUS_RE = re.compile(
    r"<(?P<state>[^|>]+)\|"
    r"(?:(?:WPos:(?P<wx>-?\d+(?:\.\d+)?),(?P<wy>-?\d+(?:\.\d+)?),(?P<wz>-?\d+(?:\.\d+)?))|(?:[^|>]*))\|?"
    r"(?:(?:MPos:(?P<mx>-?\d+(?:\.\d+)?),(?P<my>-?\d+(?:\.\d+)?),(?P<mz>-?\d+(?:\.\d+)?))|(?:[^|>]*))\|?"
    r"[^|>]*?FS:(?P<feed>\d+(?:\.\d+)?),(?P<spindle>\d+(?:\.\d+)?)"
    r"(?:\|[^|>]*?WCO:(?P<wcox>-?\d+(?:\.\d+)?),(?P<wcoy>-?\d+(?:\.\d+)?),(?P<wcoz>-?\d+(?:\.\d+)?))?"
    r"[^>]*>"
)

REALTIME = {
    'hold': b'!',
    'start': b'~',
    'status': b'?',
    'soft-reset': b'\x18',
    'reset': b'\x18',
    'unlock': b'$X\n',
    'jog-cancel': b'\x85',
}

def _mk_qos(reliability: str, depth: int = 10) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.history = HistoryPolicy.KEEP_LAST
    if reliability.lower() == "reliable":
        qos.reliability = ReliabilityPolicy.RELIABLE
    else:
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
    return qos


class GrblBridge(Node):
    def __init__(self):
        super().__init__('grbl_bridge')

        # ---- Parameters ----
        self.declare_parameters('', [
            ('port', '/dev/ttyUSB0'),
            ('baud', 115200),
            ('enable_polling', True),
            ('status_hz', 5.0),
            ('startup_gcode', ['$X']),
            ('publish_joint_states', True),
            ('joint_names', ['joint_x', 'joint_y', 'joint_z']),
            ('frame_id', 'base_link'),
            ('steps_per_mm', [80.0, 80.0, 400.0]),
            ('mm_per_rev', [1.0, 1.0, 1.0]),
            ('zero_is_machine_zero', True),
            ('log_rx_lines', False),
            ('qos_reliability', 'best_effort'),
        ])

        # Access parameters
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.enable_polling = bool(self.get_parameter('enable_polling').value)
        status_hz = float(self.get_parameter('status_hz').value)
        self.report_dt = 1.0 / max(status_hz, 0.1)
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
        self.joint_names = [str(x) for x in self.get_parameter('joint_names').value]
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.zero_is_machine_zero = bool(self.get_parameter('zero_is_machine_zero').value)
        self.log_rx_lines = bool(self.get_parameter('log_rx_lines').value)
        qos_reliability = str(self.get_parameter('qos_reliability').value).lower()

        # ---- State ----
        self._ser = None
        self._ser_lock = threading.Lock()
        self._stop = threading.Event()
        self._rx_q = queue.Queue(maxsize=200)
        self._tx_q = queue.Queue(maxsize=400)
        self._last_js_t = 0.0
        self._mpos = [0.0, 0.0, 0.0]
        self._wco = [0.0, 0.0, 0.0]  # Work Coordinate Offset if present
        self._last_state = "Idle"

        # ---- Publishers ----
        qos = _mk_qos(qos_reliability, depth=20)
        self.status_pub = self.create_publisher(GrblStatus, 'grbl/status', qos)
        self.raw_pub = self.create_publisher(String, 'grbl/raw', qos)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10) if self.publish_joint_states else None

        # ---- Services ----
        self.create_service(SendGcode, 'grbl/send_gcode', self.on_send_gcode)
        self.create_service(JogXYZ,    'grbl/jog_xyz',    self.on_jog)
        self.create_service(Home,      'grbl/home',       self.on_home)
        self.create_service(Realtime,  'grbl/realtime',   self.on_realtime)
        self.create_service(SetUnits,  'grbl/set_units',  self.on_set_units)

        # ---- Threads ----
        self._rx_t = threading.Thread(target=self.rx_loop, daemon=True)
        self._tx_t = threading.Thread(target=self.tx_loop, daemon=True)
        self._poll_t = threading.Thread(target=self.poll_loop, daemon=True)

        self.get_logger().info(f"Opening {self.port} @ {self.baud}")
        self._open_serial_and_greet()

        self._rx_t.start()
        self._tx_t.start()
        if self.enable_polling:
            self._poll_t.start()

        # Startup G-code (queued)
        for line in self.get_parameter('startup_gcode').value:
            self.enqueue(line)

        self.get_logger().info("GRBL bridge started")

    # ---------- Serial bring-up ----------
    def _open_serial_and_greet(self):
        # Try open with modest timeouts; we will reconnect if needed
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.5)
            time.sleep(0.1)

            # Toggle DTR/RTS (many Arduino clones rely on this)
            try:
                self._ser.dtr = False
                self._ser.rts = False
                time.sleep(0.1)
                self._ser.dtr = True
                self._ser.rts = True
            except Exception as e:
                self.get_logger().warn(f"Could not toggle DTR/RTS: {e}")

            # Soft reset and allow banner to print (do NOT flush it)
            try:
                with self._ser_lock:
                    self._ser.write(REALTIME['reset'])
                self.get_logger().info("Sent Ctrl-X (soft reset)")
            except Exception as e:
                self.get_logger().warning(f"Could not send soft reset: {e}")

            time.sleep(1.0)

            # Kick a status so we know RX works
            try:
                with self._ser_lock:
                    self._ser.write(REALTIME['status'])
                    self._ser.write(b'\n')
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self._ser = None

    def _reconnect_loop(self, delay_s: float = 1.0):
        # Backoff reconnect loop
        while not self._stop.is_set() and self._ser is None:
            self.get_logger().info(f"Trying to (re)open {self.port} @ {self.baud}...")
            self._open_serial_and_greet()
            if self._ser is None:
                time.sleep(min(delay_s, 5.0))
                delay_s = min(delay_s * 1.5, 5.0)

    # ---------- Serial loops ----------
    def rx_loop(self):
        buf = b''
        while not self._stop.is_set():
            if self._ser is None:
                self._reconnect_loop()
                continue
            try:
                b = self._ser.read(256)
                if not b:
                    continue
                buf += b
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    s = line.decode(errors='ignore').strip()
                    if not s:
                        continue
                    if self.log_rx_lines:
                        self.get_logger().info(f"GRBL<< {s}")
                    self.raw_pub.publish(String(data=s))
                    self.parse_status(s)
            except Exception as e:
                self.get_logger().error(f"RX error: {e}")
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(0.3)

    def tx_loop(self):
        while not self._stop.is_set():
            if self._ser is None:
                time.sleep(0.1)
                continue
            try:
                data = self._tx_q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                with self._ser_lock:
                    if isinstance(data, (bytes, bytearray)):
                        self._ser.write(data)
                    else:
                        # Ensure single LF line ending for GRBL
                        s = str(data)
                        if not s.endswith('\n'):
                            s += '\n'
                        self._ser.write(s.encode())
            except Exception as e:
                self.get_logger().error(f"TX error: {e}")
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(0.3)

    def poll_loop(self):
        while not self._stop.is_set():
            if self._ser is None:
                time.sleep(0.2)
                continue
            try:
                with self._ser_lock:
                    self._ser.write(REALTIME['status'])  # '?'
            except Exception as e:
                self.get_logger().error(f"poll_loop write failed: {e}")
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
            time.sleep(self.report_dt)

    # ---------- Helpers ----------
    def enqueue(self, data: str):
        # Accepts str or bytes; always queues one line
        if isinstance(data, (bytes, bytearray)):
            if not data.endswith(b'\n'):
                data += b'\n'
            self._tx_q.put(data)
        else:
            s = str(data)
            if not s.endswith('\n'):
                s += '\n'
            self._tx_q.put(s)

    def _publish_joint_state(self, pos):
        if not self.joint_pub:
            return
        t = time.time()
        if (t - self._last_js_t) < 0.02:  # ~50 Hz cap
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = self.frame_id
        js.name = self.joint_names
        js.position = pos[:3]
        self.joint_pub.publish(js)
        self._last_js_t = t

    def parse_status(self, s: str):
        m = STATUS_RE.search(s)
        if not m:
            return

        self._last_state = m['state']
        # Capture offsets if present
        if m['wcox'] is not None:
            self._wco = [float(m['wcox']), float(m['wcoy']), float(m['wcoz'])]

        # Prefer machine position (present on all status lines)
        mx = m['mx']; my = m['my']; mz = m['mz']
        if mx is not None and my is not None and mz is not None:
            self._mpos = [float(mx), float(my), float(mz)]

        # Compute work position if user prefers zero != machine zero and WCO exists
        if self.zero_is_machine_zero:
            pub_pos = self._mpos
        else:
            # If we have MPos & WCO, WPos = MPos - WCO; otherwise fallback to MPos
            pub_pos = [self._mpos[i] - self._wco[i] for i in range(3)]

        msg = GrblStatus()
        msg.state = self._last_state
        msg.x, msg.y, msg.z = pub_pos
        msg.feed = float(m['feed']) if m['feed'] else 0.0
        msg.spindle = float(m['spindle']) if m['spindle'] else 0.0
        msg.raw = s
        self.status_pub.publish(msg)

        self._publish_joint_state(pub_pos)

    # ---------- Services ----------
    def on_send_gcode(self, req, res):
        gcode = (req.gcode or '').strip()
        if not gcode:
            res.ok = False; res.reply = 'empty gcode'
            return res
        self.enqueue(gcode)
        res.ok = True; res.reply = gcode
        return res

    def on_jog(self, req, res):
        # Build a robust, space-separated $J line
        axes = []
        if abs(req.x) > 0.0: axes.append(f"X{req.x:.4f}")
        if abs(req.y) > 0.0: axes.append(f"Y{req.y:.4f}")
        if abs(req.z) > 0.0: axes.append(f"Z{req.z:.4f}")

        if not axes:
            res.ok = False; res.reply = "no axis delta provided"
            return res

        feed = max(1.0, float(req.feed))
        mode = "G91" if req.relative else "G90"
        parts = ["$J=", "G21", mode] + axes + [f"F{feed:.3f}"]
        g = " ".join(parts)
        self.enqueue(g)
        res.ok = True; res.reply = g
        return res

    def on_home(self, req, res):
        axes = ''
        if req.home_x: axes += 'X'
        if req.home_y: axes += 'Y'
        if req.home_z: axes += 'Z'
        # Note: vanilla GRBL supports $H (all axes). Axis-specific $H may be firmware-dependent.
        g = "$H" if axes == '' else f"$H{axes}"
        self.enqueue(g)
        res.ok = True; res.reply = g
        return res

    def on_realtime(self, req, res):
        cmd = (req.cmd or '').strip().lower()
        data = REALTIME.get(cmd)
        if not data:
            res.ok = False; res.reply = f"unknown '{cmd}'"
            return res
        try:
            with self._ser_lock:
                if self._ser is not None:
                    self._ser.write(data)
                else:
                    raise RuntimeError("serial not connected")
        except Exception as e:
            self.get_logger().error(f"Realtime write failed: {e}")
            res.ok = False; res.reply = str(e)
            return res
        res.ok = True; res.reply = cmd
        return res

    def on_set_units(self, req, res):
        u = (req.units or '').strip().lower()
        if u in ('mm', 'millimeter', 'millimetre'):
            self.enqueue('G21')
        elif u in ('inch', 'inches'):
            self.enqueue('G20')
        else:
            res.ok = False; res.reply = 'units must be mm or inch'
            return res
        res.ok = True; res.reply = f'units {u}'
        return res

    # ---------- Shutdown ----------
    def destroy_node(self):
        self._stop.set()
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GrblBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
