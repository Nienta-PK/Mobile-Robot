#!/usr/bin/env python3
import rclpy, serial
from rclpy.node import Node
from sensor_msgs.msg import Range

from ros2_grbl_ramps_interfaces.srv import JogXYZ, Realtime  # from your bridge

class SerialUltrasonicGuard(Node):
    def __init__(self):
        super().__init__('serial_ultrasonic_guard')

        # --- Params (you can also override with --ros-args -p ...) ---
        self.declare_parameter('port', '/dev/ttyACM0')      # Arduino Mega USB
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', 'ultrasonic/range')
        self.declare_parameter('min_range_cm', 2.0)
        self.declare_parameter('max_range_cm', 400.0)
        self.declare_parameter('fov', 0.5)

        # GRBL control
        self.declare_parameter('grbl_namespace', 'grbl')    # matches your bridge
        self.declare_parameter('stop_distance_cm', 3.0)     # STOP at <= 3 cm
        self.declare_parameter('jog_feed_mm_min', 800.0)    # jogging feed
        self.declare_parameter('jog_distance_mm', 1000.0)   # relative X distance
        self.declare_parameter('x_positive', True)          # True -> +X forward

        p = {p.name: p.value for p in self.get_parameters([
            'port','baud','topic','min_range_cm','max_range_cm','fov',
            'grbl_namespace','stop_distance_cm','jog_feed_mm_min','jog_distance_mm','x_positive'
        ])}
        self.port = p['port']; self.baud=int(p['baud'])
        self.topic = p['topic']
        self.min_m = float(p['min_range_cm'])/100.0
        self.max_m = float(p['max_range_cm'])/100.0
        self.fov   = float(p['fov'])
        self.stop_cm = float(p['stop_distance_cm'])
        self.jog_feed = float(p['jog_feed_mm_min'])
        self.jog_dx = float(p['jog_distance_mm']) * (1.0 if bool(p['x_positive']) else -1.0)
        self.grbl_ns = p['grbl_namespace']

        # Publisher
        self.pub = self.create_publisher(Range, self.topic, 10)

        # Serial to Mega (reads "DIST_CM: <val>")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)

        # GRBL clients (provided by your bridge node)
        self.cli_jog  = self.create_client(JogXYZ,   f'/{self.grbl_ns}/jog_xyz')
        self.cli_rt   = self.create_client(Realtime, f'/{self.grbl_ns}/realtime')

        # State
        self._moving = False
        self._last_cm = None

        # Timers
        self.create_timer(0.05, self.tick)   # 20 Hz read/logic

        self.get_logger().info(
            f"Reading {self.port} @ {self.baud}, publishing {self.topic}; "
            f"Jog X {'+' if self.jog_dx>0 else '-'} until <= {self.stop_cm:.1f} cm."
        )

    # ---------- helpers ----------
    def _publish_range(self, cm: float):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_m
        msg.max_range = self.max_m
        msg.range = max(self.min_m, min(self.max_m, cm/100.0))
        self.pub.publish(msg)

    def _grbl_ready(self) -> bool:
        ok = self.cli_jog.wait_for_service(timeout_sec=0.1) and \
             self.cli_rt.wait_for_service(timeout_sec=0.1)
        if not ok:
            self.get_logger().warn("Waiting for GRBL services...")
        return ok

    def _send_hold_and_cancel(self):
        # feed-hold first (gentle), then jog-cancel to stop the running $J
        req = Realtime.Request(cmd='hold')
        self.cli_rt.call_async(req)
        req2 = Realtime.Request(cmd='jog-cancel')
        self.cli_rt.call_async(req2)

    def _start_jog_if_needed(self):
        if self._moving:  return
        if not self._grbl_ready(): return
        # ask GRBL to jog relative X by a long distance; we'll cancel on stop
        j = JogXYZ.Request()
        j.x = self.jog_dx; j.y = 0.0; j.z = 0.0
        j.feed = self.jog_feed
        j.relative = True                  # G91 relative move (bridge will format $J= ... )  :contentReference[oaicite:3]{index=3}
        fut = self.cli_jog.call_async(j)
        self._moving = True
        self.get_logger().info(f"Jogging X by {self.jog_dx:.1f} mm @ F{self.jog_feed:.0f}")

    # ---------- main loop ----------
    def tick(self):
        # 1) read one line from Mega
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line or not line.startswith('DIST_CM:'):
            return
        try:
            cm = float(line.split(':',1)[1].strip())
        except Exception:
            return

        self._last_cm = cm
        self._publish_range(cm)
        self.get_logger().info(f"Range: {cm:.1f} cm   moving={self._moving}")

        # 2) control logic
        if cm <= self.stop_cm:
            # STOP
            if self._moving and self._grbl_ready():
                self._send_hold_and_cancel()            # uses /grbl/realtime commands  :contentReference[oaicite:4]{index=4}
                self._moving = False
                self.get_logger().warn(f"Stopped at {cm:.1f} cm (<= {self.stop_cm:.1f} cm)")
        else:
            # START / KEEP MOVING FORWARD
            self._start_jog_if_needed()

    def destroy_node(self):
        try:
            if self.ser: self.ser.close()
        except: pass
        return super().destroy_node()

def main():
    rclpy.init()
    n = SerialUltrasonicGuard()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
