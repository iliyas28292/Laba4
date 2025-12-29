import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen


class RurMover(Node):
    def __init__(self):
        super().__init__('rur_mover')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # скорости
        self.lin_speed = 2.0        # units/s
        self.ang_speed_deg = 90.0   # deg/s

        # клиент на "перо"
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen...')

        self.main()

    def set_pen(self, off: bool, width: int = 3, r: int = 255, g: int = 255, b: int = 255):
        req = SetPen.Request()
        req.r = int(r)
        req.g = int(g)
        req.b = int(b)
        req.width = int(width)
        req.off = bool(off)
        fut = self.pen_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def hold_twist(self, lin: float, ang_deg_s: float, duration_s: float, hz: float = 20.0):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = math.radians(float(ang_deg_s))  # rad/s

        period = 1.0 / hz
        t_end = time.monotonic() + float(duration_s)

        while time.monotonic() < t_end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.pub.publish(Twist())
        time.sleep(0.05)

    def rotate(self, angle_deg: float):
        duration = abs(angle_deg) / self.ang_speed_deg
        sign = 1.0 if angle_deg >= 0 else -1.0
        self.hold_twist(0.0, sign * self.ang_speed_deg, duration)

    def forward(self, dist: float):
        duration = abs(dist) / self.lin_speed
        sign = 1.0 if dist >= 0 else -1.0
        self.hold_twist(sign * self.lin_speed, 0.0, duration)

    def move_no_draw(self, dist: float):
        self.set_pen(True)      # pen OFF
        self.forward(dist)
        self.set_pen(False)     # pen ON

    def main(self):
        # Буква "И" (зеркало N):  | / |
        h = 3.0
        diag = math.sqrt(2.0) * h

        # 1) левая вертикаль вверх
        self.rotate(+90.0)      # вверх
        self.set_pen(False)
        self.forward(h)

        # 2) вернуться вниз БЕЗ рисования
        self.rotate(+180.0)     # вниз
        self.move_no_draw(h)
        self.rotate(+180.0)     # снова вверх (как было)

        # 3) диагональ снизу-влево -> вверх-вправо  ( / )
        self.rotate(-45.0)      # на северо-восток
        self.set_pen(False)
        self.forward(diag)

        # 4) опуститься по правой стойке БЕЗ рисования и нарисовать её вверх
        self.rotate(-135.0)     # вниз
        self.move_no_draw(h)
        self.rotate(+180.0)     # вверх
        self.set_pen(False)
        self.forward(h)


def main(args=None):
    rclpy.init(args=args)
    mover = RurMover()
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
