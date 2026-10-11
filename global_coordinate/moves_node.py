import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveNode(Node):

    def __init__(self):
        super().__init__('move_node')

        self.publisher_ = self.create_publisher(
            Twist, '/cmd_vel', 5
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            5
        )

        self.side_length = 2.0             
        self.max_linear = 1.0           
        self.max_angular = 0.50             

        self.k_distance = 0.5              
        self.k_angle = 1.5                 

        self.position_tolerance = 0.01      
        self.angle_tolerance = math.radians(0.5)  

        self.odom_received = False
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.yaw = 0.0

        self.initialized = False
        self.x_origin = 0.0
        self.y_origin = 0.0
        self.yaw_origin = 0.0

        self.waypoints = []
        self.waypoint_index = 0

        self.phase = 'ALIGN'
        self.finished = False
        self.warned_no_odom = False
        self.last_reported_waypoint = -1

        self.timer = self.create_timer(0.1, self.timer_callback)

    @staticmethod
    def normalize_angle(angle):
        """Normalisasi sudut ke rentang [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        """Menyimpan posisi dan yaw dari odometri."""
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.odom_received = True

    def initialize_waypoints(self):
        """Membuat persegi relatif terhadap posisi dan arah awal robot."""
        self.x_origin = self.x_odom
        self.y_origin = self.y_odom
        self.yaw_origin = self.yaw

        L = self.side_length
        self.waypoints = [
            (L, 0.0),
            (L, L),
            (0.0, L),
            (0.0, 0.0)
        ]

        self.initialized = True

        self.get_logger().info(
            f'Waypoint terbentuk: panjang sisi={L:.2f} m'
        )

    def local_to_odom(self, x_local, y_local):
        """Mengubah titik lokal persegi menjadi koordinat odometri."""
        c = math.cos(self.yaw_origin)
        s = math.sin(self.yaw_origin)

        x_global = self.x_origin + c * x_local - s * y_local
        y_global = self.y_origin + s * x_local + c * y_local

        return x_global, y_global

    def get_local_position(self):
        """Mengubah posisi odometri ke koordinat relatif terhadap titik awal."""
        dx = self.x_odom - self.x_origin
        dy = self.y_odom - self.y_origin

        c = math.cos(self.yaw_origin)
        s = math.sin(self.yaw_origin)

        x_local = c * dx + s * dy
        y_local = -s * dx + c * dy

        return x_local, y_local

    def stop_robot(self):
        """Mengirim perintah berhenti."""
        self.publisher_.publish(Twist())

    def timer_callback(self):
        msg = Twist()

        if not self.odom_received:
            if not self.warned_no_odom:
                self.get_logger().warn('Menunggu odometri dari /odom...')
                self.warned_no_odom = True
            self.stop_robot()
            return

        if not self.initialized:
            self.initialize_waypoints()

        x_local, y_local = self.get_local_position()

        if self.finished:
            self.stop_robot()
            return

        if self.waypoint_index >= len(self.waypoints):
            target_yaw = self.yaw_origin
            angle_error = self.normalize_angle(target_yaw - self.yaw)

            if abs(angle_error) > self.angle_tolerance:
                msg.angular.z = max(
                    -self.max_angular,
                    min(self.max_angular, self.k_angle * angle_error)
                )
                status = 'Mengembalikan orientasi awal'
            else:
                self.finished = True
                self.phase = 'FINISHED'
                self.stop_robot()
                self.get_logger().info(
                    'Lintasan persegi selesai; posisi dan orientasi akhir '
                    'berada dalam toleransi.'
                )
                self.timer.cancel()

                rclpy.shutdown()
                return

        else:
            wx_local, wy_local = self.waypoints[self.waypoint_index]
            wx_global, wy_global = self.local_to_odom(wx_local, wy_local)

            dx = wx_global - self.x_odom
            dy = wy_global - self.y_odom
            distance = math.hypot(dx, dy)

            if distance <= self.position_tolerance:
                msg = Twist()
                if self.last_reported_waypoint != self.waypoint_index:
                    self.get_logger().info(
                        f'Waypoint {self.waypoint_index + 1} tercapai '
                        f'pada posisi lokal ({x_local:.3f}, {y_local:.3f})'
                    )
                    self.last_reported_waypoint = self.waypoint_index

                self.waypoint_index += 1
                self.phase = 'ALIGN'
                target_yaw = self.yaw
                angle_error = 0.0
                status = 'Waypoint tercapai; bersiap berotasi'
            else:
                target_yaw = math.atan2(dy, dx)
                angle_error = self.normalize_angle(target_yaw - self.yaw)

                if self.phase == 'ALIGN':
                    if abs(angle_error) > self.angle_tolerance:
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = max(
                            -self.max_angular,
                            min(self.max_angular, self.k_angle * angle_error)
                        )
                        status = 'Rotasi menuju arah sisi berikutnya'
                    else:
                        self.phase = 'MOVE'
                        status = 'Arah tercapai, bersiap bergerak'
                else:
                    if abs(angle_error) > self.angle_tolerance:
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = max(
                            -self.max_angular,
                            min(self.max_angular, self.k_angle * angle_error)
                        )
                        status = 'Koreksi orientasi'
                    else:
                        msg.linear.x = min(
                            self.max_linear,
                            self.k_distance * distance
                        )
                        msg.linear.y = 0.0
                        msg.angular.z = 0.0
                        status = 'Bergerak lurus menuju waypoint'

        self.publisher_.publish(msg)

        if abs(msg.linear.x) > 0.0:
            vx_global = msg.linear.x * math.cos(self.yaw)
            vy_global = msg.linear.x * math.sin(self.yaw)
        else:
            vx_global = 0.0
            vy_global = 0.0

        self.get_logger().info(
            status + ' | ' +
            'Posisi lokal=(x=' + str(round(x_local, 3)) +
            ', y=' + str(round(y_local, 3)) + ') m | ' +
            'Vx_global=' + str(round(vx_global, 3)) + ' m/s | ' +
            'Vy_global=' + str(round(vy_global, 3)) + ' m/s | ' +
            'Omega=' + str(round(msg.angular.z, 3)) + ' rad/s | ' +
            'Galat sudut=' + str(round(math.degrees(angle_error), 2)) + ' derajat'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher_.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
