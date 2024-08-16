import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import numpy as np
from scipy.ndimage import distance_transform_edt
import quadprog
import threading
from qpsolvers import solve_qp
from pynput import keyboard
import tty
import sys
import termios
import matplotlib.pyplot as plt
from matplotlib import colors
from scipy import ndimage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class OccupancyGridProcessor(Node):

    def __init__(self):
        super().__init__('occupancy_grid_processor')
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # PREV ODOM, FROM TURTLEBOT3
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 10 HZ A SECOND, THIS CAN BE HIGHER
        self.create_timer(0.1, self.update_robot_pose)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.occupancy_grid = None
        self.robot_pose = None
        self.u_teleop = np.array([0.0, 0.0])
        self.u_safe = np.array([0.0, 0.0])
        # TRY CHANGING BOUND TO LOWER
        self.truncation_bound = 5.0
        self.max_linear_velocity = 0.22
        self.max_angular_velocity = 2.84
        self.linear_increment = 0.01
        self.angular_increment = 0.01
        self.robot_radius = 0.1

        # INCREASE FOR MORE CAUTION IN SAFE REGION
        self.p1 = 0.8
        # DECREASE FOR MORE CAUTION IN UNSAFE REGION
        self.p2 = 0.001
        # INCREASE FOR MORE UNCERT
        self.epsilon = 0.25

        self.A = np.eye(2)  # 2x2 identity matrix for 2D system
        self.B = np.eye(2)  # 2x2 identity matrix for 2D system

        #self.listener = keyboard.Listener(on_press=self.on_press)
        #self.listener.start()

        self.orig_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        self.input_thread = threading.Thread(target=self.handle_input)
        self.input_thread.dameon = True
        self.input_thread.start()

    def map_callback(self, msg):
        self.occupancy_grid = msg
        self.process_occupancy_grid()

    def update_robot_pose(self):
        try:
            # BASE_LINK IS ROBOT BODY LINK
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',  
                rclpy.time.Time())
            
            self.robot_pose = transform.transform
            self.process_occupancy_grid()
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform from 'map' to 'base_link': {ex}")

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.u_teleop[0] = min(self.u_teleop[0] + self.linear_increment, self.max_linear_velocity)
            elif key.char == 's':
                self.u_teleop[0] = max(self.u_teleop[0] - self.linear_increment, -self.max_linear_velocity)
            elif key.char == 'a':
                self.u_teleop[1] = min(self.u_teleop[1] + self.angular_increment, self.max_angular_velocity)
            elif key.char == 'd':
                self.u_teleop[1] = max(self.u_teleop[1] - self.angular_increment, -self.max_angular_velocity)
            elif key.char == 'x':
                self.u_teleop = np.array([0.0, 0.0])
                self.u_safe = np.array([0.0, 0.0])
                self.publish_safe_velocity()
                return

            self.process_occupancy_grid()
        except AttributeError:
            pass

    def handle_input(self):
        while True:
            x = sys.stdin.read(1)[0]
            self.get_logger().info(f'You pressed: {x}')

            if x == 'w':
                self.u_teleop[0] = min(self.u_teleop[0] + self.linear_increment, self.max_linear_velocity)
            elif x == 's':
                self.u_teleop[0] = max(self.u_teleop[0] - self.linear_increment, -self.max_linear_velocity)
            elif x == 'a':
                self.u_teleop[1] = min(self.u_teleop[1] + self.angular_increment, self.max_angular_velocity)
            elif x == 'd':
                self.u_teleop[1] = max(self.u_teleop[1] - self.angular_increment, -self.max_angular_velocity)
            elif x == 'x':
                self.u_teleop = np.array([0.0, 0.0])
                self.u_safe = np.array([0.0, 0.0])
                self.publish_safe_velocity()
                #return
            # pressed esc key
            elif x == chr(27):
                break

            self.process_occupancy_grid()

    def __del__(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)


    def process_occupancy_grid(self):
        if self.occupancy_grid is None or self.robot_pose is None:
            return

        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        resolution = self.occupancy_grid.info.resolution
        origin = self.occupancy_grid.info.origin

        grid = np.array(self.occupancy_grid.data).reshape((height, width))
        robot_x = (self.robot_pose.translation.x - origin.position.x) / resolution
        robot_y = (self.robot_pose.translation.y - origin.position.y) / resolution

        # DISTANE TRANSFORMS
        free_mask = grid == 0
        not_free_mask = ~free_mask
        
        distance_to_boundary = distance_transform_edt(free_mask) * resolution
        distance_from_boundary = distance_transform_edt(not_free_mask) * resolution

        data_matrix = np.zeros((height, width), dtype=[('state', 'i4'), ('tesdf', 'f4')])

        for y in range(height):
            for x in range(width):
                # SAFE SPACE
                if free_mask[y, x]:
                    h_x = min(distance_to_boundary[y, x], self.truncation_bound)
                # DANGER SPACE (OCCUPIED OR UNKNOWN)
                else:
                    h_x = -min(distance_from_boundary[y, x], self.truncation_bound)
                data_matrix[y, x] = (grid[y, x], h_x)

        # GRADIENT OF TESDF
        # SOBEL FILTER, WORKED BETTER?
        # DERIVATIVE SMOOTHING ON AXIS MAKES STRONGER (FAST AND GRADUAL) GRADIENT RENDER AT EDGE
        gradient_x = ndimage.sobel(data_matrix['tesdf'], axis=1) / (8 * resolution)
        gradient_y = ndimage.sobel(data_matrix['tesdf'], axis=0) / (8 * resolution)
        gradient = np.stack((gradient_x, gradient_y), axis=-1)

        self.apply_safety_filter(data_matrix, gradient, robot_x, robot_y)

        # STORE FOR VISUAL
        self.tesdf = data_matrix['tesdf']
        self.gradient = gradient
        self.robot_x = robot_x
        self.robot_y = robot_y

    def apply_safety_filter(self, data_matrix, gradient, robot_x, robot_y):
        if self.robot_pose is None or self.occupancy_grid is None:
            return

        # TESDF
        closest_x = int(round(robot_x))
        closest_y = int(round(robot_y))
        h = data_matrix[closest_y, closest_x]['tesdf']
        grad_h = gradient[closest_y, closest_x]

        # ALPHA FROM TESDF
        alpha = -self.p1 if h >= 0 else self.p2

        # QP
        # 2x2 IDENTITY MATRIX
        P = np.eye(2)  
        q = -self.u_teleop

        # CONSTRAINT
        C = grad_h.T @ self.B
        c1 = grad_h.T @ (np.eye(2) - self.A) @ np.array([robot_x, robot_y]) + alpha * h
        c2 = np.linalg.norm(grad_h) * self.epsilon

        # SET UP FOR QP SOLVER 
        G = np.vstack([-C, np.eye(2), -np.eye(2)])
        h_qp = np.array([-(c1 + c2), 
                        self.max_linear_velocity, self.max_angular_velocity,
                        self.max_linear_velocity, self.max_angular_velocity])

        self.get_logger().info(f'h: {h}')
        self.get_logger().info(f'grad_h: {grad_h}')
        self.get_logger().info(f'alpha: {alpha}')
        self.get_logger().info(f'C: {C}')
        self.get_logger().info(f'c1: {c1}')
        self.get_logger().info(f'c2: {c2}')
        self.get_logger().info(f'G: {G}')
        self.get_logger().info(f'h_qp: {h_qp}')

        try:
            u_safe = solve_qp(P, q, G, h_qp, solver="cvxopt")
            
            if u_safe is None:
                raise ValueError("QP solver couldn't find a solution")
            
            # CHECK IF SAFETY SATISFIED
            if np.dot(C, u_safe) < c1 + c2:
                self.get_logger().warn('Safety constraint not satisfied')
                u_safe = self.project_onto_constraint(u_safe, C, c1 + c2)
        except Exception as e:
            self.get_logger().error(f'QP solver error: {e}')
            # IF QP FAILS
            # PROJECT TELEOP ONTO CONSTRAINT
            u_safe = self.u_teleop #self.project_onto_constraint(self.u_teleop, C, c1 + c2)

        self.u_safe = u_safe
        self.get_logger().info(f'TESDF={h}, Gradient={grad_h}, Safe input={u_safe}, User input={self.u_teleop}')
        self.publish_safe_velocity()

    def publish_safe_velocity(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.u_safe[0]
        cmd_vel_msg.angular.z = self.u_safe[1]  
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
    def visualize_tesdf_and_gradient(self):
        plt.figure(figsize=(15, 5))
        
        plt.subplot(131)
        plt.imshow(self.tesdf, cmap='RdBu', origin='lower')
        plt.colorbar(label='TESDF')
        plt.title('TESDF')
        plt.plot(self.robot_x, self.robot_y, 'go', markersize=10)

        plt.subplot(132)
        gradient_magnitude = np.sqrt(self.gradient[:,:,0]**2 + self.gradient[:,:,1]**2)
        plt.imshow(gradient_magnitude, cmap='viridis', origin='lower')
        plt.colorbar(label='Gradient Magnitude')
        plt.title('Gradient Magnitude')
        plt.plot(self.robot_x, self.robot_y, 'go', markersize=10)

        # BROKEN SO FAR
        plt.subplot(133)
        step = 5
        plt.quiver(self.gradient[::step, ::step, 0], self.gradient[::step, ::step, 1])
        plt.title('Gradient Direction')
        plt.plot(self.robot_x, self.robot_y, 'go', markersize=10)

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.visualize_tesdf_and_gradient()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.orig_settings)

if __name__ == '__main__':
    main()