import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np
from scipy.stats import multivariate_normal
from scipy.optimize import minimize, NonlinearConstraint
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

class RobotPositionMapper(Node):
    def __init__(self):
        super().__init__('robot_position_mapper')
        
        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Attributes
        # Map
        self.map = None
        self.map_resolution = None
        self.map_origin = None
        self.num_samples = 400

        # Current position
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_covariance = None

        self.current_mean = None
        self.samples = None
        self.current_safety = None
        self.safety_gradient = None
        self.safety_prob = None

        self.max_linear_velocity = 0.22
        self.max_angular_velocity = 2.84
        
        # Projected position
        self.projected_x = None
        self.projected_y = None
        self.projected_mean = None
        self.projected_yaw = None
        
        self.projected_samples = None
        self.projected_safety = None

        self.qp_projected_mean = None
        self.qp_projected_safety = None
        self.qp_projected_gradient = None
        
        #BCBF
        self.alpha = 1.0
        self.desired_safety = 0.95
        self.update_frequency = 0.75 # relates with prediction_time

        # Plot (set up for after exit plotting)
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.canvas = FigureCanvasAgg(self.fig)
        # interactive mode
        plt.ion()

    def map_callback(self, msg):
        self.map = msg
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        occupancy_array = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)

        # inverted for safety instead of collision
        self.safety_prob = np.select(
            [occupancy_array == -1, occupancy_array == 0],
            [0.5, 1.0],
            default=0.0
        )
        
        self.get_logger().info('Received new map')

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        # Start QP
        safe_control = self.compute_safe_control(np.array([self.linear_velocity, self.angular_velocity]))
        self.get_logger().info(f'found safe control!: {safe_control[0]:.4f}, {safe_control[1]:.4f}')

    def pose_callback(self, msg):
        # Extract the robot position, orientation, and covariance
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Uncomment if to take cov from robot... but we artifically set as high for dev
        #cov_matrix = np.array(msg.pose.covariance).reshape(6, 6)
        #position_covariance = cov_matrix[:2, :2]
        self.position_covariance = [[0.009, 0.0], [0.0, 0.009]]

        # Calculate safety at current position
        self.current_mean = np.array([self.robot_x, self.robot_y])
        self.samples = multivariate_normal.rvs(mean=self.current_mean, cov=self.position_covariance, size=self.num_samples)
        current_probability_grid = self.calculate_probability_grid(self.samples)
        self.current_safety = self.calculate_safety_for_grid(current_probability_grid)

        # Project the robot's position forward
        self.projected_x, self.projected_y, self.projected_yaw = self.project_position(
            self.robot_x, self.robot_y, self.robot_yaw,
            self.linear_velocity, self.angular_velocity,
            1.0 / self.update_frequency
        )

        # Generate samples for the projected position
        self.projected_mean = np.array([self.projected_x, self.projected_y])
        self.projected_samples = multivariate_normal.rvs(mean=self.projected_mean, cov=self.position_covariance, size=self.num_samples)
        projected_probability_grid = self.calculate_probability_grid(self.projected_samples)
        self.projected_safety = self.calculate_safety_for_grid(projected_probability_grid)

        self.safety_gradient = self.calculate_safety_gradient(self.current_mean, self.current_safety, self.projected_mean, self.projected_safety)

        self.update_visualization()


        # Log safety and gradient information
        #self.log_safety_and_gradient()

    # Differential Drive
    # Assuming Unicycle Model Kinematics
    # Param: curr x, curr y, curr yaw, linear velcoity, angular velocity, time to project
    def project_position(self, x, y, yaw, v, w, t):
        # Moving in a straight line
        if abs(w) < 1e-6:
            new_x = x + v * t * math.cos(yaw)
            new_y = y + v * t * math.sin(yaw)
            new_yaw = yaw
        # Moving in an arc
        else:
            new_x = x + (v / w) * (math.sin(yaw + w * t) - math.sin(yaw))
            new_y = y + (v / w) * (math.cos(yaw) - math.cos(yaw + w * t))
            new_yaw = yaw + w * t
        return new_x, new_y, new_yaw

    # Calculate robot yaw (for projection)
    # quaterion in this case is the orientation of the robot in 3D/2D space
    # (Can scale to 3D with roll and pitch)
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        # in radians
        return roll_x, pitch_y, yaw_z

    # Converts points to grid data
    # Relies on world_to_grid
    def calculate_probability_grid(self, samples):
        if self.map is None:
            return None

        prob_grid = np.zeros((self.map.info.height, self.map.info.width))
        
        for sample in samples:
            grid_x, grid_y = self.world_to_grid(sample[0], sample[1])
            if 0 <= grid_x < self.map.info.width and 0 <= grid_y < self.map.info.height:
                prob_grid[grid_y, grid_x] += 1

        prob_grid /= len(samples)
        return prob_grid

    # Converts sample point into nearest grid for P(x)
    def world_to_grid(self, x, y):
        grid_x = int((x - self.map_origin[0]) / self.map_resolution)
        grid_y = int((y - self.map_origin[1]) / self.map_resolution)
        return grid_x, grid_y

    # Takes P(x) as paramter
    # Calculates P(x) -- in context of safety at cell, not of collision
    # Returns P(x) * O(x)
    def calculate_safety_for_grid(self, probability_grid):
        if self.map is None or probability_grid is None or self.safety_prob is None:
            return 0.0
        
        #occupancy_array = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        # Invert occupancy for safety
        #safety_prob = np.where(occupancy_array == -1, 0.5, 
        #                       np.where(occupancy_array == 0, 1.0, 0.0))
        
        return np.dot(probability_grid.flatten(), self.safety_prob.flatten())
    

    def calculate_safety_gradient(self, current_mean, current_safety, projected_mean, projected_safety):
        # Calculate the vector from current to projected position
        direction = projected_mean - current_mean
        
        # Calculate the change in safety
        safety_difference = projected_safety - current_safety

        # Avoid division by zero
        # normalize direction calculation for consistent result
        distance = np.linalg.norm(direction)
        # If distance is very small, return zero gradient
        if distance < 1e-6 or safety_difference == 0:
            return np.zeros(2)  
        
        # Calculate the gradient
        gradient = (safety_difference / distance) * (direction / distance)
        return gradient

    def compute_safe_control(self, u_tele):
        # Bounds for linear and angular velocities
        bounds = [
            (-self.max_linear_velocity, self.max_linear_velocity),
            (-self.max_angular_velocity, self.max_angular_velocity)
        ]

        nonlinear_constraint = NonlinearConstraint(
            self.control_constraint,
            0,
            np.inf,
            keep_feasible=False
        )

        # solve QP
        result = minimize(
            self.control_cost,
            u_tele,
            args=(u_tele,),
            # try different methods still, go back to least common squares
            # method='trust-constr', BEST
            # method='SLSQP', BAD, Positive directional derivative for linesearch at EDGES
            # method='COBYLA', BAD, couldnt converge at EDGES
            method='trust-constr',
            constraints=[nonlinear_constraint],
            bounds=bounds,
            options={
                #'verbose': 2,
                'maxiter': 1000
                #'disp': True
            }
        )
        
        self.get_logger().info(f"Current safety: {self.current_safety}")
        if not result.success:
            self.get_logger().warn(f"Optimization failed: {result.message}")
            # Default to sending user input (not safe)
            return u_tele
        
        self.get_logger().info(f"Optimization succeeded. Safe control: {result.x[0]:.4f}, {result.x[1]:.4f}")
        #self.update_visualization()

        # safety input
        return result.x

    def control_cost(self, u, u_tele):
        # Define the cost function to minimize (u_safe - u_tele)^2
        return np.sum((u - u_tele) ** 2)

    def control_constraint(self, u):
        # project the robot's position using the proposed control input

        projected_x, projected_y, _ = self.project_position(
            self.robot_x, self.robot_y, self.robot_yaw,
            u[0], u[1], 1.0 / self.update_frequency
        )

        # Get interpolated safety value and gradient for the projected position

        projected_mean = np.array([projected_x, projected_y])
        self.qp_projected_mean = projected_mean
        
        projected_samples = multivariate_normal.rvs(mean=projected_mean, cov=self.position_covariance, size=self.num_samples)
        projected_probability_grid = self.calculate_probability_grid(projected_samples)

        projected_safety = self.calculate_safety_for_grid(projected_probability_grid)
        self.qp_projected_safety = projected_safety
        h_b = projected_safety
        #h_b = self.current_safety

        # calculate_safety_gradient(self, current_mean, current_safety, projected_mean, projected_safety):
        #safety_gradient = self.calculate_safety_gradient(self.projected_mean, self.current_safety, projected_mean, projected_safety)
        safety_gradient = self.calculate_safety_gradient(self.current_mean, self.current_safety, projected_mean, projected_safety)
        self.qp_projected_gradient = safety_gradient

        # Calculate safety of new porjected point
        # Calculate gradient between original point and test point

        # Constraint: grad_h(f'(b) + g'(b)u) + alpha * h(b) >= 0
        # translates to
        # Constraint: grad_h(projected position with u) * u + alpha * h(belief at u) >= 0
        cbf_constraint = np.dot(safety_gradient, u[:2]) + self.alpha * h_b
        
        # Additional constraint: h(projected position with u) >= desired_safety
        safety_constraint = h_b - self.desired_safety
        
        # Shouldn't uncomment, slow solver
        #self.get_logger().info(f"CBF Constraint Value: {cbf_constraint:.4f} (should be >= 0)")
        #self.get_logger().info(f"Safety Constraint Value: {safety_constraint:.4f} (should be >= 0)")
        
        return np.array([cbf_constraint, safety_constraint])
    
    def update_visualization(self):
        if self.map is None:
            return

        self.ax.clear()

        # size by occupancy grid
        occupancy_array = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        self.ax.imshow(occupancy_array, cmap='gray_r', origin='lower', extent=[
            self.map_origin[0],
            self.map_origin[0] + self.map.info.width * self.map_resolution,
            self.map_origin[1],
            self.map_origin[1] + self.map.info.height * self.map_resolution
        ])

        # Plot current vs projected position
        self.ax.plot(self.robot_x, self.robot_y, 'ro', markersize=10, label='Current')
        self.ax.plot(self.projected_mean[0], self.projected_mean[1], 'bo', markersize=10, label='Projected')

        # Plot safety gradient
        gradient_scale = 0.5  # the gradient arrow size scalar
        self.ax.arrow(self.robot_x, self.robot_y, 
                      self.safety_gradient[0] * gradient_scale, 
                      self.safety_gradient[1] * gradient_scale, 
                      head_width=0.1, head_length=0.5, fc='g', ec='g', label='Gradient')

        # Add safety values as text
        self.ax.text(self.robot_x, self.robot_y + 0.2, f'Safety: {self.current_safety:.2f}', 
                     ha='center', va='bottom', color='red')
        self.ax.text(self.projected_mean[0], self.projected_mean[1] + 0.2, f'Safety: {self.projected_safety:.2f}', 
                     ha='center', va='bottom', color='blue')

        self.ax.set_title('Robot Position and Safety Gradient')
        self.ax.legend()
        self.ax.set_aspect('equal')

        # Update the plot
        self.canvas.draw()
        plt.pause(0.1)  # Small pause to allow the plot to update

    def log_safety_and_gradient(self):
        self.get_logger().info(f"Current robot position: ({self.robot_x:.2f}, {self.robot_y:.2f})")
        self.get_logger().info(f"Current safety value: {self.current_safety:.6f}")
        self.get_logger().info(f"Projected robot position: ({self.projected_x:.2f}, {self.projected_y:.2f})")
        #self.get_logger().info(f"Projected safety value: {self.projected_safety:.6f}")

def main(args=None):
    rclpy.init(args=args)
    robot_position_mapper = RobotPositionMapper()
    
    try:
        rclpy.spin(robot_position_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        robot_position_mapper.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        # plot open after shutdown
        plt.show()


if __name__ == '__main__':
    main()
