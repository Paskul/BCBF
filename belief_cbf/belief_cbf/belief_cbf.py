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


'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np
from scipy.stats import multivariate_normal
from scipy.optimize import minimize, NonlinearConstraint
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt

class RBFInterpolator:
    # Radial Basis Function
    # Currently working epsilon of 5.0
        # controls the width of the gaussian
    # We do this to have smooth safety, and differentiability
    
    def __init__(self, epsilon=1.0):
        self.epsilon = epsilon
        self.test_points = None
        self.weights = None
        self.values = None

    def fit(self, test_points, values):
        self.test_points = np.array(test_points)
        self.values = np.array(values)
        # kernel matrix
        # is the similarity or influence between all pairs of known points
        kernel_matrix = self.rbf_kernel(self.test_points, self.test_points)
        # Get weights
        self.weights = np.linalg.solve(kernel_matrix, self.values)

    def rbf_kernel(self, X, Y):
        distances = cdist(X, Y)
        return np.exp(-(self.epsilon * distances) ** 2)

    def interpolate(self, points):
        if self.test_points is None or self.weights is None:
            raise ValueError("need to fit interpolator first.")

        points = np.atleast_2d(points)
        kernel_values = self.rbf_kernel(points, self.test_points)
        return np.dot(kernel_values, self.weights)

    def gradient(self, points):
        if self.test_points is None or self.weights is None:
            raise ValueError("need to fit interpolator first.")

        points = np.atleast_2d(points)
        # (1, n_test_points, n_dimensions)
        # (n_points, 1, n_dimensions)
        # differences is (n_points, n_test_points, n_dimensions)
        differences = self.test_points[np.newaxis, :, :] - points[:, np.newaxis, :]
        kernel_values = self.rbf_kernel(points, self.test_points)

        gradients = 2 * self.epsilon**2 * np.sum(
            self.weights[np.newaxis, :, np.newaxis] * kernel_values[:, :, np.newaxis] * differences,
            axis=1
        )

        # convert back to 1D array
        return gradients.squeeze()

def create_rbf_interpolator(test_points, value, epsilon=5.0):
    test_points = np.array(test_points)
    values = np.array([value[tuple(test_point)]['safety'] for test_point in test_points])

    interpolator = RBFInterpolator(epsilon)
    interpolator.fit(test_points, values)

    return interpolator

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
        self.map = None
        self.map_resolution = None
        self.map_origin = None
        self.samples = None
        self.num_samples = 500
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.max_linear_velocity = 0.22
        self.max_angular_velocity = 2.84
        self.alpha = 1.0
        self.desired_safety = 0.97
        self.update_frequency = 2 # relates with prediction_time
        
        # Projected position
        self.projected_x = None
        self.projected_y = None
        self.projected_yaw = None
        
        # Prediction time
        self.prediction_time = 0.5  # 0.1 seconds into the future
        
        # Parameters for gradient calculation
        self.velocity_step = 0.25  # Velocity step size for gradient calculation
        self.grid_size = 10  # Size of the grid for gradient calculation (10x10 grid)
        # ^^ Final product, need to finalize required size for gradient calc (from max velocity)
        self.safety_gradient = np.zeros(2)
        self.rbf_interpolator = None

        # Safety values
        self.current_safety = None

    def map_callback(self, msg):
        self.map = msg
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.get_logger().info('Received new map')

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        safe_control = self.compute_safe_control(np.array([self.linear_velocity, self.angular_velocity]))
        self.get_logger().info(f'safe control!: {safe_control[0]:.4f}, {safe_control[1]:.4f}')

    def pose_callback(self, msg):
        # Extract the robot position, orientation, and covariance
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Uncomment if to take cov from robot... but we artifically set as high for dev
        #cov_matrix = np.array(msg.pose.covariance).reshape(6, 6)
        #position_covariance = cov_matrix[:2, :2]
        position_covariance = [[0.009, 0.0], [0.0, 0.009]]

        # Calculate safety at current position
        current_mean = np.array([self.robot_x, self.robot_y])
        current_samples = multivariate_normal.rvs(mean=current_mean, cov=position_covariance, size=self.num_samples)
        current_probability_grid = self.calculate_probability_grid(current_samples)
        self.current_safety = self.calculate_safety_for_grid(current_probability_grid)

        # Project the robot's position forward
        self.projected_x, self.projected_y, self.projected_yaw = self.project_position(
            self.robot_x, self.robot_y, self.robot_yaw,
            self.linear_velocity, self.angular_velocity,
            1.0 / self.update_frequency
        )

        # Generate samples for the projected position
        projected_mean = np.array([self.projected_x, self.projected_y])
        self.samples = multivariate_normal.rvs(mean=projected_mean, cov=position_covariance, size=self.num_samples)

        # Calculate safety gradient at the projected position
        self.calculate_safety_gradient()

        # Log safety and gradient information
        #self.log_safety_and_gradient()

    # Differential Drive
    # Assuming Unicycle Model Kinematics
    # Param: x, y, yaw, linear velcoity, angular velocity, time to project
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
     
        return roll_x, pitch_y, yaw_z  # in radians

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
        if self.map is None or probability_grid is None:
            return 0.0
        
        occupancy_array = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        # Invert occupancy for safety
        safety_prob = np.where(occupancy_array == -1, 0.5, 
                               np.where(occupancy_array == 0, 1.0, 0.0))
        
        return np.sum(probability_grid * safety_prob)
    
    # Creates 'box' around projection mean for 'safety' testing
    # velocity_step determines range, grid_size determines quantity
    # Allows for smooth safety/gradient in radial basis function
    def calculate_grid_points(self):
        x = np.linspace(self.projected_x - self.velocity_step, self.projected_x + self.velocity_step, self.grid_size)
        y = np.linspace(self.projected_y - self.velocity_step, self.projected_y + self.velocity_step, self.grid_size)
        xx, yy = np.meshgrid(x, y)
        grid_points = np.column_stack((xx.ravel(), yy.ravel()))
        return grid_points

    # Depends on calculate_grid_points
    # Finds safety at each test grid point made
    # Allows for smooth safety/gradient in radial basis function
    def calculate_safety_values(self, grid_points):
        safety_values = np.zeros(grid_points.shape[0])
        for i, point in enumerate(grid_points):
            shifted_samples = self.samples + (point - np.array([self.projected_x, self.projected_y]))
            shifted_probability_grid = self.calculate_probability_grid(shifted_samples)
            safety_values[i] = self.calculate_safety_for_grid(shifted_probability_grid)
        return safety_values

    # From our test points and their safety, creates radial basis function to follow
    def calculate_safety_gradient(self):
        if self.samples is None or self.projected_x is None or self.projected_y is None:
            self.get_logger().warn('Samples or projected position not initialized.')
            return
        
        # Grid of points around the testing projection
        grid_points = self.calculate_grid_points()
        safety_values = self.calculate_safety_values(grid_points)

        # Give the RBF interpolator with the grid points and their corresponding safety values
        # data prep, giving each test point a safety value
        # Epsilon 5 works better
        self.rbf_interpolator = create_rbf_interpolator(grid_points, 
                                                        {tuple(point): {'safety': safety} for point, safety in zip(grid_points, safety_values)}, 
                                                        epsilon=5.0)

        # Calculate the safety gradient at the projected position
        projected_point = np.array([[self.projected_x, self.projected_y]])
        self.safety_gradient = self.rbf_interpolator.gradient(projected_point)

        # for logging
        # safety values at the current pos
        robot_point = np.array([[self.robot_x, self.robot_y]])
        #self.current_safety = self.rbf_interpolator.interpolate(robot_point)[0]
        self.projected_safety = self.rbf_interpolator.interpolate(projected_point)[0]

        #self.log_safety_and_gradient()

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
            method='trust-constr',
            constraints=[nonlinear_constraint],
            bounds=bounds,
            options={
                #'verbose': 2,
                'maxiter': 500
                #'disp': True
            }
        )
        
        self.get_logger().info(f"Current safety: {self.current_safety}")
        if not result.success:
            self.get_logger().warn(f"Optimization failed: {result.message}")
            return u_tele
        
        self.get_logger().info(f"Optimization succeeded. Safe control: {result.x[0]:.4f}, {result.x[1]:.4f}")
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
        h_b = self.rbf_interpolator.interpolate(np.array([[projected_x, projected_y]]))[0]
        safety_gradient = self.rbf_interpolator.gradient(np.array([[projected_x, projected_y]])).flatten()

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

    def log_safety_and_gradient(self):
        self.get_logger().info(f"Current robot position: ({self.robot_x:.2f}, {self.robot_y:.2f})")
        self.get_logger().info(f"Current safety value: {self.current_safety:.6f}")
        self.get_logger().info(f"Projected robot position: ({self.projected_x:.2f}, {self.projected_y:.2f})")
        self.get_logger().info(f"Projected safety value: {self.projected_safety:.6f}")

    
    #def visualize_gradients(self):
    #    if self.map is None or self.rbf_interpolator is None:
    #        self.get_logger().warn("Map or RBF interpolator not available for visualization")
    #        return

    #    fig, ax = plt.subplots(figsize=(12, 12))

        # occupancy grid
    #    occupancy_array = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
    #    ax.imshow(occupancy_array, cmap='gray_r', origin='lower', extent=[
    #        self.map_origin[0],
    #        self.map_origin[0] + self.map.info.width * self.map_resolution,
    #        self.map_origin[1],
    #        self.map_origin[1] + self.map.info.height * self.map_resolution
    #    ])

        # Plot points with safety values
    #    grid_points = self.calculate_grid_points()
    #    safety_values = self.calculate_safety_values(grid_points)
    #    scatter = ax.scatter(grid_points[:, 0], grid_points[:, 1], c=safety_values, cmap='viridis', label='Safety Values')

    #    cbar = fig.colorbar(scatter, ax=ax, label='Safety Value')
    #    ax.set_xlabel('X (m)')
    #    ax.set_ylabel('Y (m)')
    #    ax.set_title('Test Points Safety Visualization')
    #    ax.legend()
    #    ax.set_aspect('equal')
    #    plt.tight_layout()
    #    plt.show()
    #    self.get_logger().info("Displayed test points safety visualization")

    def visualize_gradients(self):
        if self.map is None or self.rbf_interpolator is None:
            self.get_logger().warn("Map or RBF interpolator not available for visualization")
            return

        fig, ax = plt.subplots(figsize=(12, 12))

        # plot occup grid
        occupancy_array = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        ax.imshow(occupancy_array, cmap='gray_r', origin='lower', extent=[
            self.map_origin[0],
            self.map_origin[0] + self.map.info.width * self.map_resolution,
            self.map_origin[1],
            self.map_origin[1] + self.map.info.height * self.map_resolution
        ])

        # Same as earlier, make grid for smooth visualization
        x = np.linspace(self.projected_x - self.velocity_step, self.projected_x + self.velocity_step, 100)
        y = np.linspace(self.projected_y - self.velocity_step, self.projected_y + self.velocity_step, 100)
        X, Y = np.meshgrid(x, y)
        points = np.column_stack((X.ravel(), Y.ravel()))

        # Interpolate safety values using smoother
        Z = self.rbf_interpolator.interpolate(points).reshape(X.shape)

        # Plot interpolated safety values
        safety_plot = ax.contourf(X, Y, Z, levels=50, cmap='RdYlGn', alpha=0.7)

        # Plot gradients
        # skip depends on linspace size, 100/skip 10 == 10 grad layers
        skip = 10
        gradient_points = points[::skip]
        gradients = self.rbf_interpolator.gradient(gradient_points)
        gradient_norms = np.linalg.norm(gradients, axis=1)
        
        # Avoid division by zero
        gradient_norms[gradient_norms == 0] = 1
        normalized_gradients = gradients / gradient_norms[:, np.newaxis]

        for point, gradient in zip(gradient_points, normalized_gradients):
            dx, dy = gradient
            if np.isfinite(dx) and np.isfinite(dy):
                ax.arrow(point[0], point[1],
                        dx * self.velocity_step / 10, dy * self.velocity_step / 10,
                        head_width=self.velocity_step/100, head_length=self.velocity_step/100,
                        fc='black', ec='black', alpha=0.3)

        # plot with projected position
        ax.plot(self.robot_x, self.robot_y, 'bo', markersize=10, label='Robot')
        ax.plot(self.projected_x, self.projected_y, 'go', markersize=8, label='Projected Position')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Robot Safety Visualization with RBF Interpolation')
        ax.legend()
        ax.set_aspect('equal')
        cbar = fig.colorbar(safety_plot, ax=ax, label='Safety Value')
        plt.tight_layout()
        plt.show()
        self.get_logger().info("Displayed robot safety visualization")

    def on_shutdown(self):
        self.get_logger().info("Node is shutting down. Generating final visualization...")
        self.visualize_gradients()
        self.get_logger().info("Visualization complete. Shutting down.")
        plt.show(block=True)

def main(args=None):
    rclpy.init(args=args)
    robot_position_mapper = RobotPositionMapper()
    
    try:
        rclpy.spin(robot_position_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        robot_position_mapper.on_shutdown()
        robot_position_mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''