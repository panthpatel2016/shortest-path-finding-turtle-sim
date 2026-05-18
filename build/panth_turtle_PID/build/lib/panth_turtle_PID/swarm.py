


import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import random
import math

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
       
        # Number of turtles to spawn
        self.num_turtles = 10
       
        # Dictionary to store each turtle's state:
        # key: turtle name,
        # value: dict with 'pose', 'velocity' (as [vx, vy]), and 'publisher'
        self.turtle_states = {}

        # Create a client for the spawn service (provided by turtlesim_node)
        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
       
        # Define safe boundaries for spawning and for wall-bouncing.
        # The turtlesim window is roughly 11x11.
        self.x_min = 1.0
        self.x_max = 10.0
        self.y_min = 1.0
        self.y_max = 10.0

        # Speed (magnitude of the velocity) for the turtles.
        self.speed = 1.0
       
        # --- Parameters for the swarm (flocking) rules ---
        # Neighborhood radius: turtles within this distance are considered neighbors.
        self.neighbor_radius = 2.0
        # Separation distance: if turtles are closer than this, they steer away.
        self.separation_distance = 1.0
       
        # Weights for the boids rules.
        self.weight_separation = 2.0
        self.weight_alignment = 1.0
        self.weight_cohesion = 1.0
       
        # --- New Rule Parameters: Group Limit ---
        # Group distance: if turtles are within this distance, they are considered part of a group.
        self.group_distance = 1.5
        # Weight for the group limit repulsion force.
        self.weight_group = 5.0

        # --- New Parameter: Random Perturbation ---
        # A small random term helps break symmetry and avoids stagnation.
        self.weight_random = 0.3

        # --- New Parameter: Inertia Smoothing ---
        # This factor (between 0 and 1) blends the previous velocity with the computed force.
        # A higher value means slower reaction (more inertia).
        self.inertia_factor = 0.8

        # Spawn turtles and set up subscribers/publishers for each.
        for i in range(self.num_turtles):
            # Generate random initial position and orientation.
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            theta = random.uniform(-math.pi, math.pi)
           
            # Unique name for each turtle.
            turtle_name = f'swarm_turtle_{i+1}'
            self.spawn_turtle(x, y, theta, turtle_name)
           
            # Initialize each turtle with a random velocity (providing initial inertia).
            random_angle = random.uniform(-math.pi, math.pi)
            vx = self.speed * math.cos(random_angle)
            vy = self.speed * math.sin(random_angle)
            self.turtle_states[turtle_name] = {'pose': None, 'velocity': [vx, vy]}
           
            # Create a publisher for the turtle's cmd_vel topic.
            pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
            self.turtle_states[turtle_name]['publisher'] = pub
           
            # Create a subscription to the turtle's pose.
            self.create_subscription(
                Pose,
                f'/{turtle_name}/pose',
                self.create_pose_callback(turtle_name),
                10
            )
       
        # Timer: update turtle commands at 10 Hz.
        self.timer = self.create_timer(0.1, self.update_turtles)
        self.get_logger().info("Enhanced swarm controller with inertia smoothing started!")

    def spawn_turtle(self, x, y, theta, name):
        """Call the spawn service to create a new turtle."""
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = name

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {future.result().name}")
        else:
            self.get_logger().error("Failed to spawn turtle")

    def create_pose_callback(self, turtle_name):
        """Create a closure to capture the turtle_name in the callback."""
        def callback(msg):
            self.turtle_states[turtle_name]['pose'] = msg
        return callback

    def update_turtles(self):
        """Timer callback: compute and publish cmd_vel for each turtle based on swarm rules."""
        for turtle_name, state in self.turtle_states.items():
            pose = state['pose']
            if pose is None:
                continue  # Wait until we have a pose measurement.

            current_x = pose.x
            current_y = pose.y
            current_velocity = state['velocity']

            # Initialize accumulators for the flocking rules.
            sep_x, sep_y = 0.0, 0.0
            align_x, align_y = 0.0, 0.0
            coh_x, coh_y = 0.0, 0.0
            neighbor_count = 0

            # For the Group Limit rule: collect positions of neighbors within group_distance.
            group_positions = []

            # Loop over all other turtles to accumulate neighbor influences.
            for other_name, other_state in self.turtle_states.items():
                if other_name == turtle_name:
                    continue
                other_pose = other_state['pose']
                if other_pose is None:
                    continue

                neighbor_x = other_pose.x
                neighbor_y = other_pose.y
                dx = current_x - neighbor_x
                dy = current_y - neighbor_y
                distance = math.sqrt(dx * dx + dy * dy)

                # Consider only neighbors within the general neighborhood radius.
                if distance < self.neighbor_radius and distance > 0:
                    neighbor_count += 1

                    # --- Enhanced Separation ---
                    if distance < self.separation_distance:
                        # Increase repulsion when very close.
                        repulsion = 1.0 / (distance + 0.01)
                        sep_x += (dx * repulsion)
                        sep_y += (dy * repulsion)

                    # --- Alignment ---
                    neighbor_vel = other_state['velocity']
                    align_x += neighbor_vel[0]
                    align_y += neighbor_vel[1]

                    # --- Cohesion ---
                    coh_x += neighbor_x
                    coh_y += neighbor_y

                # --- Group Limit: Collect positions for neighbors within a slightly larger distance.
                if distance < self.group_distance and distance > 0:
                    group_positions.append((neighbor_x, neighbor_y))

            # Start with the turtle's current velocity (inertia).
            desired_x = current_velocity[0]
            desired_y = current_velocity[1]

            if neighbor_count > 0:
                # --- Alignment: Average neighbors' velocities.
                align_x /= neighbor_count
                align_y /= neighbor_count

                # --- Cohesion: Steer toward the center of mass of neighbors.
                coh_x /= neighbor_count
                coh_y /= neighbor_count
                coh_x = coh_x - current_x
                coh_y = coh_y - current_y

                # Combine the separation, alignment, and cohesion forces.
                desired_x += (self.weight_separation * sep_x +
                              self.weight_alignment * align_x +
                              self.weight_cohesion * coh_x)
                desired_y += (self.weight_separation * sep_y +
                              self.weight_alignment * align_y +
                              self.weight_cohesion * coh_y)

            # --- Group Limit Rule ---
            # If more than 2 turtles (i.e. at least 2 neighbors) are within group_distance,
            # compute the group's center and add a repulsive force away from it.
            if len(group_positions) >= 2:
                sum_x = sum([pos[0] for pos in group_positions])
                sum_y = sum([pos[1] for pos in group_positions])
                avg_x = sum_x / len(group_positions)
                avg_y = sum_y / len(group_positions)
                group_force_x = current_x - avg_x
                group_force_y = current_y - avg_y
                desired_x += self.weight_group * group_force_x
                desired_y += self.weight_group * group_force_y

            # --- Additional Rule: Random Perturbation ---
            desired_x += self.weight_random * random.uniform(-1, 1)
            desired_y += self.weight_random * random.uniform(-1, 1)

            # --- Normalize the computed desired vector to the set speed ---
            mag = math.sqrt(desired_x ** 2 + desired_y ** 2)
            if mag > 0:
                computed_vx = (desired_x / mag) * self.speed
                computed_vy = (desired_y / mag) * self.speed
            else:
                computed_vx, computed_vy = current_velocity

            # --- Inertia Smoothing ---
            # Blend the current velocity with the computed desired velocity.
            new_vx = self.inertia_factor * current_velocity[0] + (1 - self.inertia_factor) * computed_vx
            new_vy = self.inertia_factor * current_velocity[1] + (1 - self.inertia_factor) * computed_vy

            # Normalize the new velocity.
            mag_new = math.sqrt(new_vx ** 2 + new_vy ** 2)
            if mag_new > 0:
                new_vx = (new_vx / mag_new) * self.speed
                new_vy = (new_vy / mag_new) * self.speed
            else:
                new_vx, new_vy = current_velocity

            # --- Wall Avoidance (Bounce) ---
            if pose.x <= self.x_min and new_vx < 0:
                new_vx = -new_vx
            if pose.x >= self.x_max and new_vx > 0:
                new_vx = -new_vx
            if pose.y <= self.y_min and new_vy < 0:
                new_vy = -new_vy
            if pose.y >= self.y_max and new_vy > 0:
                new_vy = -new_vy

            # Save the updated (smoothed) velocity.
            state['velocity'] = [new_vx, new_vy]

            # Compute the desired heading angle.
            desired_angle = math.atan2(new_vy, new_vx)
            angle_error = self.normalize_angle(desired_angle - pose.theta)
           
            # Proportional control for angular velocity.
            k_ang = 4.0
            angular_z = k_ang * angle_error
           
            # Optionally, adjust linear speed when turning.
            if abs(angle_error) < 0.1:
                linear_x = self.speed
            else:
                linear_x = self.speed * math.cos(angle_error)
           
            # Publish the command velocity.
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            state['publisher'].publish(twist)

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down enhanced swarm controller...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
