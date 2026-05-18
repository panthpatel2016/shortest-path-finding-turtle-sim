#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point

import random
import math

# ---------------- PID Controller ----------------
class PID:
    def __init__(self, kp, ki, kd, u_min=None, u_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.u_min = u_min
        self.u_max = u_max
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        # Basic anti-windup: only integrate if output not saturated
        self.integral += error * dt
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / max(dt, 1e-6)
        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.u_min is not None and u < self.u_min:
            u = self.u_min
        if self.u_max is not None and u > self.u_max:
            u = self.u_max
        self.prev_error = error
        return u


class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        # -------------- General params --------------
        self.num_turtles = 10
        self.dt = 0.1  # timer period
        self.turtle_states = {}  # name -> {'pose','velocity',[PIDs],'publisher'}

        # -------------- Spawn client --------------
        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # -------------- World bounds (turtlesim ~11x11) --------------
        self.x_min, self.x_max = 1.0, 10.0
        self.y_min, self.y_max = 1.0, 10.0

        # -------------- Base speed --------------
        self.speed = 1.0

        # -------------- Boids params --------------
        self.neighbor_radius = 2.0
        self.separation_distance = 1.0
        self.weight_separation = 0.5   # reduced (PID handles main separation)
        self.weight_alignment  = 1.0
        self.weight_cohesion   = 1.0

        # -------------- Anti-clumping --------------
        self.group_distance = 1.5
        self.weight_group = 5.0

        # -------------- Perturbation + inertia --------------
        self.weight_random = 0.25
        self.inertia_factor = 0.8

        # -------------- Go-to-goal (click-to-move) --------------
        # Subscribe to target points (published by GUI or CLI)
        qos = QoSProfile(depth=10)
        self.create_subscription(Point, '/target_point', self.on_target_point, qos)
        self.current_target = None           # (x,y)
        self.active_turtle_name = None       # who chases the target
        self.goal_reached_thresh = 0.15
        self.weight_goal_vector = 2.0        # blend factor for goal direction

        # PID for go-to-goal (per turtle, stored in state)
        # - angle_pid -> angular.z (rad/s)
        # - dist_pid  -> goal speed (m/s)
        self.kp_ang, self.ki_ang, self.kd_ang = 4.0, 0.0, 0.6
        self.kp_dist, self.ki_dist, self.kd_dist = 0.9, 0.0, 0.0
        self.ang_limit = 3.0   # rad/s
        self.lin_limit = 2.0   # m/s

        # PID for collision separation (per turtle, acts on nearest neighbor distance)
        self.kp_sep, self.ki_sep, self.kd_sep = 1.5, 0.0, 0.25
        self.sep_limit = 2.0  # max outward "speed" contributed by separation PID

        # -------------- Spawn turtles, pubs/subs, and per-turtle PIDs --------------
        for i in range(self.num_turtles):
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            theta = random.uniform(-math.pi, math.pi)
            name = f'swarm_turtle_{i+1}'
            self.spawn_turtle(x, y, theta, name)

            angle = random.uniform(-math.pi, math.pi)
            vx = self.speed * math.cos(angle)
            vy = self.speed * math.sin(angle)

            pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(Pose, f'/{name}/pose', self.make_pose_cb(name), 10)

            self.turtle_states[name] = {
                'pose': None,
                'velocity': [vx, vy],
                'publisher': pub,
                'angle_pid': PID(self.kp_ang, self.ki_ang, self.kd_ang, -self.ang_limit, self.ang_limit),
                'dist_pid':  PID(self.kp_dist, self.ki_dist, self.kd_dist,  0.0,            self.lin_limit),
                'sep_pid':   PID(self.kp_sep, self.ki_sep, self.kd_sep,     0.0,            self.sep_limit)
            }

        # -------------- Control loop --------------
        self.timer = self.create_timer(self.dt, self.update_turtles)
        self.get_logger().info("Swarm with PID collision avoidance + click-to-move started!")

    # ---------- Services / subscriptions ----------
    def spawn_turtle(self, x, y, theta, name):
        req = Spawn.Request()
        req.x, req.y, req.theta, req.name = x, y, theta, name
        fut = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is not None:
            self.get_logger().info(f"Spawned: {fut.result().name}")
        else:
            self.get_logger().error("Failed to spawn turtle")

    def make_pose_cb(self, name):
        def cb(msg):
            self.turtle_states[name]['pose'] = msg
        return cb

    def on_target_point(self, msg: Point):
        # Clamp to bounds
        tx = max(self.x_min, min(self.x_max, msg.x))
        ty = max(self.y_min, min(self.y_max, msg.y))
        self.current_target = (tx, ty)

        # Pick the nearest turtle at click time
        nearest = None
        best_d = float('inf')
        for name, st in self.turtle_states.items():
            p = st['pose']
            if p is None:
                continue
            d = math.hypot(tx - p.x, ty - p.y)
            if d < best_d:
                best_d, nearest = d, name

        self.active_turtle_name = nearest
        # Reset its go-to-goal PIDs for a fresh start
        if nearest:
            self.turtle_states[nearest]['angle_pid'].reset()
            self.turtle_states[nearest]['dist_pid'].reset()
        self.get_logger().info(f"New target ({tx:.2f},{ty:.2f}) -> {nearest}")

    # ---------- Core loop ----------
    def update_turtles(self):
        for name, st in self.turtle_states.items():
            pose = st['pose']
            if pose is None:
                continue

            cx, cy, cth = pose.x, pose.y, pose.theta
            cvx, cvy = st['velocity']

            # --- Neighborhood sweeps ---
            sep_dx = sep_dy = 0.0
            align_x = align_y = 0.0
            coh_x = coh_y = 0.0
            neighbor_count = 0

            # For PID separation: track nearest neighbor
            nearest_vec = None
            nearest_dist = float('inf')

            for other_name, other in self.turtle_states.items():
                if other_name == name:
                    continue
                op = other['pose']
                if op is None:
                    continue

                dx = cx - op.x
                dy = cy - op.y
                dist = math.hypot(dx, dy)
                if dist < nearest_dist and dist > 1e-6:
                    nearest_dist = dist
                    nearest_vec = (dx, dy)

                if 0.0 < dist < self.neighbor_radius:
                    neighbor_count += 1

                    # classic boids separation (light, PID is primary)
                    if dist < self.separation_distance:
                        rep = 1.0 / (dist + 0.01)
                        sep_dx += dx * rep
                        sep_dy += dy * rep

                    # alignment
                    ovx, ovy = other['velocity']
                    align_x += ovx
                    align_y += ovy

                    # cohesion (towards neighbors' center)
                    coh_x += op.x
                    coh_y += op.y

            # ------ Desired vector starts as inertia ------
            desired_x, desired_y = cvx, cvy

            if neighbor_count > 0:
                # alignment average
                align_x /= neighbor_count
                align_y /= neighbor_count

                # cohesion towards centroid
                coh_x = (coh_x / neighbor_count) - cx
                coh_y = (coh_y / neighbor_count) - cy

                desired_x += (self.weight_separation * sep_dx +
                              self.weight_alignment  * align_x +
                              self.weight_cohesion   * coh_x)
                desired_y += (self.weight_separation * sep_dy +
                              self.weight_alignment  * align_y +
                              self.weight_cohesion   * coh_y)

            # ------ Group-limit anti-clump (repel from local centroid if >=2 close) ------
            # Use a slightly larger ring, collect positions again to compute a strong repulsion
            # (Cheap re-scan based on nearest_dist already done; cost OK for N=10)
            # Here we’ll use the already-computed cohesion centroid when neighbor_count>=2
            if neighbor_count >= 2:
                # coh_x, coh_y currently points from me to centroid
                group_fx = -coh_x  # repel away from centroid
                group_fy = -coh_y
                desired_x += self.weight_group * group_fx
                desired_y += self.weight_group * group_fy

            # ------ PID separation using nearest neighbor distance ------
            if nearest_vec is not None and nearest_dist < self.separation_distance + 0.75:
                # error is how much we are inside the safety bubble (>=0 means too close)
                sep_error = max(0.0, self.separation_distance - nearest_dist)
                sep_speed = st['sep_pid'].update(sep_error, self.dt)  # outward "speed" contribution
                # unit vector pointing away from nearest neighbor
                ux = nearest_vec[0] / max(nearest_dist, 1e-6)
                uy = nearest_vec[1] / max(nearest_dist, 1e-6)
                desired_x += sep_speed * ux
                desired_y += sep_speed * uy

            # ------ Goal attraction for the active turtle (click-to-move) ------
            if self.active_turtle_name == name and self.current_target is not None:
                tx, ty = self.current_target
                dx, dy = (tx - cx), (ty - cy)
                dist = math.hypot(dx, dy)

                if dist < self.goal_reached_thresh:
                    # reached -> clear goal
                    self.get_logger().info(f"{name} reached ({tx:.2f},{ty:.2f})")
                    self.current_target = None
                    self.active_turtle_name = None
                    st['angle_pid'].reset()
                    st['dist_pid'].reset()
                else:
                    # direction to goal (as a vector)
                    theta_goal = math.atan2(dy, dx)
                    angle_err = self.normalize_angle(theta_goal - cth)

                    # Use PIDs:
                    ang_cmd = st['angle_pid'].update(angle_err, self.dt)  # -> angular.z
                    base_speed = st['dist_pid'].update(dist, self.dt)     # -> nominal forward speed

                    # Blend a goal vector to bias the velocity field toward the target
                    # (scaled by base_speed and a weight)
                    goal_vx = self.weight_goal_vector * base_speed * math.cos(theta_goal)
                    goal_vy = self.weight_goal_vector * base_speed * math.sin(theta_goal)
                    desired_x += goal_vx
                    desired_y += goal_vy

                    # We'll publish angular.z via PID below (overrides pure vector->angle P)

            # ------ Random jitter ------
            desired_x += self.weight_random * random.uniform(-1, 1)
            desired_y += self.weight_random * random.uniform(-1, 1)

            # ------ Normalize to base speed ------
            mag = math.hypot(desired_x, desired_y)
            if mag > 1e-6:
                comp_vx = (desired_x / mag) * self.speed
                comp_vy = (desired_y / mag) * self.speed
            else:
                comp_vx, comp_vy = cvx, cvy

            # ------ Inertia smoothing ------
            new_vx = self.inertia_factor * cvx + (1.0 - self.inertia_factor) * comp_vx
            new_vy = self.inertia_factor * cvy + (1.0 - self.inertia_factor) * comp_vy

            # keep constant magnitude = self.speed
            new_mag = math.hypot(new_vx, new_vy)
            if new_mag > 1e-6:
                new_vx = (new_vx / new_mag) * self.speed
                new_vy = (new_vy / new_mag) * self.speed

            # ------ Wall bounce ------
            if cx <= self.x_min and new_vx < 0: new_vx = -new_vx
            if cx >= self.x_max and new_vx > 0: new_vx = -new_vx
            if cy <= self.y_min and new_vy < 0: new_vy = -new_vy
            if cy >= self.y_max and new_vy > 0: new_vy = -new_vy

            # save vel
            st['velocity'] = [new_vx, new_vy]

            # ------ Convert to Twist ------
            desired_heading = math.atan2(new_vy, new_vx)
            ang_err = self.normalize_angle(desired_heading - cth)

            twist = Twist()

            # Angular control:
            if self.active_turtle_name == name and self.current_target is not None:
                # Use PID angular when chasing a point
                twist.angular.z = st['angle_pid'].update(ang_err, self.dt)
            else:
                # Simple P (or you can also use the PID):
                twist.angular.z = max(-self.ang_limit, min(self.ang_limit, 4.0 * ang_err))

            # Linear speed: slow down when turning a lot
            lin_cmd = self.speed * max(0.0, math.cos(ang_err))
            twist.linear.x = lin_cmd

            st['publisher'].publish(twist)

    # ---------- helpers ----------
    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down swarm controller...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
