import math
import heapq
import random
from typing import List, Tuple, Set, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute

# Forces a value between a lower and upper bound
# param x: value to bound
# param lo: lower bound
# param hi: upper bound
# returns: x clamped to [lo, hi]
def bound(x, lo, hi):
    return max(lo, min(hi, x))

# Computes hypotennuse distance between two 2D points
# param a: first point as (x, y)
# param b: second point as (x, y)
# returns: straight-line distance between a and b
def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

# Tests whether a 2D point lies within an axis-aligned rectangle
# param p: point as (x, y)
# param r: rectangle as (x0, y0, x1, y1)
# returns: True if p is inside or on the boundary of r
def point_in_rect(p: Tuple[float, float], r: Tuple[float, float, float, float]) -> bool:
    x, y = p
    x0, y0, x1, y1 = r
    return (x0 <= x <= x1) and (y0 <= y <= y1)


#segment-segment test

# Computes the signed cross product (b-a) x (c-a) to determine turn direction
# param a: first point of reference segment
# param b: second point of reference segment
# param c: point to test orientation against
# returns: positive if left turn, negative if right turn, zero if collinear
def _orient(a: Tuple[float,float], b: Tuple[float,float], c: Tuple[float,float]) -> float:
    # Cross product (b - a) x (c - a); sign tells turn direction
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

# Checks if point c is collinear with segment a-b and lies within its bounding box
# param a: first endpoint of segment
# param b: second endpoint of segment
# param c: point to test
# param eps: tolerance for floating-point collinearity check
# returns: True if c lies on segment a-b
def _on_seg(a: Tuple[float,float], b: Tuple[float,float], c: Tuple[float,float], eps=1e-9) -> bool:
    # Is c colinear with a-b and within the bounding box of a-b?
    if abs(_orient(a,b,c)) > eps:
        return False
    return (min(a[0],b[0]) - eps <= c[0] <= max(a[0],b[0]) + eps and
            min(a[1],b[1]) - eps <= c[1] <= max(a[1],b[1]) + eps)

# Tests whether segment p-q intersects segment r-s, including collinear touching
# param p: first endpoint of segment 1
# param q: second endpoint of segment 1
# param r: first endpoint of segment 2
# param s: second endpoint of segment 2
# param eps: tolerance for collinearity detection
# returns: True if the two segments intersect or touch
def _seg_seg_intersect(p: Tuple[float,float], q: Tuple[float,float],
                       r: Tuple[float,float], s: Tuple[float,float], eps=1e-9) -> bool:
    o1 = _orient(p, q, r)
    o2 = _orient(p, q, s)
    o3 = _orient(r, s, p)
    o4 = _orient(r, s, q)

    # Proper intersection
    if (o1*o2 < 0) and (o3*o4 < 0):
        return True
    # Colinear cases (touching counts as collision)
    if abs(o1) <= eps and _on_seg(p, q, r): return True
    if abs(o2) <= eps and _on_seg(p, q, s): return True
    if abs(o3) <= eps and _on_seg(r, s, p): return True
    if abs(o4) <= eps and _on_seg(r, s, q): return True
    return False

# Tests whether segment p-q intersects or enters an axis-aligned rectangle
# param p: first endpoint of segment
# param q: second endpoint of segment
# param rect: rectangle as (x0, y0, x1, y1)
# returns: True if the segment intersects or has an endpoint inside the rectangle
def segment_intersects_rect(p: Tuple[float,float], q: Tuple[float,float],
                            rect: Tuple[float,float,float,float]) -> bool:
    x0, y0, x1, y1 = rect
    # If either endpoint is inside, it's an intersection
    if point_in_rect(p, rect) or point_in_rect(q, rect):
        return True
    # Check against the 4 rectangle edges
    e1 = ((x0, y0), (x1, y0))
    e2 = ((x1, y0), (x1, y1))
    e3 = ((x1, y1), (x0, y1))
    e4 = ((x0, y1), (x0, y0))
    return (_seg_seg_intersect(p, q, *e1) or
            _seg_seg_intersect(p, q, *e2) or
            _seg_seg_intersect(p, q, *e3) or
            _seg_seg_intersect(p, q, *e4))

# Checks if a segment is free of collision with all obstacle rectangles
# param p: first endpoint of segment
# param q: second endpoint of segment
# param rects: list of obstacle rectangles as (x0, y0, x1, y1)
# returns: True if segment does not intersect any rectangle
def segment_is_free(p: Tuple[float, float], q: Tuple[float, float],
                    rects: List[Tuple[float, float, float, float]]) -> bool:
    # Free if the segment does NOT intersect any rectangle
    for r in rects:
        if segment_intersects_rect(p, q, r):
            return False
    return True











#Dijkstra's Algorithm 

# Finds the shortest path between two nodes using Dijkstra's algorithm
# param start_id: index of the start node
# param goal_id: index of the goal node
# param adj: adjacency list where adj[i] is a list of (neighbor_index, edge_weight) tuples
# returns: ordered list of node indices from start to goal, or empty list if no path exists
def dijkstra_path(start_id: int, goal_id: int, adj: List[List[Tuple[int, float]]]) -> List[int]:
    N = len(adj)
    dist_to = [float('inf')] * N
    prev = [-1] * N
    dist_to[start_id] = 0.0
    pq = [(0.0, start_id)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal_id:
            break
        if d != dist_to[u]:
            continue
        for v, w in adj[u]:
            nd = d + w
            if nd < dist_to[v]:
                dist_to[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if dist_to[goal_id] == float('inf'):
        return []
    path = []
    cur = goal_id
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path












#ROS2 Node

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('Path_Planning_Turtlesim_Init')

        #Setup init
        self.world_min = 0.0
        self.world_max = 11.088889
        self.grid_w = 22
        self.grid_h = 22
        self.cell_size = (self.world_max - self.world_min) / self.grid_w

        #Map init
        self.start_cell = (2, 2)
        self.goal_cell  = (19, 19)
        self.num_blocked_cells = 32
        self.max_sampling_tries = 100

        #Probabilistic Roadmap init
        self.num_samples = 450
        self.k_neighbors = 10
        self.shortcut_attempts = 150

        #Pure pursuit controller
        self.lookahead = 0.55
        self.lookahead_min = 0.20
        self.lookahead_shrink = 0.75
        self.v_max = 1.1
        self.omega_max = 2.8
        self.goal_tol = 0.08
        self.slowdown_radius = 1.0
        self.timer_period = 0.05  # 20 Hz

        #ROS i/o
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        #Pen sketching services
        self.spawn_cli = self.create_client(Spawn, 'spawn'); self._wait(self.spawn_cli, 'spawn')
        self.t1_tel = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute'); self._wait(self.t1_tel, '/turtle1/teleport_absolute')
        self.t1_pen = self.create_client(SetPen, '/turtle1/set_pen'); self._wait(self.t1_pen, '/turtle1/set_pen')

        #Obstacles
        self.raw_cells = self._make_random_obstacles()
        self.raw_cells.discard(self.start_cell)  # defensive
        self.raw_cells.discard(self.goal_cell)

        self._spawn_marker('walls', 0.6, 0.6)
        self._setup_marker_clients('walls')
        self._draw_obstacles(self.raw_cells)  # neat boxes only

        # Rectangles used for collision checks
        self.rects = [self._cell_rect(c) for c in self.raw_cells]

        #Planning
        start_xy = self._cell_center(self.start_cell)
        goal_xy  = self._cell_center(self.goal_cell)
        path_xy  = self._plan_with_prm(start_xy, goal_xy, self.rects)

        # Start turtle at start, drawing path as it moves
        self._set_pen(self.t1_pen, 0, 0, 255, 3, off=1)
        self._teleport(self.t1_tel, start_xy[0], start_xy[1], 0.0)

        # SAFETY: if start inside an obstacle, move to nearest free cell center
        if any(point_in_rect(start_xy, r) for r in self.rects):
            safe_xy = self._nearest_free_center(start_xy, self.raw_cells)
            self.get_logger().warn("Start inside obstacle; moving to nearest free cell.")
            self._teleport(self.t1_tel, safe_xy[0], safe_xy[1], 0.0)

        self._set_pen(self.t1_pen, 0, 0, 255, 3, off=0)

        # Flags (optional) — no connecting lines
        self._spawn_marker('flags', 0.7, 0.7)
        self._setup_marker_clients('flags')
        self._draw_flag(start_xy, color='green')
        self._draw_flag(goal_xy,  color='red')

        # Densify for smooth following
        self.path_pts = self._densify(path_xy, step=0.08)
        self.path_idx = 0
        self.pose: Optional[Pose] = None

        # Control loop
        self.create_timer(self.timer_period, self._control_loop)
        self.get_logger().info("PRM + Dijkstra + Pure Pursuit (exact collisions, obstacles only).")






    #ROS Helpers 

    # Blocks until the given ROS service becomes available
    # param client: ROS service client to wait on
    # param name: human-readable service name for logging
    # returns: None
    def _wait(self, client, name: str):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service: {name} ...")

    # Callback that stores the latest turtle pose from the /turtle1/pose topic
    # param msg: Pose message containing x, y, and theta of the turtle
    # returns: None
    def _pose_cb(self, msg: Pose):
        self.pose = msg

        

    #Grid formation

    # Computes the axis-aligned bounding box of a grid cell in world coordinates
    # param cell: grid cell as (col, row) indices
    # returns: bounding box as (x0, y0, x1, y1)
    def _cell_bounds(self, cell: Tuple[int, int]) -> Tuple[float, float, float, float]:
        i, j = cell
        x0 = self.world_min + i * self.cell_size
        y0 = self.world_min + j * self.cell_size
        x1 = x0 + self.cell_size
        y1 = y0 + self.cell_size
        return (x0, y0, x1, y1)

    # Returns the world-space center coordinates of a grid cell
    # param cell: grid cell as (col, row) indices
    # returns: center point as (x, y)
    def _cell_center(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        x0, y0, x1, y1 = self._cell_bounds(cell)
        return (0.5 * (x0 + x1), 0.5 * (y0 + y1))

    # Returns the rectangle of a grid cell (alias for _cell_bounds)
    # param cell: grid cell as (col, row) indices
    # returns: bounding box as (x0, y0, x1, y1)
    def _cell_rect(self, cell: Tuple[int, int]) -> Tuple[float, float, float, float]:
        return self._cell_bounds(cell)

    #Obstacle formation

    # Generates a random set of blocked grid cells that still allows a path from start to goal
    # param: none (uses self.num_blocked_cells, self.start_cell, self.goal_cell)
    # returns: set of (col, row) tuples representing blocked cells
    def _make_random_obstacles(self) -> Set[Tuple[int, int]]:
        must_be_free = {self.start_cell, self.goal_cell}
        cells = [(x, y) for x in range(self.grid_w) for y in range(self.grid_h)
                 if (x, y) not in must_be_free]
        for _ in range(self.max_sampling_tries):
            random.shuffle(cells)
            chosen = set(cells[:self.num_blocked_cells])
            if self._coarse_has_path(chosen):
                return chosen
        self.get_logger().warn("Could not find a solvable random map quickly; using no obstacles.")
        return set()

    # Runs a grid-level Dijkstra to check if start and goal are connected given blocked cells
    # param blocked: set of (col, row) cells treated as impassable
    # returns: True if a path exists from start_cell to goal_cell
    def _coarse_has_path(self, blocked: Set[Tuple[int, int]]) -> bool:
        start, goal = self.start_cell, self.goal_cell
        if start in blocked or goal in blocked:
            return False
        W, H = self.grid_w, self.grid_h
        INF = 10**9
        distg = [[INF] * H for _ in range(W)]
        pq = []
        distg[start[0]][start[1]] = 0
        heapq.heappush(pq, (0, start))
        while pq:
            d, (x, y) = heapq.heappop(pq)
            if (x, y) == goal:
                return True
            if d != distg[x][y]:
                continue
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and (nx, ny) not in blocked:
                    nd = d + 1
                    if nd < distg[nx][ny]:
                        distg[nx][ny] = nd
                        heapq.heappush(pq, (nd, (nx, ny)))
        return False

    # Finds the closest unblocked grid cell center to a given world position
    # param xy: query position as (x, y) in world coordinates
    # param blocked: set of (col, row) cells to exclude
    # returns: world-space (x, y) of the nearest free cell center
    def _nearest_free_center(self, xy: Tuple[float, float], blocked: Set[Tuple[int,int]]) -> Tuple[float,float]:
        nearest = None
        best_d = float('inf')
        for i in range(self.grid_w):
            for j in range(self.grid_h):
                if (i, j) in blocked: continue
                c = self._cell_center((i, j))
                d = distance(xy, c)
                if d < best_d:
                    best_d = d
                    nearest = c
        return nearest if nearest is not None else xy

    # Draws outlines of all obstacle cells in turtlesim using the walls turtle
    # param blocked: set of (col, row) obstacle cells to draw
    # returns: None
    def _draw_obstacles(self, blocked: Set[Tuple[int, int]]):
        """Draw only obstacle rectangles; pen toggled to avoid connecting lines."""
        pen = getattr(self, 'walls_pen')
        tel = getattr(self, 'walls_tel')
        for cell in blocked:
            x0, y0, x1, y1 = self._cell_bounds(cell)
            self._set_pen(pen, 0, 0, 0, 3, off=1); self._teleport(tel, x0, y0, 0.0)
            self._set_pen(pen, 0, 0, 0, 3, off=0)
            self._teleport(tel, x1, y0, 0.0); self._teleport(tel, x1, y1, 0.0)
            self._teleport(tel, x0, y1, 0.0); self._teleport(tel, x0, y0, 0.0)
            self._set_pen(pen, 0, 0, 0, 3, off=1)
        self._teleport(tel, 0.6, 0.6, 0.0)



    #PRM Planning
    # Plans a collision-free path using a Probabilistic Roadmap with Dijkstra and shortcutting
    # param start_xy: start position as (x, y) in world coordinates
    # param goal_xy: goal position as (x, y) in world coordinates
    # param rects: list of obstacle rectangles as (x0, y0, x1, y1)
    # returns: list of (x, y) waypoints from start to goal
    def _plan_with_prm(self, start_xy, goal_xy, rects) -> List[Tuple[float, float]]:
        nodes = [start_xy, goal_xy]

        # 1) Random free samples
        pad = 0.05
        while len(nodes) < 2 + self.num_samples:
            x = random.uniform(self.world_min + pad, self.world_max - pad)
            y = random.uniform(self.world_min + pad, self.world_max - pad)
            if all(not point_in_rect((x, y), r) for r in rects):
                nodes.append((x, y))

        # 2) k-NN graph with exact collision checks
        adj: List[List[Tuple[int, float]]] = [[] for _ in range(len(nodes))]
        for i in range(len(nodes)):
            dlist = sorted(((distance(nodes[i], nodes[j]), j) for j in range(len(nodes)) if j != i),
                           key=lambda t: t[0])
            added = 0
            for _, j in dlist:
                if added >= self.k_neighbors:
                    break
                if segment_is_free(nodes[i], nodes[j], rects):
                    w = distance(nodes[i], nodes[j])
                    adj[i].append((j, w))
                    adj[j].append((i, w))
                    added += 1

        # 3) Shortest path
        idx_path = dijkstra_path(0, 1, adj)
        if not idx_path:
            self.get_logger().warn("PRM failed; using straight line fallback.")
            return [start_xy, goal_xy]
        path = [nodes[i] for i in idx_path]

        # 4) Shortcuts (still exact collision-checked)
        for _ in range(self.shortcut_attempts):
            if len(path) <= 2: break
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if j == i + 1: continue
            if segment_is_free(path[i], path[j], rects):
                path = path[:i+1] + path[j:]
        return path

    # Inserts intermediate points along a polyline so no segment exceeds a given step length
    # param pts: list of (x, y) waypoints defining the path
    # param step: maximum distance between consecutive output points
    # returns: densified list of (x, y) points
    def _densify(self, pts: List[Tuple[float, float]], step: float = 0.08) -> List[Tuple[float, float]]:
        if not pts:
            return []
        out = [pts[0]]
        for i in range(1, len(pts)):
            p, q = pts[i-1], pts[i]
            L = distance(p, q)
            if L < step:
                out.append(q); continue
            n = max(1, int(L / step))
            for k in range(1, n + 1):
                t = k / n
                out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
        return out

    #Flags for start and finish

    # Spawns a named marker turtle at the given position in turtlesim
    # param name: name to assign to the spawned turtle
    # param x: x position in world coordinates
    # param y: y position in world coordinates
    # returns: None
    def _spawn_marker(self, name: str, x: float, y: float):
        req = Spawn.Request()
        req.x = float(x); req.y = float(y); req.theta = 0.0; req.name = name
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

    # Creates and stores set_pen and teleport_absolute service clients for a named turtle
    # param name: name of the turtle whose clients should be created
    # returns: None
    def _setup_marker_clients(self, name: str):
        pen = self.create_client(SetPen, f'/{name}/set_pen'); self._wait(pen, f'/{name}/set_pen')
        tel = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute'); self._wait(tel, f'/{name}/teleport_absolute')
        setattr(self, f'{name}_pen', pen)
        setattr(self, f'{name}_tel', tel)

    # Draws a small flag (pole + triangle) at the given world position using the flags turtle
    # param center: position as (x, y) where the flag base will be placed
    # param color: flag color as a string ('red', 'green', or other for orange)
    # returns: None
    def _draw_flag(self, center: Tuple[float, float], color: str = 'red'):
        pen = getattr(self, 'flags_pen')
        tel = getattr(self, 'flags_tel')
        if color == 'red':   r,g,b = (255,0,0)
        elif color == 'green': r,g,b = (0,180,0)
        else:               r,g,b = (255,140,0)

        cx, cy = center
        h = 0.9 * self.cell_size
        base_x = cx - 0.2 * self.cell_size
        base_y = cy - 0.4 * self.cell_size
        top_y  = base_y + h

        # Move to base with pen OFF
        self._set_pen(pen, 0, 0, 0, 3, off=1); self._teleport(tel, base_x, base_y, 0.0)
        # Pole with pen ON
        self._set_pen(pen, 0, 0, 0, 3, off=0); self._teleport(tel, base_x, top_y, 0.0)
        # Triangle with pen ON
        self._set_pen(pen, r, g, b, 2, off=0)
        self._teleport(tel, base_x + 0.45 * self.cell_size, top_y - 0.12 * self.cell_size, 0.0)
        self._teleport(tel, base_x, top_y - 0.24 * self.cell_size, 0.0)
        self._teleport(tel, base_x, top_y, 0.0)
        # Pen OFF before leaving
        self._set_pen(pen, r, g, b, 2, off=1); self._teleport(tel, 0.7, 0.7, 0.0)

    #Services

    # Teleports a turtle to an absolute position and angle using its teleport service
    # param tel: TeleportAbsolute service client
    # param x: target x position in world coordinates
    # param y: target y position in world coordinates
    # param theta: target heading in radians
    # returns: None
    def _teleport(self, tel, x: float, y: float, theta: float):
        req = TeleportAbsolute.Request()
        req.x = float(x); req.y = float(y); req.theta = float(theta)
        fut = tel.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

    # Configures a turtle's pen color, width, and on/off state via the set_pen service
    # param pen: SetPen service client
    # param r: red channel (0-255)
    # param g: green channel (0-255)
    # param b: blue channel (0-255)
    # param width: pen stroke width in pixels
    # param off: 1 to lift the pen (no drawing), 0 to lower it (drawing enabled)
    # returns: None
    def _set_pen(self, pen, r: int, g: int, b: int, width: int, off: int):
        req = SetPen.Request()
        req.r = int(r); req.g = int(g); req.b = int(b)
        req.width = int(width); req.off = int(off)
        fut = pen.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

    #Control Loop

    # Timer callback that runs the pure pursuit controller to follow the planned path
    # param: none (reads self.pose and self.path_pts)
    # returns: None
    def _control_loop(self):
        if self.pose is None or not self.path_pts:
            return

        gx, gy = self.path_pts[-1]
        if distance((self.pose.x, self.pose.y), (gx, gy)) < self.goal_tol:
            self.cmd_pub.publish(Twist()); return

        self.path_idx = self._advance_index(self.path_idx, (self.pose.x, self.pose.y))

        target, used_Ld = self._safe_lookahead(self.path_idx, (self.pose.x, self.pose.y), self.lookahead)

        cmd = Twist()
        if target is None:
            pnext = self.path_pts[min(self.path_idx + 1, len(self.path_pts) - 1)]
            ang = math.atan2(pnext[1] - self.pose.y, pnext[0] - self.pose.x)
            ang_err = self._wrap_angle(ang - self.pose.theta)
            cmd.angular.z = bound(2.0 * ang_err, -self.omega_max, self.omega_max)
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd); return

        dx = target[0] - self.pose.x; dy = target[1] - self.pose.y
        ct = math.cos(-self.pose.theta); st = math.sin(-self.pose.theta)
        x_ld =  ct*dx - st*dy; y_ld =  st*dx + ct*dy

        if x_ld <= 1e-3:
            ang = math.atan2(dy, dx)
            ang_err = self._wrap_angle(ang - self.pose.theta)
            cmd.angular.z = bound(2.2 * ang_err, -self.omega_max, self.omega_max)
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd); return

        kappa = 2.0 * y_ld / max(1e-6, (used_Ld * used_Ld))
        d_goal = distance((self.pose.x, self.pose.y), (gx, gy))
        goal_scale = bound(d_goal / self.slowdown_radius, 0.25, 1.0)
        turn_scale = 1.0 / (1.0 + 1.2 * abs(kappa))
        v = self.v_max * goal_scale * turn_scale
        omega = bound(kappa * v, -self.omega_max, self.omega_max)

        cmd.linear.x = v; cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    # Finds the furthest collision-free lookahead point by shrinking the lookahead distance if blocked
    # param idx: current path index to start searching from
    # param pos: current turtle position as (x, y)
    # param Ld_init: initial lookahead distance
    # returns: tuple of (lookahead point or None if none found, actual lookahead distance used)
    def _safe_lookahead(self, idx: int, pos: Tuple[float, float], Ld_init: float
                        ) -> Tuple[Optional[Tuple[float, float]], float]:
        Ld = Ld_init
        while Ld >= self.lookahead_min - 1e-6:
            t = self._point_at_arc_len(idx, Ld)
            if t is None: t = self.path_pts[-1]
            if segment_is_free(pos, t, self.rects):
                return t, Ld
            Ld *= self.lookahead_shrink
        return None, Ld_init

    # Advances the path tracking index to the closest upcoming point along the path
    # param idx: current path index
    # param pos: current turtle position as (x, y)
    # returns: updated path index at the nearest point within the next 50 steps
    def _advance_index(self, idx: int, pos: Tuple[float, float]) -> int:
        best = idx; best_d = distance(self.path_pts[best], pos)
        end = min(len(self.path_pts) - 1, idx + 50)
        for i in range(idx + 1, end + 1):
            d = distance(self.path_pts[i], pos)
            if d <= best_d: best_d, best = d, i
            else: break
        return best

    # Interpolates a point along the path at a given arc length from the current index
    # param idx: starting path index for arc length measurement
    # param Ld: arc length distance to travel along the path
    # returns: interpolated (x, y) point at distance Ld, or None if path ends before reaching it
    def _point_at_arc_len(self, idx: int, Ld: float) -> Optional[Tuple[float, float]]:
        acc = 0.0
        for i in range(idx, len(self.path_pts) - 1):
            p, q = self.path_pts[i], self.path_pts[i + 1]
            seg = distance(p, q)
            if acc + seg >= Ld:
                t = (Ld - acc) / seg
                return (p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))
            acc += seg
        return None

    # Wraps an angle to the range [-pi, pi]
    # param a: angle in radians
    # returns: equivalent angle wrapped to [-pi, pi]
    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a


# Main
# Initializes ROS2, creates the PathPlannerNode, and spins until interrupted
# param args: command-line arguments passed to rclpy.init (default None)
# returns: None
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down planner...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
3
if __name__ == '__main__':
    main()
