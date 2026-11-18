"""Translation of obstacle_guidance.cpp: eight-connected Dijkstra guidance."""
import heapq
import math
from ...include.python_ver.obstacle_guidance import Neighbor, Offset


class ObstacleGuidance:
    def __init__(self):
        self.width_ = self.height_ = 0
        self.resolution_ = 0.0
        self.blocked_, self.penalties_, self.distances_ = [], [], []
        self.clearance_radius_ = self.penalty_weight_ = -1.0
        self.clearance_offsets_, self.neighbors_, self.queue_ = [], [], []
        self.penalty_lookup_ = [0.0] * 256

    # clearance: distance to maintain form the obstacles for the robot's center
    # cost_weight: penalty to high cost cells
    def update(self, width, height, resolution, costs, clearance, cost_weight):
        if (width <= 0 or height <= 0 or costs is None or
                not math.isfinite(resolution) or resolution <= 0 or
                not math.isfinite(clearance) or clearance < 0 or
                not math.isfinite(cost_weight) or cost_weight < 0):
            raise ValueError("Invalid obstacle guidance grid")
        if len(costs) != width * height or any(c < 0 or c > 255 for c in costs):
            raise ValueError("Expected one unsigned byte per grid cell")
        self.width_, self.height_, self.resolution_ = width, height, resolution
        self.blocked_ = [False] * (width * height)
        self.penalties_ = [0.0] * (width * height)
        self.distances_ = [math.inf] * (width * height)
        radius = clearance / resolution + math.sqrt(0.5)
        if radius != self.clearance_radius_:
            cells = math.ceil(radius)
            self.clearance_offsets_ = [Offset(dx, dy)
                for dy in range(-cells, cells + 1) for dx in range(-cells, cells + 1)
                if math.hypot(dx, dy) <= radius]
            self.clearance_radius_ = radius
        if self.penalty_weight_ != cost_weight:
            self.penalty_lookup_ = [1.0 + cost_weight * min(c, 252) / 252.0 for c in range(256)]
            self.penalty_weight_ = cost_weight
        self.neighbors_ = []
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                if dx or dy:
                    length = resolution * math.hypot(dx, dy)
                    self.neighbors_.append(Neighbor(dx, dy, length, length * 0.5, math.atan2(dy, dx)))
        for y in range(height):
            for x in range(width):
                index = y * width + x
                cost = costs[index]
                self.blocked_[index] = (self.blocked_[index] or cost >= 253 or
                    (x + 0.5) * resolution < clearance or (y + 0.5) * resolution < clearance or
                    (width - x - 0.5) * resolution < clearance or
                    (height - y - 0.5) * resolution < clearance)
                self.penalties_[index] = self.penalty_lookup_[cost]
                if cost >= 254:
                    for offset in self.clearance_offsets_:
                        nx, ny = x + offset.x, y + offset.y
                        if 0 <= nx < width and 0 <= ny < height:
                            self.blocked_[ny * width + nx] = True

    def traversable(self, x, y):
        return (0 <= x < self.width_ and 0 <= y < self.height_ and
                not self.blocked_[y * self.width_ + x])

    def _neighbors(self, cx, cy):
        for step in self.neighbors_:
            nx, ny = cx + step.x, cy + step.y
            if not self.traversable(nx, ny):
                continue
            # Diagonal motion cannot cut across a blocked corner.
            if step.x and step.y and (not self.traversable(nx, cy) or not self.traversable(cx, ny)):
                continue
            yield step, ny * self.width_ + nx

    def setTarget(self, x, y):
        self.distances_ = [math.inf] * (self.width_ * self.height_)
        if not self.traversable(x, y):
            return False
        self.queue_.clear()
        target = y * self.width_ + x
        self.distances_[target] = 0.0
        heapq.heappush(self.queue_, (0.0, target))
        while self.queue_:
            distance, index = heapq.heappop(self.queue_)
            if distance > self.distances_[index]:
                continue
            for step, nxt in self._neighbors(index % self.width_, index // self.width_):
                candidate = distance + step.half_length * (self.penalties_[index] + self.penalties_[nxt])
                if candidate < self.distances_[nxt]:
                    self.distances_[nxt] = candidate
                    heapq.heappush(self.queue_, (candidate, nxt))
        return True

    def _valid_coordinates(self, x, y):
        return (math.isfinite(x) and math.isfinite(y) and x >= -0.5 and y >= -0.5 and
                x < self.width_ - 0.5 and y < self.height_ - 0.5)

    def distance(self, x, y):
        if not self._valid_coordinates(x, y):
            return math.inf
        cx, cy = math.floor(x + 0.5), math.floor(y + 0.5)
        if not self.traversable(cx, cy) or not math.isfinite(self.distances_[cy * self.width_ + cx]):
            return math.inf
        x0, y0 = math.floor(x), math.floor(y)
        total = weights = 0.0
        for dy in range(2):
            for dx in range(2):
                nx, ny = x0 + dx, y0 + dy
                if not self.traversable(nx, ny):
                    continue
                value = self.distances_[ny * self.width_ + nx]
                weight = (x - x0 if dx else 1 - (x - x0)) * (y - y0 if dy else 1 - (y - y0))
                if math.isfinite(value) and weight > 0:
                    total += weight * value
                    weights += weight
        return total / weights if weights > 0 else math.inf

    def route(self, x, y, direction):
        """Extract the descending route for telemetry without changing Dijkstra."""
        if not self.traversable(x, y) or not math.isfinite(self.distances_[y*self.width_+x]):
            return []
        result = []
        for _ in range(len(self.distances_)):
            result.append((x, y))
            index = y*self.width_+x
            if self.distances_[index] == 0:
                return result
            best = None
            for step, nxt in self._neighbors(x, y):
                if self.distances_[nxt] >= self.distances_[index]:
                    continue
                cost = self.distances_[nxt]+step.half_length*(self.penalties_[index]+self.penalties_[nxt])
                angle = abs(math.atan2(math.sin(step.heading-direction), math.cos(step.heading-direction)))
                if best is None or cost < best[0]-1e-9 or (abs(cost-best[0])<=1e-9 and angle<best[1]):
                    best = cost, angle, step
            if best is None:
                return []
            step = best[2]
            x, y, direction = x+step.x, y+step.y, step.heading
        return []

    def heading(self, x, y, current_heading):
        if not self._valid_coordinates(x, y):
            return current_heading
        cx, cy = math.floor(x + 0.5), math.floor(y + 0.5)
        if not self.traversable(cx, cy):
            return current_heading
        start_x, start_y = cx, cy
        length, direction = 0.0, current_heading
        # Follow a short route section to smooth individual grid-cell angles.
        while length < 0.35:
            index = cy * self.width_ + cx
            best_cost = best_angle = math.inf
            step_x = step_y = 0
            for step, nxt in self._neighbors(cx, cy):
                if self.distances_[nxt] >= self.distances_[index]:
                    continue
                cost = self.distances_[nxt] + step.half_length * (self.penalties_[index] + self.penalties_[nxt])
                angle = abs(math.atan2(math.sin(step.heading - direction), math.cos(step.heading - direction)))
                if cost < best_cost - 1e-9 or (abs(cost - best_cost) <= 1e-9 and angle < best_angle):
                    best_cost, best_angle = cost, angle
                    step_x, step_y = step.x, step.y
            if not step_x and not step_y:
                break
            cx, cy = cx + step_x, cy + step_y
            length += self.resolution_ * math.hypot(step_x, step_y)
            direction = math.atan2(step_y, step_x)
        return math.atan2(cy - start_y, cx - start_x) if length > 0 else current_heading
