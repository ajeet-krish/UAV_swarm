import numpy as np


class BoxObstacle:
    def __init__(self, bounds, label=""):
        pts = np.array(bounds, dtype=np.float64)
        self.min_corner = pts[0]
        self.max_corner = pts[1]
        self.label = label
        self.bounds = np.array([self.min_corner, self.max_corner])

    def distance(self, point):
        p = np.asarray(point, dtype=np.float64)
        closest = np.clip(p, self.min_corner, self.max_corner)
        diff = p - closest
        dist = np.linalg.norm(diff)
        inside = np.all((self.min_corner <= p) & (p <= self.max_corner))
        if inside:
            dist_to_faces = min(p[0] - self.min_corner[0], self.max_corner[0] - p[0],
                                p[1] - self.min_corner[1], self.max_corner[1] - p[1],
                                p[2] - self.min_corner[2], self.max_corner[2] - p[2])
            dist = -dist_to_faces
            grad = diff / (np.linalg.norm(diff) + 1e-12)
            return dist, grad
        return dist, diff / (dist + 1e-12)

    def compute_forces(self, point, eta, rho0):
        dist, grad = self.distance(point)
        if dist <= 0:
            return eta * 100.0 * grad
        if 0 < dist < rho0:
            magnitude = eta * (1.0 / dist - 1.0 / rho0) / (dist ** 2)
            return magnitude * grad
        return np.zeros(3)


class CylinderObstacle:
    def __init__(self, center, radius, height, label=""):
        self.center = np.array(center, dtype=np.float64)
        self.radius = radius
        self.height = height
        self.label = label

    def distance(self, point):
        p = np.asarray(point, dtype=np.float64)
        dx = p[0] - self.center[0]
        dz = p[2] - self.center[2]
        horiz_dist = np.sqrt(dx * dx + dz * dz) - self.radius

        dy_low = p[1] - (self.center[1] - self.height / 2.0)
        dy_high = (self.center[1] + self.height / 2.0) - p[1]
        vert_dist = max(-dy_low, -dy_high, 0.0) if dy_low > 0 and dy_high > 0 else min(dy_low if dy_low > 0 else 0.0, dy_high if dy_high > 0 else 0.0)

        if horiz_dist <= 0 and dy_low > 0 and dy_high > 0:
            dist = -min(-horiz_dist, dy_low, dy_high)
        elif horiz_dist > 0 and dy_low > 0 and dy_high > 0:
            dist = horiz_dist
        else:
            if horiz_dist > 0:
                dist = np.sqrt(horiz_dist ** 2 + max(-dy_low, -dy_high, 0.0) ** 2)
            else:
                dist = max(-dy_low, -dy_high, 0.0)

        grad_h = np.array([dx, 0.0, dz]) / (np.sqrt(dx * dx + dz * dz) + 1e-12)
        return dist, grad_h

    def compute_forces(self, point, eta, rho0):
        dist, grad = self.distance(point)
        if dist <= 0:
            return eta * 100.0 * grad
        if 0 < dist < rho0:
            magnitude = eta * (1.0 / dist - 1.0 / rho0) / (dist ** 2)
            return magnitude * grad
        return np.zeros(3)


class Environment:
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def compute_obstacle_forces(self, point, eta, rho0):
        total = np.zeros(3, dtype=np.float64)
        for obs in self.obstacles:
            total += obs.compute_forces(point, eta, rho0)
        return total

    def get_obstacle_data(self):
        data = []
        for obs in self.obstacles:
            if isinstance(obs, BoxObstacle):
                data.append({
                    "type": "box",
                    "min_corner": obs.min_corner.tolist(),
                    "max_corner": obs.max_corner.tolist(),
                    "label": obs.label,
                })
            elif isinstance(obs, CylinderObstacle):
                data.append({
                    "type": "cylinder",
                    "center": obs.center.tolist(),
                    "radius": obs.radius,
                    "height": obs.height,
                    "label": obs.label,
                })
        return data
