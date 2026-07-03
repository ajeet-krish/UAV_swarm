import numpy as np
import json


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        return super().default(obj)


class FrameExporter:
    def __init__(self):
        self.frames = []
        self.meta = {}

    def add_meta(self, key, value):
        self.meta[key] = value

    def record_frame(self, t, agents, metrics, wind, leader_pos, vector_field=None):
        frame = {
            "t": float(t),
            "drones": [
                {
                    "id": a.id,
                    "pos": a.pos.tolist(),
                    "vel": a.vel.tolist(),
                    "u": a.recorded_u.tolist(),
                    "is_leader": a.is_leader,
                    "d_ij": a.d_ij.tolist(),
                }
                for a in agents
            ],
            "metrics": {
                "formation_error": float(metrics.get("formation_error", 0.0)),
                "lambda_2": float(metrics.get("lambda_2", 0.0)),
                "max_control_effort": float(metrics.get("max_control_effort", 0.0)),
            },
            "wind": wind.tolist() if isinstance(wind, np.ndarray) else wind,
            "leader_pos": leader_pos.tolist() if isinstance(leader_pos, np.ndarray) else leader_pos,
        }
        if vector_field is not None:
            frame["vector_field"] = [
                {
                    "pos": v["pos"].tolist(),
                    "grad": v["grad"].tolist(),
                    "strength": float(v["strength"]),
                }
                for v in vector_field
            ]
        self.frames.append(frame)

    def to_dict(self):
        return {
            "meta": self.meta,
            "frames": self.frames,
        }

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, cls=NumpyEncoder, separators=(",", ":"))


def sample_vector_field(grid_points, agents, obstacles, env, params):
    vectors = []
    obstacle_eta = params.get("eta", 10.0)
    obstacle_rho0 = params.get("rho0", 2.0)
    for point in grid_points:
        p = np.array(point, dtype=np.float64)
        grad_obs = env.compute_obstacle_forces(p, obstacle_eta, obstacle_rho0)

        grad_consensus = np.zeros(3, dtype=np.float64)
        for agent in agents:
            dist = np.linalg.norm(p - agent.pos)
            if dist > 0.1:
                rep = params.get("kc", 2.5) * (p - agent.pos) / (dist ** 3 + 1e-12)
                grad_consensus += rep

        total_grad = grad_obs + grad_consensus
        strength = np.linalg.norm(total_grad)
        vectors.append({
            "pos": p,
            "grad": total_grad,
            "strength": strength,
        })
    return vectors
