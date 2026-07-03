import numpy as np


class QuadcopterAgent:
    def __init__(self, agent_id, initial_pos, formation_offset, is_leader=False):
        self.id = agent_id
        self.is_leader = is_leader

        # State: [x, y, z, vx, vy, vz]
        self.state = np.array([
            initial_pos[0], initial_pos[1], initial_pos[2],
            0.0, 0.0, 0.0
        ], dtype=np.float64)

        # Current formation offset relative to leader
        self.d_ij = np.array(formation_offset, dtype=np.float64)

        # Target formation offset (for morphing)
        self.d_ij_target = np.array(formation_offset, dtype=np.float64)

        # Physical parameters
        self.m = 1.2       # kg
        self.b = 0.15      # Aerodynamic damping coefficient (Ns/m)
        self.T_max = 15.0  # Maximum thrust per axis (N)

        # Ambient wind (set by environment each step)
        self.wind_local = np.zeros(3, dtype=np.float64)

        # Recorded telemetry
        self.recorded_u = np.zeros(3, dtype=np.float64)

    @property
    def pos(self):
        return self.state[0:3]

    @property
    def vel(self):
        return self.state[3:6]

    def derivatives(self, u_cmd):
        dx = self.state[3:6].copy()
        dv = (u_cmd - self.b * self.state[3:6]) / self.m + self.wind_local
        dstate = np.concatenate([dx, dv])
        return dstate

    def step_rk4(self, dt, u_cmd):
        def f(s):
            old = self.state.copy()
            self.state = s
            d = self.derivatives(u_cmd)
            self.state = old
            return d

        s0 = self.state.copy()
        k1 = f(s0)
        k2 = f(s0 + 0.5 * dt * k1)
        k3 = f(s0 + 0.5 * dt * k2)
        k4 = f(s0 + dt * k3)
        self.state = s0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def saturate_u(self, u):
        return np.clip(u, -self.T_max, self.T_max)
