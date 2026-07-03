import numpy as np


class DrydenGustModel:
    def __init__(self, sigma=2.0, tau=5.0, seed=42):
        self.sigma = sigma
        self.tau = tau
        self.rng = np.random.default_rng(seed)

        self.w_x = 0.0
        self.w_y = 0.0
        self.w_z = 0.0

    def sample(self, dt):
        phi = np.exp(-dt / self.tau)
        noise_scale = self.sigma * np.sqrt(1.0 - phi * phi)

        self.w_x = phi * self.w_x + noise_scale * self.rng.normal()
        self.w_y = phi * self.w_y + noise_scale * self.rng.normal() * 0.5
        self.w_z = phi * self.w_z + noise_scale * self.rng.normal()

        return np.array([self.w_x, self.w_y, self.w_z], dtype=np.float64)
