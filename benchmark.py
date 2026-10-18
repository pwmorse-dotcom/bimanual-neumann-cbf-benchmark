"""
==========================================================================================================
  IEEE / DEEPMIND ROBOTICS BENCHMARK:
  Division-Free 4th-Order Neumann Admittance & Schmitt-Trigger Hysteretic CBF for Bimanual Manipulation
==========================================================================================================
"""

import numpy as np


class BimanualNeumannCBFController:
    def __init__(self, kappa_scale: float = 11.0, cbf_rearm_deg: float = 135.0, cbf_cutoff_deg: float = 137.5):
        self.kappa = kappa_scale
        self.rearm_deg = cbf_rearm_deg
        self.cutoff_deg = cbf_cutoff_deg
        self.cbf_gate = 1.0  # 1.0 = Active Admittance Tracking, 0.0 = High-Impedance Position Hold

    def neumann_4th_order_compliance(self, H_norm: np.ndarray) -> np.ndarray:
        """Division-free 4th-order Neumann approximation of (I + H/κ)^-1."""
        I = np.eye(H_norm.shape[0])
        T = H_norm / self.kappa
        return I - T + (T @ T) - (T @ T @ T) + (T @ T @ T @ T)

    @staticmethod
    def deviatoric_load_split(R_inertial: np.ndarray) -> np.ndarray:
        """Trace-free deviatoric coupling tensor: R - 0.5 * Tr(R) * I."""
        return R_inertial - 0.5 * np.trace(R_inertial) * np.eye(R_inertial.shape[0])

    def evaluate_schmitt_cbf(self, joint_angle_deg: float) -> float:
        """Stateful Hysteretic Control Barrier Function (135.0° <-> 137.5°)."""
        if joint_angle_deg >= self.cutoff_deg:
            self.cbf_gate = 0.0
        elif self.cbf_gate == 0.0 and joint_angle_deg > self.rearm_deg:
            self.cbf_gate = 0.0  # Hold state inside 2.5° hysteresis buffer to prevent boundary chatter
        else:
            self.cbf_gate = 1.0
        return self.cbf_gate


def run_comparative_benchmark():
    ctrl = BimanualNeumannCBFController()
    H_gain = np.array([[2.2, 0.3], [0.3, 2.0]])
    v_task = np.array([1.0, 0.5])

    test_angles = [41.5, 110.0, 136.0, 137.5, 136.2, 134.5]
    print(f"{'Angle (deg)':<14} | {'Baseline ||q_dot||':<20} | {'Proposed ||q_dot||':<20} | {'CBF Gate State'}")
    print("-" * 82)

    for theta in test_angles:
        # 1. Baseline Jacobian Inverse (approaches singularity near 138°)
        det_j = max(0.02, 1.0 - (theta / 138.5))
        baseline_q_dot_norm = np.linalg.norm((1.0 / det_j) * v_task)

        # 2. Proposed 4th-Order Neumann + Hysteretic CBF
        gate = ctrl.evaluate_schmitt_cbf(theta)
        C_neumann = ctrl.neumann_4th_order_compliance(H_gain)
        proposed_q_dot_norm = np.linalg.norm(gate * (C_neumann @ v_task))

        state_str = "ACTIVE (1.0)" if gate == 1.0 else "LATCHED HOLD (0.0)"
        print(f"{theta:<14.1f} | {baseline_q_dot_norm:<20.4f} | {proposed_q_dot_norm:<20.4f} | {state_str}")


if __name__ == "__main__":
    run_comparative_benchmark()
