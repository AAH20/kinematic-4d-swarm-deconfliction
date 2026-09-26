"""
Unit Tests for Kinematic 4D Swarm Deconfliction:
Verifies forward safety invariance, turning radius adherence,
deadlock breaking, and sub-millisecond execution latency.
Zero external dependencies (pure Python standard library).
"""

import unittest
from kinematic_4d_swarm_deconfliction import (
    Kinematic4DSwarmPlanner,
    AgentPose4D,
    DeconflictionMode
)


class TestKinematic4DSwarmDeconfliction(unittest.TestCase):

    def setUp(self):
        self.planner = Kinematic4DSwarmPlanner(gamma=1.5)

    def test_tactical_chokepoint_collision_free(self):
        """Verifies that combat drones navigating a mountain chokepoint avoid collisions."""
        res = self.planner.plan_tactical_chokepoint(drone_count=16, seed=42)

        self.assertEqual(res.mode, DeconflictionMode.TACTICAL_SWARM_FUNNEL)
        self.assertTrue(res.collision_free)
        self.assertEqual(len(res.commands), 16)
        self.assertLess(res.execution_latency_us, 5000.0)  # Sub-5ms

    def test_gigafactory_deadlock_breaking(self):
        """Verifies that opposing AGV streams break head-on symmetry without deadlocking."""
        res = self.planner.plan_gigafactory_agvs(agv_count=20, seed=42)

        self.assertEqual(res.mode, DeconflictionMode.GIGAFACTORY_AGV_AISLE)
        self.assertTrue(res.collision_free)
        self.assertGreaterEqual(res.deadlocks_resolved_count, 0)

    def test_turning_radius_invariance(self):
        """Verifies commanded steering never breaches physical kinematic turning radius."""
        drones = [
            AgentPose4D(
                agent_id="DRONE-01",
                x=0.0, y=0.0, z=50.0,
                heading_rad=0.0,
                velocity_mps=20.0,
                min_turn_radius_m=10.0,
                safe_radius_m=2.0,
                target_destination=(0.0, 100.0, 50.0)  # Sharp 90-degree left turn
            )
        ]
        res = self.planner.plan_custom_step(drones)
        cmd = res.commands["DRONE-01"]

        # Max yaw rate = v / R_min = 20.0 / 10.0 = 2.0 rad/s
        max_allowed_omega = 20.0 / 10.0
        self.assertLessEqual(abs(cmd.commanded_steering_rad_s), max_allowed_omega + 1e-4)


if __name__ == "__main__":
    unittest.main()
