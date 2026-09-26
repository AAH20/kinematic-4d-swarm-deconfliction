"""
Tactical Swarm Chokepoint Adapter:
Simulates high-speed combat drone formations funneling through
narrow radar-shadow canyons and mountain passes.
Zero external dependencies (pure Python standard library).
"""

from __future__ import annotations
import math
import random
from typing import List
from ..core.models import AgentPose4D


class SwarmChokepointAdapter:
    """Generates high-dynamic combat drone swarm chokepoint funneling scenarios."""

    @staticmethod
    def generate_chokepoint_swarm(
        drone_count: int = 16,
        seed: int = 42
    ) -> List[AgentPose4D]:
        """
        Creates a swarm converging on a narrow mountain gorge at (500, 0, 100).
        """
        rng = random.Random(seed)
        drones: List[AgentPose4D] = []

        target_gorge = (500.0, 0.0, 100.0)

        for i in range(drone_count):
            # Spread across a wide fan at start X = 0
            start_y = rng.uniform(-150.0, 150.0)
            start_z = rng.uniform(80.0, 120.0)
            drones.append(AgentPose4D(
                agent_id=f"COMBAT-UAV-{i+1:02d}",
                x=rng.uniform(-20.0, 20.0),
                y=start_y,
                z=start_z,
                heading_rad=rng.uniform(-0.2, 0.2),
                velocity_mps=30.0,  # 108 km/h
                min_turn_radius_m=12.0,
                safe_radius_m=3.0,
                target_destination=target_gorge
            ))

        return drones
