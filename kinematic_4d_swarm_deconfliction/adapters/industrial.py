"""
Gigafactory AGV & AMR Fleet Adapter:
Simulates heavy bi-directional AGV traffic converging at a central 2-way
warehouse intersection prone to symmetric gridlock and deadlock freezes.
Zero external dependencies (pure Python standard library).
"""

from __future__ import annotations
import math
import random
from typing import List
from ..core.models import AgentPose4D


class GigafactoryAGVAdapter:
    """Generates complex bi-directional warehouse AGV floor traffic."""

    @staticmethod
    def generate_aisle_traffic(
        agv_count: int = 20,
        seed: int = 42
    ) -> List[AgentPose4D]:
        """
        Creates opposing streams of AGVs traveling along a 4-meter wide aisle.
        Eastbound AGVs: traveling from X = -50 to X = +50
        Westbound AGVs: traveling from X = +50 to X = -50
        """
        rng = random.Random(seed)
        agvs: List[AgentPose4D] = []

        half = agv_count // 2

        # Eastbound AGVs (spaced along X corridor)
        for i in range(half):
            x = -45.0 + (i * 3.5)
            y = rng.uniform(-1.0, 1.0)
            agvs.append(AgentPose4D(
                agent_id=f"AGV-EAST-{i+1:02d}",
                x=x,
                y=y,
                z=0.0,
                heading_rad=0.0,  # Facing East (+X)
                velocity_mps=1.5,
                min_turn_radius_m=1.0,
                safe_radius_m=1.2,
                target_destination=(50.0, y, 0.0)
            ))

        # Westbound AGVs (spaced along X corridor)
        for j in range(half):
            x = 45.0 - (j * 3.5)
            y = rng.uniform(-1.0, 1.0)
            agvs.append(AgentPose4D(
                agent_id=f"AGV-WEST-{j+1:02d}",
                x=x,
                y=y,
                z=0.0,
                heading_rad=math.pi,  # Facing West (-X)
                velocity_mps=1.5,
                min_turn_radius_m=1.0,
                safe_radius_m=1.2,
                target_destination=(-50.0, y, 0.0)
            ))

        return agvs
