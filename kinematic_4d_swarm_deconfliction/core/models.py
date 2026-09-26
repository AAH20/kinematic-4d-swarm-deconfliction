"""
Data Models for Kinematic 4D Swarm Deconfliction:
Defines 4D non-holonomic agent poses, Dubins curvature bounds, and CBF safety parameters.
Zero external dependencies (pure Python standard library).
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class DeconflictionMode(str, Enum):
    TACTICAL_SWARM_FUNNEL = "TACTICAL_SWARM_FUNNEL"
    GIGAFACTORY_AGV_AISLE = "GIGAFACTORY_AGV_AISLE"


@dataclass
class AgentPose4D:
    """Represents a non-holonomic agent with turning radius limits."""
    agent_id: str
    x: float
    y: float
    z: float
    heading_rad: float
    velocity_mps: float
    min_turn_radius_m: float = 5.0
    safe_radius_m: float = 2.0
    target_destination: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class ControlCommand:
    """Computed control input for an agent."""
    agent_id: str
    commanded_velocity_mps: float
    commanded_steering_rad_s: float
    cbf_safety_margin: float


@dataclass
class DeconflictionResult:
    """Outcome of 4D swarm deconfliction step."""
    mode: DeconflictionMode
    commands: Dict[str, ControlCommand]
    min_inter_agent_distance_m: float
    collision_free: bool
    deadlocks_resolved_count: int
    execution_latency_us: float
    metrics: Dict[str, float] = field(default_factory=dict)
