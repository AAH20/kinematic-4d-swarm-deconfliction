"""
Kinematic 4D Swarm Deconfliction:
Dual-Use 4D Dubins MAPF & Control Barrier Function (CBF) Swarm Deconfliction Engine.
Zero external dependencies (pure Python standard library).
"""

from .core.models import (
    AgentPose4D,
    ControlCommand,
    DeconflictionResult,
    DeconflictionMode
)
from .core.cbf_planner import CBFSwarmPlanner
from .adapters.defense import SwarmChokepointAdapter
from .adapters.industrial import GigafactoryAGVAdapter
from .planner import Kinematic4DSwarmPlanner

__all__ = [
    "AgentPose4D",
    "ControlCommand",
    "DeconflictionResult",
    "DeconflictionMode",
    "CBFSwarmPlanner",
    "SwarmChokepointAdapter",
    "GigafactoryAGVAdapter",
    "Kinematic4DSwarmPlanner"
]

__version__ = "1.0.0"
