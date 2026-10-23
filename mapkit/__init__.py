"""mapkit: pure-Python parser for plans and ``map/`` items (no ROS imports).

Sits beside ``controller/`` and ``teensy_link/`` at the repo root.
"""
from __future__ import annotations

from mapkit.frontmatter import FrontmatterError, split_frontmatter
from mapkit.items import (KINDS, REQUIRED_FIELDS, STATUS_SETS, Item, ItemError,
                          item_uuid, load_item, load_items)
from mapkit.plans import (BOARDS, Plan, PlanError, Step, load_plan, load_plans,
                          normalise_plan_meta, plan_paths)
from mapkit.steps import (STEP_HEADING_RE, STEP_KINDS, STEP_STATUSES,
                          StepHeading, check_mirror, check_step_vocab,
                          extract_step_headings, normalise_step_id)

__version__ = "0.1.0"

__all__ = [
    "__version__", "FrontmatterError", "split_frontmatter",
    "KINDS", "REQUIRED_FIELDS", "STATUS_SETS", "Item", "ItemError",
    "item_uuid", "load_item", "load_items",
    "BOARDS", "Plan", "PlanError", "Step", "load_plan", "load_plans",
    "normalise_plan_meta", "plan_paths",
    "STEP_HEADING_RE", "STEP_KINDS", "STEP_STATUSES", "StepHeading",
    "check_mirror", "check_step_vocab", "extract_step_headings",
    "normalise_step_id",
]
