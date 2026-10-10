import copy
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEGS = (("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1))


def load_design(path=None):
    design = json.loads(Path(path or ROOT / "config/design.json").read_text(encoding="utf-8"))
    validate(design)
    return design


def validate(d):
    for key in ("body_length_m", "body_width_m", "upper_length_m", "lower_length_m", "standing_leg_height_m"):
        if not math.isfinite(d[key]) or d[key] <= 0:
            raise ValueError(f"Invalid dimension: {key}")
    if d["standing_leg_height_m"] >= d["upper_length_m"] + d["lower_length_m"]:
        raise ValueError("Standing height exceeds leg reach")
    if d["actuator_layout"] not in ("all00", "hybrid", "all02", "power_knees"):
        raise ValueError("Unknown actuator layout")
    if not 0.5 <= d["control"]["duty_factor"] < 0.9:
        raise ValueError("Unsupported duty factor")


def variant(d, layout, length, height=None):
    result = copy.deepcopy(d)
    result["actuator_layout"] = layout
    result["upper_length_m"] = result["lower_length_m"] = length
    result["standing_leg_height_m"] = height if height is not None else 1.60 * length
    validate(result)
    return result


def motor_names(d):
    triple = {"all00": ["RS00"] * 3, "hybrid": ["RS00", "RS02", "RS02"], "all02": ["RS02"] * 3, "power_knees": ["RS00", "RS02", "RS06"]}
    return triple[d["actuator_layout"]] * 4


def mass_and_cost(d):
    motors = motor_names(d)
    actuator_mass = sum(d["actuators"][name]["mass_kg"] for name in motors)
    frame_mass = d["base_without_actuators_kg"] + 4 * sum(d[key] for key in ("hip_bracket_kg", "upper_structure_kg", "lower_structure_kg"))
    # A quantity discount applies per SKU, not to a mixed total of twelve motors.
    cost = sum(motors.count(name) * d["actuators"][name]["bulk_10_price_usd" if motors.count(name) >= 10 else "unit_price_usd"] for name in set(motors))
    return {"estimated_mass_kg": actuator_mass + frame_mass, "actuator_mass_kg": actuator_mass, "motor_cost_usd": cost, "mass_measured": False}
