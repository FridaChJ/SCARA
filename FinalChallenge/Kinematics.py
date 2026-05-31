import numpy as np
from typing import Optional


class Scara2D:
    def __init__(self, l1: float, l2: float):
        """
        Planar 2-DOF SCARA arm.

        Parameters
        ----------
        l1 : float  Link 1 length (any consistent unit, e.g. cm)
        l2 : float  Link 2 length
        """
        self.l1 = l1
        self.l2 = l2

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------
    def forward_kinematics(self, theta1_deg: float, theta2_deg: float) -> np.ndarray:
        """Joint angles (degrees) → end-effector (x, y)."""
        t1 = np.radians(theta1_deg)
        t2 = np.radians(theta2_deg)
        x = self.l1 * np.cos(t1) + self.l2 * np.cos(t1 + t2)
        y = self.l1 * np.sin(t1) + self.l2 * np.sin(t1 + t2)
        return np.array([x, y])

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------
    def inverse_kinematics(
        self,
        x: float,
        y: float,
        elbow_up: bool = False,
    ) -> tuple[float, float]:
        """
        Cartesian target (x, y) → (theta1_deg, theta2_deg).

        Angles are ABSOLUTE, referenced to the fully-extended zero position:
          • 0°  = arm fully stretched along the positive X axis
          • All results normalised to [0°, 360°)

        Parameters
        ----------
        x, y      : target coordinates (same unit as l1/l2)
        elbow_up  : False → elbow-down (default), True → elbow-up

        Raises
        ------
        ValueError  if the target is geometrically unreachable.
        """
        d_sq = x ** 2 + y ** 2

        # Law of cosines for theta2
        cos_t2 = (d_sq - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)

        if not (-1.0 <= cos_t2 <= 1.0):
            raise ValueError(
                f"Point ({x:.3f}, {y:.3f}) is outside the robot's workspace. "
                f"|cos_t2| = {abs(cos_t2):.4f} > 1"
            )

        # sin_t2 sign selects elbow-down (+) or elbow-up (-)
        sin_t2 = -np.sqrt(1 - cos_t2 ** 2) if elbow_up else np.sqrt(1 - cos_t2 ** 2)
        theta2 = np.arctan2(sin_t2, cos_t2)

        # theta1
        k1 = self.l1 + self.l2 * cos_t2
        k2 = self.l2 * sin_t2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        # Convert to degrees and normalise to [0°, 360°)
        t1_deg = np.degrees(theta1) % 360.0
        t2_deg = np.degrees(theta2) % 360.0

        return t1_deg, t2_deg


# ======================================================================
# Main function: batch IK over a list of detected objects
# ======================================================================
def compute_kinematics(
    coords: list[dict],
    l1: float,
    l2: float,
    elbow_up: bool = False,
) -> dict[str, Optional[dict[str, float]]]:
    """
    Compute absolute joint angles (J1, J3) for every detected object.

    Objects are labelled per-colour in appearance order:
        red1, red2, …  /  blue1, blue2, …  /  green1, green2, …

    Parameters
    ----------
    coords   : list of {'label': str, 'x': float, 'y': float}
    l1, l2   : robot link lengths (same unit as coords)
    elbow_up : elbow configuration (must be consistent for all points)

    Returns
    -------
    dict keyed by object name, values {'j1': float, 'j3': float} or None
    if the point is unreachable.
    """
    robot = Scara2D(l1=l1, l2=l2)

    # ── Assign per-colour indices ──────────────────────────────────────
    colour_counter: dict[str, int] = {}
    labelled: list[tuple[str, float, float]] = []

    for obj in coords:
        colour = obj["label"]
        colour_counter[colour] = colour_counter.get(colour, 0) + 1
        name = f"{colour}{colour_counter[colour]}"
        labelled.append((name, obj["x"], obj["y"]))

    # ── Compute IK for every point ─────────────────────────────────────
    result: dict[str, Optional[dict[str, float]]] = {}

    for name, x, y in labelled:
        try:
            t1, t3 = robot.inverse_kinematics(x, y, elbow_up=elbow_up)
            result[name] = {"j1": round(t1, 4), "j3": round(t3, 4)}
        except ValueError as err:
            print(f"[WARN] {name} at ({x}, {y}) skipped: {err}")
            result[name] = None

    return result


# ======================================================================
# Entry point (demo)
# ======================================================================
if __name__ == "__main__":
    # ── Robot parameters ──────────────────────────────────────────────
    L1 = 25.0   # cm  (link 1)
    L2 = 20.0   # cm  (link 2)

    # ── Input from vision system ──────────────────────────────────────
    coords_data = [
        {"label": "red",   "x": 33.24, "y": 16.35},
        {"label": "red",   "x": 13.74, "y": 16.10},
        {"label": "blue",  "x": 25.23, "y": 25.40},
        {"label": "blue",  "x": 25.34, "y": 20.49},
        {"label": "blue",  "x": 25.40, "y":  8.80},
        {"label": "green", "x": 25.32, "y": 30.77},
        {"label": "green", "x": 25.26, "y": 15.45},
        {"label": "green", "x": 25.54, "y":  3.32},
    ]

    # ── Run ───────────────────────────────────────────────────────────
    angles = compute_kinematics(coords_data, l1=L1, l2=L2, elbow_up=False)

    # ── Pretty-print ─────────────────────────────────────────────────
    print(f"\n{'Object':<10} {'J1 (°)':>10} {'J3 (°)':>10}")
    print("-" * 32)
    for name, vals in angles.items():
        if vals is None:
            print(f"{name:<10} {'UNREACHABLE':>10}")
        else:
            print(f"{name:<10} {vals['j1']:>10.4f} {vals['j3']:>10.4f}")

    # ── Verification (FK round-trip) ──────────────────────────────────
    robot = Scara2D(l1=L1, l2=L2)
    print("\n── FK verification ──")
    original = {obj["label"]: [] for obj in coords_data}
    for obj in coords_data:
        original[obj["label"]].append((obj["x"], obj["y"]))

    colour_counter: dict = {}
    for obj in coords_data:
        colour = obj["label"]
        colour_counter[colour] = colour_counter.get(colour, 0) + 1
        name = f"{colour}{colour_counter[colour]}"
        if angles[name]:
            fk = robot.forward_kinematics(angles[name]["j1"], angles[name]["j3"]) # pyright: ignore[reportOptionalSubscript]
            print(
                f"{name:<10} target=({obj['x']:.2f}, {obj['y']:.2f})  "
                f"FK=({fk[0]:.2f}, {fk[1]:.2f})  "
                f"err={np.linalg.norm([fk[0]-obj['x'], fk[1]-obj['y']]):.4f}"
            )