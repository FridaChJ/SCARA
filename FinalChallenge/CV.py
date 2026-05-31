"""
cv_module.py  —  Production CV module for robot workspace perception
====================================================================
Public API:
    summary_data, coordinates_data            = process_image(image_path)
    summary_data, coordinates_data, debug_img = process_image(image_path, debug=True)
"""

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

#   ID 3 ------- ID 2
#   |   workspace  |
#   ID 0 ------- ID 1
ROBOT_COORDS = {0: [0, 0], 1: [50, 0], 2: [50, 33], 3: [0, 33]}

# Workspace bounds (mm) — detections outside are discarded
WORKSPACE_MARGIN_MM = 2.0
WORKSPACE_X = (-WORKSPACE_MARGIN_MM, 50 + WORKSPACE_MARGIN_MM)
WORKSPACE_Y = (-WORKSPACE_MARGIN_MM, 33 + WORKSPACE_MARGIN_MM)

# ── HSV colour ranges ── tune S/V lower bounds for your lighting ───────────
COLOR_RANGES = {
    "red": {
        "lower":  np.array([0,   60, 40]),
        "upper":  np.array([10,  255, 255]),
        "lower2": np.array([160, 60, 40]),
        "upper2": np.array([180, 255, 255]),
    },
    "blue": {
        "lower": np.array([95,  60, 40]),
        "upper": np.array([135, 255, 255]),
    },
    "green": {
        "lower": np.array([35,  50, 40]),
        "upper": np.array([90,  255, 255]),
    },
}

EXPECTED_SHAPE = {"red": "circle", "blue": "square", "green": "triangle"}

# ── Shape classification thresholds ───────────────────────────────────────
CIRCLE_THR   = 0.82    # raise if circles still labelled square; lower if circles missed
POLY_EPSILON = 0.025   # lower = more vertices preserved (better for triangles)
                       # raise = fewer vertices (merges corners, better for squares)

# Minimum blob area (px²)
MIN_AREA = 500

COLOR_ORDER = ["red", "blue", "green"]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _build_mask(hsv, ranges):
    mask = cv2.inRange(hsv, ranges["lower"], ranges["upper"])
    if "lower2" in ranges:
        mask |= cv2.inRange(hsv, ranges["lower2"], ranges["upper2"])
    return mask


def _clean_mask(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def _circularity(contour):
    area  = cv2.contourArea(contour)
    perim = cv2.arcLength(contour, True)
    return (4.0 * np.pi * area / (perim ** 2)) if perim > 0 else 0.0


def _vertices(contour):
    perim  = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, POLY_EPSILON * perim, True)
    return len(approx)


def _classify_shape_for(contour, color):
    circ = _circularity(contour)
    v    = _vertices(contour)

    if color == "red":
        return "circle" if circ >= CIRCLE_THR else f"unknown(circ={circ:.2f})"

    if color == "blue":
        if v == 3:
            return "triangle"
        if v in (4, 5, 6):
            return "square"
        if v > 6 and circ < 0.75:
            return "square"
        return f"unknown({v}v)"

    if color == "green":
        if v == 3:
            return "triangle"
        if v == 4 and circ < 0.72:
            return "triangle"
        if v in (4, 5, 6):
            return "square"
        return f"unknown({v}v)"

    # fallback
    if circ >= CIRCLE_THR:
        return "circle"
    if v == 3:
        return "triangle"
    if v in (4, 5, 6):
        return "square"
    return "unknown"


def _centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])


def _to_robot(px, py, H):
    pt = np.array([[[px, py]]], dtype=np.float32)
    r  = cv2.perspectiveTransform(pt, H)
    return float(r[0][0][0]), float(r[0][0][1])


def _in_workspace(rx, ry):
    return WORKSPACE_X[0] <= rx <= WORKSPACE_X[1] and WORKSPACE_Y[0] <= ry <= WORKSPACE_Y[1]


def _compute_homography(img):
    gray     = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(ARUCO_DICT)
    corners, ids, _ = detector.detectMarkers(gray)

    found = 0 if ids is None else len(ids)
    if found < 4:
        raise Exception(f"ArUco detection failed: found {found} marker(s), need 4.")

    pixel_pts, robot_pts = [], []
    for i in range(len(ids)):
        mid = int(ids[i][0])
        if mid in ROBOT_COORDS:
            pixel_pts.append(corners[i][0].mean(axis=0))
            robot_pts.append(ROBOT_COORDS[mid])

    if len(pixel_pts) < 4:
        raise Exception(f"Valid markers found: {len(pixel_pts)} — need IDs 0-3 present.")

    H, _ = cv2.findHomography(
        np.array(pixel_pts, dtype=np.float32),
        np.array(robot_pts,  dtype=np.float32)
    )
    if H is None:
        raise Exception("cv2.findHomography returned None.")

    print(f"[INFO] Homography computed from {len(pixel_pts)} markers.")
    return H


def _detect_objects(hsv, H):
    results = {c: [] for c in COLOR_ORDER}

    for color in COLOR_ORDER:
        mask = _build_mask(hsv, COLOR_RANGES[color])
        mask = _clean_mask(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_AREA:
                continue
            c = _centroid(cnt)
            if c is None:
                continue
            cx, cy = c

            rx, ry = _to_robot(cx, cy, H)
            if not _in_workspace(rx, ry):
                print(f"[SKIP] {color} at pixel ({cx},{cy}) → ({rx:.1f},{ry:.1f}) outside workspace.")
                continue

            shape    = _classify_shape_for(cnt, color)
            expected = EXPECTED_SHAPE[color]
            if shape != expected:
                print(f"[WARN] {color} at ({cx},{cy}): expected {expected}, got {shape}.")

            results[color].append({"label": color, "x": round(rx, 2), "y": round(ry, 2), "shape": shape})
            print(f"[{color.upper()}] shape={shape}  pixel=({cx},{cy})  robot=({rx:.2f},{ry:.2f}) mm")

    return results


def _draw_results(img, objects_by_color, H):
    out   = img.copy()
    H_inv = np.linalg.inv(H)
    font  = cv2.FONT_HERSHEY_SIMPLEX
    BGR   = {"red": (0, 0, 255), "blue": (255, 0, 0), "green": (0, 200, 0)}

    def r2p(rx, ry):
        pt = cv2.perspectiveTransform(np.array([[[rx, ry]]], dtype=np.float32), H_inv)
        return int(pt[0][0][0]), int(pt[0][0][1])

    for color, objs in objects_by_color.items():
        for obj in objs:
            x, y = r2p(obj["x"], obj["y"])
            cv2.circle(out, (x, y), 10, BGR[color], -1)
            cv2.putText(out, f"{color} ({obj['x']:.1f},{obj['y']:.1f})",
                        (x + 8, y - 8), font, 0.55, BGR[color], 2)
    return out


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def process_image(image_path: str, debug: bool = False):
    """
    Parameters
    ----------
    image_path : str   Path to input image.
    debug      : bool  If True, saves _debug.jpg and returns it as 3rd value.

    Returns
    -------
    summary_data : list[dict]
        [{"red": int}, {"blue": int}, {"green": int}, {"total": int}, {"extra": None}]

    coordinates_data : list[dict]
        [{"label": str, "x": float, "y": float}, ...]  ordered red → blue → green

    debug_img : np.ndarray  (only when debug=True)
    """
    img = cv2.imread(image_path)
    if img is None:
        raise Exception(f"Could not load image: '{image_path}'")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H   = _compute_homography(img)

    objects_by_color = _detect_objects(hsv, H)

    n_red   = len(objects_by_color["red"])
    n_blue  = len(objects_by_color["blue"])
    n_green = len(objects_by_color["green"])

    summary_data = [
        {"red":   n_red},
        {"blue":  n_blue},
        {"green": n_green},
        {"total": n_red + n_blue + n_green},
        {"extra": None},
    ]

    coordinates_data = []
    for color in COLOR_ORDER:
        for obj in objects_by_color[color]:
            coordinates_data.append({"label": obj["label"], "x": obj["x"], "y": obj["y"]})

    if debug:
        dbg  = _draw_results(img, objects_by_color, H)
        path = image_path.rsplit(".", 1)[0] + "_debug.jpg"
        cv2.imwrite(path, dbg)
        print(f"[INFO] Debug image saved → {path}")
        return summary_data, coordinates_data, dbg

    return summary_data, coordinates_data


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    path = r"D:\Reto\foto.jpg"
    if len(sys.argv) > 1:
        path = sys.argv[1]

    try:
        result = process_image(path, debug=True)
        summary, coordinates, debug_img = result

        print("\n===== SUMMARY =====")
        for item in summary:
            print(item)

        print("\n===== COORDINATES =====")
        for obj in coordinates:
            print(obj)

        cv2.imshow("Detections", debug_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"\n[ERROR] {e}")