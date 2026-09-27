"""Joint-coordinate signs at the operator's camera-direction boundary.

The current mixed-drive station was measured in MANUAL/HOLD: positive GM6020
yaw turns the camera left; positive CyberGear pitch points it down. Pixels use
right/down-positive coordinates. This conversion belongs at the display/input
boundary; motor feedback and tracking continue to use joint coordinates.
"""

YAW_POSITIVE_SCREEN_SIGN = -1
PITCH_POSITIVE_SCREEN_SIGN = 1

DIRECTION_JS = f"""
const otaJointScreenSign = Object.freeze({{
  yaw: {YAW_POSITIVE_SCREEN_SIGN}, pitch: {PITCH_POSITIVE_SCREEN_SIGN}
}});
function otaJogForArrow(arrow) {{
  const directions = {{left: ["yaw", -1], right: ["yaw", 1],
                      up: ["pitch", -1], down: ["pitch", 1]}};
  const d = directions[arrow];
  if (!d) throw new Error("unknown camera direction: " + arrow);
  return d[0] + (d[1] * otaJointScreenSign[d[0]] > 0 ? "+" : "-");
}}
function otaAxisArrow(axis, jointDirection) {{
  const screenDirection = jointDirection * otaJointScreenSign[axis];
  if (axis === "yaw") return screenDirection < 0 ? "\\u2190" : "\\u2192";
  if (axis === "pitch") return screenDirection < 0 ? "\\u2191" : "\\u2193";
  throw new Error("unknown joint axis: " + axis);
}}
"""
